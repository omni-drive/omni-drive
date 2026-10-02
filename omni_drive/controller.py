import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, replace

from omni_drive.channels import get_esp_command_publisher
from omni_drive.generated import robot_pb2
from omni_drive.hardware import FULL_SPEED_YAW_RATE_RAD_S
from omni_drive.kinematics import wheel_speeds
from omni_drive.models import DriveVector, wrap_angle

CONTROL_RATE_HZ = 50.0
MAX_CONTROL_STEP_S = 0.1
SPEED_DEADBAND = 1e-3
ZERO_COMMANDS_BEFORE_SILENCE = 10
MAX_ACCELERATION_PER_S = 1.5
MAX_ANGULAR_ACCELERATION_PER_S = 3.0
HEADING_KP_PER_S = 8.0
HEADING_KD = 0.4
MAX_HEADING_CORRECTION_RAD_S = 2.0

type WheelSpeeds = tuple[float, float, float]


@dataclass(frozen=True)
class BodyVelocity:
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0

    @property
    def is_translating(self) -> bool:
        return math.hypot(self.vx, self.vy) >= SPEED_DEADBAND

    @property
    def is_rotating(self) -> bool:
        return abs(self.omega) >= SPEED_DEADBAND

    def rotated(self, angle: float) -> "BodyVelocity":
        c, s = math.cos(angle), math.sin(angle)
        return replace(self, vx=c * self.vx - s * self.vy, vy=s * self.vx + c * self.vy)


def wheel_commands(velocity: BodyVelocity) -> WheelSpeeds:
    speeds = wheel_speeds(velocity.vx, velocity.vy, velocity.omega)
    fastest = max(abs(s) for s in speeds)
    if fastest > 1.0:
        speeds = tuple(s / fastest for s in speeds)
    return _without_creep(speeds)


def _without_creep(speeds: WheelSpeeds) -> WheelSpeeds:
    return tuple(0.0 if abs(s) < SPEED_DEADBAND else s for s in speeds)


def _step_towards(current: float, target: float, max_step: float) -> float:
    return current + max(-max_step, min(max_step, target - current))


class AccelerationLimiter:
    def __init__(self) -> None:
        self._current = BodyVelocity()

    def reset(self) -> None:
        self._current = BodyVelocity()

    def limit(self, target: BodyVelocity, dt: float) -> BodyVelocity:
        dvx, dvy = target.vx - self._current.vx, target.vy - self._current.vy
        change = math.hypot(dvx, dvy)
        max_change = MAX_ACCELERATION_PER_S * dt
        scale = 1.0 if change <= max_change else max_change / change
        self._current = BodyVelocity(
            self._current.vx + dvx * scale,
            self._current.vy + dvy * scale,
            _step_towards(self._current.omega, target.omega, MAX_ANGULAR_ACCELERATION_PER_S * dt),
        )
        return self._current


class HeadingHold:
    def __init__(self) -> None:
        self._held_yaw: float | None = None
        self._last_error = 0.0

    def release(self) -> None:
        self._held_yaw = None

    def correct(self, command: BodyVelocity, yaw: float, dt: float) -> BodyVelocity:
        if command.is_rotating or not command.is_translating:
            self.release()
            return command
        if self._held_yaw is None:
            self._held_yaw = yaw
            self._last_error = 0.0
        error = wrap_angle(self._held_yaw - yaw)
        error_rate = (error - self._last_error) / dt if dt > 0 else 0.0
        self._last_error = error
        yaw_rate = HEADING_KP_PER_S * error + HEADING_KD * error_rate
        yaw_rate = max(-MAX_HEADING_CORRECTION_RAD_S, min(MAX_HEADING_CORRECTION_RAD_S, yaw_rate))
        return replace(command, omega=yaw_rate / FULL_SPEED_YAW_RATE_RAD_S)


class MotionController:
    def __init__(self, read_yaw: Callable[[], float | None]):
        self.field_centric = True
        self._read_yaw = read_yaw
        self._publisher = get_esp_command_publisher()
        self._limiter = AccelerationLimiter()
        self._heading_hold = HeadingHold()
        self._lock = threading.Lock()
        self._target: BodyVelocity | WheelSpeeds = BodyVelocity()
        self._yaw = 0.0
        self._yaw_offset = 0.0
        self._yaw_reset_requested = True
        self._stopping = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="OmniDrive-MotionController", daemon=True
        )
        self._thread.start()

    @property
    def yaw(self) -> float:
        with self._lock:
            return self._yaw

    def request_yaw_reset(self) -> None:
        with self._lock:
            self._yaw_reset_requested = True

    def set_target(self, vector: DriveVector) -> None:
        with self._lock:
            self._target = BodyVelocity(vector.vx, vector.vy, vector.omega)

    def set_wheel_speeds(self, w1: float, w2: float, w3: float) -> None:
        with self._lock:
            self._target = (w1, w2, w3)

    def close(self) -> None:
        self._stopping.set()
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=1.0)
        self._send((0.0, 0.0, 0.0))
        self._publisher.close()

    def _run(self) -> None:
        period = 1.0 / CONTROL_RATE_HZ
        zero_commands_sent = 0
        last = time.monotonic()
        while not self._stopping.is_set():
            now = time.monotonic()
            speeds = self._next_wheel_speeds(min(now - last, MAX_CONTROL_STEP_S))
            last = now
            zero_commands_sent = 0 if any(speeds) else zero_commands_sent + 1
            if zero_commands_sent <= ZERO_COMMANDS_BEFORE_SILENCE:
                self._send(speeds)
            self._stopping.wait(max(0.0, period - (time.monotonic() - now)))

    def _next_wheel_speeds(self, dt: float) -> WheelSpeeds:
        yaw = self._update_yaw()
        with self._lock:
            target = self._target
        if not isinstance(target, BodyVelocity):
            self._limiter.reset()
            self._heading_hold.release()
            return _without_creep(target)
        command = self._limiter.limit(target, dt)
        command = self._heading_hold.correct(command, yaw, dt)
        if self.field_centric:
            command = command.rotated(-yaw)
        return wheel_commands(command)

    def _update_yaw(self) -> float:
        raw_yaw = self._read_yaw()
        with self._lock:
            if raw_yaw is not None:
                if self._yaw_reset_requested:
                    self._yaw_reset_requested = False
                    self._yaw_offset = raw_yaw
                    self._heading_hold.release()
                self._yaw = wrap_angle(raw_yaw - self._yaw_offset)
            return self._yaw

    def _send(self, speeds: WheelSpeeds) -> None:
        command = robot_pb2.VelocityCommand()
        raw = command.raw_speeds
        raw.wheel_1, raw.wheel_2, raw.wheel_3 = speeds
        self._publisher.send(command)
