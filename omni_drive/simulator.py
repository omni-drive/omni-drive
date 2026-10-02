import argparse
import logging
import math
import threading
import time
from dataclasses import dataclass, field, replace

import numpy as np
import zmq

from omni_drive.generated import robot_pb2
from omni_drive.hardware import (
    BATTERY_CRITICAL_V,
    BATTERY_FULL_V,
    COMMAND_WATCHDOG_S,
    ENCODER_TICKS_PER_M,
    FULL_SPEED_RIM_M_S,
    HOKUYO_MAX_RANGE_M,
    HOKUYO_MIN_RANGE_M,
    HOKUYO_STEP_ANGLES_RAD,
    ROBOT_RADIUS_M,
    WHEEL_BASE_RADIUS_M,
)
from omni_drive.ipc import IPC, bind_publisher
from omni_drive.kinematics import body_velocity
from omni_drive.logs import configure_logging
from omni_drive.messages import MM_PER_M, LidarScan
from omni_drive.models import Pose, UltrasonicSensorId
from omni_drive.sensors import DEFAULT_ULTRASONIC, noisy_reading

logger = logging.getLogger("Simulator")

MOTOR_TIME_CONSTANT_S = 0.08
STOPPED_WHEEL_SPEED = 0.01
PHYSICS_STEP_S = 0.002
MAX_FRAME_S = 0.1
TELEMETRY_HZ = 20.0
LIDAR_HZ = 10.0
MAX_POLL_S = 0.005

LIDAR_ANGLES = np.array(HOKUYO_STEP_ANGLES_RAD)
LIDAR_DROPOUT_PROBABILITY = 0.01
LIDAR_RELATIVE_NOISE = 0.01

BATTERY_SAG_V_PER_S = 0.4 / 600.0
INTERNAL_RESISTANCE_OHM = 0.08
IDLE_CURRENT_A = 0.35
CURRENT_PER_FULL_SPEED_WHEEL_A = 1.2

type Segment = tuple[tuple[float, float], tuple[float, float]]


def box_walls(x0: float, y0: float, x1: float, y1: float) -> list[Segment]:
    return [
        ((x0, y0), (x1, y0)),
        ((x1, y0), (x1, y1)),
        ((x1, y1), (x0, y1)),
        ((x0, y1), (x0, y0)),
    ]


@dataclass(frozen=True)
class World:
    name: str
    segments: np.ndarray = field(repr=False)


WORLDS: dict[str, list[Segment]] = {
    "box": box_walls(-0.5, -0.5, 0.5, 0.5) + box_walls(0.4, 0.4, 0.5, 0.5),
    "room": (
        box_walls(-1.6, -1.3, 2.4, 1.7)
        + box_walls(0.9, 0.7, 1.3, 1.1)
        + box_walls(-1.2, -1.0, -0.9, -0.6)
        + box_walls(1.6, -0.9, 1.7, -0.4)
    ),
}


def make_world(name: str) -> World:
    if name not in WORLDS:
        raise ValueError(f"Unknown world {name!r}; choose one of: {', '.join(WORLDS)}")
    return World(name, np.array(WORLDS[name], dtype=float))


def cast_rays(segments: np.ndarray, origins: np.ndarray, angles: np.ndarray) -> np.ndarray:
    origins = np.broadcast_to(np.asarray(origins, dtype=float), (len(angles), 2))
    starts, edges = segments[:, 0], segments[:, 1] - segments[:, 0]
    directions = np.stack([np.cos(angles), np.sin(angles)], axis=1)
    to_start = starts[None] - origins[:, None]
    denominator = (
        directions[:, None, 0] * edges[None, :, 1] - directions[:, None, 1] * edges[None, :, 0]
    )
    with np.errstate(divide="ignore", invalid="ignore"):
        distance = (
            to_start[..., 0] * edges[None, :, 1] - to_start[..., 1] * edges[None, :, 0]
        ) / denominator
        along = (
            to_start[..., 0] * directions[:, None, 1] - to_start[..., 1] * directions[:, None, 0]
        ) / denominator
    hit = (np.abs(denominator) > 1e-12) & (distance > 0) & (along >= 0) & (along <= 1)
    return np.where(hit, distance, np.inf).min(axis=1)


def clearance(segments: np.ndarray, point: np.ndarray) -> float:
    starts, edges = segments[:, 0], segments[:, 1] - segments[:, 0]
    along = np.clip(((point - starts) * edges).sum(axis=1) / (edges * edges).sum(axis=1), 0, 1)
    nearest = starts + along[:, None] * edges
    return float(np.hypot(*(nearest - point).T).min())


def body_step(rim: np.ndarray, wheel_base_radius: float) -> tuple[float, float, float]:
    dx, dy, rim_turn = body_velocity(*rim.tolist())
    return dx, dy, rim_turn / wheel_base_radius


class Simulator:
    def __init__(
        self,
        world: str | World = "box",
        seed: int | None = 0,
        publish_ultrasonic: bool = False,
        speed_factor: float = 1.0,
        wheel_scale_error: tuple[float, float, float] = (0.05, 0.03, 0.02),
        robot_radius_error: float = 0.01,
        slip_noise: float = 0.015,
        imu_drift_deg_per_min: float = 0.5,
        imu_noise_rad: float = 0.002,
        lidar_noise_m: float = 0.01,
    ) -> None:
        self.world = world if isinstance(world, World) else make_world(world)
        self.publish_ultrasonic = publish_ultrasonic
        self.speed_factor = speed_factor
        self.wheel_scale_error = np.asarray(wheel_scale_error, dtype=float)
        self.true_wheel_base_radius = WHEEL_BASE_RADIUS_M * (1.0 + robot_radius_error)
        self.slip_noise = slip_noise
        self.imu_drift_rad_s = math.radians(imu_drift_deg_per_min) / 60.0
        self.imu_noise_rad = imu_noise_rad
        self.lidar_noise_m = lidar_noise_m
        self._seed = seed
        self._lock = threading.RLock()
        self._stopping = threading.Event()
        self._thread: threading.Thread | None = None
        self._context: zmq.Context | None = None
        self._time_s = 0.0
        self.reset()

    def reset(self) -> None:
        with self._lock:
            self._rng = np.random.default_rng(self._seed)
            self._pose = Pose()
            self._odometry = Pose()
            self._partial_ticks = np.zeros(3)
            self._imu_bias = 0.0
            self._target = np.zeros(3)
            self._actual = np.zeros(3)
            self._last_command_s = -math.inf

    @property
    def true_pose(self) -> tuple[float, float, float]:
        with self._lock:
            return self._pose.x, self._pose.y, self._pose.yaw

    @property
    def odometry(self) -> tuple[float, float, float]:
        with self._lock:
            return self._odometry.x, self._odometry.y, self._odometry.yaw

    @property
    def imu_yaw(self) -> float:
        with self._lock:
            return self._pose.yaw + self._imu_bias

    @property
    def wheel_speeds(self) -> tuple[float, float, float]:
        with self._lock:
            w1, w2, w3 = self._actual
            return float(w1), float(w2), float(w3)

    def set_wheel_speeds(self, w1: float, w2: float, w3: float) -> None:
        with self._lock:
            self._target = np.clip([w1, w2, w3], -1.0, 1.0).astype(float)
            self._last_command_s = self._time_s

    def step(self, duration: float) -> None:
        with self._lock:
            substeps = max(1, round(duration / PHYSICS_STEP_S))
            for _ in range(substeps):
                self._substep(duration / substeps)

    def _substep(self, dt: float) -> None:
        self._time_s += dt
        if self._time_s - self._last_command_s > COMMAND_WATCHDOG_S:
            self._target[:] = 0.0
        self._actual += (self._target - self._actual) * (
            1.0 - math.exp(-dt / MOTOR_TIME_CONSTANT_S)
        )
        self._imu_bias += self.imu_drift_rad_s * dt
        if not self._target.any() and np.abs(self._actual).max() < STOPPED_WHEEL_SPEED:
            self._actual[:] = 0.0
            return
        rim = self._actual * FULL_SPEED_RIM_M_S * dt
        self._move(*body_step(rim, self.true_wheel_base_radius))
        self._count_encoder_ticks(rim)

    def _move(self, dx_body: float, dy_body: float, dyaw: float) -> None:
        moved = self._pose.advanced(dx_body, dy_body, dyaw)
        position = np.array([self._pose.x, self._pose.y])
        step = np.array([moved.x, moved.y]) - position
        self._pose = replace(self._pose, yaw=moved.yaw)
        if not step.any():
            return
        current_gap = clearance(self.world.segments, position)
        for attempt in (step, np.array([step[0], 0.0]), np.array([0.0, step[1]])):
            gap = clearance(self.world.segments, position + attempt)
            if gap >= ROBOT_RADIUS_M or gap > current_gap:
                x, y = (position + attempt).tolist()
                self._pose = Pose(x, y, moved.yaw)
                return

    def _count_encoder_ticks(self, rim: np.ndarray) -> None:
        slip = self._rng.normal(0.0, 1.0, 3) * self.slip_noise * np.sqrt(np.abs(rim))
        self._partial_ticks += (rim * (1.0 + self.wheel_scale_error) + slip) * ENCODER_TICKS_PER_M
        ticks = np.trunc(self._partial_ticks)
        self._partial_ticks -= ticks
        if ticks.any():
            step = body_step(ticks / ENCODER_TICKS_PER_M, WHEEL_BASE_RADIUS_M)
            self._odometry = self._odometry.advanced(*step)

    def telemetry(self) -> robot_pb2.TelemetryResponse:
        with self._lock:
            message = robot_pb2.TelemetryResponse()
            odometry = message.odometry
            odometry.x, odometry.y, odometry.yaw_rad = self.odometry
            gyro_noise = self._rng.normal(0.0, self.imu_noise_rad)
            message.imu.yaw_rad = self.imu_yaw + gyro_noise
            self._fill_power(message.diagnostics.power)
            target, actual = message.diagnostics.target_speeds, message.diagnostics.actual_speeds
            target.wheel_1, target.wheel_2, target.wheel_3 = self._target
            actual.wheel_1, actual.wheel_2, actual.wheel_3 = self._actual
            if self.publish_ultrasonic:
                readings = message.ultrasonic
                readings.sensor_1, readings.sensor_2, readings.sensor_3 = self.ultrasonic_ranges()
            return message

    def _fill_power(self, power: robot_pb2.PowerStatus) -> None:
        current = IDLE_CURRENT_A + CURRENT_PER_FULL_SPEED_WHEEL_A * float(
            np.abs(self._actual).sum()
        )
        current += self._rng.normal(0, 0.02)
        voltage = BATTERY_FULL_V - self._time_s * BATTERY_SAG_V_PER_S
        voltage -= INTERNAL_RESISTANCE_OHM * current
        voltage += self._rng.normal(0, 0.01)
        power.voltage, power.current = voltage, max(0.0, current)
        power.is_critical = voltage < BATTERY_CRITICAL_V

    def ultrasonic_ranges(self) -> tuple[float, float, float]:
        with self._lock:
            r1, r2, r3 = (self._ultrasonic_range(sensor) for sensor in UltrasonicSensorId)
            return r1, r2, r3

    def _ultrasonic_range(self, sensor: UltrasonicSensorId) -> float:
        config = DEFAULT_ULTRASONIC[sensor]
        face = self._pose.compose(Pose(*config.face))
        beam = self._pose.yaw + config.beam_angles()
        true_range = float(cast_rays(self.world.segments, np.array([face.x, face.y]), beam).min())
        return noisy_reading(true_range, config, self._rng)

    def lidar_scan(self) -> LidarScan:
        with self._lock:
            position = np.array([self._pose.x, self._pose.y])
            ranges = cast_rays(self.world.segments, position, self._pose.yaw + LIDAR_ANGLES)
            hit = np.isfinite(ranges)
            ranges, angles = ranges[hit], LIDAR_ANGLES[hit]
            noise_std = np.maximum(self.lidar_noise_m, LIDAR_RELATIVE_NOISE * ranges)
            ranges = ranges + self._rng.normal(0.0, 1.0, len(ranges)) * noise_std
            kept = (
                (ranges >= HOKUYO_MIN_RANGE_M)
                & (ranges <= HOKUYO_MAX_RANGE_M)
                & (self._rng.random(len(ranges)) > LIDAR_DROPOUT_PROBABILITY)
            )
        ranges_mm, angles = ranges[kept] * MM_PER_M, angles[kept]
        xs = np.round(ranges_mm * np.cos(angles), 1)
        ys = np.round(ranges_mm * np.sin(angles), 1)
        return LidarScan(
            points=list(zip(xs.tolist(), ys.tolist(), strict=True)), timestamp=time.monotonic()
        )

    def start(self) -> "Simulator":
        if self._thread is not None:
            return self
        self._context = zmq.Context()
        try:
            telemetry = bind_publisher(self._context, IPC.ESP_TELEMETRY)
            commands = self._context.socket(zmq.PULL)
            commands.setsockopt(zmq.LINGER, 0)
            commands.bind(IPC.ESP_COMMANDS)
            scans = bind_publisher(self._context, IPC.LIDAR_SCANS)
        except zmq.ZMQError:
            self._context.destroy(linger=0)
            self._context = None
            raise
        self._stopping.clear()
        self._thread = threading.Thread(
            target=self._run, args=(telemetry, commands, scans), name="Simulator", daemon=True
        )
        self._thread.start()
        logger.info(
            "Simulator (%s) up. PUB %s | PULL %s | PUB %s",
            self.world.name,
            IPC.ESP_TELEMETRY,
            IPC.ESP_COMMANDS,
            IPC.LIDAR_SCANS,
        )
        return self

    def stop(self) -> None:
        self._stopping.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._context is not None:
            self._context.destroy(linger=0)
            self._context = None

    def __enter__(self) -> "Simulator":
        return self.start()

    def __exit__(self, *exc: object) -> None:
        self.stop()

    def _run(self, telemetry: zmq.Socket, commands: zmq.Socket, scans: zmq.Socket) -> None:
        poller = zmq.Poller()
        poller.register(commands, zmq.POLLIN)
        last = next_telemetry = next_scan = time.monotonic()
        try:
            while not self._stopping.is_set():
                now = time.monotonic()
                self._apply_commands(commands)
                self.step(min(now - last, MAX_FRAME_S) * self.speed_factor)
                last = now
                if now >= next_telemetry:
                    telemetry.send(self.telemetry().SerializeToString())
                    next_telemetry = max(next_telemetry + 1.0 / TELEMETRY_HZ, now)
                if now >= next_scan:
                    scans.send(self.lidar_scan().to_bytes())
                    next_scan = max(next_scan + 1.0 / LIDAR_HZ, now)
                wait_s = min(next_telemetry, next_scan) - time.monotonic()
                poller.poll(timeout=max(0.0, min(wait_s, MAX_POLL_S)) * 1000.0)
        except zmq.ZMQError as error:
            if not self._stopping.is_set():
                logger.warning("Simulator stopped on ZMQ error: %s", error)

    def _apply_commands(self, commands: zmq.Socket) -> None:
        while True:
            try:
                frame = commands.recv(zmq.NOBLOCK)
            except zmq.Again:
                return
            try:
                speeds = robot_pb2.VelocityCommand.FromString(frame).raw_speeds
            except Exception as error:
                logger.warning("Ignored an unreadable VelocityCommand: %s", error)
                continue
            self.set_wheel_speeds(speeds.wheel_1, speeds.wheel_2, speeds.wheel_3)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Simulated omni-drive robot (ESP32 firmware and Hokuyo LiDAR) on the real "
        "ZMQ endpoints. On Windows set OMNI_DRIVE_IPC=tcp://127.0.0.1:5650 first."
    )
    parser.add_argument("--world", choices=tuple(WORLDS), default="box")
    parser.add_argument(
        "--ultrasonic", action="store_true", help="publish ultrasonic readings in telemetry"
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--speed-factor", type=float, default=1.0, help="run the physics this many times faster"
    )
    args = parser.parse_args()
    configure_logging()
    simulator = Simulator(
        world=args.world,
        seed=args.seed,
        publish_ultrasonic=args.ultrasonic,
        speed_factor=args.speed_factor,
    ).start()
    try:
        while True:
            time.sleep(5.0)
            x, y, yaw = simulator.true_pose
            ox, oy, oyaw = simulator.odometry
            logger.info(
                "true (%.3f, %.3f, %.1f°)  odometry (%.3f, %.3f, %.1f°)",
                x,
                y,
                math.degrees(yaw),
                ox,
                oy,
                math.degrees(oyaw),
            )
    except KeyboardInterrupt:
        logger.info("Stopping simulator...")
    finally:
        simulator.stop()


if __name__ == "__main__":
    main()
