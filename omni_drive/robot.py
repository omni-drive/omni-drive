import asyncio
import atexit
import math
import threading
import time
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from functools import partial

from omni_drive.channels import get_esp_telemetry_subscriber, get_lidar_subscriber
from omni_drive.controller import MotionController
from omni_drive.generated import robot_pb2
from omni_drive.hardware import BATTERY_EMPTY_V, BATTERY_FULL_V
from omni_drive.messages import LidarScan
from omni_drive.models import DriveVector, Pose, Time, UltrasonicSensorId
from omni_drive.sensors import DEFAULT_ULTRASONIC, UltrasonicSensor

STATUS_RULE = "-" * 56


class Wheel:
    def __init__(self, on_change: Callable[[], None]):
        self._speed = 0.0
        self._on_change = on_change

    @property
    def speed(self) -> float:
        return self._speed

    @speed.setter
    def speed(self, value: float) -> None:
        self._speed = max(-1.0, min(1.0, float(value)))
        self._on_change()

    def forget(self) -> None:
        self._speed = 0.0


class OmniDrive:
    _instance: "OmniDrive | None" = None
    _instance_lock = threading.Lock()

    def __new__(cls) -> "OmniDrive":
        with cls._instance_lock:
            if cls._instance is None:
                robot = super().__new__(cls)
                robot._connect()
                cls._instance = robot
            return cls._instance

    def _connect(self) -> None:
        self._closed = False
        self._batching = False
        self._telemetry = get_esp_telemetry_subscriber()
        self._lidar = get_lidar_subscriber()
        self._motion = MotionController(read_yaw=self._odometry_yaw)
        self._wheels = tuple(Wheel(on_change=self._send_wheel_speeds) for _ in range(3))
        self._ultrasonic = tuple(
            UltrasonicSensor(
                sensor_id,
                DEFAULT_ULTRASONIC[sensor_id],
                read_hardware=partial(self._hardware_ultrasonic, sensor_id.value),
                latest_scan=self._lidar.latest_with_count,
                seed=sensor_id.value,
            )
            for sensor_id in UltrasonicSensorId
        )
        atexit.register(self.close)

    def close(self) -> None:
        cls = type(self)
        with cls._instance_lock:
            if self._closed:
                return
            self._closed = True
            if cls._instance is self:
                cls._instance = None
        atexit.unregister(self.close)
        self._motion.close()
        self._telemetry.close()
        self._lidar.close()

    @property
    def is_current(self) -> bool:
        return type(self)._instance is self and not self._closed

    def set_vector(self, vector: DriveVector) -> None:
        self._ensure_open()
        self._forget_wheel_speeds()
        self._motion.set_target(vector)

    def stop(self) -> None:
        if not self._closed:
            self._forget_wheel_speeds()
            self._motion.set_target(DriveVector.zero())

    async def drive_for(self, vector: DriveVector, duration: Time) -> None:
        self.set_vector(vector)
        try:
            await asyncio.sleep(duration.seconds)
        finally:
            self.stop()

    @contextmanager
    def batch(self) -> Iterator[None]:
        self._batching = True
        try:
            yield
        finally:
            self._batching = False
            self._send_wheel_speeds()

    def reset_yaw(self) -> None:
        self._motion.request_yaw_reset()

    @property
    def field_centric(self) -> bool:
        return self._motion.field_centric

    @field_centric.setter
    def field_centric(self, enabled: bool) -> None:
        self._motion.field_centric = enabled

    @property
    def wheel_1(self) -> Wheel:
        return self._wheels[0]

    @property
    def wheel_2(self) -> Wheel:
        return self._wheels[1]

    @property
    def wheel_3(self) -> Wheel:
        return self._wheels[2]

    @property
    def us_1(self) -> UltrasonicSensor:
        return self._ultrasonic[0]

    @property
    def us_2(self) -> UltrasonicSensor:
        return self._ultrasonic[1]

    @property
    def us_3(self) -> UltrasonicSensor:
        return self._ultrasonic[2]

    @property
    def ultrasonic_sensors(self) -> tuple[UltrasonicSensor, UltrasonicSensor, UltrasonicSensor]:
        return self._ultrasonic

    @property
    def yaw(self) -> float:
        return self._motion.yaw

    @property
    def odometry(self) -> Pose | None:
        message = self._telemetry.latest
        if message is None:
            return None
        odometry = message.odometry
        return Pose(odometry.x, odometry.y, odometry.yaw_rad)

    @property
    def imu_yaw(self) -> float | None:
        message = self._telemetry.latest
        return message.imu.yaw_rad if message is not None else None

    def telemetry(self) -> tuple[robot_pb2.TelemetryResponse | None, int]:
        return self._telemetry.latest_with_count()

    def lidar(self) -> tuple[LidarScan | None, int]:
        return self._lidar.latest_with_count()

    @property
    def lidar_scan(self) -> LidarScan | None:
        return self._lidar.latest

    def print_status(self) -> None:
        message = self._telemetry.latest
        if message is None:
            print("No telemetry from the robot. Is it switched on? Wait a few seconds and retry.")
            return
        power = message.diagnostics.power
        odometry = message.odometry
        target = message.diagnostics.target_speeds
        actual = message.diagnostics.actual_speeds
        charge = (power.voltage - BATTERY_EMPTY_V) / (BATTERY_FULL_V - BATTERY_EMPTY_V) * 100.0
        charge = max(0.0, min(100.0, charge))
        critical = "  CRITICAL" if power.is_critical else ""
        readings = "  ".join(
            f"{d.meters:4.2f}" if d else " -- " for d in (s.read() for s in self._ultrasonic)
        )
        source = "emulated from LiDAR" if self.us_1.emulated else "from the robot"
        print(
            f"{STATUS_RULE}\n"
            f"Battery:    {power.voltage:5.2f} V  {power.current:5.2f} A  {charge:3.0f}%{critical}\n"
            f"Odometry:   x {odometry.x:+6.3f} m  y {odometry.y:+6.3f} m"
            f"  yaw {math.degrees(odometry.yaw_rad):+6.1f}°\n"
            f"Wheels:     target {target.wheel_1:+5.2f} {target.wheel_2:+5.2f} {target.wheel_3:+5.2f}"
            f"  actual {actual.wheel_1:+5.2f} {actual.wheel_2:+5.2f} {actual.wheel_3:+5.2f}"
            " (of max)\n"
            f"LiDAR:      {self._lidar_status()}\n"
            f"Ultrasonic: {readings} m  (front, left, right; {source})\n"
            f"{STATUS_RULE}"
        )

    def _lidar_status(self) -> str:
        scan = self._lidar.latest
        if scan is None:
            return "no scans"
        return f"{len(scan.points)} points, {time.monotonic() - scan.timestamp:.1f} s ago"

    def _odometry_yaw(self) -> float | None:
        pose = self.odometry
        return pose.yaw if pose else None

    def _hardware_ultrasonic(self, sensor_index: int) -> float | None:
        message = self._telemetry.latest
        if message is None or not message.HasField("ultrasonic"):
            return None
        readings = message.ultrasonic
        return (readings.sensor_1, readings.sensor_2, readings.sensor_3)[sensor_index]

    def _send_wheel_speeds(self) -> None:
        if self._batching:
            return
        self._ensure_open()
        self._motion.set_wheel_speeds(*(wheel.speed for wheel in self._wheels))

    def _forget_wheel_speeds(self) -> None:
        for wheel in self._wheels:
            wheel.forget()

    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError(
                "This OmniDrive connection is closed. Call OmniDrive() to connect again."
            )
