import asyncio
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from omni_drive.generated.robot_pb2 import TelemetryResponse
from omni_drive.hardware import LIDAR_SELF_RETURN_RANGE_M
from omni_drive.kalman import PoseKalmanFilter, PoseNoise
from omni_drive.lidar_slam import LidarSlam, MapSnapshot
from omni_drive.models import Pose, wrap_angle
from omni_drive.robot import OmniDrive
from omni_drive.sensors import BEAM_RAYS, UltrasonicSensor

IDLE_POLL_S = 0.002
READY_POLL_S = 0.05
JOIN_TIMEOUT_S = 1.0
SCAN_RATE_SMOOTHING = 0.2
MATCH_TIME_SMOOTHING = 0.1
NO_LIDAR_POSE = Pose(math.nan, math.nan, math.nan)


@dataclass(frozen=True)
class Estimate:
    t: float
    odometry: Pose
    lidar: Pose | None
    kalman: Pose
    kalman_std: tuple[float, float, float]
    kalman_cov: np.ndarray = field(repr=False)
    lidar_confident: bool
    lidar_score: float
    lidar_accepted: bool

    def as_dict(self) -> dict[str, float | bool]:
        lidar = self.lidar or NO_LIDAR_POSE
        return {
            "t": self.t,
            "odom_x": self.odometry.x,
            "odom_y": self.odometry.y,
            "odom_yaw": self.odometry.yaw,
            "lidar_x": lidar.x,
            "lidar_y": lidar.y,
            "lidar_yaw": lidar.yaw,
            "kf_x": self.kalman.x,
            "kf_y": self.kalman.y,
            "kf_yaw": self.kalman.yaw,
            "kf_std_x": self.kalman_std[0],
            "kf_std_y": self.kalman_std[1],
            "kf_std_yaw": self.kalman_std[2],
            "lidar_confident": self.lidar_confident,
            "lidar_accepted": self.lidar_accepted,
        }


@dataclass(frozen=True)
class ScanFrame:
    seq: int
    points: np.ndarray
    pose: Pose


@dataclass
class _Odometry:
    origin: Pose
    previous: Pose
    first_gyro_yaw: float
    previous_gyro_yaw: float
    gyro_alive: bool = False


@dataclass
class _LidarState:
    pose: Pose | None = None
    confident: bool = False
    accepted: bool = False


class Localizer:
    _default: "Localizer | None" = None

    def __init__(
        self,
        robot: OmniDrive | None = None,
        slam: LidarSlam | None = None,
        kalman: PoseKalmanFilter | None = None,
        noise: PoseNoise | None = None,
        use_imu: bool = True,
        min_range: float = LIDAR_SELF_RETURN_RANGE_M,
        history: int = 12000,
    ):
        self.robot = robot or OmniDrive()
        self.slam = slam or LidarSlam()
        self.kalman = kalman or PoseKalmanFilter(noise=noise)
        self.use_imu = use_imu
        self.min_range = min_range
        self.scan_hz = 0.0
        self.slam_ms = 0.0
        self._lock = threading.Lock()
        self._slam_lock = threading.Lock()
        self._stopping = threading.Event()
        self._thread: threading.Thread | None = None
        self._reset_requested = True
        self._history: deque[Estimate] = deque(maxlen=history)
        self._estimate: Estimate | None = None
        self._scan: ScanFrame | None = None

    @classmethod
    def default(cls) -> "Localizer":
        shared = cls._default
        if shared is None or not shared.running or not shared.robot.is_current:
            if shared is not None:
                shared.stop()
            cls._default = cls().start()
        return cls._default

    @classmethod
    def for_robot(cls, robot: OmniDrive) -> "Localizer":
        return cls.default() if robot.is_current else cls(robot).start()

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> "Localizer":
        if not self.running:
            self._stopping.clear()
            self._thread = threading.Thread(target=self._run, name="Localizer", daemon=True)
            self._thread.start()
        return self

    def stop(self) -> None:
        self._stopping.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=JOIN_TIMEOUT_S)
        self._thread = None

    def reset(self) -> None:
        with self._lock:
            self._reset_requested = True
            self._estimate = None
            self._scan = None
            self._history.clear()

    async def wait_ready(self, timeout: float = 5.0) -> Estimate:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            estimate = self.estimate
            if estimate is not None and estimate.lidar is not None:
                return estimate
            await asyncio.sleep(READY_POLL_S)
        raise TimeoutError("No odometry or LiDAR data. Is the robot (or the simulator) running?")

    @property
    def estimate(self) -> Estimate | None:
        with self._lock:
            return self._estimate

    @property
    def scan(self) -> ScanFrame | None:
        with self._lock:
            return self._scan

    def history(self, since: float | None = None) -> list[Estimate]:
        with self._lock:
            estimates = list(self._history)
        return estimates if since is None else [e for e in estimates if e.t >= since]

    def dataframe(self, since: float | None = None):
        import pandas as pd

        return pd.DataFrame([e.as_dict() for e in self.history(since)])

    @property
    def map_version(self) -> int:
        return self.slam.map_version

    def map_snapshot(self) -> MapSnapshot | None:
        with self._slam_lock:
            return self.slam.snapshot()

    def expected_range(self, sensor: UltrasonicSensor, pose: Pose, rays: int = BEAM_RAYS) -> float:
        c = sensor.config
        face = pose.compose(Pose(*c.face))
        angles = pose.yaw + c.beam_angles(rays)
        with self._slam_lock:
            ranges = self.slam.map.raycast((face.x, face.y), angles, c.max_range)
        return float(min(ranges.min(), c.max_range))

    def _run(self) -> None:
        last_telemetry = last_scan = -1
        last_scan_time: float | None = None
        start_time = time.monotonic()
        odometry: _Odometry | None = None
        lidar = _LidarState()
        seq = 0
        while not self._stopping.is_set():
            if self._take_reset_request():
                start_time = time.monotonic()
                odometry, lidar = None, _LidarState()

            busy = False
            message, count = self.robot.telemetry()
            if message is not None and count != last_telemetry:
                last_telemetry, busy = count, True
                odometry = self._predict(message, odometry)
                self._publish_estimate(time.monotonic() - start_time, odometry, lidar)

            scan, count = self.robot.lidar()
            if scan is not None and count != last_scan and odometry is not None:
                last_scan, busy = count, True
                if last_scan_time is not None and scan.timestamp > last_scan_time:
                    self.scan_hz = _smoothed(
                        self.scan_hz, 1.0 / (scan.timestamp - last_scan_time), SCAN_RATE_SMOOTHING
                    )
                last_scan_time = scan.timestamp
                points = scan.points_in_meters()
                points = points[np.hypot(points[:, 0], points[:, 1]) >= self.min_range]
                if self._match_scan(points, self._heading_hint(odometry), lidar):
                    seq += 1
                    self._publish_scan(ScanFrame(seq, points, lidar.pose))

            if not busy:
                time.sleep(IDLE_POLL_S)

    def _take_reset_request(self) -> bool:
        with self._lock:
            if not self._reset_requested:
                return False
            self._reset_requested = False
            self._history.clear()
            with self._slam_lock:
                self.slam.reset()
            self.kalman.reset()
            return True

    def _predict(self, message: TelemetryResponse, odometry: _Odometry | None) -> _Odometry:
        raw = Pose(message.odometry.x, message.odometry.y, message.odometry.yaw_rad)
        gyro_yaw = message.imu.yaw_rad
        if odometry is None:
            return _Odometry(raw, raw, gyro_yaw, gyro_yaw)
        dx, dy, dyaw = odometry.previous.increment_to(raw)
        odometry.gyro_alive = odometry.gyro_alive or gyro_yaw != odometry.first_gyro_yaw
        if self.use_imu and odometry.gyro_alive:
            dyaw = wrap_angle(gyro_yaw - odometry.previous_gyro_yaw)
        odometry.previous, odometry.previous_gyro_yaw = raw, gyro_yaw
        self.kalman.predict(dx, dy, dyaw)
        return odometry

    def _heading_hint(self, odometry: _Odometry) -> float:
        if self.use_imu and odometry.gyro_alive:
            return odometry.previous_gyro_yaw
        return odometry.previous.yaw

    def _match_scan(self, points: np.ndarray, heading_hint: float, lidar: _LidarState) -> bool:
        started = time.perf_counter()
        try:
            with self._slam_lock:
                x, y, yaw = self.slam.update(points, heading_hint)
        except Exception as error:
            print(f"Skipped a LiDAR scan the scan matcher could not use: {error}")
            return False
        elapsed_ms = (time.perf_counter() - started) * 1000
        self.slam_ms = _smoothed(self.slam_ms, elapsed_ms, MATCH_TIME_SMOOTHING)
        lidar.pose = Pose(x, y, yaw)
        lidar.confident = self.slam.confident
        lidar.accepted = lidar.confident and self.kalman.update_pose(lidar.pose)
        return True

    def _publish_estimate(self, t: float, odometry: _Odometry, lidar: _LidarState) -> None:
        estimate = Estimate(
            t=t,
            odometry=odometry.previous.relative_to(odometry.origin),
            lidar=lidar.pose,
            kalman=self.kalman.pose,
            kalman_std=self.kalman.std,
            kalman_cov=self.kalman.P.copy(),
            lidar_confident=lidar.confident,
            lidar_score=self.slam.score,
            lidar_accepted=lidar.accepted,
        )
        with self._lock:
            if not self._reset_requested:
                self._estimate = estimate
                self._history.append(estimate)

    def _publish_scan(self, frame: ScanFrame) -> None:
        with self._lock:
            if not self._reset_requested:
                self._scan = frame


def _smoothed(average: float, sample: float, weight: float) -> float:
    return average + weight * (sample - average) if average else sample
