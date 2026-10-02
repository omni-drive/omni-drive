import asyncio
import math
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from omni_drive.hardware import LIDAR_SELF_RETURN_RANGE_M
from omni_drive.messages import LidarScan
from omni_drive.models import Distance, Time, UltrasonicSensorId, wrap_angle

POLL_INTERVAL = Time.ms(20)
FALSE_ECHO_MAX_M = 0.5
BEAM_RAYS = 7


@dataclass(frozen=True)
class UltrasonicConfig:
    angle: float
    offset: float = 0.15
    cone: float = math.radians(25)
    min_range: float = 0.02
    max_range: float = 4.0
    noise_std: float = 0.01
    outlier_prob: float = 0.03

    @property
    def face(self) -> tuple[float, float]:
        return self.offset * math.cos(self.angle), self.offset * math.sin(self.angle)

    def beam_angles(self, rays: int = BEAM_RAYS) -> np.ndarray:
        return self.angle + np.linspace(-self.cone / 2, self.cone / 2, rays)

    def in_beam(self, relative_points: np.ndarray) -> np.ndarray:
        bearing = wrap_angle(np.arctan2(relative_points[:, 1], relative_points[:, 0]) - self.angle)
        return np.abs(bearing) <= self.cone / 2


def noisy_reading(true_range: float, config: UltrasonicConfig, rng: np.random.Generator) -> float:
    true_range = float(np.clip(true_range, config.min_range, config.max_range))
    if rng.random() < config.outlier_prob:
        if rng.random() < 0.5:
            return config.max_range
        return float(rng.uniform(config.min_range, min(true_range, FALSE_ECHO_MAX_M)))
    if true_range >= config.max_range:
        return config.max_range
    noisy = true_range + rng.normal(0.0, config.noise_std)
    return float(np.clip(noisy, config.min_range, config.max_range))


DEFAULT_ULTRASONIC = {
    UltrasonicSensorId.US_1: UltrasonicConfig(angle=0.0),
    UltrasonicSensorId.US_2: UltrasonicConfig(angle=math.pi / 2),
    UltrasonicSensorId.US_3: UltrasonicConfig(angle=-math.pi / 2),
}


class UltrasonicSensor:
    def __init__(
        self,
        sensor_id: UltrasonicSensorId,
        config: UltrasonicConfig,
        read_hardware: Callable[[], float | None],
        latest_scan: Callable[[], tuple[LidarScan | None, int]],
        seed: int | None = None,
    ):
        self.sensor_id = sensor_id
        self.config = config
        self.emulated = True
        self._read_hardware = read_hardware
        self._latest_scan = latest_scan
        self._rng = np.random.default_rng(seed)
        self._emulated_scan_count = -1
        self._emulated_reading: Distance | None = None

    def read(self) -> Distance | None:
        measured = self._read_hardware()
        self.emulated = measured is None
        if measured is not None:
            return Distance.m(measured) if measured > 0 else None
        scan, count = self._latest_scan()
        if scan is None:
            return None
        if count != self._emulated_scan_count:
            self._emulated_scan_count = count
            true_range = self.ideal_range(scan.points_in_meters())
            self._emulated_reading = Distance.m(noisy_reading(true_range, self.config, self._rng))
        return self._emulated_reading

    async def wait_for(
        self, condition: Callable[[Distance], bool], polling_interval: Time = POLL_INTERVAL
    ) -> Distance:
        while True:
            reading = self.read()
            if reading is not None and condition(reading):
                return reading
            await asyncio.sleep(polling_interval.seconds)

    def ideal_range(self, points: np.ndarray) -> float:
        c = self.config
        points = points[np.hypot(points[:, 0], points[:, 1]) >= LIDAR_SELF_RETURN_RANGE_M]
        relative = points - np.array(c.face)
        distance = np.hypot(relative[:, 0], relative[:, 1])
        seen = c.in_beam(relative) & (distance >= c.min_range)
        if not seen.any():
            return c.max_range
        return float(min(distance[seen].min(), c.max_range))
