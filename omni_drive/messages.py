import pickle
from dataclasses import dataclass

import numpy as np

MM_PER_M = 1000.0

type Point2D = tuple[float, float]


@dataclass(frozen=True, slots=True)
class LidarScan:
    points: list[Point2D]
    timestamp: float

    def points_in_meters(self) -> np.ndarray:
        return np.asarray(self.points, dtype=float).reshape(-1, 2) / MM_PER_M

    def to_bytes(self) -> bytes:
        return pickle.dumps(self, protocol=5)

    @classmethod
    def from_bytes(cls, data: bytes) -> "LidarScan":
        return pickle.loads(data)
