import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

from omni_drive.models import Pose, wrap_angle

CHI2_GATE_3_DOF = 14.16
MIN_PROCESS_VARIANCE = 1e-10
RESYNC_INFLATION = 4.0


@dataclass(frozen=True)
class PoseNoise:
    per_meter: float = 0.05
    yaw_per_meter: float = math.radians(3.0)
    yaw_per_rad: float = 0.08
    lidar_xy: float = 0.02
    lidar_yaw: float = math.radians(1.5)
    initial_xy: float = 0.01
    initial_yaw: float = math.radians(1.0)


class PoseKalmanFilter:
    def __init__(
        self,
        pose: Pose | None = None,
        noise: PoseNoise | None = None,
        gate: float | None = CHI2_GATE_3_DOF,
        max_rejections: int = 20,
    ):
        self.noise = noise or PoseNoise()
        self.gate = gate
        self.max_rejections = max_rejections
        self.reset(pose)

    def reset(self, pose: Pose | None = None) -> None:
        pose = pose or Pose()
        n = self.noise
        self.x = np.array([pose.x, pose.y, pose.yaw], dtype=float)
        self.P = np.diag([n.initial_xy**2, n.initial_xy**2, n.initial_yaw**2])
        self.accepted = 0
        self.rejected = 0
        self.rejected_in_row = 0
        self.last_mahalanobis2 = 0.0

    @property
    def pose(self) -> Pose:
        return Pose(float(self.x[0]), float(self.x[1]), float(self.x[2]))

    @property
    def std(self) -> tuple[float, float, float]:
        sx, sy, syaw = np.sqrt(np.maximum(np.diag(self.P), 0.0))
        return float(sx), float(sy), float(syaw)

    def predict(self, dx: float, dy: float, dyaw: float) -> None:
        mid_yaw = self.x[2] + dyaw / 2
        c, s = math.cos(mid_yaw), math.sin(mid_yaw)
        moved = self.pose.advanced(dx, dy, dyaw)
        world_dx, world_dy = moved.x - self.x[0], moved.y - self.x[1]
        self.x = np.array([moved.x, moved.y, moved.yaw])

        F = np.array([[1.0, 0.0, -world_dy], [0.0, 1.0, world_dx], [0.0, 0.0, 1.0]])
        V = np.array([[c, -s, -world_dy / 2], [s, c, world_dx / 2], [0.0, 0.0, 1.0]])
        n = self.noise
        M = np.diag(
            [
                n.per_meter**2 * abs(dx),
                n.per_meter**2 * abs(dy),
                n.yaw_per_meter**2 * math.hypot(dx, dy) + n.yaw_per_rad**2 * abs(dyaw),
            ]
        )
        M += MIN_PROCESS_VARIANCE * np.eye(3)
        self.P = F @ self.P @ F.T + V @ M @ V.T

    def update_pose(self, z: Pose, R: ArrayLike | None = None) -> bool:
        if R is None:
            n = self.noise
            R = np.diag([n.lidar_xy**2, n.lidar_xy**2, n.lidar_yaw**2])
        innovation = np.array([z.x - self.x[0], z.y - self.x[1], wrap_angle(z.yaw - self.x[2])])
        return self._correct(innovation, np.eye(3), np.asarray(R, dtype=float), self.gate)

    def _correct(
        self, innovation: np.ndarray, H: np.ndarray, R: np.ndarray, gate: float | None
    ) -> bool:
        S = H @ self.P @ H.T + R
        self.last_mahalanobis2 = float(innovation @ np.linalg.solve(S, innovation))
        if gate is not None and self.last_mahalanobis2 > gate:
            self.rejected_in_row += 1
            if self.rejected_in_row < self.max_rejections:
                self.rejected += 1
                return False
            disagreement = H.T @ innovation
            self.P = self.P + RESYNC_INFLATION * np.outer(disagreement, disagreement)
            S = H @ self.P @ H.T + R
        self.rejected_in_row = 0
        self.accepted += 1
        K = np.linalg.solve(S, H @ self.P).T
        self.x = self.x + K @ innovation
        self.x[2] = wrap_angle(self.x[2])
        joseph = np.eye(len(self.x)) - K @ H
        self.P = joseph @ self.P @ joseph.T + K @ R @ K.T
        return True
