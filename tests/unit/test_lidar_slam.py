import math

import numpy as np
import pytest

from omni_drive.lidar_slam import LidarSlam
from omni_drive.models import Pose, wrap_angle
from omni_drive.simulator import box_walls, cast_rays

ROOM_HALF_X, ROOM_HALF_Y = 3.013, 2.021
BEAMS = np.linspace(-2.1, 2.1, 680)
YAW_TOL = math.radians(1.5)
SEGMENTS = np.array(
    box_walls(-ROOM_HALF_X, -ROOM_HALF_Y, ROOM_HALF_X, ROOM_HALF_Y) + box_walls(1.8, -0.3, 2.3, 0.2)
)


def scan_room(x: float, y: float, yaw: float, noise: float, seed: int) -> np.ndarray:
    ranges = cast_rays(SEGMENTS, np.array([x, y]), yaw + BEAMS)
    ranges = ranges + np.random.default_rng(seed).normal(0, noise, len(BEAMS))
    return np.stack([ranges * np.cos(BEAMS), ranges * np.sin(BEAMS)], axis=1)


def run(poses, imu=None, noise=0.005):
    slam = LidarSlam()
    estimates = [
        Pose(
            *slam.update(scan_room(*pose, noise, seed=i), None if imu is None else imu(i, pose[2]))
        )
        for i, pose in enumerate(poses)
    ]
    start = Pose(*poses[0])
    truth = [Pose(*pose).relative_to(start) for pose in poses]
    return estimates, truth


def assert_pose(est, true, pos_tol=0.03, yaw_tol=YAW_TOL):
    assert est.distance_to(true) < pos_tol, (est, true)
    assert abs(wrap_angle(est.yaw - true.yaw)) < yaw_tol, (est, true)


def test_stationary_robot_does_not_drift():
    est, truth = run([(0.0, 0.0, 0.0)] * 50, noise=0.01)
    assert_pose(est[-1], truth[-1], pos_tol=0.01)


@pytest.mark.parametrize(
    "vx, vy, yaw", [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (1.0, 0.5, 0.7), (-1.0, 0.3, 0.7)]
)
def test_tracks_straight_motion_along_featureless_walls(vx, vy, yaw):
    poses = [(i * 0.03 * vx, i * 0.03 * vy, yaw) for i in range(41)]
    est, truth = run(poses)
    assert_pose(est[-1], truth[-1])


def test_follows_a_fast_spin_in_place_without_imu():
    poses = [(0.5, 0.2, math.radians(-30) * i) for i in range(50)]
    est, truth = run(poses)
    for e, t in zip(est, truth, strict=True):
        assert_pose(e, t)


def test_drives_a_lap_with_a_drifting_imu_and_returns_to_start():
    corners = [(-1.0, -0.8), (1.0, -0.8), (1.0, 0.8), (-1.0, 0.8), (-1.0, -0.8)]
    poses, heading = [], 0.0
    for (ax, ay), (bx, by) in zip(corners, corners[1:], strict=False):
        target = math.atan2(by - ay, bx - ax)
        while abs(wrap_angle(target - heading)) > 1e-6:
            heading += max(-0.15, min(0.15, wrap_angle(target - heading)))
            poses.append((ax, ay, heading))
        n = round(math.hypot(bx - ax, by - ay) / 0.03)
        poses += [(ax + (bx - ax) * k / n, ay + (by - ay) * k / n, heading) for k in range(n)]
    poses.append((*corners[-1], heading))

    est, truth = run(poses, imu=lambda i, yaw: yaw + math.radians(0.3) * i)
    for e, t in zip(est, truth, strict=True):
        assert_pose(e, t, pos_tol=0.05, yaw_tol=math.radians(2))
    assert_pose(est[-1], truth[-1])
