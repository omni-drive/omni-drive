import math

import pytest

from omni_drive.kalman import PoseKalmanFilter
from omni_drive.models import Pose


def test_yaw_innovation_wraps_across_the_180_degree_seam():
    ekf = PoseKalmanFilter(Pose(0, 0, math.radians(179)))
    for _ in range(20):
        ekf.predict(0.01, 0.0, 0.0)
    assert ekf.update_pose(Pose(ekf.pose.x, ekf.pose.y, math.radians(-179)))
    assert abs(math.degrees(ekf.pose.yaw)) > 179


def test_gate_rejects_an_outlier_then_resyncs_to_a_persistent_measurement():
    ekf = PoseKalmanFilter()
    for _ in range(10):
        ekf.predict(0.01, 0.0, 0.0)
    jumped = Pose(2.0, 0.0, 0.0)

    assert not ekf.update_pose(jumped)
    assert ekf.rejected == 1
    assert ekf.pose.x == pytest.approx(0.1)

    results = [ekf.update_pose(jumped) for _ in range(ekf.max_rejections - 1)]
    assert not any(results[:-1]) and results[-1]
    assert ekf.pose.x == pytest.approx(2.0, abs=0.05)
    assert ekf.update_pose(jumped)
