import time

import numpy as np
import pytest

from omni_drive import Angle, DriveVector


def test_set_vector_drives_the_robot_forward(robot, sim, wait_for):
    robot.set_vector(DriveVector(velocity=0.5, heading=Angle.deg(0)))
    assert wait_for(lambda: sim.true_pose[0] > 0.12)
    robot.stop()
    time.sleep(0.4)

    x, y, _ = sim.true_pose
    assert 0.12 < x < 0.34
    assert abs(y) < 0.03
    assert robot.odometry.x == pytest.approx(x, abs=0.03)
    assert robot.odometry.y == pytest.approx(y, abs=0.03)


def test_print_status_reports_battery_odometry_wheels_and_sensors(robot, capsys):
    robot.print_status()
    out = capsys.readouterr().out
    for label in ("Battery:", "Odometry:", "Wheels:", "LiDAR:", "Ultrasonic:"):
        assert label in out
    assert "16." in out
    assert "CRITICAL" not in out


def test_front_ultrasonic_is_emulated_from_lidar(robot):
    readings = []
    for _ in range(8):
        readings.append(robot.us_1.read().meters)
        time.sleep(0.1)
    assert robot.us_1.emulated
    assert np.median(readings) == pytest.approx(0.35, abs=0.04)


def test_stop_forgets_per_wheel_speeds(robot, wait_for):
    robot.wheel_1.speed = 0.5
    robot.stop()
    robot.wheel_2.speed = 0.2
    try:
        assert wait_for(lambda: _target_speeds(robot) == pytest.approx((0.0, 0.2, 0.0)))
    finally:
        robot.stop()


def _target_speeds(robot) -> tuple[float, float, float]:
    target = robot.telemetry()[0].diagnostics.target_speeds
    return target.wheel_1, target.wheel_2, target.wheel_3
