import numpy as np
import pytest

from omni_drive.models import UltrasonicSensorId
from omni_drive.sensors import DEFAULT_ULTRASONIC, UltrasonicSensor


def test_emulated_range_ignores_lidar_hits_on_the_robot_body():
    front = UltrasonicSensorId.US_1
    sensor = UltrasonicSensor(
        front, DEFAULT_ULTRASONIC[front], read_hardware=lambda: None, latest_scan=lambda: (None, 0)
    )
    points = np.array([[0.175, 0.0], [0.9, 0.0]])
    assert sensor.ideal_range(points) == pytest.approx(0.75)
