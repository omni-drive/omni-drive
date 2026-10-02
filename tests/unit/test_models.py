import math

import pytest

from omni_drive import Pose


def test_increment_to_inverts_advanced():
    start = Pose(0.4, -0.2, math.radians(170))
    moved = start.advanced(0.05, -0.02, math.radians(30))
    assert moved.yaw == pytest.approx(math.radians(-160))
    assert start.increment_to(moved) == pytest.approx((0.05, -0.02, math.radians(30)))
