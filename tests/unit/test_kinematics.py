import pytest

from omni_drive.kinematics import body_velocity, wheel_speeds


@pytest.mark.parametrize(
    "velocity", [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (0.3, -0.4, 0.2)]
)
def test_body_velocity_inverts_wheel_speeds(velocity):
    assert body_velocity(*wheel_speeds(*velocity)) == pytest.approx(velocity)


def test_forward_motion_spins_the_side_wheels_in_opposite_directions():
    w1, w2, w3 = wheel_speeds(0.5, 0.0, 0.0)
    assert w2 == pytest.approx(0.0)
    assert w1 == pytest.approx(-w3)
    assert w3 > 0
