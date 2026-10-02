import asyncio
import math

from omni_drive import Angle, DriveVector, Localizer, OmniDrive, Pose, Time


def _error(estimate: Pose, truth: Pose) -> float:
    return math.hypot(estimate.x - truth.x, estimate.y - truth.y)


def test_localizer_estimates_follow_a_square_drive(robot, sim):
    localizer = Localizer(robot).start()
    try:
        asyncio.run(localizer.wait_ready())
        for heading in (0, 90, 180, 270):
            vector = DriveVector(velocity=0.6, heading=Angle.deg(heading))
            asyncio.run(robot.drive_for(vector, Time.s(0.7)))
            asyncio.run(asyncio.sleep(0.4))

            estimate, truth = localizer.estimate, Pose(*sim.true_pose)
            assert _error(estimate.lidar, truth) < 0.04
            assert _error(estimate.kalman, truth) < 0.03
            assert _error(estimate.odometry, truth) < 0.05
            assert abs(estimate.kalman.yaw - truth.yaw) < math.radians(3)
    finally:
        localizer.stop()


def test_shared_localizer_follows_a_reconnected_robot(robot):
    first = Localizer.for_robot(robot)
    try:
        assert Localizer.for_robot(robot) is first
        robot.close()
        reconnected = OmniDrive()
        second = Localizer.default()
        try:
            assert second.robot is reconnected
            assert not first.running
        finally:
            second.stop()
            reconnected.close()
    finally:
        first.stop()
