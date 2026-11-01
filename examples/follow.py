import asyncio
import math

import numpy as np

from omni_drive import Angle, Clock, Distance, DriveVector, OmniDrive

robot = OmniDrive()
robot.field_centric = False

ANGLE_DEADBAND = Angle.deg(5)
FULL_SPEED_ANGLE = Angle.deg(90)

TARGET_DISTANCE = Distance.cm(30)
DISTANCE_DEADBAND = Distance.cm(5)
MAX_DETECTION_DISTANCE = Distance.cm(50)
FULL_SPEED_DISTANCE = Distance.cm(25)
FACING_THRESHOLD = Angle.deg(30)


async def main():
    try:
        async for _ in Clock():
            polar = robot.lidar.polar
            if len(polar) == 0:
                continue

            valid = (polar[:, 0] > 0.05) & (polar[:, 0] <= MAX_DETECTION_DISTANCE.meters)
            if not np.any(valid):
                robot.stop()
                continue

            filtered_polar = polar[valid]
            nearest_idx = np.argmin(filtered_polar[:, 0])
            dist, theta = filtered_polar[nearest_idx]

            if abs(theta) < ANGLE_DEADBAND.radians:
                omega = 0.0
            else:
                omega = max(-1.0, min(1.0, theta / FULL_SPEED_ANGLE.radians))

            dist_error = dist - TARGET_DISTANCE.meters
            if abs(theta) <= FACING_THRESHOLD.radians:
                if abs(dist_error) < DISTANCE_DEADBAND.meters:
                    velocity = 0.0
                else:
                    raw_velocity = max(-1.0, min(1.0, dist_error / FULL_SPEED_DISTANCE.meters))
                    velocity = raw_velocity * math.cos(theta)
            else:
                velocity = 0.0

            robot.set_vector(DriveVector(velocity=velocity, heading=Angle(0), omega=omega))
    finally:
        robot.stop()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
