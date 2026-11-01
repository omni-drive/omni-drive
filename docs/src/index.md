# omni_drive

`omni_drive` is a Python library for controlling the OmniDrive robot and reading its sensors
from a Jupyter notebook. The robot has three omni wheels, a 2D LiDAR, a gyroscope and three
ultrasonic sensors.

## Quickstart

Run the following code in a notebook on the robot:

```python
import asyncio

from omni_drive import Angle, DriveVector, OmniDrive

robot = OmniDrive()
robot.set_vector(DriveVector(0.3, Angle.deg(90)))  # drive left at 30 % of full speed
await asyncio.sleep(1)
robot.stop()
print(robot.odometry)
```

The robot drives left for one second, stops and prints its odometry pose.
`asyncio.run` is not required, because Jupyter allows `await` at the top level of a cell.

`robot.drive_for` performs the same motion in a single call and also stops the robot if the
cell is interrupted:

```python
from omni_drive import Time

await robot.drive_for(DriveVector(0.3, Angle.deg(90)), Time.s(1))
```

Without a robot, the same code runs against the [simulator](getting-started/simulator.md).

## Where to go next

| Task | Page |
| --- | --- |
| Open a notebook on the robot | [On the robot](getting-started/robot.md) |
| Make the robot move | [Driving](guides/driving.md) |
| Read the LiDAR, gyroscope or ultrasonic sensors | [Reading sensors](guides/sensors.md) |
| Know where the robot is | [Localization](guides/localization.md) |
| Plot what the robot sees | [Visualizing](guides/visualizing.md) |
| Understand which way x, y and yaw point | [Frames and units](concepts/frames.md) |
| Look up a function | [API reference](reference/omni_drive.md) |
