# Frames and units

## Units

The public API passes physical quantities as `Distance`, `Angle` and `Time` objects, not as
plain numbers:

| Quantity | Type | Create | Read |
| --- | --- | --- | --- |
| distance, position | `Distance` | `Distance.m()`, `.cm()`, `.mm()` | `.meters`, `.as_cm`, `.as_mm` |
| angle, heading | `Angle` | `Angle.rad()`, `.deg()`, `.rev()` | `.radians`, `.degrees`, `.revolutions` |
| time | `Time` | `Time.s()`, `.ms()` | `.seconds` |

```python
from omni_drive import Angle, Distance, Time

Angle.deg(90).radians   # 1.5707...
Angle.rad(1.0).degrees  # 57.29...
Angle.rev(0.5).radians  # 3.1415..., half a revolution
Distance.cm(25).meters  # 0.25
Distance.m(0.25).as_mm  # 250.0
Time.ms(100).seconds    # 0.1
```

`Distance.inf` and `Time.inf` are infinite values. Objects of the same type can be compared:
`Distance.cm(20) < Distance.m(1)` is `True`. Comparing a `Distance` with a plain number raises
`TypeError`.

`Angle.degrees` is always between 0 and 360: `Angle.deg(-10).degrees` is `350.0`. Use
`.radians` when you need the sign.

A few values have no unit type:

| Value | Unit |
| --- | --- |
| `DriveVector.velocity`, `DriveVector.omega`, `wheel_N.speed` | fraction of full wheel speed, −1 to 1 |
| `LidarScan.points` | **millimeters** |
| `LidarScan.point_cloud` | meters |
| `LidarScan.polar` | `r` in meters, minus 0.1242 m and clamped at 0; `theta` in radians |
| `LidarScan.timestamp` | seconds of `time.monotonic()` |

## Axes

Every frame in the library uses the same axes:

- **x** points forward;
- **y** points left;
- **yaw** is measured from x towards y, so positive yaw is a counter-clockwise turn seen from
  above.

`robot.pose.yaw` and `robot.imu_yaw` are wrapped to −π … π in radians.

## Wheels

Positive wheel speeds turn the robot counter-clockwise. The wheels sit at the same distance
from the center:

| Wheel | Position, measured from the front |
| --- | --- |
| `wheel_1` | 60° (front left) |
| `wheel_2` | 180° (rear) |
| `wheel_3` | −60° (front right) |

Working out the speed of each wheel for a given `vx`, `vy` and `omega` (the inverse
kinematics) is task 1 of the lab.

## Frames

**Robot frame.** Fixed to the robot, origin at its center. LiDAR points and the directions of
the distance sensors are in this frame: a LiDAR point at `(1000, 0)` mm is 1 m straight ahead.
With `robot.field_centric = False`, `DriveVector.heading` is in this frame too.

**Start frame.** Fixed to the floor. It is the robot frame at a moment when a value was started
or reset. Each part of the library has its own moment:

| Value | Start frame is the robot's pose when |
| --- | --- |
| `robot.pose` | `robot.reset_pose()` was last called, or `robot.pose` was first read (`print_status()` and `Localizer` also read it) |
| `robot.imu_yaw` | `robot.reset_pose()` was last called, or shortly after `OmniDrive()` was created |
| field-centric `heading`, LiDAR beams, the `cos(imu_yaw)` factor of the time-of-flight sensors | same as `robot.imu_yaw` |
| odometry on the ESP32 (`Odom` line of `print_status()`) | the ESP32 powered on |
| `Localizer` estimates | the localizer started, or `loc.reset()` |

`robot.pose` and `robot.imu_yaw` usually share a start frame, but not always: if you never call
`reset_pose()` and read `robot.pose` for the first time after the robot has moved, the two
differ. Call `robot.reset_pose()` before an experiment to align them.

## Poses

A `Pose(x, y, yaw)` is a position and heading in some frame, with `x` and `y` as `Distance` and
`yaw` as `Angle`. It unpacks into its three fields:

```python
from omni_drive import Angle, Distance, Pose

start = Pose(Distance.m(1.0), Distance.m(0.0), Angle.deg(90))  # 1 m ahead, facing left
end = Pose(Distance.m(1.0), Distance.m(0.5), Angle.deg(90))    # 0.5 m further left

end.relative_to(start)  # Pose(x=0.500m, y=0.000m, yaw=0.0°)
x, y, yaw = end
```

`b.relative_to(a)` expresses `b` in the frame of `a`: how far `b` is ahead of and to the left of
`a`, and how much it is turned. `robot.pose` itself is the firmware's odometry pose made relative
to the origin that `reset_pose()` stores.

The [`Localizer`](../guides/localization.md) works with `RawPose` from
`omni_drive.processing.models`, a pose made of plain floats in meters and radians. Besides
`relative_to` it has `compose`, which does the opposite:

```python
from omni_drive.processing.models import RawPose

p = RawPose(1.0, 0.0, 1.5708)          # the robot's pose in the start frame
w = p.compose(RawPose(1.0, 0.2))       # a point 1 m ahead and 0.2 m left of the robot
print(w.x, w.y)                        # the same point in the start frame
w.relative_to(p)                       # RawPose(x=1.0, y=0.2, ...) again
```

`a.compose(b)` applies `b`, given in `a`'s frame, on top of `a`. `RawPose.from_pose(pose)`
converts a `Pose`.
