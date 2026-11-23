# Frames and units

## Units

The public API passes physical quantities as unit objects, not as plain numbers:

| Quantity | Type | Create | Read |
| --- | --- | --- | --- |
| distance, position | `Distance` | `Distance.m()`, `.cm()`, `.mm()` | `.meters`, `.centimeters`, `.millimeters` |
| angle, heading | `Angle` | `Angle.deg()`, `.rad()`, `.rev()` | `.degrees`, `.radians`, `.revolutions` |
| time, duration | `Time` | `Time.s()`, `.ms()` | `.seconds`, `.milliseconds` |
| linear speed | `Speed` | `Speed.mps()`, `.cmps()` | `.meters_per_second` |

```python
from omni_drive import Angle, Distance, Speed, Time

Distance.cm(25).meters       # 0.25
Angle.deg(90).radians        # 1.5707...
Time.ms(100).seconds         # 0.1

Distance.cm(30) + Distance.cm(20)      # 0.5 m
Distance.cm(30) * 2                    # 0.6 m
Distance.cm(30) / Distance.cm(10)      # 3.0, a plain number
Distance.m(0.78) / Time.s(2)           # 0.39 m/s, a Speed
Distance.cm(10) / Speed.mps(0.39)      # 0.256 s, a Time
Speed.mps(0.39) * Time.s(2)            # 0.78 m, a Distance
Distance.cm(3) ** 2                    # 0.0009, a plain number in m²
```

The rules:

- `+`, `-` and comparisons work only between values of the same type. `Distance.cm(5) + 0.1`
  or `robot.ir_front.distance < 0.3` raises `TypeError` with a hint such as
  `Distance.cm(5)`: write the unit of the number.
- Multiplying or dividing by a plain number keeps the type; dividing two values of the same
  type gives a plain number.
- `Distance / Time` is a `Speed`, `Distance / Speed` is a `Time`, `Speed * Time` is a
  `Distance`.
- `Distance * Distance` and `Distance ** 2` give a plain number in m², for example a variance.
- `-d`, `abs(d)`, `round(d, 3)` and `sum([...])` work as expected.
- `float(q)` is the value in SI units (meters, radians, seconds, m/s), so `math.cos(angle)`
  and `np.asarray(distances, dtype=float)` work.
- `Distance.infinite()` is what a sensor returns without a reading; check it with
  `d.is_finite` or `math.isfinite(d)`.
- Values print with their unit: `f"{d:.3f}"` gives `0.123 m`, an `Angle` prints in degrees
  (`90.0°`).

See [`omni_drive.units`](../reference/units.md) for every member.

A few values have no unit type:

| Value | Unit |
| --- | --- |
| `DriveVector.velocity`, `DriveVector.omega`, `wheel_N.speed` | fraction of full wheel speed, −1 to 1 |
| `LidarScan.points` | **millimeters**, internal frame (below) |
| `LidarScan.point_cloud` | meters |
| `LidarScan.polar` | `r` in meters, minus 0.1242 m and clamped at 0; `theta` in radians |
| `LidarScan.timestamp` | seconds of `time.monotonic()` |
| recorder data (`rec.data`) | plain floats in SI units |

## Axes

Everything you read from or send to the robot uses the same axes:

- **x** points forward;
- **y** points **right**;
- angles (`heading`, `yaw`) and turns (`omega`) are positive **clockwise** seen from above:
  0° is forward, 90° right, 180° back, −90° (or 270°) left.

| Direction | Angle | Distance sensors | LiDAR beam |
| --- | --- | --- | --- |
| front | 0° | `ir_front`, `us_front` | `beam_front` |
| right | 90° | `ir_right`, `us_right` | `beam_right` |
| left | −90° | `ir_left`, `us_left` | `beam_left` |

`robot.pose.yaw`, `robot.heading` and `robot.imu_yaw` are `Angle` values wrapped to
−180° … 180°.

!!! note "Internal frame"
    The firmware, the motion controller, the LiDAR driver and the SLAM code use the ROS
    convention (x forward, y left, counter-clockwise positive). The library converts at the
    public API, which only flips the sign of y, of angles and of turn rates. You meet the
    internal frame only in `LidarScan.points`, `LidarScan.raw_point_cloud`, the `Odom` line of
    `print_status()` and the [`Localizer`](../guides/localization.md) estimates.

## Wheels

Positive wheel speeds turn the robot clockwise. The wheels sit at the same distance from the
center:

| Wheel | Position, clockwise from the front |
| --- | --- |
| `wheel_1` | 300° = −60° (front left) |
| `wheel_2` | 180° (rear) |
| `wheel_3` | 60° (front right) |

Working out the speed of each wheel for a given `vx`, `vy` and `omega` (the inverse
kinematics) is task 1 of the lab.

## Frames

**Robot frame.** Fixed to the robot, origin at its center. The directions of the distance
sensors and `LidarScan.point_cloud` are in this frame: a point at `[1.0, 0.2]` is 1 m ahead
and 0.2 m to the right. With `robot.field_centric = False`, `DriveVector.heading` is in this
frame too.

**Start frame.** Fixed to the floor. It is the robot frame at a moment when a value was started
or reset. Each part of the library has its own moment:

| Value | Start frame is the robot's pose when |
| --- | --- |
| `robot.pose` | `robot.reset_pose()` was last called, or `robot.pose` was first read (`print_status()` and `Localizer` also read it) |
| `robot.heading` | same as `robot.pose`; after `calibrate_imu()` it comes from the gyroscope |
| `robot.imu_yaw` | `robot.reset_pose()` was last called, or shortly after `OmniDrive()` was created |
| field-centric `heading`, LiDAR beams, the `cos(heading)` factor of the time-of-flight sensors | same as `robot.heading` |
| odometry on the ESP32 (`Odom` line of `print_status()`) | the ESP32 powered on |
| `Localizer` estimates | the localizer started, or `loc.reset()` |

Call `robot.reset_pose()` before an experiment so that all of them start together.

## Poses

A `Pose(x, y, yaw)` is a position and heading, with `x` and `y` as `Distance` and `yaw` as
`Angle`. It unpacks into its three fields:

```python
from omni_drive import Distance

x, y, yaw = robot.pose
print(f"{x:.3f} ahead, {y:.3f} to the right, turned {yaw:.1f}")
robot.pose.distance_to(robot.pose)  # Distance between two poses
```

`robot.pose` is the firmware's odometry pose made relative to the origin that `reset_pose()`
stores.

The [`Localizer`](../guides/localization.md) works with `RawPose` from
`omni_drive.processing.models`, a pose made of plain floats in meters and radians in the
internal frame (y left, counter-clockwise). Besides `relative_to` it has `compose`, which does
the opposite:

```python
from omni_drive.processing.models import RawPose

p = RawPose(1.0, 0.0, 1.5708)          # the robot's pose in the start frame
w = p.compose(RawPose(1.0, 0.2))       # a point 1 m ahead and 0.2 m left of the robot
print(w.x, w.y)                        # the same point in the start frame
w.relative_to(p)                       # RawPose(x=1.0, y=0.2, ...) again
```

`a.compose(b)` applies `b`, given in `a`'s frame, on top of `a`.
