# omni_drive.models

Value types and identifiers. `Time`, `Distance`, `Angle`, `LidarScan`, `Pose` and `PowerStatus`
are frozen dataclasses: changing a field requires creating a new object. `DriveVector` is a
plain mutable class.

## Time

```python
Time(seconds: float)
```

| Member | Description |
| --- | --- |
| `Time.s(value)`, `Time.ms(value)` | from seconds, milliseconds |
| `Time.inf` | infinite time (class attribute) |
| `seconds: float` | the value in seconds |

## Distance

```python
Distance(meters: float)
```

| Member | Description |
| --- | --- |
| `Distance.m(value)`, `Distance.cm(value)`, `Distance.mm(value)` | from meters, centimeters, millimeters |
| `Distance.inf` | infinite distance (class attribute); sensors return it when there is no reading |
| `meters: float` | the value in meters |
| `as_cm: float`, `as_mm: float` | the value in centimeters, millimeters |

## Angle

```python
Angle(radians: float)
```

| Member | Description |
| --- | --- |
| `Angle.rad(value)` | from radians |
| `Angle.deg(value)` | from degrees |
| `Angle.rev(value)` | from revolutions (1 = 360°) |
| `radians: float` | the value in radians, as given (not wrapped) |
| `degrees: float` | the value in degrees, wrapped to [0, 360) |
| `revolutions: float` | the value in revolutions (not wrapped) |

```python
Angle.deg(-90).degrees  # 270.0
Angle.deg(-90).radians  # -1.5708
```

`Time`, `Distance` and `Angle` support comparison (`<`, `>=`, ...) between values of the same
type, by their stored field. They have no arithmetic operators.

## DriveVector

```python
DriveVector(velocity: float, heading: Angle, omega: float = 0.0)
```

A drive command for [`set_vector`](omni_drive.md#set_vector) and
[`drive_for`](omni_drive.md#drive_for).

| Member | Description |
| --- | --- |
| `velocity: float` | translation speed as a fraction of full speed; clamped to [−1, 1] on every assignment |
| `heading: Angle` | direction of travel; 0 is forward, `Angle.deg(90)` is left. Measured from the yaw reference when [`field_centric`](omni_drive.md#field_centric) is on |
| `omega: float` | turn rate as a fraction of full speed; clamped to [−1, 1] on every assignment |
| `vx: float` | the forward component of the command |
| `vy: float` | the left component of the command |
| `DriveVector.zero()` | a vector with every field 0 |

```python
DriveVector(2.0, Angle.deg(0)).velocity  # 1.0
```

## Point2D

```python
Point2D(x: float, y: float)
```

A `NamedTuple`. Its unit depends on where it is used: millimeters in `LidarScan.points`,
meters in [`PoseRecorder`](recorder.md#poserecorder) data.

## LidarScan

```python
LidarScan(points: list[Point2D] = [], timestamp: float = 0.0, yaw: float = 0.0)
```

One LiDAR scan. Read it from [`robot.lidar`](omni_drive.md#lidar).

| Member | Description |
| --- | --- |
| `points: list[Point2D]` | scan points in **millimeters**; x forward, y left |
| `timestamp: float` | timestamp of the scan, seconds, as delivered by the LiDAR service |
| `yaw: float` | `robot.imu_yaw` in radians when the scan was read; 0 for scans created directly |
| `point_cloud: np.ndarray` | the points as an `(N, 2)` array in meters |
| `polar: np.ndarray` | the points as an `(N, 2)` array `[r, theta]`: `r` is the range minus `LIDAR_OFFSET_M` (0.1242 m), clamped at 0, in meters; `theta` is `atan2(y, x)` in radians |
| `beam_front: Distance` | distance in the 0° direction, averaged over a sector (below) |
| `beam_left: Distance` | the same in the +90° direction |
| `beam_right: Distance` | the same in the −90° direction |

The `beam_*` directions are measured from the yaw reference, not from the robot's current front:
the code subtracts `yaw` from the target direction. For each beam, the 51 points nearest in angle
to that direction are taken. If the scan has fewer than 51 points, or any of them is more than
16° off, the result is `Distance.inf`. Otherwise each point is projected onto the beam
direction, `LIDAR_OFFSET_M` is subtracted, the 7 lowest and 7 highest of the 51 values are dropped, and
the mean of the remaining 37 is returned, clamped at 0.

`MM_PER_M = 1000.0` and `LIDAR_OFFSET_M = 0.1242` are module constants.

## Pose

```python
Pose(x: Distance = Distance(0.0), y: Distance = Distance(0.0), yaw: Angle = Angle(0.0))
```

A position and a heading: x forward, y left, yaw counter-clockwise. Iterating a pose yields
`x`, `y`, `yaw`, so `x, y, yaw = robot.pose` works.

| Member | Description |
| --- | --- |
| `relative_to(origin: Pose) -> Pose` | this pose seen from `origin`; yaw wrapped to [−π, π) |
| `Pose.from_odometry(odometry) -> Pose` | from the firmware's protobuf `Odometry` message (`x_m`, `y_m`, `yaw_rad`) |

`str(pose)` and `repr(pose)` give `Pose(x=1.000m, y=0.500m, yaw=90.0°)`, with yaw in degrees
in [0, 360).

For pose arithmetic in plain floats (`compose`, `distance_to`, `advanced`, `increment_to`), see
[`RawPose`](localization.md#rawpose).

## PowerStatus

```python
PowerStatus(voltage: float = 0.0, current: float = 0.0, percentage: float = 0.0, is_critical: bool = False)
```

Battery status, read from [`robot.power`](omni_drive.md#power). Iterating yields the four fields
in order, so `voltage, current, percentage, is_critical = robot.power` works.

| Field | Description |
| --- | --- |
| `voltage` | battery voltage, volts |
| `current` | current, amperes |
| `percentage` | battery charge, percent |
| `is_critical` | the firmware reports a critical battery |

## wrap_angle

```python
wrap_angle(radians: float) -> float
```

Returns the same angle in the range [−π, π). Defined in `omni_drive.common` and importable from
`omni_drive.models`. It also works element-wise on NumPy arrays.

## Enumerations

| Enum | Members (value) |
| --- | --- |
| `WheelId` | `WHEEL_1` (0), `WHEEL_2` (1), `WHEEL_3` (2) |
| `IrSensorId` | `LEFT` (0), `FRONT` (1), `RIGHT` (2) |
| `UltrasonicSensorId` | `LEFT` (0), `FRONT` (1), `RIGHT` (2) |

`robot.us_front.index` is `UltrasonicSensorId.FRONT`, and `index.name` is `"FRONT"`.
