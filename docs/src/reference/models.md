# omni_drive.models

Value types and identifiers. `LidarScan`, `Pose` and `PowerStatus` are frozen dataclasses:
changing a field requires creating a new object. `DriveVector` is a plain mutable class.

The unit types `Distance`, `Angle`, `Time` and `Speed` are documented in
[`omni_drive.units`](units.md).

## DriveVector

```python
DriveVector(velocity: float, heading: Angle, omega: float = 0.0)
```

A drive command for [`set_vector`](omni_drive.md#set_vector) and
[`drive_for`](omni_drive.md#drive_for).

| Member | Description |
| --- | --- |
| `velocity: float` | translation speed as a fraction of full speed; clamped to [−1, 1] on every assignment |
| `heading: Angle` | direction of travel; `Angle.deg(0)` is forward, `Angle.deg(90)` is right, `Angle.deg(-90)` is left. Must be an `Angle` (a plain number raises `TypeError`). Measured from the yaw reference when [`field_centric`](omni_drive.md#field_centric) is on |
| `omega: float` | turn rate as a fraction of full speed, positive = clockwise; clamped to [−1, 1] on every assignment |
| `vx: float` | the forward component of the command |
| `vy: float` | the rightward component of the command |
| `DriveVector.zero()` | a vector with every field 0 |

```python
DriveVector(2.0, Angle.deg(0)).velocity  # 1.0
```

## Point2D

```python
Point2D(x: float, y: float)
```

A `NamedTuple`. Its unit depends on where it is used: millimeters (internal frame) in
`LidarScan.points`, meters (x forward, y right) in [`PoseRecorder`](recorder.md#poserecorder) data.

## LidarScan

```python
LidarScan(points: list[Point2D] = [], timestamp: float = 0.0, yaw: float = 0.0)
```

One LiDAR scan. Read it from [`robot.lidar`](omni_drive.md#lidar).

| Member | Description |
| --- | --- |
| `points: list[Point2D]` | raw scan points in **millimeters**, internal frame (x forward, y **left**) |
| `timestamp: float` | timestamp of the scan, seconds, as delivered by the LiDAR service |
| `yaw: float` | the robot's heading in radians, internal frame (counter-clockwise), when the scan was read; 0 for scans created directly |
| `point_cloud: np.ndarray` | the points as an `(N, 2)` array in meters: column 0 = x (forward), column 1 = y (right) |
| `raw_point_cloud: np.ndarray` | the same in the internal frame (y left) |
| `polar: np.ndarray` | the points as an `(N, 2)` array `[r, theta]`: `r` is the range minus `LIDAR_OFFSET_M` (0.1242 m), clamped at 0, in meters; `theta` is `atan2(y, x)` in radians, clockwise from forward |
| `beam_front: Distance` | distance in the 0° direction, averaged over a sector (below) |
| `beam_right: Distance` | the same in the +90° direction |
| `beam_left: Distance` | the same in the −90° direction |
| `beam(angle: Angle) -> Distance` | the same in any direction, e.g. `scan.beam(Angle.deg(45))` |

The `beam_*` directions are measured from the yaw reference, not from the robot's current front:
the code subtracts `yaw` from the target direction. For each beam, the 51 points nearest in angle
to that direction are taken. If the scan has fewer than 51 points, or any of them is more than
16° off, the result is `Distance.infinite()`. Otherwise each point is projected onto the beam
direction, `LIDAR_OFFSET_M` is subtracted, the 7 lowest and 7 highest of the 51 values are dropped, and
the mean of the remaining 37 is returned, clamped at 0.

`MM_PER_M = 1000.0` and `LIDAR_OFFSET_M = 0.1242` are module constants.

## Pose

```python
Pose(x: Distance = Distance(0.0), y: Distance = Distance(0.0), yaw: Angle = Angle(0.0))
```

A position and a heading: x forward, y right, yaw clockwise. Iterating a pose yields
`x`, `y`, `yaw`, so `x, y, yaw = robot.pose` works.

| Member | Description |
| --- | --- |
| `distance_to(other: Pose) -> Distance` | straight-line distance between the two positions |

`str(pose)` and `repr(pose)` give `Pose(x=1.000 m, y=0.500 m, yaw=90.0°)`.

`OdometryPose` (same module) is the internal float pose built from the firmware's `Odometry`
message, in the internal frame; `to_public()` converts it to a `Pose`.

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
