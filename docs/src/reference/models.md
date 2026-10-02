# omni_drive.models

Small value types. All of them are frozen dataclasses; changing a field requires creating a new
object.

## DriveVector

```python
DriveVector(velocity: float, heading: Angle, omega: float = 0.0)
```

A drive command for [`set_vector`](omni_drive.md#set_vector) and
[`drive_for`](omni_drive.md#drive_for).

| Field | Description |
| --- | --- |
| `velocity` | speed as a fraction of full speed, 0 to 1; 1.0 is about 0.39 m/s |
| `heading` | direction of travel; 0 is forward, `Angle.deg(90)` is left |
| `omega` | turn rate as a fraction of full speed; 1.0 is about 3.9 rad/s, positive is counter-clockwise |

The exact conversion constants are defined in [`omni_drive.hardware`](hardware.md).

| Member | Description |
| --- | --- |
| `vx: float` | `velocity * cos(heading)`, the forward component |
| `vy: float` | `velocity * sin(heading)`, the left component |
| `DriveVector.zero()` | a vector with every field 0 |

```python
DriveVector(0.3, Angle.deg(45)).vx  # 0.212
```

## Angle

```python
Angle(radians: float)
```

| Member | Description |
| --- | --- |
| `Angle.rad(value)` | from radians |
| `Angle.deg(value)` | from degrees |
| `Angle.rev(value)` | from revolutions (1 = 360°) |
| `radians: float` | the value in radians |
| `degrees: float` | the value in degrees |
| `revolutions: float` | the value in revolutions |

## Distance

```python
Distance(meters: float)
```

| Member | Description |
| --- | --- |
| `Distance.m(value)`, `Distance.cm(value)`, `Distance.mm(value)` | from meters, centimeters, millimeters |
| `meters: float` | the value in meters |
| `as_cm: float`, `as_mm: float` | the value in centimeters, millimeters |

## Time

```python
Time(seconds: float)
```

| Member | Description |
| --- | --- |
| `Time.s(value)`, `Time.ms(value)` | from seconds, milliseconds |
| `seconds: float` | the value in seconds |

## Pose

```python
Pose(x: float = 0.0, y: float = 0.0, yaw: float = 0.0)
```

A position in meters and a heading in radians: x forward, y left, yaw counter-clockwise. See
[Frames and units](../concepts/frames.md#poses).

| Method | Description |
| --- | --- |
| `compose(local: Pose) -> Pose` | applies `local`, given in this pose's frame, on top of this pose |
| `relative_to(origin: Pose) -> Pose` | this pose seen from `origin`; the inverse of `compose` |
| `distance_to(other: Pose) -> float` | straight-line distance between the two positions, in meters |
| `advanced(dx: float, dy: float, dyaw: float) -> Pose` | moves by an increment given in this pose's frame (meters, radians), using the heading halfway through the turn, as the firmware integrates odometry |
| `increment_to(target: Pose) -> tuple[float, float, float]` | the increment `(dx, dy, dyaw)` that `advanced` needs to reach `target`; the inverse of `advanced` |

```python
a = Pose(1.0, 0.0, Angle.deg(90).radians)
b = a.compose(Pose(0.5, 0.0))  # Pose(x=1.0, y=0.5, yaw=1.5708)
b.relative_to(a)               # Pose(x=0.5, y=0.0, yaw=0.0), up to rounding

c = a.advanced(0.5, 0.0, Angle.deg(90).radians)  # drive 0.5 m while turning 90°
a.increment_to(c)                                # (0.5, 0.0, 1.5708), up to rounding
```

## wrap_angle

```python
wrap_angle(radians: float) -> float
```

Returns the same angle in the range [−π, π). The function also operates element-wise on NumPy
arrays.

## Enumerations

| Enum | Members |
| --- | --- |
| `WheelId` | `WHEEL_1`, `WHEEL_2`, `WHEEL_3` |
| `UltrasonicSensorId` | `US_1`, `US_2`, `US_3` |

`robot.us_1.sensor_id` is `UltrasonicSensorId.US_1`, and `sensor_id.name` is `"US_1"`.
