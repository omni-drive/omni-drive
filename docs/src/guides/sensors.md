# Reading sensors

Every reading on this page returns the latest value the robot sent and does not wait for new
data. The ESP32 sends telemetry 50 times a second. Before the first message arrives, each
reading returns a default value (listed below) instead of `None`.

The library does not notice when telemetry stops. If the robot disconnects, every reading keeps
returning the last value it received.

Directions follow [Frames and units](../concepts/frames.md). `robot.print_status()` prints
all readings at once.

## Pose

```python
robot.pose            # Pose(x=0.152m, y=-0.004m, yaw=2.3°)
x, y, yaw = robot.pose
print(x.meters, y.meters, yaw.degrees)
```

`robot.pose` is a `Pose` with `x` and `y` as `Distance` and `yaw` as `Angle`. The ESP32
computes it from the wheel encoders (see [Odometry and drift](../concepts/odometry.md)).

The pose is relative to an origin that the library keeps:

- `robot.reset_pose()` sets the origin to the current pose, so `robot.pose` reads
  `(0, 0, 0)` right after it. Call it once telemetry is arriving: before the first message it
  only clears the origin, which is then set at the next read of `robot.pose`.
- If you never call `reset_pose()`, the origin is the pose at the first read of `robot.pose`.
  `print_status()` and a running `Localizer` also read it.

`x` points along the robot's front at the origin and `y` to its left. `reset_pose()` also
resets `robot.imu_yaw` and the field-centric direction. Before the first telemetry message,
`robot.pose` is `Pose()`, all zeros.

## Gyroscope

```python
robot.imu_yaw.degrees  # e.g. 87.5
```

`robot.imu_yaw` is an `Angle`: the heading integrated from the gyroscope, relative to the
heading at the last `robot.reset_pose()`, or at the first telemetry message after `OmniDrive()`
was created (the motion controller reads it 50 times a second). `.radians` is wrapped to −π … π,
but `.degrees` is always 0–360, so a small turn to the right reads about 355°. For a signed
value in degrees use `math.degrees(robot.imu_yaw.radians)`.

The gyroscope is not affected by wheel slip, but its reading slowly drifts over time. The
ESP32 measures the gyroscope bias once, during the first second after it powers on. Keep the
robot still while it starts. If the gyroscope failed to start, `imu_yaw` stays at 0.

`robot.pose.yaw` comes from the wheels, `robot.imu_yaw` from the gyroscope. The two drift apart
as the robot moves.

## Distance sensors

The robot has three pairs of distance sensors, at the left, front and right:

| Left | Front | Right | Type |
| --- | --- | --- | --- |
| `robot.ir_left` | `robot.ir_front` | `robot.ir_right` | laser time-of-flight (VL53L1X) |
| `robot.us_left` | `robot.us_front` | `robot.us_right` | ultrasonic |

The `ir_*` sensors are time-of-flight laser sensors. `print_status()` lists them as "ToF".
`robot.ultrasonic_sensors` is the tuple `(us_left, us_front, us_right)`.

```python
d = robot.us_front.distance  # Distance
print(d.meters, d.as_cm)
```

`.distance` returns a `Distance`. It returns `Distance.inf` when the sensor has no valid
reading: before the first telemetry message, when the sensor gave no echo, or when it timed
out. Compare readings with other `Distance` values, not with plain numbers:

```python
from omni_drive import Distance

if robot.ir_front.distance < Distance.cm(30):
    print("obstacle ahead")
```

| | Time-of-flight (`ir_*`) | Ultrasonic (`us_*`) |
| --- | --- | --- |
| New reading | every 25 ms | each sensor about every 100 ms (the three are polled in turn) |
| No reading | `Distance.inf` | `Distance.inf`, also beyond 2 m |
| Correction in Python | multiplied by `cos(robot.imu_yaw)` | 0.0132 m subtracted |

The firmware adds Gaussian noise to every finite reading: a standard deviation of 15 mm for the
time-of-flight sensors and 8 mm for the ultrasonic sensors.

!!! note "Time-of-flight readings depend on `imu_yaw`"
    `ir_*.distance` multiplies the measured range by the cosine of `robot.imu_yaw`. This turns
    the slanted range to a wall into the distance along the sensor's direction in the frame of
    the last `reset_pose()`. Call `robot.reset_pose()` to make the current heading the reference.

### Wait for a reading

```python
from omni_drive import Angle, Distance, DriveVector

async with robot:
    robot.set_vector(DriveVector(velocity=0.2, heading=Angle.deg(0)))
    d = await robot.us_front.wait_for(lambda d: d < Distance.cm(30))
print(f"wall at {d.as_cm:.0f} cm")
```

`sensor.wait_for(condition, polling_interval=Time.ms(20))` reads `.distance` every
`polling_interval` and returns the first reading for which `condition` is true. It has no
timeout. Because readings freeze when telemetry stops, add one yourself if the condition may
never become true:

```python
import asyncio

async with robot:
    robot.set_vector(DriveVector(velocity=0.2, heading=Angle.deg(0)))
    d = await asyncio.wait_for(robot.us_front.wait_for(lambda d: d < Distance.cm(30)), timeout=5)
```

`asyncio.wait_for` raises `TimeoutError` after 5 s, and `async with robot:` stops the robot.

## LiDAR

```python
scan = robot.lidar          # LidarScan
pts = scan.point_cloud      # (N, 2) NumPy array, meters
print(len(scan.points), scan.timestamp)
```

`robot.lidar` returns the latest scan as a `LidarScan`. Before the first scan it returns an
empty `LidarScan()`, with no points and `timestamp` 0.0.

| Attribute | Content |
| --- | --- |
| `points` | list of `Point2D(x, y)` in **millimeters**, in the robot frame |
| `point_cloud` | the same points as an `(N, 2)` NumPy array in meters |
| `polar` | `(N, 2)` array of `[r, theta]`: `theta` in radians, `r` in meters minus a fixed 0.1242 m offset (clipped at 0) |
| `timestamp` | `time.monotonic()` on the robot when the scan arrived |
| `beam_left`, `beam_front`, `beam_right` | `Distance` in one direction, see below |

The scanner covers about 240°, centered on the robot's front. The robot's service keeps
ranges from 20 mm to 5.6 m and drops the rest, so the number of points changes from scan to
scan. Points within about 0.18 m of the center are reflections from the robot itself; the
[`Localizer`](localization.md) ignores them.

Each access to `robot.lidar` builds a new `LidarScan` object, even when no new scan has
arrived. To process each scan once, compare `timestamp`:

```python
from omni_drive import Clock, Time

last = None
async for _ in Clock(period=Time.ms(20), duration=Time.s(10)):
    scan = robot.lidar
    if scan.points and scan.timestamp != last:
        last = scan.timestamp
        ...  # handle the new scan
```

### Beams

`beam_left`, `beam_front` and `beam_right` give one distance each, at +90°, 0° and −90°.
Each one:

1. takes the 51 points closest in angle to the beam direction;
2. returns `Distance.inf` if the scan has fewer than 51 points, or if any of them is more than
   16° from the direction;
3. projects each point onto the beam direction and subtracts 0.1242 m;
4. drops the 7 lowest and 7 highest of the 51 projections and returns the mean of the other 37.

The beam directions are **not** fixed to the robot. They are measured in the frame of the last
`robot.reset_pose()`: the library subtracts `robot.imu_yaw` from each direction. When the robot
has turned by 30°, `beam_front` still looks towards the wall that was ahead at the reset. A beam
returns `Distance.inf` once its direction leaves the scanner's field of view.

## Power

```python
voltage, current, percentage, is_critical = robot.power
print(f"{robot.power.percentage:.0f} % ({robot.power.voltage:.2f} V)")
```

`robot.power` is a `PowerStatus` with `voltage` (V), `current` (A), `percentage` (estimated
charge, 0–100) and `is_critical`. `is_critical` is `True` when the battery voltage is below
13.0 V. Before the first telemetry message all fields are 0 and `is_critical` is `False`.

See [`omni_drive`](../reference/omni_drive.md), [`models`](../reference/models.md) and
[`sensors`](../reference/sensors.md) in the API reference.
