# Reading sensors

Every reading on this page returns the latest value sent by the robot and does not wait for
new data. Most readings are `None` until the first message arrives. Lengths are in meters and
angles in radians, unless the name says otherwise. Directions follow
[Frames and units](../concepts/frames.md).

## Odometry

```python
pose = robot.odometry  # Pose(x, y, yaw) or None
if pose is not None:
    print(pose.x, pose.y, pose.yaw)
```

The ESP32 integrates the wheel encoders into a pose. It starts at `(0, 0, 0)` when the ESP32
powers on, not when the notebook starts. To measure displacement from a starting point, store
the first pose and use `Pose.relative_to`:

```python
start = robot.odometry
...
moved = robot.odometry.relative_to(start)  # in the frame of the start pose
print(f"{moved.x:.3f} m forward, {moved.y:.3f} m left")
```

`robot.yaw` is the odometry heading relative to the last `robot.reset_yaw()`.

Odometry drifts. [Odometry and drift](../concepts/odometry.md) explains why.

## Gyroscope

```python
robot.imu_yaw  # float or None
```

This is the heading integrated from the gyroscope since the ESP32 powered on. It is not
wrapped to ±π, so two full counter-clockwise turns read about `4 * math.pi`. Unlike the wheels,
the gyroscope is not affected by slip, but its reading slowly drifts over time. If the gyroscope failed to start, it reads
`0.0`.

## LiDAR

```python
scan = robot.lidar_scan
if scan is not None:
    points = scan.points_in_meters()  # (N, 2) NumPy array in meters
```

`robot.lidar_scan` is the latest scan, or `None` before the first one. `points_in_meters()`
returns its points as an `(N, 2)` array in meters, in the robot frame. `scan.points` is the same data as
a list of `(x, y)` tuples in **millimeters**, and `scan.timestamp` is the Pi's
`time.monotonic()` when the scan arrived.

The LiDAR covers a field of view of about 240°, centered on the robot's front, with a range of
up to 5.6 m, and sends a scan about 10 times a second. Directions without an echo are omitted,
so the number of points varies from scan to scan. Points within 0.18 m of the center are
reflections from the robot itself.

To process each scan only once, use `robot.lidar()`. It returns the scan together with the
number of scans received so far; the number changes when a new scan arrives:

```python
import asyncio

last = -1
for _ in range(500):  # about 10 seconds
    scan, count = robot.lidar()
    if scan is not None and count != last:
        last = count
        ...  # handle the new scan
    await asyncio.sleep(0.02)
```

`robot.telemetry()` provides the same for the raw telemetry message.

## Ultrasonic sensors

| Sensor | Faces |
| --- | --- |
| `robot.us_1` | front |
| `robot.us_2` | left |
| `robot.us_3` | right |

```python
d = robot.us_1.read()  # Distance or None
if d is not None:
    print(d.meters, d.as_cm)
```

A reading is the distance from the sensor face (0.15 m from the robot center) to the nearest
object inside a 25° cone. With no echo, the sensor reports its maximum range, 4 m.

`robot.ultrasonic_sensors` is the tuple `(us_1, us_2, us_3)`.

### Emulated readings

The firmware does not currently read the ultrasonic sensors. Until it does, `read()` computes a
reading from the latest LiDAR scan instead: the nearest LiDAR point in the sensor's cone,
measured from the sensor face, plus noise modeled on an HC-SR04 sensor (1 cm standard
deviation, and about 3 % false echoes). One new value is produced per LiDAR scan.

`robot.us_1.emulated` indicates the kind of the last reading and is updated on every `read()`:
`True` when the value came from the LiDAR, `False` when it came from the sensor. Once the
firmware sends real readings, the library uses them without any change to user code.

### Wait for a reading

```python
from omni_drive import Angle, DriveVector

try:
    robot.set_vector(DriveVector(0.2, Angle.deg(0)))
    d = await robot.us_1.wait_for(lambda d: d.meters < 0.3)
finally:
    robot.stop()
print(f"wall at {d.as_cm:.0f} cm")
```

`wait_for` checks the sensor every 20 ms and returns the first reading that makes the condition
true.
