# omni_drive

```python
from omni_drive import (
    Angle, Distance, DriveVector, Estimate, Localizer,
    OmniDrive, Pose, PoseKalmanFilter, PoseNoise, Time, WheelId,
)
```

This page documents `OmniDrive` (defined in `omni_drive.robot`). The other names are documented in
[`omni_drive.models`](models.md), [`omni_drive.localization`](localization.md) and
[`omni_drive.kalman`](kalman.md).

## OmniDrive

```python
OmniDrive()
```

Returns the robot. The first call connects to the robot's services and starts a background
thread that sends wheel commands 50 times a second. Subsequent calls return the same object
until `close()` is called.

```python
robot = OmniDrive()
assert OmniDrive() is robot
assert isinstance(robot, OmniDrive)
```

### Driving

#### set_vector

```python
robot.set_vector(vector: DriveVector) -> None
```

Sets the drive command and returns immediately. The robot follows the command until the next
one arrives. The method also sets the three `wheel_N.speed` values back to 0 and switches to vector mode,
with acceleration limits, heading hold and, if enabled, field-centric driving. See [Driving](../guides/driving.md).

#### drive_for

```python
await robot.drive_for(vector: DriveVector, duration: Time) -> None
```

Drives with `vector` for `duration` and then stops. The robot also stops if the wait is cancelled,
for example when the cell is interrupted.

```python
await robot.drive_for(DriveVector(0.3, Angle.deg(0)), Time.s(2))
```

#### stop

```python
robot.stop() -> None
```

Sets the drive command to zero and the three `wheel_N.speed` values to 0. The robot
decelerates over a fraction of a second. The method has no effect on a closed robot.

#### field_centric

```python
robot.field_centric: bool  # read and write, default True
```

When `True`, `DriveVector.heading` is measured from the yaw reference (see `reset_yaw`).
When `False`, it is measured from the robot's current front.

#### reset_yaw

```python
robot.reset_yaw() -> None
```

Makes the robot's current heading the new yaw reference: `robot.yaw` becomes 0 and
field-centric headings are measured from it. The change takes effect in the next control
cycle, within 20 ms.

#### wheel_1, wheel_2, wheel_3

```python
robot.wheel_1: Wheel
```

The three wheels, for setting raw speeds. See [Wheel](#wheel).

#### batch

```python
with robot.batch():
    ...
```

Context manager. Wheel speeds set inside the block are sent together when the block ends.

```python
with robot.batch():
    robot.wheel_1.speed = 0.2
    robot.wheel_2.speed = -0.4
    robot.wheel_3.speed = 0.2
```

### Pose and heading

#### odometry

```python
robot.odometry: Pose | None
```

Wheel odometry pose, integrated by the ESP32 since it powered on. `None` before the first
telemetry message.

#### yaw

```python
robot.yaw: float
```

Odometry heading in radians relative to the last `reset_yaw()`, wrapped to [−π, π).

#### imu_yaw

```python
robot.imu_yaw: float | None
```

Heading integrated from the gyroscope since the ESP32 powered on, in radians. The value is not wrapped.
`None` before the first telemetry message.

### LiDAR

#### lidar_scan

```python
robot.lidar_scan: LidarScan | None
```

The latest scan, or `None` before the first one. A `LidarScan` has:

| Member | Description |
| --- | --- |
| `points: list[tuple[float, float]]` | `(x, y)` in **millimeters**, robot frame; directions without an echo are left out |
| `timestamp: float` | the Pi's `time.monotonic()` in seconds when the scan was read |
| `points_in_meters() -> np.ndarray` | the points as an `(N, 2)` array in meters |

#### lidar

```python
robot.lidar() -> tuple[LidarScan | None, int]
```

The latest scan and the number of scans received so far. The number increases when a new scan
arrives.

### Ultrasonic sensors

#### us_1, us_2, us_3

```python
robot.us_1: UltrasonicSensor  # front
robot.us_2: UltrasonicSensor  # left
robot.us_3: UltrasonicSensor  # right
```

See [`UltrasonicSensor`](sensors.md#ultrasonicsensor).

#### ultrasonic_sensors

```python
robot.ultrasonic_sensors: tuple[UltrasonicSensor, UltrasonicSensor, UltrasonicSensor]
```

`(us_1, us_2, us_3)`.

### Status and connection

#### print_status

```python
robot.print_status() -> None
```

Prints the battery voltage, current and charge (with `CRITICAL` when the firmware reports a
critical battery), the odometry pose, target and actual wheel speeds as fractions of full
speed, the size and age of the last LiDAR scan, and the three ultrasonic readings with their
source. If no telemetry has arrived, a hint is printed instead.

#### telemetry

```python
robot.telemetry() -> tuple[TelemetryResponse | None, int]
```

The latest raw telemetry message and the number received so far. `TelemetryResponse` is the
protobuf message defined in `omni_drive/proto/robot.proto`, with `odometry`, `imu`, `diagnostics` and
`ultrasonic` fields.

#### close

```python
robot.close() -> None
```

Stops the robot, stops the background thread and closes the connection. Repeated calls have
no effect. The method is also called automatically when the kernel exits.

After `close()`, commands (`set_vector`, `drive_for`, wheel speeds) raise `RuntimeError`.
A new connection is opened with `OmniDrive()`, which then returns a new object.

#### is_current

```python
robot.is_current: bool
```

`True` while this object is the open connection that `OmniDrive()` returns. `False` after
`close()`.

## Wheel

```python
robot.wheel_1.speed = 0.5
robot.wheel_1.speed  # 0.5
```

`speed: float` is the commanded speed as a fraction of full speed, clipped to [−1, 1]. The
attribute is readable and writable.

Setting a speed switches the robot to raw mode and takes effect immediately, except
inside [`batch()`](#batch). A positive speed on all three wheels turns the robot counter-clockwise.
Full speed is about 0.39 m/s at the wheel rim (see [`omni_drive.hardware`](hardware.md)).
