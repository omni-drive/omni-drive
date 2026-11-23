# omni_drive

```python
from omni_drive import (
    Angle, Clock, Distance, DriveVector, Kalman2d, LidarScan, LivePlot, Localizer,
    OmniDrive, Point2D, Pose, PoseNoise, PoseRecorder, PowerStatus, Recorder,
    SensorRecorder, Speed, Time, WheelId, processing,
)
```

| Name | Documented in |
| --- | --- |
| `OmniDrive` | this page; defined in `omni_drive.omni_drive` |
| `Distance`, `Angle`, `Time`, `Speed` | [`omni_drive.units`](units.md) |
| `DriveVector`, `Pose`, `Point2D`, `LidarScan`, `PowerStatus`, `WheelId` | [`omni_drive.models`](models.md) |
| `Clock` | [`omni_drive.clock`](clock.md) |
| `Recorder`, `SensorRecorder`, `PoseRecorder` | [`omni_drive.recorder`](recorder.md) |
| `LivePlot` | [`omni_drive.live`](live.md) |
| `Kalman2d` | [`omni_drive.kalman`](kalman.md) |
| `PoseNoise` | [`omni_drive.processing.kalman`](kalman.md#posenoise) |
| `Localizer` | [`omni_drive.processing.localization`](localization.md) |
| `processing` | the `omni_drive.processing` subpackage; it exports only `RawPose` ([`localization`](localization.md#rawpose)) |

`IrSensorId`, `UltrasonicSensorId` and the sensor classes are not exported here. Import them from
[`omni_drive.models`](models.md) and [`omni_drive.sensors`](sensors.md).

## OmniDrive

```python
OmniDrive()
```

Returns the robot. `OmniDrive` is a singleton: the first call subscribes to the ESP32 telemetry and
LiDAR streams and starts a background thread that sends wheel commands 50 times a second.
Every later call returns the same object until `close()` is called.

```python
robot = OmniDrive()
assert OmniDrive() is robot
```

The object is also an async context manager. Leaving the block calls `stop()`; it does not close
the connection.

```python
async with OmniDrive() as robot:
    await robot.drive_for(Time.s(2), DriveVector(0.3, Angle.deg(0)))
```

### Driving

#### set_vector

```python
robot.set_vector(vector: DriveVector) -> None
```

Sets the drive command and returns immediately. The robot follows it until the next command.
Switches to [vector mode](#vector-mode) and sets the stored speed of all three wheels to 0.

#### drive_for

```python
await robot.drive_for(duration: Time, vector: DriveVector) -> None
```

Drives with `vector` for `duration` and then stops. `duration` comes first. The robot also stops
if the wait is cancelled, for example when the cell is interrupted. `duration` must be a `Time`;
a plain number raises `TypeError`.

```python
await robot.drive_for(Time.s(2), DriveVector(0.3, Angle.deg(0)))
```

#### pause_for

```python
await robot.pause_for(duration: Time) -> None
```

Calls `stop()` and waits for `duration` (a `Time`; a plain number raises `TypeError`).

#### stop

```python
robot.stop() -> None
```

Same as `set_vector(DriveVector.zero())`. In vector mode the robot decelerates along the
acceleration ramp.

#### field_centric

```python
robot.field_centric: bool  # read and write, default True
```

When `True`, `DriveVector.heading` is measured from the heading at the last
[`reset_pose`](#reset_pose), using [`heading`](#heading). When `False`, it is measured from the
robot's current front.

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

The public commands use the robot frame (y right, clockwise positive); before they go to the
firmware, which uses the ROS convention, the library flips the sign of `vy`, `omega` and the raw
wheel speeds.

#### Vector mode

The background thread runs at 50 Hz. In vector mode, each cycle:

1. Ramps the commanded `vx`, `vy` and `omega` toward the target at no more than 9.0 per second
   (in fractions of full speed), so going from 0 to 1 takes about 0.11 s.
2. While translating with `omega = 0`, adds a proportional heading-hold correction (at most
   0.5) towards the held heading: the heading at the start of the move, or, after
   [`calibrate_imu`](#calibrate_imu), the heading of the last `reset_pose()`.
3. Clamps `omega` to [−1, 1] and scales the translation down to at most `1 − |omega|`.
4. If `field_centric` is on, rotates `(vx, vy)` from the yaw-reference frame into the robot
   frame using [`heading`](#heading), extrapolated 50 ms ahead at the commanded turn rate.
5. Converts `(vx, vy, omega)` to the three wheel speeds with the inverse kinematics of the
   wheel layout (see [Frames and units](../concepts/frames.md)) and clips each to [−1, 1].

Speeds below 0.001 are sent as 0. After 10 consecutive all-zero commands, the thread stops sending
until a non-zero command appears. In raw mode (wheel speeds set directly), steps 1 to 4 are
skipped and the three stored wheel speeds are sent as they are (sign-flipped for the firmware).

### Pose and heading

#### pose

```python
robot.pose: Pose
```

Wheel odometry pose relative to the origin: `x` forward and `y` right as `Distance`, `yaw`
clockwise as `Angle`. The origin is the odometry pose at the first read of `pose` (or at the
last `reset_pose()`). `Pose()` before the first telemetry message.

#### heading

```python
robot.heading: Angle
```

The robot's heading since the last `reset_pose()`, clockwise positive, wrapped to
[−180°, 180°). Before `calibrate_imu()` it is the odometry yaw (`robot.pose.yaw`); afterwards
it comes from the drift-corrected gyroscope. The heading hold, field-centric driving,
`LidarScan.yaw` and the IR sensor correction all use this value.

#### calibrate_imu

```python
await robot.calibrate_imu(duration: Time = Time.s(1.5)) -> float
```

Stops the robot and measures the gyroscope drift for `duration` (plus 0.3 s to settle); the
robot must not move. Afterwards `heading` comes from the drift-corrected gyroscope and the
heading hold keeps the heading of the last `reset_pose()` across moves. Returns the measured
drift in rad/s, positive = clockwise. Raises `RuntimeError` if no IMU telemetry arrived.

```python
await robot.calibrate_imu()
robot.reset_pose()
```

#### imu_yaw

```python
robot.imu_yaw: Angle
```

Raw gyroscope heading relative to the IMU baseline, clockwise positive, wrapped to
[−180°, 180°), without drift correction. The baseline is the heading in the first telemetry
message, or at the last `reset_pose()`. `Angle(0.0)` before the first telemetry message.

#### reset_pose

```python
robot.reset_pose() -> None
```

Makes the current odometry pose the new origin of `pose` and the current gyroscope heading the
new IMU baseline (and, after `calibrate_imu()`, the zero of `heading`), and resets the heading
held by the motion controller. Before the first
telemetry message, it clears both, so they are set again from the first message that arrives.

### Sensors

#### lidar

```python
robot.lidar: LidarScan
```

The latest LiDAR scan, with `yaw` set to the robot's heading (in radians, internal frame) at the
time of the read. An empty
`LidarScan()` before the first scan. Each read creates a new object. See
[`LidarScan`](models.md#lidarscan).

#### ir_left, ir_front, ir_right

```python
robot.ir_left: IrSensor
```

The three time-of-flight distance sensors. See [`Sensor`](sensors.md#sensor).

#### us_left, us_front, us_right

```python
robot.us_left: UltrasonicSensor
```

The three ultrasonic sensors. See [`Sensor`](sensors.md#sensor).

#### ultrasonic_sensors

```python
robot.ultrasonic_sensors: tuple[UltrasonicSensor, ...]
```

`(us_left, us_front, us_right)`.

### Status and connection

#### power

```python
robot.power: PowerStatus
```

The latest battery status. `PowerStatus()` (all zeros) before the first telemetry message. See
[`PowerStatus`](models.md#powerstatus).

#### print_status

```python
robot.print_status() -> None
```

Prints the battery voltage, current and charge (`CRITICAL!` when the firmware reports a critical
battery), the pose relative to the origin (`Pose:` line, robot frame), the raw firmware odometry
(`Odom:` line, internal frame), the target and actual wheel speeds
reported by the firmware, and the ToF and ultrasonic distances. Prints
`[Status] No telemetry received.` if no telemetry has arrived.

#### close

```python
robot.close() -> None
```

Stops the robot, stops the background thread, sends zero wheel speeds and closes the
connections. The next `OmniDrive()` call opens a new connection and returns a new object. Commands
given to the old object after `close()` are no longer sent. `close()` is also called
automatically when the interpreter exits.

## Wheel

```python
robot.wheel_1.speed = 0.5
robot.wheel_1.speed  # 0.5
```

Wheels are created by `OmniDrive`; get them from `robot.wheel_1` to `robot.wheel_3`.

| Member | Description |
| --- | --- |
| `speed: float` | commanded speed as a fraction of full speed, clipped to [−1, 1]; read and write |
| `set_speed(speed: float) -> None` | same as assigning `speed` |

Setting a speed switches the robot to raw mode; all three stored wheel speeds go out with the
next 50 Hz control cycle (within 20 ms), except inside [`batch()`](#batch), where they are
applied when the block ends. The other wheels keep their stored speeds;
`set_vector()` and `stop()` set them to 0. A positive speed on all three wheels turns the robot
clockwise.
