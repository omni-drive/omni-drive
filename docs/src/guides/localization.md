# Localization

`Localizer` estimates where the robot is. It runs in a background thread and keeps three
estimates of the robot pose at the same time:

| Estimate | Source | Behaviour |
| --- | --- | --- |
| `odometry` | wheel encoders (`robot.pose`) | smooth, but drifts with every meter driven |
| `lidar` | LiDAR scans matched against a map that the localizer builds | does not drift while the map is accurate, but is noisy and can get lost |
| `kalman` | an extended Kalman filter that combines the two | smooth and free of drift, if `PoseNoise` fits your robot |

All three start at `(0, 0, 0)` at the robot's pose when the localizer starts or resets. x points
forward, y points left, and yaw is counter-clockwise.

The poses are `RawPose` objects with plain floats: `x` and `y` in meters, `yaw` in radians. This
is different from `robot.pose`, which uses `Distance` and `Angle`.

If you are new to the robot, start with [Getting started](../index.md).

## Start it

```python
loc = Localizer.default()   # the shared localizer, already started
e = await loc.wait_ready()  # waits for odometry and the first scan
print(e.odometry, e.lidar, e.kalman)
```

`wait_ready(timeout=5.0)` raises `TimeoutError` if no odometry and LiDAR data arrive in time.

Keep the robot still while the localizer starts. The first scan becomes the origin of the map.

`Localizer()` with no `robot` argument uses `OmniDrive()`. The [live views](visualizing.md) in
`omni_drive.ui` use `Localizer.default()`, one shared localizer that starts on first use. Use it
in your own code too, otherwise scan matching runs twice:

```python
loc = Localizer.default()         # shared localizer for OmniDrive(), already started
loc = Localizer.for_robot(robot)  # the shared one if robot is the current connection
```

`for_robot()` returns the shared localizer when `robot` is the current `OmniDrive()` connection.
For any other robot object it creates and starts a new localizer. After `robot.close()` and a new
`OmniDrive()`, `Localizer.default()` stops the old shared localizer and starts one for the new
connection.

`loc.running` is `True` while the background thread runs.

## Read the latest estimate

```python
e = loc.estimate   # Estimate, or None before the first odometry message
e.kalman.x, e.kalman.y, e.kalman.yaw
```

| Field | Type | Meaning |
| --- | --- | --- |
| `t` | `float` | seconds since the last start or reset |
| `odometry` | `RawPose` | wheel odometry since the last reset |
| `lidar` | `RawPose` or `None` | scan-matched pose, `None` before the first scan |
| `kalman` | `RawPose` | Kalman filter pose |
| `kalman_std` | `(float, float, float)` | standard deviations of x [m], y [m] and yaw [rad] |
| `kalman_cov` | 3×3 NumPy array | full covariance matrix of the filter |
| `lidar_score` | `float` | fraction of scan points that landed on walls of the map |
| `lidar_confident` | `bool` | `lidar_score` is at least 0.4 |
| `lidar_accepted` | `bool` | the filter used this LiDAR pose |

When the match is not confident, the scan matcher reports its prediction as the LiDAR pose and the
filter ignores it. `lidar_accepted` can also be `False` for a confident match if the filter finds
it too far from its own prediction. [The Kalman filter](../concepts/kalman.md) explains this
gate.

`e.as_dict()` returns the estimate as a flat dictionary, with the same keys as the
[`dataframe()` columns](#record-a-run).

Other read-only values:

| Attribute | Meaning |
| --- | --- |
| `loc.scan` | the latest matched scan as `ScanFrame` (`seq`, `points` in meters in the scanner's frame, `pose`), or `None` |
| `loc.scan_hz` | smoothed scan rate in Hz |
| `loc.slam_ms` | smoothed time of one scan match in milliseconds |
| `loc.slam` | the `LidarSlam` scan matcher |
| `loc.kalman` | the `PoseKalmanFilter` |

## Reset

```python
loc.reset()
```

The robot's current pose becomes the origin of all three estimates. The history is cleared, the
map is discarded and the filter starts again. Reset only while the robot is still.

`loc.reset()` does not call `robot.reset_pose()`. The localizer follows the changes of
`robot.pose`, so a `robot.reset_pose()` while it runs looks to it like a jump back to the start.
If you call `robot.reset_pose()`, call `loc.reset()` right after it.

## Record a run

The localizer stores each estimate it publishes since the last reset. It keeps the last 12 000
(the `history` argument of `Localizer`) and drops older ones.

```python
from omni_drive import Angle, DriveVector, Time

loc.reset()
await loc.wait_ready()

async with robot:
    await robot.drive_for(Time.s(2), DriveVector(velocity=0.3, heading=Angle.deg(0)))

df = loc.dataframe()
df[["t", "odom_x", "lidar_x", "kf_x"]].tail()
```

Columns of `dataframe()`:

| Columns | Content |
| --- | --- |
| `t` | seconds since the last reset |
| `odom_x`, `odom_y`, `odom_yaw` | odometry pose |
| `lidar_x`, `lidar_y`, `lidar_yaw` | LiDAR pose, `NaN` before the first scan |
| `kf_x`, `kf_y`, `kf_yaw` | Kalman filter pose |
| `kf_std_x`, `kf_std_y`, `kf_std_yaw` | Kalman filter standard deviations |
| `lidar_confident`, `lidar_accepted` | LiDAR flags, as in `Estimate` |

`loc.history(since=10.0)` returns the same data as a list of `Estimate` objects with
`t >= 10 s`. `loc.dataframe(since=10.0)` filters the same way. Without `since`, both return
everything.

To plot a recording, see [Offline plots](visualizing.md#offline-plots).

## Get the map

```python
snap = loc.map_snapshot()  # MapSnapshot, or None before the first scan
if snap is not None:
    print(snap.probability.shape, snap.resolution, snap.x_min, snap.y_min)
```

`map_snapshot()` copies the mapped part of the occupancy grid:

| Field | Meaning |
| --- | --- |
| `probability` | 2D array, one value per cell: 0 free, 0.5 unknown, 1 occupied |
| `resolution` | cell size in meters, 0.03 by default |
| `x_min`, `y_min` | world position of the corner of cell `[0, 0]` |
| `version` | `map_version` at the time of the copy |

`probability[i, j]` is the cell at `x = x_min + i * resolution` and
`y = y_min + j * resolution`. Rows follow x and columns follow y.

The copy does not change while the localizer keeps mapping. `loc.map_version` increases each time
the scan matcher adds a scan to the map: after the robot moves 5 cm, turns 3°, or after 20 scans,
whichever comes first. Take a new snapshot when `map_version` has changed.

### Expected sensor readings

`loc.expected_range(sensor, pose)` casts rays through the map from the sensor's position and
returns the distance the sensor should read if the robot stood at `pose`:

```python
e = loc.estimate
loc.expected_range(robot.us_front, e.kalman)  # meters
```

`pose` is a `RawPose`. The result is at most the sensor's maximum range (4.0 m) and is that value
if no ray hits a wall. [Expected ultrasonic ranges](../concepts/ultrasonic.md) explains the
method.

## Tune the filter

`PoseNoise` holds the standard deviations that the filter assumes. Pass your own values when you
create the localizer:

```python
from omni_drive import Localizer, PoseNoise

loc = Localizer(robot, noise=PoseNoise(per_meter=0.08, lidar_xy=0.03)).start()
```

| Field | Default | Meaning |
| --- | --- | --- |
| `per_meter` | 0.05 m | odometry position error after driving 1 m |
| `yaw_per_meter` | 3° | heading error after driving 1 m |
| `yaw_per_rad` | 0.08 rad | heading error after turning 1 rad |
| `lidar_xy` | 0.02 m | error of a LiDAR position |
| `lidar_yaw` | 1.5° | error of a LiDAR heading |
| `initial_xy` | 0.01 m | position uncertainty at reset |
| `initial_yaw` | 1° | heading uncertainty at reset |

All fields are in meters and radians, so pass angles in radians, for example
`yaw_per_meter=math.radians(5)`.

- Larger odometry values make the filter follow the LiDAR more closely.
- Larger LiDAR values make it smoother, but slower to correct drift.
- If `lidar_accepted` is often `False` while `lidar_confident` is `True`, the filter finds the
  LiDAR poses too far from its prediction. Increase the odometry values.

[The Kalman filter](../concepts/kalman.md) shows where each value enters the equations.

## Constructor options

`Localizer(robot=None, slam=None, kalman=None, noise=None, use_imu=True, min_range=0.18, history=12000)`

| Argument | Meaning |
| --- | --- |
| `robot` | the robot to read; `OmniDrive()` if omitted |
| `slam` | a `LidarSlam` scan matcher; a default one if omitted |
| `kalman` | a `PoseKalmanFilter`; if you pass one, `noise` is ignored |
| `noise` | `PoseNoise` for the default filter |
| `use_imu` | take heading changes from the gyroscope instead of the wheels |
| `min_range` | drop LiDAR points closer than this many meters |
| `history` | how many estimates to keep |

With `use_imu=True`, the filter and the scan matcher take heading changes from the gyroscope,
because the wheels slip when the robot turns. The localizer switches to the gyroscope once its
reading changes from its first value. Until then, or with `use_imu=False`, it uses the wheels.
The `odometry` estimate always uses the wheels.

`min_range` defaults to 0.18 m, the robot radius plus 1 cm. Closer points are reflections from
the robot itself.

## Stop it

```python
loc.stop()
```

`start()` starts the thread again. After a restart, `t` and the odometry estimate start from zero,
but the map and the filter keep their state. For a consistent fresh start, call `reset()` and
then `start()`.

See also the API pages [localization](../reference/localization.md),
[lidar_slam](../reference/lidar_slam.md) and [kalman](../reference/kalman.md).
