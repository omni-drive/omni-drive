# Localization

`Localizer` runs in a background thread and maintains three simultaneous estimates of the
robot pose:

| Estimate | Source | Notes |
| --- | --- | --- |
| `odometry` | wheel encoders | smooth, 20 times a second, drifts with every meter |
| `lidar` | LiDAR scans matched against a map it builds | does not drift while the map is accurate, but noisy and occasionally lost |
| `kalman` | a Kalman filter that combines the two | smooth and free of drift, provided `PoseNoise` is set appropriately |

All three start at `(0, 0, 0)` at the robot's pose when the localizer starts or resets, with x
forward, y left and yaw counter-clockwise.

## Start it

```python
from omni_drive import Localizer

loc = Localizer().start()
e = await loc.wait_ready()  # waits for odometry and the first scan; TimeoutError after 5 s
print(e.odometry, e.lidar, e.kalman)
```

Keep the robot stationary during startup, because the first scan becomes the origin of the map.

`Localizer()` uses `OmniDrive()` unless a robot is passed. The [live views](visualizing.md)
use `Localizer.default()`, a single shared localizer that starts on first use. Use it in other
code as well; otherwise the scan matching runs twice:

```python
loc = Localizer.default()            # shared localizer for OmniDrive()
loc = Localizer.for_robot(robot)     # the same object if robot is the current connection
```

After `robot.close()` and a new `OmniDrive()`, `Localizer.default()` stops the old shared
localizer and starts one for the new connection.

## Read the latest estimate

```python
e = loc.estimate  # Estimate or None before the first telemetry message
e.t               # seconds since the last reset
e.kalman          # Pose
e.kalman_std      # standard deviations (x [m], y [m], yaw [rad])
e.lidar           # Pose, or None before the first scan
e.lidar_confident # the scan matched the map well
e.lidar_accepted  # the filter used that LiDAR pose
```

`e.lidar_score` is the fraction of scan points that landed on walls in the map. Below 0.4 the
match is not considered confident, and the filter ignores it.

## Reset

```python
loc.reset()
```

The robot's current pose becomes the origin of all three estimates, the history is cleared and
the map is discarded. Reset only while the robot is stationary.

## Record a run

The localizer keeps every estimate since the last reset, 20 per second, up to 12 000 of them
(10 minutes).

```python
loc.reset()
await asyncio.sleep(0.5)
...  # drive
df = loc.dataframe()
df[["t", "odom_x", "lidar_x", "kf_x"]].tail()
```

Columns: `t`, `odom_x`, `odom_y`, `odom_yaw`, `lidar_x`, `lidar_y`, `lidar_yaw`, `kf_x`,
`kf_y`, `kf_yaw`, `kf_std_x`, `kf_std_y`, `kf_std_yaw`, `lidar_confident`, `lidar_accepted`.
LiDAR columns are `NaN` before the first scan.

`loc.history(since=10.0)` returns the same data as a list of `Estimate` objects from
`t = 10 s` on. `loc.dataframe(since=10.0)` filters the same way.

To plot a recording, see [Offline plots](visualizing.md#offline-plots).

## Get the map

```python
snap = loc.map_snapshot()  # MapSnapshot or None before the first scan
if snap is not None:
    print(snap.probability.shape, snap.resolution, snap.x_min, snap.y_min)
```

`map_snapshot()` copies the mapped area of the occupancy grid: a probability per 3 cm cell
(0 free, 0.5 unknown, 1 occupied) and the world position of its corner. The copy does not
change while the localizer continues mapping. `loc.map_version` increases each time a scan is
added to the map, which indicates whether a new snapshot is worth taking. See
[`MapSnapshot`](../reference/lidar_slam.md#mapsnapshot).

## Tune the filter

`PoseNoise` holds the standard deviations that the filter assumes. Custom values can be passed
when the localizer is created:

```python
from omni_drive import Localizer, PoseNoise

loc = Localizer(noise=PoseNoise(per_meter=0.08, lidar_xy=0.03)).start()
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

Pass angles in radians, for example `yaw_per_meter=math.radians(5)`.

- Larger odometry values make the filter follow the LiDAR more closely.
- Larger LiDAR values make it smoother, but slower to correct drift.
- If `lidar_accepted` is often `False` while `lidar_confident` is `True`, the filter considers
  the LiDAR poses too far from its prediction. Increase the odometry values.

[The Kalman filter](../concepts/kalman.md) shows where each value enters the equations.

Other options of `Localizer`:

- `use_imu=True` (default): the filter takes heading changes from the gyroscope instead of the
  wheels, because the wheels slip when the robot turns. It switches to the gyroscope once the
  gyroscope's reading changes from its first value. Until then, or if no gyroscope is present,
  it uses the wheels. The `odometry` estimate always uses the wheels.
- `min_range=0.18` (`hardware.LIDAR_SELF_RETURN_RANGE_M`): LiDAR points closer than this, in
  meters, are dropped, because they are reflections from the robot itself.

## Stop it

```python
loc.stop()
```
