# omni_drive.localization

Usage is described in [Localization](../guides/localization.md).

## Localizer

```python
Localizer(
    robot: OmniDrive | None = None,
    slam: LidarSlam | None = None,
    kalman: PoseKalmanFilter | None = None,
    noise: PoseNoise | None = None,
    use_imu: bool = True,
    min_range: float = LIDAR_SELF_RETURN_RANGE_M,
    history: int = 12000,
)
```

Keeps three pose estimates (odometry, LiDAR, Kalman filter) up to date in a background
thread. The localizer is created stopped; `start()` must be called to run it.

| Parameter | Description |
| --- | --- |
| `robot` | the robot to read from; default `OmniDrive()` |
| `slam` | the scan matcher; default `LidarSlam()` |
| `kalman` | the filter; default `PoseKalmanFilter(noise=noise)` |
| `noise` | noise settings for the default filter; ignored if `kalman` is passed |
| `use_imu` | predict the filter's heading with the gyroscope instead of the wheels, once the gyroscope reading has changed from its first value (a reading that never changes means no gyroscope) |
| `min_range` | LiDAR points closer than this, in meters, are dropped because they are reflections from the robot itself; default 0.18 |
| `history` | number of estimates to keep; 12 000 corresponds to 10 minutes at 20 per second |

### Control

| Member | Description |
| --- | --- |
| `Localizer.default() -> Localizer` | a started localizer for `OmniDrive()`, shared by the live views. Created on first use, and again if the previous one stopped or its robot was closed |
| `Localizer.for_robot(robot: OmniDrive) -> Localizer` | `Localizer.default()` if `robot` is the current connection, otherwise a new started localizer for `robot` |
| `start() -> Localizer` | starts the background thread; returns `self` |
| `stop() -> None` | stops the thread |
| `running: bool` | whether the thread is running |
| `reset() -> None` | makes the current pose the origin of all three estimates, clears the history and the map |
| `await wait_ready(timeout: float = 5.0) -> Estimate` | waits until odometry and a first LiDAR pose exist; raises `TimeoutError` after `timeout` seconds |

### Results

| Member | Description |
| --- | --- |
| `estimate: Estimate | None` | the latest estimate; `None` before the first telemetry message after a reset |
| `history(since: float | None = None) -> list[Estimate]` | every estimate since the reset, or those with `t >= since` |
| `dataframe(since: float | None = None) -> pandas.DataFrame` | the history as a table, one row per estimate, columns as in `Estimate.as_dict()` |
| `scan: ScanFrame | None` | the latest scan and the LiDAR pose it was matched at |
| `map_version: int` | increases each time a scan is added to the map |
| `map_snapshot() -> MapSnapshot | None` | a copy of the mapped area, taken under the map's lock; `None` before the first scan. See [`MapSnapshot`](lidar_slam.md#mapsnapshot) |
| `expected_range(sensor: UltrasonicSensor, pose: Pose, rays: int = 7) -> float` | what `sensor` should read at `pose`, ray-cast on the map, in meters ([how](../concepts/ultrasonic.md)) |
| `scan_hz: float` | measured LiDAR scan rate |
| `slam_ms: float` | average time to match one scan, in milliseconds |
| `robot`, `slam`, `kalman` | the objects passed in or created; `slam.map` is the live occupancy grid |

## Estimate

```python
Estimate(
    t: float,
    odometry: Pose,
    lidar: Pose | None,
    kalman: Pose,
    kalman_std: tuple[float, float, float],
    kalman_cov: np.ndarray,
    lidar_confident: bool,
    lidar_score: float,
    lidar_accepted: bool,
)
```

The three estimates at one moment. The dataclass is frozen.

| Field | Description |
| --- | --- |
| `t` | seconds since the last reset |
| `odometry` | wheel odometry pose in the start frame |
| `lidar` | latest LiDAR pose, `None` before the first scan |
| `kalman` | Kalman filter pose |
| `kalman_std` | standard deviations of the Kalman pose: x [m], y [m], yaw [rad] |
| `lidar_confident` | the latest scan matched the map with a score of at least 0.4 |
| `lidar_score` | fraction of the latest scan's points that landed on occupied map cells |
| `lidar_accepted` | the filter used the latest LiDAR pose |
| `kalman_cov` | the full 3 × 3 covariance matrix of the Kalman pose |

`as_dict() -> dict[str, float]` returns the fields as a flat dictionary with the keys `t`,
`odom_x`, `odom_y`, `odom_yaw`, `lidar_x`, `lidar_y`, `lidar_yaw`, `kf_x`, `kf_y`, `kf_yaw`,
`kf_std_x`, `kf_std_y`, `kf_std_yaw`, `lidar_confident`, `lidar_accepted`. LiDAR values are
`nan` when `lidar` is `None`. `kalman_cov` and `lidar_score` are not included.

## ScanFrame

| Field | Description |
| --- | --- |
| `seq` | increases by one with every scan |
| `points` | the scan as an `(N, 2)` array in meters, robot frame, without points closer than `min_range` |
| `pose` | the LiDAR pose the scan was matched at |
