# omni_drive.processing.localization

Fuses wheel odometry, the gyroscope and LiDAR scan matching into one pose estimate. `Localizer`
is also exported from `omni_drive`. All poses on this page are [`RawPose`](#rawpose) values in
meters and radians, in the localizer's start frame.

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
thread. The localizer is created stopped; call `start()` to run it.

On every loop pass, the thread reads `robot.pose`, feeds the increment to the Kalman filter
and publishes an estimate. It then reads `robot.lidar`, drops points closer than `min_range`,
matches the scan with [`LidarSlam`](lidar_slam.md#lidarslam), and, if the match is confident,
corrects the filter with the LiDAR pose.

| Parameter | Description |
| --- | --- |
| `robot` | the robot to read from; default `OmniDrive()` |
| `slam` | the scan matcher; default `LidarSlam()` |
| `kalman` | the filter; default `PoseKalmanFilter(noise=noise)` |
| `noise` | noise settings for the default filter; ignored if `kalman` is passed |
| `use_imu` | use the gyroscope heading change instead of the wheels' for the filter prediction and the scan matcher's heading hint, once the gyroscope reading has changed from its first value |
| `min_range` | LiDAR points closer than this to the scan origin, in meters, are dropped as reflections from the robot itself; default 0.18 (`ROBOT_RADIUS_M + 0.01`) |
| `history` | number of estimates to keep |

### Control

| Member | Description |
| --- | --- |
| `Localizer.default() -> Localizer` | a started localizer for `OmniDrive()`, shared by the views in `omni_drive.ui`. Created on first use, and again if the previous one stopped or the robot connection changed |
| `Localizer.for_robot(robot: OmniDrive) -> Localizer` | `Localizer.default()` if `robot` is the current `OmniDrive()` object, otherwise a new started localizer for `robot` |
| `start() -> Localizer` | starts the background thread; returns `self` |
| `stop() -> None` | stops the thread (waits up to 1 s) |
| `running: bool` | whether the thread is running |
| `reset() -> None` | clears the estimates, the history, the map and the filter; the next odometry pose becomes the origin of all three estimates and `t` restarts at 0. Does not change `robot.pose` |
| `await wait_ready(timeout: float = 5.0) -> Estimate` | waits until an estimate with a LiDAR pose exists and returns it; raises `TimeoutError` after `timeout` seconds |

### Results

| Member | Description |
| --- | --- |
| `estimate: Estimate | None` | the latest estimate; `None` after a reset until the thread publishes the next one |
| `history(since: float | None = None) -> list[Estimate]` | the stored estimates, or those with `t >= since` |
| `dataframe(since: float | None = None) -> pandas.DataFrame` | the history as a table, one row per estimate, columns as in `Estimate.as_dict()` |
| `scan: ScanFrame | None` | the latest matched scan and the LiDAR pose it was matched at |
| `map_version: int` | `slam.map_version`: increases each time a scan is added to the map |
| `map_snapshot() -> MapSnapshot | None` | a copy of the mapped area, taken under the map's lock; `None` before the first scan. See [`MapSnapshot`](lidar_slam.md#mapsnapshot) |
| `expected_range(sensor: UltrasonicSensor, pose: RawPose, rays: int = 7) -> float` | what `sensor` should read at `pose`, in meters: `rays` rays across the sensor's cone, cast on the map from the sensor face, the shortest hit, capped at `config.max_range` |
| `scan_hz: float` | LiDAR scan rate, smoothed |
| `slam_ms: float` | time to match one scan, smoothed, milliseconds |
| `robot`, `slam`, `kalman`, `use_imu`, `min_range` | as passed in or created; `slam.map` is the live occupancy grid |

```python
loc = Localizer().start()
await loc.wait_ready()
loc.estimate.kalman  # RawPose(x=..., y=..., yaw=...)
```

## Estimate

```python
Estimate(
    t: float,
    odometry: RawPose,
    lidar: RawPose | None,
    kalman: RawPose,
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
| `t` | seconds since the localizer started or was reset |
| `odometry` | wheel odometry pose in the start frame |
| `lidar` | latest LiDAR pose; `None` before the first scan |
| `kalman` | Kalman filter pose |
| `kalman_std` | standard deviations of the Kalman pose: x [m], y [m], yaw [rad] |
| `kalman_cov` | the 3 × 3 covariance of the Kalman pose |
| `lidar_confident` | the latest scan matched the map with a score of at least `slam.min_score` |
| `lidar_score` | `slam.score`: fraction of the latest scan's points on occupied map cells |
| `lidar_accepted` | the filter used the latest LiDAR pose |

`as_dict() -> dict[str, float | bool]` returns a flat dictionary with the keys `t`, `odom_x`,
`odom_y`, `odom_yaw`, `lidar_x`, `lidar_y`, `lidar_yaw`, `kf_x`, `kf_y`, `kf_yaw`, `kf_std_x`,
`kf_std_y`, `kf_std_yaw`, `lidar_confident`, `lidar_accepted`. LiDAR values are `nan` when
`lidar` is `None`. `kalman_cov` and `lidar_score` are not included.

## ScanFrame

```python
ScanFrame(seq: int, points: np.ndarray, pose: RawPose)
```

The dataclass is frozen.

| Field | Description |
| --- | --- |
| `seq` | increases by one with every matched scan |
| `points` | the scan as an `(N, 2)` array in meters, robot frame, without points closer than `min_range` |
| `pose` | the LiDAR pose the scan was matched at |

## RawPose

```python
RawPose(x: float = 0.0, y: float = 0.0, yaw: float = 0.0)
```

A pose in plain floats: meters and radians, x forward, y left, yaw counter-clockwise. Defined in
`omni_drive.processing.models` and exported from `omni_drive.processing`. The dataclass is
frozen.

| Member | Description |
| --- | --- |
| `RawPose.from_pose(pose: Pose) -> RawPose` | from a [`Pose`](models.md#pose) |
| `compose(local: RawPose) -> RawPose` | applies `local`, given in this pose's frame, on top of this pose |
| `relative_to(origin: RawPose) -> RawPose` | this pose seen from `origin`; the inverse of `compose` |
| `distance_to(other: RawPose) -> float` | straight-line distance between the two positions, meters |
| `advanced(dx: float, dy: float, dyaw: float) -> RawPose` | moves by an increment in this pose's frame, using the heading halfway through the turn |
| `increment_to(target: RawPose) -> tuple[float, float, float]` | the increment `(dx, dy, dyaw)` that `advanced` needs to reach `target`; the inverse of `advanced` |

`compose`, `relative_to` and `advanced` wrap the resulting yaw to [−π, π).

```python
from omni_drive.processing import RawPose

a = RawPose(1.0, 0.0, math.pi / 2)
b = a.compose(RawPose(0.5, 0.0))  # RawPose(x=1.0, y=0.5, yaw=1.5708)
b.relative_to(a)                  # RawPose(x=0.5, y=0.0, yaw=0.0), up to rounding
```
