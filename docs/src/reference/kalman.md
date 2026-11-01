# omni_drive.kalman

An extended Kalman filter for the 2D pose (x, y, yaw). The equations are in
[The Kalman filter](../concepts/kalman.md). For linear problems, `pykalman.KalmanFilter` can be used instead
([example](../concepts/kalman.md#linear-problems-pykalman)).

## PoseNoise

```python
PoseNoise(
    per_meter: float = 0.05,
    yaw_per_meter: float = math.radians(3.0),
    yaw_per_rad: float = 0.08,
    lidar_xy: float = 0.02,
    lidar_yaw: float = math.radians(1.5),
    initial_xy: float = 0.01,
    initial_yaw: float = math.radians(1.0),
)
```

Standard deviations assumed by the filter. The dataclass is frozen.

| Field | Description |
| --- | --- |
| `per_meter` | odometry position error after driving 1 m, meters |
| `yaw_per_meter` | odometry heading error after driving 1 m, radians |
| `yaw_per_rad` | odometry heading error after turning 1 rad, radians |
| `lidar_xy` | error of a LiDAR position, meters |
| `lidar_yaw` | error of a LiDAR heading, radians |
| `initial_xy` | position uncertainty after a reset, meters |
| `initial_yaw` | heading uncertainty after a reset, radians |

## PoseKalmanFilter

```python
PoseKalmanFilter(
    pose: Pose | None = None,
    noise: PoseNoise | None = None,
    gate: float | None = CHI2_GATE_3_DOF,
    max_rejections: int = 20,
)
```

| Parameter | Description |
| --- | --- |
| `pose` | start pose; default `Pose()` |
| `noise` | noise settings; default `PoseNoise()` |
| `gate` | largest squared Mahalanobis distance a pose measurement may have to be accepted; `None` accepts every measurement |
| `max_rejections` | number of consecutive rejections after which the filter re-synchronises to the measurement |

### Methods

| Method | Description |
| --- | --- |
| `predict(dx: float, dy: float, dyaw: float) -> None` | applies an odometry increment: `dx`, `dy` in meters in the robot frame, `dyaw` in radians |
| `update_pose(z: Pose, R: ArrayLike | None = None) -> bool` | corrects with a measured pose; returns `False` if the gate rejected it. `R` is the 3 × 3 measurement covariance; default `diag(lidar_xy², lidar_xy², lidar_yaw²)` from `noise` |
| `reset(pose: Pose | None = None) -> None` | sets the state to `pose` with the initial uncertainty and zeroes the counters |

### Attributes

| Attribute | Description |
| --- | --- |
| `pose: Pose` | the current estimate |
| `std: tuple[float, float, float]` | standard deviations of x [m], y [m], yaw [rad] |
| `x: np.ndarray` | the state `[x, y, yaw]` |
| `P: np.ndarray` | the 3 × 3 covariance |
| `noise`, `gate`, `max_rejections` | as passed in |
| `accepted`, `rejected: int` | counts since the last reset |
| `rejected_in_row: int` | number of consecutive rejections so far |
| `last_mahalanobis2: float` | squared Mahalanobis distance of the last measurement |

`CHI2_GATE_3_DOF = 14.16` is the default gate: the 99.7 % point of the χ² distribution with 3
degrees of freedom.
