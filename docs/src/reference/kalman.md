# omni_drive.kalman and omni_drive.processing.kalman

Two modules with Kalman filters:

* `omni_drive.kalman`: [`Kalman2d`](#kalman2d), a linear filter for the (x, y) position driven by
  odometry and user-supplied measurements. Exported from `omni_drive`.
* `omni_drive.processing.kalman`: [`PoseKalmanFilter`](#posekalmanfilter), the extended filter for
  (x, y, yaw) used by [`Localizer`](localization.md#localizer), its noise settings
  [`PoseNoise`](#posenoise) (exported from `omni_drive`), and a scalar filter
  [`Kalman1D`](#kalman1d).

## Kalman2d

```python
Kalman2d(
    robot: OmniDrive,
    *,
    z: Sequence[Callable[[], float]],
    Q: ArrayLike,
    R: ArrayLike,
    H: ArrayLike,
    gate: float | None = None,
)
```

A 2D multi-sensor Kalman filter for the robot position. The state is `[x, y]` in meters and
starts at `[0, 0]`. The prediction step adds the change of `robot.pose` since the previous
step; the update step uses the measurements.

| Parameter | Description |
| --- | --- |
| `robot` | the robot whose `pose` provides the odometry |
| `z` | measurement functions; each returns one number, `m` in total |
| `Q` | process noise covariance, 2 × 2; also the initial `P` |
| `R` | measurement noise covariance, `m` × `m` |
| `H` | measurement matrix, `m` × 2: measurement `i` is expected to equal `H[i] @ [x, y]` |
| `gate` | if given, an update whose squared Mahalanobis distance exceeds `gate` is rejected |

| Member | Description |
| --- | --- |
| `step() -> tuple[float, float]` | reads `robot.pose`, predicts with the displacement since the last step, updates with `z`, and returns `(x, y)` |
| `state: tuple[float, float]` | same as `step()`: every read advances the filter |
| `predict(displacement: ArrayLike) -> None` | adds `[dx, dy]` to the state and `Q` to `P` |
| `update(measurements: Sequence[float] | None = None) -> bool` | corrects with `measurements`, or with the values of `z` if `None`. Measurements that are not finite (for example a sensor's `Distance.inf`) are left out. Returns `False` if no measurement is finite or the gate rejected the update. Uses the Joseph form for `P` |
| `P: np.ndarray` | the 2 × 2 covariance |
| `robot`, `z`, `Q`, `R`, `H`, `gate` | as passed in (matrices as float arrays) |

```python
import numpy as np
from omni_drive import Kalman2d, OmniDrive

robot = OmniDrive()
kf = Kalman2d(
    robot,
    z=[lambda: 2.0 - robot.ir_front.distance.meters],  # x, given a wall 2 m ahead of the origin
    Q=np.diag([1e-4, 1e-4]),
    R=[[0.02**2]],
    H=[[1.0, 0.0]],
)
x, y = kf.step()
```

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

Standard deviations assumed by `PoseKalmanFilter`. The dataclass is frozen.

| Field | Description |
| --- | --- |
| `per_meter` | odometry position error per meter driven, meters |
| `yaw_per_meter` | odometry heading error per meter driven, radians |
| `yaw_per_rad` | odometry heading error per radian turned, radians |
| `lidar_xy` | error of a LiDAR position, meters |
| `lidar_yaw` | error of a LiDAR heading, radians |
| `initial_xy` | position uncertainty after a reset, meters |
| `initial_yaw` | heading uncertainty after a reset, radians |

The odometry variances grow with the distance (`per_meter²·|dx|`, `per_meter²·|dy|`) and the
turn (`yaw_per_meter²·√(dx²+dy²) + yaw_per_rad²·|dyaw|`) of each step.

## PoseKalmanFilter

```python
PoseKalmanFilter(
    pose: RawPose | None = None,
    noise: PoseNoise | None = None,
    gate: float | None = CHI2_GATE_3_DOF,
    max_rejections: int = 20,
)
```

An extended Kalman filter for the pose `[x, y, yaw]`, in meters and radians. Poses are
[`RawPose`](localization.md#rawpose) values.

| Parameter | Description |
| --- | --- |
| `pose` | start pose; default `RawPose()` |
| `noise` | noise settings; default `PoseNoise()` |
| `gate` | largest squared Mahalanobis distance a pose measurement may have to be accepted; `None` accepts every measurement |
| `max_rejections` | the `max_rejections`-th consecutive out-of-gate measurement is accepted after the filter inflates its covariance in the direction of the disagreement; `1` or less never rejects |

| Member | Description |
| --- | --- |
| `predict(dx: float, dy: float, dyaw: float) -> None` | applies an odometry increment: `dx`, `dy` in meters in the robot frame, `dyaw` in radians, using the heading halfway through the turn |
| `update_pose(z: RawPose, R: ArrayLike | None = None) -> bool` | corrects with a measured pose; returns `False` if the gate rejected it. `R` is the 3 × 3 measurement covariance; default `diag(lidar_xy², lidar_xy², lidar_yaw²)` |
| `reset(pose: RawPose | None = None) -> None` | sets the state to `pose` (default `RawPose()`) with the initial uncertainty and zeroes the counters |
| `pose: RawPose` | the current estimate |
| `std: tuple[float, float, float]` | standard deviations of x [m], y [m], yaw [rad] |
| `x: np.ndarray` | the state `[x, y, yaw]` |
| `P: np.ndarray` | the 3 × 3 covariance |
| `noise`, `gate`, `max_rejections` | as passed in |
| `accepted: int`, `rejected: int` | counts since the last reset |
| `rejected_in_row: int` | consecutive rejections so far |
| `last_mahalanobis2: float` | squared Mahalanobis distance of the last measurement |

`CHI2_GATE_3_DOF = 14.16` is the default gate: the 99.7 % point of the χ² distribution with 3
degrees of freedom.

## Kalman1D

```python
Kalman1D(x: float, P: float, Q: float, R: float, n_sigma: float | None = None)
```

A scalar Kalman filter with state `x` and variance `P`.

| Member | Description |
| --- | --- |
| `predict(u: float) -> None` | adds `u` to `x` and `Q` to `P` |
| `update(z: float) -> bool` | corrects with measurement `z` (variance `R`). If `n_sigma` is set and `z` is more than `n_sigma` standard deviations of the innovation away, returns `False` and changes nothing |
| `x`, `P`, `Q`, `R`, `n_sigma` | the state and settings; plain attributes |

## kalman_1d

```python
kalman_1d(
    u: ArrayLike,
    z: ArrayLike,
    x0: float,
    P0: float,
    Q: float,
    R: float,
    n_sigma: float | None = None,
) -> Kalman1DRun
```

Runs `Kalman1D(x0, P0, Q, R, n_sigma)` over a recorded series: for each step `k` it predicts with
`u[k]` and updates with `z[k]`, skipping the update when `z[k]` is `nan`. `u` and `z` must have the
same length (`ValueError` otherwise).

`Kalman1DRun` is a `NamedTuple` of three arrays, one value per step:

| Field | Description |
| --- | --- |
| `estimate` | `x` after the step |
| `std` | `√P` after the step |
| `rejected` | `True` where a measurement was present and the gate rejected it |
