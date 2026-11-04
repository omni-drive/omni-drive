# The Kalman filter

A Kalman filter combines two sources that are each imperfect:

- odometry, which is smooth but drifts,
- a measurement (a distance sensor or the LiDAR), which does not drift but is noisy.

It keeps an estimate of the state and a variance that says how uncertain the estimate is. On
every step it does two things:

1. **Predict**: move the state by the odometry step. The uncertainty grows.
2. **Correct**: compare the state with a measurement and move part of the way towards it. The
   uncertainty shrinks.

The library uses this idea in three places:

| Filter | State | Where |
| --- | --- | --- |
| `Kalman1D`, `kalman_1d` | one number $x$ | `omni_drive.processing.kalman`, the same equations as the lab notebook |
| `Kalman2d` | position $(x, y)$ | `omni_drive.Kalman2d`, with any number of distance sensors |
| `PoseKalmanFilter` | pose $(x, y, \psi)$ | `omni_drive.processing.kalman`, used by [`Localizer`](../guides/localization.md) |

This page starts with the one-dimensional filter and builds up to the localizer's filter.

## One dimension

The lab notebook uses this form. The state is the robot's position $x$ along one axis, and $P$
is its variance in m².

### Predict

The wheels report that the robot moved by $\Delta x$ since the last step. Each step adds a little
random error, with variance $Q$:

$$
\bar x = x + \Delta x, \qquad \bar P = P + Q
$$

$\bar x$ and $\bar P$ are the prediction, before the measurement. $Q$ is per step, so after $N$
steps without a measurement the variance has grown by $N Q$. This is why the lab estimates $Q$
from the drift $\Delta_\text{drift}$ at the end of a run with $N$ samples:

$$
Q \approx \frac{\Delta_\text{drift}^2}{N}
$$

### Correct

A sensor measures $z$, the same quantity as $x$. For a wall straight ahead, at distance $d_0$ at
the start and $d$ now, the robot has moved $z = d_0 - d$. The sensor's error has variance $R$.

$$
\begin{aligned}
y &= z - \bar x && \text{innovation: how far the measurement is from the prediction} \\
K &= \frac{\bar P}{\bar P + R} && \text{Kalman gain, between 0 and 1} \\
x &= \bar x + K y \\
P &= (1 - K)\,\bar P
\end{aligned}
$$

- A noisy sensor ($R \gg \bar P$) gives $K \to 0$: the filter keeps the prediction.
- A precise sensor ($R \ll \bar P$) gives $K \to 1$: the filter follows the measurement.
- $P$ always gets smaller after a correction, because $1 - K < 1$.

$R$ is different for every sensor. The lab estimates it from the sensor's error $e$ against the
LiDAR, with the median absolute deviation, which ignores occasional false echoes:

$$
\text{MAD} = \operatorname{median}\left(|e - \operatorname{median}(e)|\right), \qquad
\sigma \approx 1.4826\,\text{MAD}, \qquad R = \sigma^2
$$

With several sensors, correct once per sensor, each with its own $z$ and $R$:

```python
x, P = kalman_predict(x, P, dx, Q)
x, P = kalman_update(x, P, z_ir, R_ir)
x, P = kalman_update(x, P, z_us, R_us)
```

### In the library

`Kalman1D(x, P, Q, R, n_sigma=None)` implements exactly these equations:

```python
from omni_drive.processing.kalman import Kalman1D

kf = Kalman1D(x=0.0, P=1e-4, Q=1e-4, R=4e-4)
kf.predict(0.01)    # x = 0.01, P = 2e-4
kf.update(0.015)    # True; K = 1/3, x = 0.01167, P = 1.33e-4
```

With `n_sigma`, `update(z)` rejects a measurement and returns `False` if
$|y| > n_\sigma \sqrt{\bar P + R}$. The state then stays at the prediction.

`kalman_1d(u, z, x0, P0, Q, R, n_sigma=None)` runs the filter over whole arrays: `u` holds the
odometry steps and `z` the measurements, with `NaN` where there is none. It returns
`estimate`, `std` and `rejected` arrays. Only `NaN` is skipped: replace infinite readings
first, for example `z = np.where(np.isfinite(z), z, np.nan)`, and pass `Kalman1D.update` only
finite values.

## Two dimensions: Kalman2d

`Kalman2d` estimates the position $\mathbf{x} = (x, y)$ from odometry and $m$ distance sensors at
once. It is the filter from the last part of the lab.

The scalars of the 1D filter become matrices:

| 1D | Kalman2d | Shape | Meaning |
| --- | --- | --- | --- |
| $x$ | $\mathbf{x}$ | 2 | position estimate |
| $P$ | $P$ | 2×2 | covariance of the estimate |
| $Q$ | `Q` | 2×2 | odometry noise per step |
| $R$ | `R` | m×m | sensor noise, usually diagonal with one $R$ per sensor |
| (1) | `H` | m×2 | which coordinate each sensor measures |
| $z$ | `z` | m | one function per sensor |

Each row of `H` belongs to one sensor and says how that sensor's measurement depends on the
state: $z \approx H\mathbf{x}$. A row of `[1, 0]` means the measurement follows x one to one.
Entries can be negative, and which row fits which sensor depends on how you define its $z$.
Working out `z` and `H` for the robot's sensors is part of the lab.

### Predict

`predict(displacement)` adds the odometry step $\Delta\mathbf{x}$, the change of
`(robot.pose.x, robot.pose.y)` since the last step:

$$
\bar{\mathbf{x}} = \mathbf{x} + \Delta\mathbf{x}, \qquad \bar P = P + Q
$$

### Correct

`update()` calls each function in `z` and keeps only the finite results. A sensor without a
reading returns `Distance.inf`, so a measurement such as `d0 - d` becomes infinite and that
sensor is skipped for this step. $H$ and $R$ are cut down to the remaining rows.

$$
\begin{aligned}
\mathbf{y} &= \mathbf{z} - H\bar{\mathbf{x}} \\
S &= H \bar P H^\top + R \\
K &= \bar P H^\top S^{-1} \\
\mathbf{x} &= \bar{\mathbf{x}} + K\mathbf{y} \\
P &= (I - KH)\,\bar P\,(I - KH)^\top + K R K^\top
\end{aligned}
$$

With one sensor and $H = [1]$, $S = \bar P + R$ and $K = \bar P / (\bar P + R)$: the 1D filter
again. The last line is the Joseph form of $P = (I - KH)\bar P$. It gives the same result in exact
arithmetic, but stays symmetric and positive definite with rounding errors.

`update()` returns `False` and leaves the state at the prediction if no measurement is finite or
the gate rejects the step.

### Gate

With `gate=` set, the filter first computes the squared Mahalanobis distance

$$
d^2 = \mathbf{y}^\top S^{-1} \mathbf{y}
$$

and rejects the whole correction if $d^2 >$ `gate`. $d^2$ follows a $\chi^2$ distribution with as
many degrees of freedom as there are finite measurements in that step. These thresholds reject
about 0.27 % of good measurements, the same share as "more than 3 σ" in one dimension:

| Measurements | 1 | 2 | 3 | 4 | 5 | 6 |
| --- | --- | --- | --- | --- | --- | --- |
| `gate` | 9.0 | 11.83 | 14.16 | 16.25 | 18.21 | 20.06 |

### Use it

```python
import numpy as np

from omni_drive import Kalman2d, OmniDrive, PoseRecorder
from omni_drive.live import LivePlot

robot.reset_pose()
d0_front = robot.ir_front.distance.meters
assert np.isfinite(d0_front), "no sensor reading yet, run the cell again"

kf = Kalman2d(
    robot,
    z=[lambda: d0_front - robot.ir_front.distance.meters],  # one sensor, measures x
    Q=np.diag([1e-6, 1e-6]),
    R=np.diag([4e-4]),
    H=[[1, 0]],
)

rec = PoseRecorder(
    odometry=lambda: (robot.pose.x.meters, robot.pose.y.meters),
    kalman=lambda: kf.state,
)
async with LivePlot(rec), robot:
    ...  # drive, with await
```

The values of `Q` and `R` here are placeholders. Use the ones you measured. With one sensor
that only measures x, y follows the odometry alone; more sensors add more rows to `z`, `R` and
`H`.

- The state starts at $(0, 0)$ and $P$ at `Q`. Call `robot.reset_pose()` and take the reference
  distances just before you create the filter, so that the state, odometry and measurements all
  start at zero.
- Each read of `kf.state` runs `step()`: one prediction from the odometry since the last step and
  one correction. Read it once per tick.
- You can also call `predict(displacement)` and `update(measurements)` yourself. `update()`
  without arguments calls the `z` functions.
- The `z` functions must return floats. Use `.meters`, not a `Distance`.

## Three dimensions: the localizer's filter

`Localizer` uses `PoseKalmanFilter`, an extended Kalman filter (EKF). The state is the whole pose,
the odometry step is in the robot's frame, and the LiDAR measures the pose directly.

### State

$$
\mathbf{x} = \begin{bmatrix} x \\ y \\ \psi \end{bmatrix}, \qquad
P = 3 \times 3 \text{ covariance matrix}
$$

`kf.x` and `kf.P` are these two arrays. `kf.pose` returns $\mathbf{x}$ as a `RawPose`, and
`kf.std` returns the square roots of the diagonal of $P$.

After `kf.reset(pose)`, $\mathbf{x}$ is `pose`, or zero without an argument (the start pose
given to the constructor is not remembered), and
$P = \operatorname{diag}(\sigma_{0,xy}^2,\ \sigma_{0,xy}^2,\ \sigma_{0,\psi}^2)$, from
`PoseNoise.initial_xy` and `initial_yaw`.

### Predict with odometry

For each odometry update, the localizer computes the robot's motion since the previous one, in
the robot's frame: $\Delta x$, $\Delta y$, $\Delta\psi$. They are
`previous.increment_to(current)` of consecutive odometry poses. With `use_imu=True` (the
default), $\Delta\psi$ comes from the gyroscope instead of the wheels, as soon as the gyroscope's
reading has changed from its first value.

`kf.predict(dx, dy, dyaw)` then moves the state the same way the firmware integrates odometry,
with the heading halfway through the step. This is `RawPose.advanced(dx, dy, dyaw)`:

$$
\begin{aligned}
\bar\psi &= \psi + \tfrac12\Delta\psi \\
\Delta x_w &= \Delta x\cos\bar\psi - \Delta y\sin\bar\psi \\
\Delta y_w &= \Delta x\sin\bar\psi + \Delta y\cos\bar\psi \\
x &\leftarrow x + \Delta x_w \\
y &\leftarrow y + \Delta y_w \\
\psi &\leftarrow \operatorname{wrap}(\psi + \Delta\psi)
\end{aligned}
$$

$\Delta x_w$ and $\Delta y_w$ are the step in the world frame. In `Kalman2d`, the odometry step
is already in the world frame and the prediction is a plain sum. Here the step depends on
$\psi$, which is not linear, so the filter linearizes it. $F$ is the derivative of the new state
with respect to the old state, and $V$ the derivative with respect to the step
$(\Delta x, \Delta y, \Delta\psi)$. The first two entries in the last column of $V$ are nonzero
because $\Delta\psi$ also turns the mid-point heading $\bar\psi$:

$$
F = \begin{bmatrix}
1 & 0 & -\Delta y_w \\
0 & 1 & \Delta x_w \\
0 & 0 & 1
\end{bmatrix},
\qquad
V = \begin{bmatrix}
\cos\bar\psi & -\sin\bar\psi & -\tfrac12\Delta y_w \\
\sin\bar\psi & \cos\bar\psi & \tfrac12\Delta x_w \\
0 & 0 & 1
\end{bmatrix}
$$

In the 1D filter, $Q$ is fixed per step. Here the noise of the step grows with the distance the
robot moved, not with the number of steps ([why](odometry.md)), so a robot that stands still does
not become more uncertain. With $d = \sqrt{\Delta x^2 + \Delta y^2}$:

$$
M = \operatorname{diag}\left(
\sigma_d^2\,|\Delta x|,\ \
\sigma_d^2\,|\Delta y|,\ \
\sigma_{\psi d}^2\, d + \sigma_{\psi r}^2\,|\Delta\psi|
\right)
$$

$$
P \leftarrow F P F^\top + V \left(M + 10^{-10} I\right) V^\top
$$

$V(M + 10^{-10} I)V^\top$ plays the role of $Q$. The $10^{-10}$ term keeps $P$ from collapsing
while the robot is still.

| Symbol | `PoseNoise` field | Default |
| --- | --- | --- |
| $\sigma_d$ | `per_meter` | 0.05 m/√m (5 cm standard deviation after 1 m) |
| $\sigma_{\psi d}$ | `yaw_per_meter` | 3°/√m (3° after 1 m) |
| $\sigma_{\psi r}$ | `yaw_per_rad` | 0.08 rad/√rad (0.08 rad after turning 1 rad) |
| $\sigma_{0,xy}$ | `initial_xy` | 0.01 m |
| $\sigma_{0,\psi}$ | `initial_yaw` | 1° |

### Correct with a LiDAR pose

For each LiDAR scan, the scan matcher returns a pose $\mathbf{z}$ and a score. Its first guess is
the previous pose plus the motion between the last two scans, with the heading change from the
gyroscope (or the wheels with `use_imu=False`). If the score is below 0.4, the match is not
confident and the filter ignores it. Otherwise the localizer calls `kf.update_pose(z)`. The LiDAR
measures the whole pose, so $H = I$:

$$
R = \operatorname{diag}(\sigma_{L,xy}^2,\ \sigma_{L,xy}^2,\ \sigma_{L,\psi}^2)
$$

$$
\mathbf{y} = \mathbf{z} - \mathbf{x} \quad (\text{yaw component wrapped to } [-\pi, \pi)),
\qquad
S = H P H^\top + R
$$

with $\sigma_{L,xy}$ = `lidar_xy` (0.02 m) and $\sigma_{L,\psi}$ = `lidar_yaw` (1.5°).
`update_pose(z, R=...)` takes a different 3×3 $R$ for one call.

### Gate

As in `Kalman2d`, the filter checks $d^2 = \mathbf{y}^\top S^{-1} \mathbf{y}$ before it uses
$\mathbf{z}$. If $d^2 > 14.16$ (`CHI2_GATE_3_DOF`, the 3 σ threshold for 3 measured values),
the measurement is rejected and the state stays unchanged. `kf.last_mahalanobis2` holds the last
$d^2$, and `kf.accepted` and `kf.rejected` count the outcomes. The gate keeps a wrong scan match
from pulling the estimate away.

### Update

$$
\begin{aligned}
K &= P H^\top S^{-1} \\
\mathbf{x} &\leftarrow \mathbf{x} + K\mathbf{y} \quad (\psi \text{ wrapped}) \\
P &\leftarrow (I - KH)\,P\,(I - KH)^\top + K R K^\top
\end{aligned}
$$

These are the `Kalman2d` equations with $H = I$, in Joseph form.

### Re-sync after repeated rejections

Sometimes the estimate is wrong and the LiDAR is right, for example after heavy wheel slip or
after you lift the robot and put it down elsewhere. Every LiDAR pose is then rejected, and with
the gate alone the filter would never recover.

The filter therefore counts consecutive rejections. At the 20th (`max_rejections`, a constructor
argument), it widens the covariance by the disagreement and accepts the measurement:

$$
P \leftarrow P + 4\,(H^\top\mathbf{y})(H^\top\mathbf{y})^\top,
\qquad S = H P H^\top + R
$$

and then runs the normal update. The estimate moves essentially all the way to the LiDAR pose in one
step.

### Use the filter on its own

```python
from omni_drive.processing import RawPose
from omni_drive.processing.kalman import PoseKalmanFilter, PoseNoise

kf = PoseKalmanFilter(noise=PoseNoise())
kf.predict(0.10, 0.0, 0.0)               # odometry: 10 cm forward
kf.std                                   # (0.0187, 0.0102, 0.0241): uncertainty grew
kf.update_pose(RawPose(0.12, 0.0, 0.0))  # True: accepted
kf.pose                                  # RawPose(x=0.109, ...): between the two
kf.update_pose(RawPose(0.50, 0.0, 0.0))  # False: d² = 260, rejected
```

`PoseKalmanFilter(pose=None, noise=None, gate=14.16, max_rejections=20)` takes a start pose
(`RawPose`), the noise settings, the gate threshold and the re-sync count. `gate=None` accepts
every measurement.

See also the API pages [kalman](../reference/kalman.md) and
[localization](../reference/localization.md).
