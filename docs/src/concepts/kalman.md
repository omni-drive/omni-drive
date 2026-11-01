# The Kalman filter

`Localizer` combines wheel odometry with LiDAR poses in an extended Kalman filter (EKF),
`PoseKalmanFilter`. This page lists the equations it uses.

## State

The filter keeps the pose and its uncertainty:

\[
\mathbf{x} = \begin{bmatrix} x \\ y \\ \psi \end{bmatrix}, \qquad
P = 3 \times 3 \text{ covariance matrix}
\]

`kf.x` and `kf.P` are these two arrays. `kf.pose` returns \(\mathbf{x}\) as a `Pose`, and
`kf.std` returns the square roots of the diagonal of \(P\).

After a reset, \(\mathbf{x}\) is the start pose and
\(P = \operatorname{diag}(\sigma_{0,xy}^2,\ \sigma_{0,xy}^2,\ \sigma_{0,\psi}^2)\), from
`PoseNoise.initial_xy` and `initial_yaw`.

## Predict with odometry

On each telemetry message (20 times a second) the localizer computes the robot's motion since
the previous message, in the robot's frame: \(\Delta x\), \(\Delta y\), \(\Delta\psi\).
They are `previous.increment_to(current)` of consecutive odometry poses, so the prediction
below gives exactly the firmware's odometry.
With `use_imu=True` (the default), \(\Delta\psi\) comes from the gyroscope instead of the
wheels, as soon as the gyroscope's reading has changed from its first value. Then
`kf.predict(dx, dy, dyaw)` moves the state the same way the firmware integrates odometry,
with the heading halfway through the step. This is `Pose.advanced(dx, dy, dyaw)`:

\[
\begin{aligned}
\bar\psi &= \psi + \tfrac12\Delta\psi \\
\Delta x_w &= \Delta x\cos\bar\psi - \Delta y\sin\bar\psi \\
\Delta y_w &= \Delta x\sin\bar\psi + \Delta y\cos\bar\psi \\
x &\leftarrow x + \Delta x_w \\
y &\leftarrow y + \Delta y_w \\
\psi &\leftarrow \operatorname{wrap}(\psi + \Delta\psi)
\end{aligned}
\]

\(\Delta x_w\) and \(\Delta y_w\) are the step in the world frame. The step is not linear in
\(\psi\), so the filter linearizes it. \(F\) is the derivative of the new state with respect
to the old state, and \(V\) the derivative with respect to the increment
\((\Delta x, \Delta y, \Delta\psi)\). The first two entries in the last column of \(V\) are nonzero because
\(\Delta\psi\) also rotates the mid-point heading \(\bar\psi\):

\[
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
\]

The noise of the increment grows with the distance the robot moved, not with time
([why](odometry.md#why-it-drifts)). With \(d = \sqrt{\Delta x^2 + \Delta y^2}\):

\[
M = \operatorname{diag}\left(
\sigma_d^2\,|\Delta x|,\ \
\sigma_d^2\,|\Delta y|,\ \
\sigma_{\psi d}^2\, d + \sigma_{\psi r}^2\,|\Delta\psi|
\right)
\]

\[
P \leftarrow F P F^\top + V \left(M + 10^{-10} I\right) V^\top
\]

The \(10^{-10}\) term prevents \(P\) from collapsing while the robot is stationary.

| Symbol | `PoseNoise` field | Default |
| --- | --- | --- |
| \(\sigma_d\) | `per_meter` | 0.05 m |
| \(\sigma_{\psi d}\) | `yaw_per_meter` | 3° |
| \(\sigma_{\psi r}\) | `yaw_per_rad` | 0.08 rad |

## Correct with a LiDAR pose

For each LiDAR scan (about 10 a second) the scan matcher returns a pose \(\mathbf{z}\) and a
score. Its first guess is the previous pose plus the motion between the last two scans, with
the heading change from the gyroscope (or the wheels with `use_imu=False`). If the score is
below 0.4 (not confident), the filter ignores the scan. Otherwise `kf.update_pose(z)` runs.
The LiDAR measures the whole pose, so \(H = I\):

\[
R = \operatorname{diag}(\sigma_{L,xy}^2,\ \sigma_{L,xy}^2,\ \sigma_{L,\psi}^2)
\]

\[
\mathbf{y} = \mathbf{z} - \mathbf{x} \quad (\text{yaw component wrapped to } [-\pi, \pi)),
\qquad
S = H P H^\top + R
\]

with \(\sigma_{L,xy}\) = `lidar_xy` (0.02 m) and \(\sigma_{L,\psi}\) = `lidar_yaw` (1.5°).

### Gate

Before using \(\mathbf{z}\), the filter checks whether the measurement agrees with the prediction,
given both uncertainties. The squared Mahalanobis distance is

\[
d^2 = \mathbf{y}^\top S^{-1} \mathbf{y}
\]

If \(d^2 > 14.16\) (`CHI2_GATE_3_DOF`), the measurement is rejected and the state remains
unchanged. 14.16 is the 99.7 % point of the \(\chi^2\) distribution with 3 degrees of freedom,
which corresponds approximately to "more than 3 standard deviations away" in 3D. `kf.last_mahalanobis2` holds the last
\(d^2\), and `kf.accepted` and `kf.rejected` count the outcomes. The gate prevents an incorrect scan
match from pulling the estimate away.

### Update

\[
\begin{aligned}
K &= P H^\top S^{-1} \\
\mathbf{x} &\leftarrow \mathbf{x} + K\mathbf{y} \quad (\psi \text{ wrapped}) \\
P &\leftarrow (I - KH)\,P\,(I - KH)^\top + K R K^\top
\end{aligned}
\]

The last line is the Joseph form of the covariance update. In exact arithmetic it gives the
same result as \(P \leftarrow (I - KH)P\), but with rounding it keeps \(P\) symmetric and
positive definite.

### Re-sync after repeated rejections

In some situations the estimate is wrong and the LiDAR is right, for example after heavy wheel
slip or after the robot has been lifted and placed elsewhere. Every LiDAR pose is then
rejected, and with the gate alone the filter would never recover.

The filter therefore counts consecutive rejections. At the 20th (`max_rejections`, a constructor
argument) it widens the covariance by the disagreement and accepts the measurement:

\[
P \leftarrow P + 4\,(H^\top\mathbf{y})(H^\top\mathbf{y})^\top,
\qquad S = H P H^\top + R
\]

and then runs the normal update. The estimate moves most of the way to the LiDAR pose in a
single step. At 10 scans a second, this happens about 2 seconds after the LiDAR starts to
disagree.

## Use the filter on its own

```python
from omni_drive import Pose, PoseKalmanFilter, PoseNoise

kf = PoseKalmanFilter(noise=PoseNoise())
kf.predict(0.10, 0.0, 0.0)        # odometry says: 10 cm forward
kf.std                            # (0.0187, 0.0102, 0.0241): uncertainty grew
kf.update_pose(Pose(0.12, 0.0, 0.0))  # True: accepted
kf.pose                           # Pose(x=0.109, ...): between the two
kf.update_pose(Pose(0.50, 0.0, 0.0))  # False: d² = 260, rejected
```

`PoseKalmanFilter(pose=None, noise=None, gate=CHI2_GATE_3_DOF, max_rejections=20)` takes a
start pose, the noise settings, the gate threshold and the re-sync count. `gate=None` accepts
every measurement.

## Linear problems: pykalman

A linear model does not require an EKF. An example is the distance to a wall ahead, predicted
from odometry and measured with the front ultrasonic sensor: a single number that changes by
the distance driven. For such problems, use `pykalman.KalmanFilter`, which is installed with
`omni_drive`:

```python
import numpy as np
from pykalman import KalmanFilter

kf = KalmanFilter(
    transition_matrices=[[1.0]],
    observation_matrices=[[1.0]],
    transition_covariance=[[1e-4]],        # odometry noise per step [m²]
    observation_covariance=[[0.02**2]],    # ultrasonic noise [m²]
)
mean, cov = np.array([1.0]), np.array([[0.05**2]])  # start: 1 m ± 5 cm

# u: distance driven towards the wall since the last step; z: sensor reading or None
for u, z in [(0.01, 0.985), (0.01, None), (0.01, 0.97)]:
    mean, cov = kf.filter_update(mean, cov, observation=z, transition_offset=np.array([-u]))
    print(mean[0], np.sqrt(cov[0, 0]))
```

`observation=None` skips the correction, for a step without a reading.
