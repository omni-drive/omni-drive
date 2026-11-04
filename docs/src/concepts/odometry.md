# Odometry and drift

## How the robot computes odometry

Each wheel has an encoder that counts the wheel's rotation. 100 times a second the ESP32
firmware:

1. converts the encoder counts since the last step into the distance each wheel rolled;
2. turns the three distances into a small motion of the robot in its own frame: \(\Delta x\)
   forward, \(\Delta y\) left and \(\Delta\psi\) turned. This is the inverse of the wheel
   kinematics you derive in the lab;
3. adds that motion to the pose, rotated by the heading halfway through the step:

\[
\begin{aligned}
\bar\psi &= \psi + \tfrac{1}{2}\Delta\psi \\
x &\leftarrow x + \Delta x\cos\bar\psi - \Delta y\sin\bar\psi \\
y &\leftarrow y + \Delta x\sin\bar\psi + \Delta y\cos\bar\psi \\
\psi &\leftarrow \psi + \Delta\psi
\end{aligned}
\]

The firmware sends this pose with every telemetry message. `robot.pose` is the same pose,
expressed relative to the origin that `robot.reset_pose()` stores (see
[Frames and units](frames.md)).

In Python, `RawPose.advanced(dx, dy, dyaw)` from `omni_drive.processing.models` does the same
step, and `RawPose.increment_to(target)` recovers \((\Delta x, \Delta y, \Delta\psi)\) from two
poses.

Each step only adds to the previous pose. The pose is never checked against the room.

## Why it drifts

Every step carries a small error, and the errors accumulate:

- Slip. A wheel that slides on the floor turns, but the robot does not move by the
  corresponding distance. Slip is largest when the robot turns.
- Calibration. If the real wheel radius or the distance from the center to the wheels differs
  slightly from the values in the firmware, every meter or every turn is measured with a small
  error, always in the same direction.
- A heading error propagates to all later motion: an error of 2° displaces every subsequent
  meter by 3.5 cm sideways.

Random slip behaves like a random walk. The variance of the error grows in proportion to the
distance driven, so its standard deviation grows with the square root of the distance. The
calibration errors grow in proportion to the distance. Neither depends on time: a stationary
robot does not drift.

The gyroscope (`robot.imu_yaw`) measures rotation without contact with the floor, so slip does
not affect it. Instead, its reading drifts slowly over time, also when the robot stands still.
`robot.pose.yaw` does not use the gyroscope; it comes from the wheels only.

## See it

Mark the robot's position on the floor, for example with tape next to the wheels, call
`robot.reset_pose()`, and drive a closed path that ends where it started. Then compare
`robot.pose` with the real displacement from the tape mark, measured with a tape measure. The
lab's drift test (task 2) does exactly this.

## What corrects it

Correcting the drift requires a sensor that measures the pose relative to the room. On this
robot that sensor is the LiDAR. The [`Localizer`](../guides/localization.md) uses the gyroscope
for heading changes (unless you pass `use_imu=False`) and the [Kalman filter](kalman.md) to
combine odometry, which is smooth, with LiDAR poses, which do not drift.
