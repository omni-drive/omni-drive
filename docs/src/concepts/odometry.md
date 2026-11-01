# Odometry and drift

## How the robot computes odometry

Each wheel has an encoder that measures the wheel's rotation. 100 times a second the ESP32
firmware:

1. converts the encoder counts since the last step into the distance each wheel rolled;
2. turns the three distances into a small motion of the robot in its own frame: \(\Delta x\)
   forward, \(\Delta y\) left and \(\Delta\psi\) turned (the wheel geometry fixes this
   conversion; working it out is one of the lab exercises);
3. adds that motion to the pose, rotated by the heading halfway through the step:

\[
\begin{aligned}
\bar\psi &= \psi + \tfrac{1}{2}\Delta\psi \\
x &\leftarrow x + \Delta x\cos\bar\psi - \Delta y\sin\bar\psi \\
y &\leftarrow y + \Delta x\sin\bar\psi + \Delta y\cos\bar\psi \\
\psi &\leftarrow \psi + \Delta\psi
\end{aligned}
\]

`Pose.advanced(dx, dy, dyaw)` does the same step in Python, and `Pose.increment_to(target)`
recovers \((\Delta x, \Delta y, \Delta\psi)\) from two poses. The result is `robot.odometry`.

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
not affect it. Instead, its reading drifts slowly over time.

## See it

With the [simulator running in the notebook](../getting-started/simulator.md#run-the-simulator-inside-the-notebook),
drive three laps of a square and compare odometry with the true pose:

```python
import asyncio

from omni_drive import Angle, DriveVector, Time

for lap in range(3):
    for heading in (0, 90, 180, 270):
        await robot.drive_for(DriveVector(0.3, Angle.deg(heading)), Time.s(2))
await asyncio.sleep(0.5)

print("odometry:", sim.odometry)
print("true:    ", sim.true_pose)
```

The simulator adds wheel calibration errors and random slip. Odometry reports that the robot
has returned to near `(0, 0)`, while the true pose is a few centimeters away.

## What corrects it

Correcting the drift requires a sensor that measures the pose relative to the room. On this
robot that sensor is the LiDAR. The [Kalman filter](kalman.md) combines odometry, which is
smooth, with LiDAR poses, which do not drift.
