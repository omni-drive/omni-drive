# omni_drive.kinematics

Conversion between the robot's body velocity and its three wheel speeds. The library uses
`wheel_speeds` 50 times a second to turn a `DriveVector` into wheel commands, and the
simulator uses `body_velocity` to move the simulated robot. Deriving the formulas is one of
the lab exercises, so this page documents only the interface.

All values are fractions of full speed: `vx` and `vy` in units of `FULL_SPEED_RIM_M_S`,
`omega` in units of `FULL_SPEED_YAW_RATE_RAD_S` (see [`omni_drive.hardware`](hardware.md)).
`vx` is forward, `vy` is left and `omega` is counter-clockwise, in the robot frame.

## wheel_speeds

```python
wheel_speeds(vx: float, vy: float, omega: float) -> tuple[float, float, float]
```

Returns the speeds of wheels 1, 2 and 3 that move the robot with body velocity `(vx, vy, omega)`. The
result is not limited to [−1, 1]; the motion controller scales all three down by the same factor
when one of them exceeds full speed.

## body_velocity

```python
body_velocity(w1: float, w2: float, w3: float) -> tuple[float, float, float]
```

Returns the body velocity `(vx, vy, omega)` produced by the wheel speeds `w1`, `w2`, `w3`. This
is the inverse of `wheel_speeds`.

```python
from omni_drive.kinematics import body_velocity, wheel_speeds

body_velocity(*wheel_speeds(0.3, -0.2, 0.1))  # (0.3, -0.2, 0.1), up to rounding
```
