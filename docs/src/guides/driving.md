# Driving

The robot accepts two kinds of commands:

- a **drive vector**: the desired direction and speed, plus a turn rate. The library computes
  the three wheel speeds.
- **raw wheel speeds**: the speed of each wheel is set individually.

## Drive with a vector

```python
from omni_drive import Angle, DriveVector, OmniDrive, Time

robot = OmniDrive()

for heading in (0, 90, 180, 270):  # forward, left, back, right
    await robot.drive_for(DriveVector(velocity=0.3, heading=Angle.deg(heading)), Time.s(1.5))
```

The robot drives a square without turning, with a short stop at each corner.

`drive_for(vector, duration)` sends the command, waits and then stops the robot, also when the
cell fails or is interrupted. `set_vector(vector)` only changes the command and returns
immediately, so the robot continues moving until the next command:

```python
robot.set_vector(DriveVector(0.3, Angle.deg(0)))
...  # the robot drives while this runs
robot.stop()
```

Use `set_vector` when the command must change while the robot moves, for example when steering
from sensor readings. In that case, stop the robot with `robot.stop()` in a `finally` block (see
[Stopping safely](safety.md)).

`DriveVector(velocity, heading, omega=0.0)`:

| Field | Meaning |
| --- | --- |
| `velocity` | speed as a fraction of full speed, `0.0` to `1.0` |
| `heading` | direction of travel as an `Angle`: `0` is forward, `Angle.deg(90)` is left |
| `omega` | turn rate as a fraction of full speed; positive turns counter-clockwise |

`velocity` and `omega` are not in meters or radians per second. With the firmware's
constants, `velocity=1.0` is 0.389 m/s and `omega=1.0` is 3.89 rad/s (all three wheels at full
speed). To work in physical units, divide by the constants in
[`omni_drive.hardware`](../reference/hardware.md):

```python
from omni_drive.hardware import FULL_SPEED_RIM_M_S, FULL_SPEED_YAW_RATE_RAD_S

await robot.drive_for(DriveVector(0.15 / FULL_SPEED_RIM_M_S, Angle.deg(0)), Time.s(2))  # 15 cm/s
```

The real robot deviates slightly from these constants, so measure the speed where accuracy
matters. If a combination of `velocity` and `omega` would ask a wheel for more than full speed, all three
wheels are scaled down together.

In vector mode the controller also:

- limits acceleration, so speed changes take a fraction of a second;
- holds the heading during translation with `omega=0`, so the robot does not slowly rotate.

## Field-centric driving

`robot.field_centric` is `True` by default. In this mode `heading` is measured from a fixed
direction in the room, not from the robot's current front. That direction is the one the robot
faced when `OmniDrive()` was created or when `robot.reset_yaw()` was last called.

The difference becomes visible when the robot turns while driving:

```python
import asyncio

robot.reset_yaw()
await asyncio.sleep(0.1)  # the reset takes effect in the next control cycle
await robot.drive_for(DriveVector(0.3, Angle.deg(0), omega=0.3), Time.s(3))
```

With `field_centric = True` the robot spins while it drives in a straight line. With
`robot.field_centric = False` the same command drives a curve, because "forward" turns with
the robot.

`robot.yaw` is the heading used for this, in radians, from wheel odometry. With
`robot.field_centric = False`, headings are relative to the robot's front.

## Set the wheels directly

```python
try:
    with robot.batch():
        robot.wheel_1.speed = 0.3
        robot.wheel_2.speed = 0.3
        robot.wheel_3.speed = 0.3
    await asyncio.sleep(1)
finally:
    robot.stop()
```

The same speed on all three wheels rotates the robot in place, counter-clockwise for positive
speeds.

- Speeds range from `-1.0` to `1.0`. Values outside this range are clipped.
- Each assignment takes effect immediately, so without `batch()` the robot would run for a moment
  with only `wheel_1` set. Inside `with robot.batch():` the three speeds are sent together
  when the block ends.
- Raw speeds skip the acceleration limit, the heading hold and field-centric driving.
- `set_vector`, `drive_for` and `stop` switch back to vector mode and set all three
  `wheel_N.speed` values to 0.
