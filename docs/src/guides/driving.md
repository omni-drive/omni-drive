# Driving

You can command the robot in two ways:

- a **drive vector**: the direction and speed of travel, plus a turn rate. The library computes
  the three wheel speeds.
- **raw wheel speeds**: you set the speed of each wheel yourself.

If you have not driven the robot before, start with [Getting started](../index.md).

## Drive with a vector

```python
from omni_drive import Angle, DriveVector, OmniDrive, Time

robot = OmniDrive()

async with robot:
    await robot.drive_for(Time.s(1), DriveVector(velocity=0.3, heading=Angle.deg(90)))   # left
    await robot.pause_for(Time.s(0.5))
    await robot.drive_for(Time.s(1), DriveVector(velocity=0.3, heading=Angle.deg(270)))  # right, back to the start
```

The robot drives to the left without turning, stops for half a second and drives back.

`OmniDrive()` always returns the same object, so every cell that calls it talks to the same
connection.

| Method | What it does |
| --- | --- |
| `await robot.drive_for(duration, vector)` | Sends `vector`, waits `duration` (a `Time`), then stops the robot. It also stops when the cell fails or you interrupt it. |
| `await robot.pause_for(duration)` | Stops the robot and waits `duration`. |
| `robot.set_vector(vector)` | Changes the command and returns at once. The robot keeps moving until the next command. |
| `robot.stop()` | Same as `robot.set_vector(DriveVector.zero())`. |

The duration comes first: `drive_for(Time.s(2), vector)`.

Use `set_vector` when the command changes while the robot moves, for example when you steer
from sensor readings. Put it inside `async with robot:`, which stops the robot when the block
ends, fails or is interrupted (see [Stopping safely](safety.md)):

```python
from omni_drive import Clock, Distance

async with robot:
    async for t in Clock(duration=Time.s(5)):  # one tick every 100 ms
        if robot.us_front.distance < Distance.cm(20):
            break
        robot.set_vector(DriveVector(velocity=0.2, heading=Angle.deg(0)))
```

### `DriveVector`

`DriveVector(velocity, heading, omega=0.0)`:

| Argument | Meaning |
| --- | --- |
| `velocity` | speed of travel, as a fraction of full wheel speed. Clipped to −1.0 … 1.0. |
| `heading` | direction of travel, as an `Angle`. `Angle.deg(0)` is forward, `Angle.deg(90)` is left. |
| `omega` | turn rate, as a fraction of full wheel speed. Positive turns counter-clockwise. Clipped to −1.0 … 1.0. |

`velocity` and `omega` are not in meters per second or radians per second. The library does not
convert them to physical units. If you need a speed in m/s, measure it: drive for a known time
and compare the distance with `robot.pose` or a tape measure.

`DriveVector.zero()` is a vector with all values at zero.

### What the controller does with a vector

A background thread sends the wheel speeds to the robot 50 times a second. In vector mode it
applies these steps, in this order, on every cycle:

1. **Acceleration limit.** The speed of travel and the turn rate each change by at most 9.0 per
   second, so going from 0 to full speed takes about 0.11 s.
2. **Turn priority.** The turn rate is clipped to −1 … 1 and the speed of travel is reduced to
   at most `1 − |omega|`. A full-speed `omega` therefore leaves no speed for travel.
3. **Field-centric rotation**, if `robot.field_centric` is `True` (see below).
4. **Inverse kinematics.** The vector becomes three wheel speeds, each clipped to −1 … 1.

The controller does not correct the robot's heading while it drives. With `omega=0` it sends no
turn command, so any small rotation from uneven wheels or floor stays; you can see it in
`robot.imu_yaw`.

Wheel speeds below 0.001 are sent as 0. When all three wheels have been 0 for 10 cycles, the
thread stops sending until the next nonzero command.

## Field-centric driving

`robot.field_centric` is `True` by default. In this mode, `heading` is measured from a fixed
direction in the room, not from the robot's current front. That direction is the one the robot
faced when the gyroscope reading was first taken (shortly after `OmniDrive()` is created), or
when you last called `robot.reset_pose()`.

The difference shows when the robot turns while it drives:

```python
robot.reset_pose()

async with robot:
    await robot.drive_for(Time.s(1), DriveVector(velocity=0.4, heading=Angle.deg(0), omega=0.5))
```

With `field_centric = True` the robot spins while it travels in a straight line. With
`robot.field_centric = False` the same command drives a curve, because "forward" turns with
the robot.

The rotation uses `robot.imu_yaw`, the gyroscope heading. The controller adds a small lead to
compensate for the delay between reading the heading and the wheels reacting. You can change
`robot.field_centric` at any time.

If the gyroscope failed to start, `robot.imu_yaw` stays at 0. Field-centric driving then
behaves like robot-centric driving.

## Set the wheels directly

```python
import asyncio

async with robot:
    with robot.batch():
        robot.wheel_1.speed = 0.3
        robot.wheel_2.speed = 0.3
        robot.wheel_3.speed = 0.3
    await asyncio.sleep(1)
```

The same speed on all three wheels rotates the robot in place, counter-clockwise for positive
speeds. [Frames and units](../concepts/frames.md#wheels) shows where each wheel sits.

- Speeds range from −1.0 to 1.0, as a fraction of full wheel speed. Values outside this range
  are clipped.
- Each assignment updates the command that the background thread sends 50 times a second.
  Without `batch()`, the robot can run for a moment with only `wheel_1` changed. Inside
  `with robot.batch():` the command is updated once, when the block ends, with all three speeds.
- Raw speeds skip the acceleration limit and field-centric driving.
- `set_vector`, `drive_for`, `pause_for` and `stop` switch back to vector mode and reset all
  three `wheel_N.speed` values to 0.

On the robot, the ESP32 runs a speed controller for each wheel. It compares the commanded speed
with the encoder speed 100 times a second and adjusts the motor power. A speed of 1.0
corresponds to a fixed encoder rate set in the firmware, not to a fixed motor power.

`robot.print_status()` shows the target and actual speed of each wheel. Both are in the same
−1 … 1 units as `wheel_N.speed`.
