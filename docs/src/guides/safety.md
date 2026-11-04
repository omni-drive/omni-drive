# Stopping safely

A background thread in the library sends the current command to the robot 50 times a second
until you change it. A cell that finishes, fails or is interrupted therefore does not stop the
robot by itself. Use one of the patterns below.

## Use `async with robot:`

```python
from omni_drive import Angle, Clock, DriveVector, OmniDrive, Time

robot = OmniDrive()

async with robot:
    async for t in Clock(duration=Time.s(3)):
        robot.set_vector(DriveVector(velocity=0.3, heading=Angle.deg(0)))
```

When the block ends, for any reason, `async with robot:` calls `robot.stop()`. This includes an
exception in the block and the Jupyter stop button. Put every piece of code that drives the
robot inside it.

`drive_for` also stops the robot at the end, including on an exception or interrupt:

```python
await robot.drive_for(Time.s(2), DriveVector(velocity=0.3, heading=Angle.deg(0)))
```

A loop over `Clock` ends quietly when you interrupt the cell while it waits for the next tick.
The code after the loop then still runs. Inside `async with robot:` the robot stops when the
block ends.

## Stop in `finally`

Outside an `async with robot:` block, stop the robot in a `finally` block:

```python
from omni_drive import Distance

try:
    robot.set_vector(DriveVector(velocity=0.3, heading=Angle.deg(0)))
    await robot.us_front.wait_for(lambda d: d < Distance.cm(30))  # until a wall is 30 cm ahead
finally:
    robot.stop()
```

To stop the robot by hand, keep a cell that contains only this line and run it:

```python
robot.stop()
```

In vector mode the robot slows down over about 0.1 s instead of stopping instantly (see
[Driving](driving.md#what-the-controller-does-with-a-vector)).

## Close the connection

```python
robot.close()
```

`close()` stops the robot, ends the background thread, sends zero wheel speeds once more and
closes the sockets. Calling it twice does no harm. Python also calls it when the kernel shuts
down normally.

After `close()`, the old object does not raise errors, but it no longer sends anything to the
robot. Commands on it are ignored, and its sensor properties keep returning the last values
received. Call `OmniDrive()` again to get a new connection,
and use the new object.

## The watchdog

The ESP32 firmware sets all wheel speeds to zero when it receives no command for 300 ms. This
stops the robot when:

- the kernel is restarted or shut down;
- the kernel process crashes;
- the service on the robot that forwards commands to the ESP32 stops.

The watchdog does not act while the kernel runs. A cell that is busy with a long computation, or
stuck in an endless loop, does not stop the background thread, so the robot keeps driving.

The stop button only interrupts a cell while it waits at an `await`; `async with robot:` or
`finally` then stops the robot. A loop that never awaits (for example
`while robot.us_front.distance > Distance.cm(20): pass`) can't be interrupted, and no other cell
can run meanwhile: restart the kernel, and the watchdog stops the wheels 300 ms later. So always
put an `await` in a loop, such as a `Clock` tick or `await asyncio.sleep(0.02)`.

Sensor readings do not expire. If telemetry stops, `wait_for` can wait forever while the robot
keeps driving. [Reading sensors](sensors.md#wait-for-a-reading) shows how to add a timeout.

## Before you drive

- Check the battery with `robot.print_status()`. `CRITICAL!` at the end of the power line
  means the battery voltage is below 13.0 V. Charge it before you drive.
- Keep the robot still for a second after you switch it on, while the gyroscope calibrates.
- Start at a low `velocity`, for example 0.2.
- Do not drive from a notebook and the phone remote at the same time. Both send commands, and
  the robot follows whichever arrived last.
