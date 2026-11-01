# Stopping safely

A background thread in the library resends the last command to the robot 50 times a second
until it is changed. Consequently, a cell that finishes, fails or is interrupted does not stop
the robot. Only `drive_for` stops the robot automatically.

## Prefer `drive_for`

```python
from omni_drive import Angle, DriveVector, OmniDrive, Time

robot = OmniDrive()
await robot.drive_for(DriveVector(0.3, Angle.deg(0)), Time.s(2))
```

`drive_for` always stops the robot at the end, including when the Jupyter stop button is pressed.

## Stop in `finally`

When using `set_vector` or raw wheel speeds, stop the robot in a `finally` block:

```python
try:
    robot.set_vector(DriveVector(0.3, Angle.deg(0)))
    await robot.us_1.wait_for(lambda d: d.meters < 0.3)  # until a wall is 30 cm ahead
finally:
    robot.stop()
```

The Jupyter stop button interrupts the `await`, and the `finally` block still runs.

To stop the robot manually, keep a cell containing only this line and run it:

```python
robot.stop()
```

In vector mode the robot decelerates over a fraction of a second instead of stopping instantly.

## Close the connection

```python
robot.close()
```

`close()` stops the wheels, stops the background thread and closes the sockets. Calling it
more than once is safe. After that, commands on this object raise `RuntimeError`; call
`OmniDrive()` to connect again. Python also calls `close()` when the kernel shuts down
normally.

## The watchdog

The ESP32 firmware sets all wheel speeds to zero when it receives no command for 200 ms
(`omni_drive.hardware.COMMAND_WATCHDOG_S`). This stops the robot when:

- the kernel is restarted or shut down;
- the kernel process crashes;
- the service on the robot that forwards the commands stops.

The watchdog does not act while the kernel is running. A cell busy with a long computation, or
stuck in an endless loop, does not stop the background thread, so the robot continues driving.
In that case, stop it with the stop button and a `finally` block, or restart the kernel.

## Before you drive

- Check the battery with `robot.print_status()`. `CRITICAL` after the battery line means
  that the battery must be charged immediately.
- Start at a low `velocity`, for example 0.2.
- Do not drive from a notebook and the phone remote at the same time. Both send commands, and
  the robot follows whichever arrived last.
