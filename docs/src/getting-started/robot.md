# On the robot

The robot runs JupyterLab on its Raspberry Pi, and `omni_drive` is already installed there.

## Connect

1. Join the robot's Wi-Fi network. The lab instructor provides its name and password.
2. Open the JupyterLab address given by the instructor in a browser.
3. Log in with the Jupyter password, which is also provided by the instructor.
4. Open the lab notebook. Changes to it are stored on the robot.

## Check that the robot answers

```python
import asyncio

from omni_drive import OmniDrive

robot = OmniDrive()
await asyncio.sleep(0.5)  # let the first messages arrive
robot.print_status()
```

This prints the battery state, the odometry pose, the wheel speeds, the age of the last LiDAR
scan and the ultrasonic readings. If it reports that no telemetry has been received, wait a
few seconds and run the cell again. If telemetry is still missing, the wheel controller or its
service is not running; contact the instructor.

## What runs on the robot

The notebook does not access the hardware directly. The robot runs services that publish
telemetry over local [ZeroMQ](https://zeromq.org/) sockets: odometry, gyroscope yaw, battery
and wheel speeds from the ESP32 wheel controller 20 times a second, and a Hokuyo LiDAR scan
about 10 times a second. The same services forward wheel speed commands from the library to
the wheel controller.

The phone remote control drives the robot through `omni_drive` in the same way as a notebook,
so the remote must not be used while a notebook is driving the robot.

The ESP32 firmware runs the wheel speed control loops and integrates the odometry. If it
receives no command for 200 ms, it stops the wheels (see [Stopping safely](../guides/safety.md)).

## Next

[Driving](../guides/driving.md).
