# On the robot

The robot runs JupyterLab on its Raspberry Pi, and `omni_drive` is already installed there.

## Switch it on

1. Turn on the main rocker switch (**ON/OFF**) on the body.
2. Press the power button underneath the robot. It lights up and the robot starts booting.

## Connect

1. Join the Wi-Fi network **OmniDrive** (password: `omnidrive`).
2. Open [http://omnidrive.local](http://omnidrive.local) in a browser. If the name doesn't
   resolve, use [http://10.42.0.1](http://10.42.0.1).
3. In JupyterLab, choose **File → New → Terminal** and run:

    ```bash
    lab-start
    ```

    A fresh `notebook.ipynb` appears in the file browser on the left (click the refresh icon if
    it doesn't). `lab-start` replaces the notebook that is there and keeps the old one as
    `notebook_backup_<date>_<time>.ipynb`. To continue your own work from an earlier session,
    skip this step. If the notebook is already open in a tab, close that tab before running
    `lab-start` and open the file again afterwards, so the old tab doesn't save over the fresh copy.
4. Open `notebook.ipynb` and start with the tasks.

The phone remote control is at [http://omnidrive.local/joystick/](http://omnidrive.local/joystick/).
Don't use it while a notebook is driving the robot: both send commands, and the robot follows
whichever arrived last.

## Check that the robot answers

```python
from omni_drive import OmniDrive

robot = OmniDrive()
robot.print_status()
```

This prints the battery state, the pose, the wheel speeds and the distance sensor readings.
If it prints `[Status] No telemetry received.`, wait a few seconds and run the cell again. If
telemetry is still missing, the wheel controller or its service isn't running; ask the
instructor.

## Switch it off

1. In the JupyterLab terminal, run `sudo poweroff` and wait until the system has shut down.
2. Turn the rocker switch back to **OFF**.

## What runs on the robot

The notebook doesn't talk to the hardware directly. Services on the Raspberry Pi publish the
telemetry from the ESP32 wheel controller and the Hokuyo LiDAR scans over local
[ZeroMQ](https://zeromq.org/) sockets, and forward the library's wheel commands to the ESP32.

The ESP32 firmware runs the wheel speed control and integrates the odometry. If it receives no
command for 300 ms, it stops the wheels (see [Stopping safely](../guides/safety.md)).

## Next

[Getting started](../index.md).
