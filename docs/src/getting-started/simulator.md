# Without a robot

`omni_drive.simulator` replaces the services that run on the robot. It simulates the wheels, odometry
with calibration errors and wheel slip, a drifting gyroscope and the LiDAR, in a small world
made of walls. User code does not change: `OmniDrive()` connects to the simulator in the same
way as to the robot.

## Install

Install the library together with Jupyter:

```bash
pip install git+https://github.com/omni-drive/omni-drive jupyterlab
```

Start Jupyter with `jupyter lab`.

## Choose the connection

On the robot, the library and the services communicate over `ipc://` sockets in `/tmp`.
Windows does not support `ipc://` sockets, so both sides must be switched to TCP with the
environment variable `OMNI_DRIVE_IPC`. The library uses three consecutive ports, starting at
the given one.

On Linux and macOS this step is optional.

## Run the simulator in a terminal

=== "PowerShell"

    ```powershell
    $env:OMNI_DRIVE_IPC = "tcp://127.0.0.1:5650"
    python -m omni_drive.simulator --world room
    ```

=== "bash"

    ```bash
    OMNI_DRIVE_IPC=tcp://127.0.0.1:5650 python -m omni_drive.simulator --world room
    ```

Then set the same variable in the notebook, before the first `omni_drive` import. The library
reads it only once, at import time.

```python
import os

os.environ["OMNI_DRIVE_IPC"] = "tcp://127.0.0.1:5650"

from omni_drive import OmniDrive

robot = OmniDrive()
```

If `omni_drive` has already been imported in the current kernel, restart the kernel. The
variable can also be set in the terminal before `jupyter lab`, in which case every
notebook uses it.

Options:

| Option | Meaning |
| --- | --- |
| `--world box` | 1 m × 1 m box with a small block in one corner (default) |
| `--world room` | 4 m × 3 m room with three obstacles |
| `--ultrasonic` | publish ultrasonic readings, as if the firmware supported the sensors |
| `--seed N` | random seed for the noise (default 0) |
| `--speed-factor F` | run the physics `F` times faster than real time |

Stop the simulator with ++ctrl+c++.

## Run the simulator inside the notebook

The simulator can also run in the notebook kernel. In this mode the true pose is available for
comparison with the estimates.

```python
import os

os.environ["OMNI_DRIVE_IPC"] = "tcp://127.0.0.1:5650"

from omni_drive import OmniDrive
from omni_drive.simulator import Simulator

sim = Simulator(world="room").start()
robot = OmniDrive()

...

print(sim.true_pose)  # (x, y, yaw) in the start frame
sim.stop()
```

## Things to know

- Collisions only stop the robot. The wheels keep turning, so odometry keeps counting.
- Ultrasonic readings are only published with `--ultrasonic` (or
  `Simulator(publish_ultrasonic=True)`). Without them the library emulates the sensors from the
  LiDAR scan, as it currently does on the robot. See [Reading sensors](../guides/sensors.md#ultrasonic-sensors).
