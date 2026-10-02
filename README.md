# omni-drive

`omni_drive` is a Python library for OmniDrive, a three-wheel omnidirectional robot used in
robotics laboratory classes. The robot has three omni wheels, a 2D LiDAR, a gyroscope and three
ultrasonic sensors.

The library provides:

- driving commands with acceleration limiting,
- access to odometry, gyroscope, LiDAR and ultrasonic readings,
- localization from wheel odometry, LiDAR scan matching and a Kalman filter,
- live Bokeh views for Jupyter notebooks,
- a simulator that replaces the robot, so the same code runs without hardware.

On the robot, services publish telemetry and receive commands over ZeroMQ. The library
connects to them, or to the simulator, in the same way.

## Installation

The library requires Python 3.12 or later.

```bash
pip install git+https://github.com/omni-drive/omni-drive
```

With [uv](https://docs.astral.sh/uv/):

```bash
uv add git+https://github.com/omni-drive/omni-drive
```

## Quickstart with the simulator

The library and the robot communicate over `ipc://` sockets by default. The environment variable
`OMNI_DRIVE_IPC` switches both sides to TCP on three consecutive ports starting at the given one.
This is required on Windows, which does not support `ipc://` sockets.

Start the simulator in a terminal:

```bash
OMNI_DRIVE_IPC=tcp://127.0.0.1:5650 python -m omni_drive.simulator --world room
```

In PowerShell:

```powershell
$env:OMNI_DRIVE_IPC = "tcp://127.0.0.1:5650"
python -m omni_drive.simulator --world room
```

Then, in a notebook or script, set the same variable before the first `omni_drive` import:

```python
import os

os.environ["OMNI_DRIVE_IPC"] = "tcp://127.0.0.1:5650"

from omni_drive import OmniDrive

robot = OmniDrive()
robot.print_status()
```

## Documentation

The documentation is available at <https://omni-drive.readthedocs.io/>. To preview it locally:

```bash
uv run --group docs mkdocs serve -f docs/mkdocs.yml
```

## Development

```bash
uv sync
uv run ruff check .
uv run ruff format --check .
uv run pytest -q
```

The unit tests run the library against the simulator.

The message schema is `omni_drive/proto/robot.proto`, and the generated module
`omni_drive/generated/robot_pb2.py` is kept in the repository. After changing the schema,
regenerate the module with:

```bash
uv run python scripts/gen_proto.py
```

A test checks that the committed module matches the schema.

## License

GNU General Public License v3.0 or later. See [LICENSE](LICENSE).
