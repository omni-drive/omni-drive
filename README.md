# omni-drive

Public materials for OmniDrive, a three-wheel omnidirectional robot used in robotics laboratory
classes. The robot has three omni wheels, a 2D LiDAR, a gyroscope, and ToF and ultrasonic distance
sensors. It is programmed in Python with the `omni_drive` library.

This repository contains:

- `docs/`: documentation of the `omni_drive` library, published at
  <https://omni-drive.readthedocs.io/>,
- `notebook.ipynb`: the lab notebook (in Polish): drive kinematics, odometry, positioning with
  the LiDAR, distance sensors and a Kalman filter,
- `lab/konspekt.pdf`: the lab outline that students read before the class.

## Running the notebook

The notebook runs on the robot, which serves Jupyter over its Wi-Fi network. To run it on another
computer, install the dependencies with [uv](https://docs.astral.sh/uv/) (Python 3.12 or later):

```bash
uv sync
uv run jupyter notebook notebook.ipynb
```

`pyproject.toml` pins the `omni_drive` release the notebook is written for. The library connects
to the robot's services over ZeroMQ `ipc://` sockets, so the notebook needs the robot's services or
a simulator running on the same machine.

## Documentation

To preview the documentation locally:

```bash
uv run --group docs mkdocs serve -f docs/mkdocs.yml
```

## License

GNU General Public License v3.0 or later. See [LICENSE](LICENSE).
