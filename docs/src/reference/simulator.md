# omni_drive.simulator

Replaces the services that run on the robot and connect the ESP32 and the LiDAR, which allows the library to be used without a
robot. Setup is described in [Without a robot](../getting-started/simulator.md).

## Command line

```text
python -m omni_drive.simulator [--world {box,room}] [--ultrasonic] [--seed N] [--speed-factor F]
```

| Option | Default | Description |
| --- | --- | --- |
| `--world` | `box` | `box`: 1 m × 1 m with a 10 cm block in the front-left corner. `room`: 4 m × 3 m with three obstacles, robot off-center |
| `--ultrasonic` | off | publish ultrasonic readings in the telemetry |
| `--seed` | `0` | random seed |
| `--speed-factor` | `1.0` | run the physics this many times faster than real time |

Every 5 seconds the simulator logs the true pose next to the odometry. It is stopped with
++ctrl+c++.

On Windows, `OMNI_DRIVE_IPC` must be set (for example `tcp://127.0.0.1:5650`) for both the
simulator and the notebook.

## What it simulates

| Part | Model |
| --- | --- |
| Wheels | each wheel follows its command with a 0.08 s lag; full speed is `FULL_SPEED_RIM_M_S`, about 0.39 m/s at the rim ([constants](hardware.md)) |
| Watchdog | wheel targets drop to zero 0.2 s after the last command, as in the firmware |
| Odometry | the firmware's integration, from encoder ticks with per-wheel calibration errors and random slip |
| Gyroscope | true heading plus a slow drift (0.5° per minute) and white noise |
| LiDAR | 682 beams over about 240°, 10 scans per second, 1 cm noise (1 % of the range when larger), 1 % dropped beams, ranges 0.02 to 5.6 m |
| Ultrasonic | minimum over a 25° cone from a face 0.15 m from the center, 1 cm noise, 3 % false echoes; only with `--ultrasonic` |
| Battery | voltage decreasing slowly with time and with load |
| Collisions | the robot body (radius 0.17 m) stops at walls and slides along them; the wheels continue to turn, so odometry continues to accumulate |

Telemetry is published 20 times a second.

## Simulator

```python
Simulator(
    world: str | World = "box",
    seed: int | None = 0,
    publish_ultrasonic: bool = False,
    speed_factor: float = 1.0,
    wheel_scale_error: tuple[float, float, float] = (0.05, 0.03, 0.02),
    robot_radius_error: float = 0.01,
    slip_noise: float = 0.015,
    imu_drift_deg_per_min: float = 0.5,
    imu_noise_rad: float = 0.002,
    lidar_noise_m: float = 0.01,
)
```

| Parameter | Description |
| --- | --- |
| `world` | a name from `WORLDS` (`"box"`, `"room"`) or a `World` |
| `seed` | random seed; `None` for a different run each time |
| `publish_ultrasonic` | include ultrasonic readings in the telemetry |
| `speed_factor` | physics time per wall-clock second |
| `wheel_scale_error` | relative error of each wheel's measured distance; bends straight lines in odometry |
| `robot_radius_error` | relative error of the center-to-wheel distance: the actual distance is larger than the firmware's 0.1 m by this fraction, so odometry overestimates every turn |
| `slip_noise` | random error per wheel, in meters per square root of a meter rolled |
| `imu_drift_deg_per_min` | gyroscope drift |
| `imu_noise_rad` | gyroscope white noise per message |
| `lidar_noise_m` | LiDAR range noise |

### Running

| Member | Description |
| --- | --- |
| `start() -> Simulator` | binds the service sockets and starts the simulation thread; returns `self` |
| `stop() -> None` | stops the thread and releases the sockets |
| `with Simulator() as sim:` | the same as `start()` and `stop()` |
| `reset() -> None` | puts the robot back at the start and zeroes odometry, gyroscope and wheel speeds |

### Ground truth and sensors

| Member | Description |
| --- | --- |
| `true_pose: tuple[float, float, float]` | the real `(x, y, yaw)` in the start frame |
| `odometry: tuple[float, float, float]` | what the firmware reports as odometry |
| `imu_yaw: float` | gyroscope heading without the white noise |
| `wheel_speeds: tuple[float, float, float]` | actual wheel speeds, fractions of full speed |
| `telemetry() -> TelemetryResponse` | one telemetry message as the firmware would send it now |
| `lidar_scan() -> LidarScan` | one scan as the LiDAR service would send it now |
| `ultrasonic_ranges() -> tuple[float, float, float]` | one reading per ultrasonic sensor, meters |

### Driving without sockets

In tests, the physics can be advanced directly, without `start()`:

| Member | Description |
| --- | --- |
| `set_wheel_speeds(w1: float, w2: float, w3: float) -> None` | the same as receiving one wheel command now |
| `step(duration: float) -> None` | advances the simulation by `duration` seconds |

```python
sim = Simulator("box", seed=0)
sim.set_wheel_speeds(0.3, 0.3, 0.3)
sim.step(0.1)          # less than the 0.2 s watchdog
print(sim.true_pose)   # turned counter-clockwise
```

## World

```python
World(name: str, segments: np.ndarray)
make_world(name: str) -> World
box_walls(x0: float, y0: float, x1: float, y1: float) -> list[Segment]
```

A map made of wall segments: `segments` has shape `(S, 2, 2)`, each row two end points in
meters in the start frame. `WORLDS` maps each built-in world's name to its segments, and
`make_world(name)` builds a `World` from it (`ValueError` for an unknown name).
`box_walls` returns the four walls of the rectangle with corners `(x0, y0)` and `(x1, y1)`.

```python
import numpy as np

from omni_drive.simulator import Simulator, World, box_walls

walls = box_walls(-1.0, -0.4, 3.0, 0.4) + box_walls(1.0, -0.4, 1.2, 0.0)  # corridor with a block
corridor = World("corridor", np.array(walls))
sim = Simulator(world=corridor).start()
```

The robot body is a circle of radius `hardware.ROBOT_RADIUS_M` (0.17 m); the start position,
which is the origin, must be at least that far from every wall.
