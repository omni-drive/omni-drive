# omni_drive.sensors

The ultrasonic sensors. They are not created directly but obtained from the robot
(`robot.us_1` to `robot.us_3`). See [Reading sensors](../guides/sensors.md).

## UltrasonicSensor

```python
UltrasonicSensor(
    sensor_id: UltrasonicSensorId,
    config: UltrasonicConfig,
    read_hardware: Callable[[], float | None],
    latest_scan: Callable[[], tuple[LidarScan | None, int]],
    seed: int | None = None,
)
```

An HC-SR04 ultrasonic sensor. It returns the sensor's reading when the robot transmits one and
otherwise emulates the reading from the latest LiDAR scan. The firmware does not currently
transmit ultrasonic readings, so on the physical robot they are always emulated. `read_hardware` returns the sensor's
reading in meters or `None`; `latest_scan` returns the latest scan and the scan count, like
`robot.lidar()`. `seed` seeds the emulation noise.

| Member | Description |
| --- | --- |
| `read() -> Distance | None` | distance from the sensor face to the nearest object in the beam. `None` before any data has arrived, or when the sensor reported 0 or less |
| `await wait_for(condition: Callable[[Distance], bool], polling_interval: Time = Time.ms(20)) -> Distance` | waits until a reading is not `None` and `condition(reading)` is true, then returns it |
| `emulated: bool` | `True` if the last `read()` came from the LiDAR, `False` if it came from the sensor |
| `sensor_id: UltrasonicSensorId` | `US_1`, `US_2` or `US_3` |
| `config: UltrasonicConfig` | mounting and noise model, described below |
| `ideal_range(points: np.ndarray) -> float` | noise-free emulated reading for a scan given as an `(N, 2)` array in meters, robot frame. Ignores points closer than 0.18 m to the robot center (`hardware.LIDAR_SELF_RETURN_RANGE_M`); `config.max_range` when the cone is empty |

```python
d = await robot.us_1.wait_for(lambda d: d.meters < 0.5)
```

An emulated sensor computes one new value per LiDAR scan and returns the same value until the
next scan. See [How emulated readings are computed](../concepts/ultrasonic.md#how-emulated-readings-are-computed).

## UltrasonicConfig

```python
UltrasonicConfig(
    angle: float,
    offset: float = 0.15,
    cone: float = math.radians(25),
    min_range: float = 0.02,
    max_range: float = 4.0,
    noise_std: float = 0.01,
    outlier_prob: float = 0.03,
)
```

| Field | Description |
| --- | --- |
| `angle` | direction the sensor faces in the robot frame, radians; 0 is forward |
| `offset` | distance of the sensor face from the robot center, meters |
| `cone` | full beam angle, radians |
| `min_range` | shortest reading, meters |
| `max_range` | reading when no echo returns, meters |
| `noise_std` | standard deviation of emulated noise, meters |
| `outlier_prob` | probability that an emulated reading is a false echo |
| `face: tuple[float, float]` | position of the sensor face in the robot frame, meters (read-only) |
| `beam_angles(rays: int = 7) -> np.ndarray` | `rays` directions spread evenly across the cone, robot frame, radians |
| `in_beam(relative_points: np.ndarray) -> np.ndarray` | for `(N, 2)` points relative to the sensor face, a boolean mask of those inside the cone |

`DEFAULT_ULTRASONIC` maps each `UltrasonicSensorId` to its configuration: `US_1` front
(`angle=0`), `US_2` left (`π/2`), `US_3` right (`−π/2`).

```python
from omni_drive.models import UltrasonicSensorId
from omni_drive.sensors import DEFAULT_ULTRASONIC

DEFAULT_ULTRASONIC[UltrasonicSensorId.US_2].face  # (9.2e-18, 0.15)
```

## noisy_reading

```python
noisy_reading(true_range: float, config: UltrasonicConfig, rng: np.random.Generator) -> float
```

Converts a true distance into a reading of the kind an HC-SR04 produces, in meters. With probability
`config.outlier_prob` it returns a false echo: half the time `config.max_range`, otherwise a
random distance between `config.min_range` and the true range (at most 0.5 m). In all other cases it
adds Gaussian noise with `config.noise_std`. A true range at `max_range` or beyond returns
`max_range`. The result is clipped to `[min_range, max_range]`. The function is used by both the
emulated sensors and the simulator.

```python
import numpy as np

from omni_drive.models import UltrasonicSensorId
from omni_drive.sensors import DEFAULT_ULTRASONIC, noisy_reading

rng = np.random.default_rng(0)
noisy_reading(1.0, DEFAULT_ULTRASONIC[UltrasonicSensorId.US_1], rng)  # about 1.0
```
