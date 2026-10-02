# Expected ultrasonic ranges

`UltrasonicView` compares each ultrasonic reading with the reading expected if the robot stood
at a given pose. `Localizer.expected_range(sensor, pose)` computes that value from
the LiDAR map.

## The sensor model

Each sensor has an `UltrasonicConfig` in `sensor.config`:

| Field | Default | Meaning |
| --- | --- | --- |
| `angle` | 0, π/2, −π/2 | direction the sensor faces in the robot frame |
| `offset` | 0.15 m | distance from the robot center to the sensor face |
| `cone` | 25° | full width of the beam |
| `min_range` | 0.02 m | shortest distance the sensor reports |
| `max_range` | 4.0 m | reported when no echo comes back |
| `noise_std` | 0.01 m | noise of an emulated reading |
| `outlier_prob` | 0.03 | chance that an emulated reading is a false echo |

A reading is the distance from the sensor face to the nearest object inside the cone.

## Ray casting on the map

The map is the scan matcher's occupancy grid (`loc.slam.map`, and `loc.map_snapshot()` gives
a copy). It has 3 cm cells, and each cell holds the probability that it is occupied.

For a pose \((x, y, \psi)\) and a sensor at angle \(\alpha\) and offset \(r\):

1. Place the sensor face in the start frame: `pose.compose(Pose(*sensor.config.face))`, where
   `face` is \((r\cos\alpha, r\sin\alpha)\) in the robot frame.
2. Spread 7 rays evenly across the cone (`config.beam_angles()`), at world angles from
   \(\psi + \alpha - \text{cone}/2\) to \(\psi + \alpha + \text{cone}/2\).
3. Walk along each ray in steps of half a cell (1.5 cm), up to `max_range`. The ray stops at the
   first cell with occupancy probability above 0.65. Cells outside the map count as
   unknown, not occupied.
4. The expected reading is the shortest of the 7 distances, or `max_range` if no ray hit
   anything.

```python
from omni_drive import Localizer, OmniDrive

robot = OmniDrive()
loc = Localizer.default()
e = loc.estimate
for sensor in robot.ultrasonic_sensors:
    print(sensor.sensor_id.name, loc.expected_range(sensor, e.kalman), sensor.read())
```

## Reading the comparison

If the pose is correct and the map is accurate, the expected reading matches the sensor up to
its noise. A pose that is 5 cm too far forward makes the front sensor's expected reading 5 cm
too short. The gap between the dots and a line in `UltrasonicView` is therefore the error of
that pose estimate, projected onto the sensor's direction.

The comparison has two limitations:

- The map contains only what the LiDAR has observed. Objects below or above the LiDAR's scan
  plane are not in it, but the ultrasonic sensor may still detect them.
- While the readings are [emulated](../guides/sensors.md#emulated-readings), they come from the
  LiDAR scan itself, so they agree with the LiDAR pose almost by construction. In that case the
  comparison verifies the implementation, not the LiDAR pose.

## How emulated readings are computed

The emulation in `UltrasonicSensor.read()` works on the raw scan, not on the map. It takes the
LiDAR points in the robot frame and drops those within 0.18 m of the robot center (returns
from the robot itself). Then it measures the rest from the sensor face, keeps the ones inside
the cone and at least `min_range` away, and takes the nearest. `sensor.ideal_range(points)`
returns this value.

`noisy_reading` then adds Gaussian noise with standard deviation `noise_std`. With
probability `outlier_prob` it returns a false echo instead: half the time `max_range`,
otherwise a random distance between `min_range` and the true range, at most 0.5 m.
