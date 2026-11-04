# Expected ultrasonic ranges

`Localizer.expected_range(sensor, pose)` computes the reading an ultrasonic sensor would give
if the robot stood at `pose`, using the LiDAR map. Comparing it with the real reading shows how
far a pose estimate is off along the sensor's direction.

## The sensor model

Each sensor has a `SensorConfig` in `sensor.config`. For the ultrasonic sensors the defaults
are:

| Field | Default | Used for |
| --- | --- | --- |
| `angle` | π/2 (left), 0 (front), −π/2 (right) | direction the sensor faces in the robot frame |
| `offset` | 0.0132 m | subtracted from every reading in `sensor.distance` |
| `cone` | 55° | full width of the beam in `expected_range` |
| `min_range` | 0.01 m | not used by `expected_range` |
| `max_range` | 4.0 m | ray length in `expected_range`, and its value when no ray hits |

`config.face` is the sensor face in the robot frame: 0.15 m from the center in the direction of
`angle`.

The real sensor reports no echo beyond 2 m. `sensor.distance` is then `Distance.inf`, while
`expected_range` returns at most 4.0 m. Compare the two only where the reading is finite.

## Ray casting on the map

The map is the scan matcher's occupancy grid (`loc.slam.map`; `loc.map_snapshot()` gives a
copy). It has 3 cm cells, and each cell holds the probability that it is occupied.

For a pose \((x, y, \psi)\) and a sensor at angle \(\alpha\):

1. Place the sensor face in the start frame: `pose.compose(RawPose(*sensor.config.face))`.
2. Spread 7 rays evenly across the cone (`config.beam_angles()`), at world angles from
   \(\psi + \alpha - \text{cone}/2\) to \(\psi + \alpha + \text{cone}/2\).
3. Walk along each ray in steps of half a cell (1.5 cm), up to `max_range`. The ray stops at the
   first point whose cell has an occupancy probability above 0.65. Points outside the map count
   as unknown (0.5), not occupied.
4. The expected reading is the shortest of the 7 distances, or `max_range` if no ray hit
   anything.

The result is a `float` in meters. `pose` is a `RawPose`, as in the `Localizer`'s estimates:

```python
from omni_drive import Localizer, OmniDrive

robot = OmniDrive()
loc = Localizer.default()
e = await loc.wait_ready()

for sensor in robot.ultrasonic_sensors:
    expected = loc.expected_range(sensor, e.kalman)
    print(f"{sensor.index.name:5}  expected {expected:.3f} m  measured {sensor.distance.meters:.3f} m")
```

`UltrasonicView` in `omni_drive.ui` plots the same comparison over time for the odometry,
LiDAR and Kalman poses.

## Reading the comparison

If the pose is correct and the map is accurate, the expected reading matches the sensor up to
its noise. A pose that is 5 cm too far forward makes the front sensor's expected reading 5 cm
too short. The gap between the measured and the expected value is therefore the error of that
pose estimate, projected onto the sensor's direction.

The comparison has limits:

- The map contains only what the LiDAR has observed. Objects below or above the LiDAR's scan
  plane are not in it, but the ultrasonic sensor may still detect them.
- The model uses the shortest of 7 rays across the full cone. A real ultrasonic sensor does not
  always return the nearest echo in its cone, especially from a wall hit at a steep angle.
- Each sensor is read only about every 100 ms, and the firmware adds Gaussian noise with a
  standard deviation of 8 mm (see [Reading sensors](../guides/sensors.md#distance-sensors)).
