# Frames and units

## Units

| Quantity | Unit |
| --- | --- |
| position, distance | meters |
| angle, heading | radians |
| time | seconds |
| LiDAR points (`LidarScan.points`; `points_in_meters()` converts) | **millimeters** |
| wheel speed, `DriveVector.velocity`, `omega` | fraction of full wheel speed, −1 to 1 |

`Angle`, `Distance` and `Time` perform the conversions:

```python
from omni_drive import Angle, Distance, Time

Angle.deg(90).radians   # 1.5707...
Angle.rad(1.0).degrees  # 57.29...
Angle.rev(0.5).radians  # pi, half a revolution
Distance.cm(25).meters  # 0.25
Distance.m(0.25).as_mm  # 250.0
Time.ms(100).seconds    # 0.1
```

## Axes

Every frame in the library uses the same axes:

- **x** points forward,
- **y** points left,
- **yaw** is measured from x towards y, so positive yaw is a counter-clockwise turn seen from
  above.

Angles are wrapped to −π to π, except `robot.imu_yaw`, which keeps counting.

## Frames

**Robot frame.** Fixed to the robot, origin at its center. LiDAR points and the
ultrasonic sensor directions are in this frame, so a point at `(1000, 0)` mm is 1 m straight
ahead.

**Start frame.** Fixed to the floor. It is the robot frame at the moment a component was
started or reset, and each part of the library has its own start moment:

| Value | Start frame is the robot's pose when |
| --- | --- |
| `robot.odometry` | the ESP32 powered on |
| `robot.imu_yaw` | the ESP32 powered on |
| `robot.yaw`, field-centric `heading` | `OmniDrive()` was first created, or `robot.reset_yaw()` |
| `Localizer` estimates | `loc.start()`, or `loc.reset()` |
| `Simulator.true_pose` | the simulator started, or `sim.reset()` |

## Poses

A `Pose(x, y, yaw)` is a position and heading in some frame. Two methods move between frames:

```python
from omni_drive import Angle, Pose

start = Pose(1.0, 0.0, Angle.deg(90).radians)  # 1 m ahead, facing left
step = Pose(0.5, 0.0, 0.0)                     # 0.5 m forward, in start's own frame

end = start.compose(step)        # Pose(x=1.0, y=0.5, yaw=1.5707...)
end.relative_to(start)           # Pose(x=0.5, y=3e-17, yaw=0.0), step again
end.distance_to(start)           # 0.5
```

`a.compose(b)` applies `b`, given in `a`'s frame, on top of `a`. `b.relative_to(a)` is the
inverse operation: it expresses `b` in the frame of `a`.

To move a LiDAR point from the robot frame to the start frame, compose it with the robot's
pose:

```python
p = loc.estimate.kalman           # the robot's pose in the start frame
w = p.compose(Pose(1.0, 0.2))     # a point 1 m ahead and 0.2 m left of the robot
print(w.x, w.y)                   # the same point in the start frame
```

## Plots

`LidarView` and `plot_trajectories` draw the start frame as a map, with world x pointing up
and world y pointing left. The axis labels show the world coordinate.
