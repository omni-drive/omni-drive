# omni_drive.hardware

Robot and firmware constants in SI units. They convert the library's fractions of full speed
into m/s and rad/s.

```python
from omni_drive.hardware import FULL_SPEED_RIM_M_S, FULL_SPEED_YAW_RATE_RAD_S
```

| Constant | Value | Meaning |
| --- | --- | --- |
| `WHEEL_RADIUS_M` | 0.035 | wheel radius, meters |
| `ENCODER_TICKS_PER_REV` | 990 | encoder ticks per wheel revolution |
| `ENCODER_TICKS_PER_M` | 4501.8 | encoder ticks per meter rolled |
| `FULL_SPEED_TICKS_PER_S` | 1750 | wheel speed 1.0, in ticks per second |
| `FULL_SPEED_RIM_M_S` | 0.389 | wheel speed 1.0, in meters per second at the rim |
| `WHEEL_BASE_RADIUS_M` | 0.1 | distance from the robot center to each wheel, meters |
| `FULL_SPEED_YAW_RATE_RAD_S` | 3.89 | turn rate with all three wheels at 1.0, radians per second |
| `COMMAND_WATCHDOG_S` | 0.2 | time without a command after which the firmware stops the wheels, seconds |
| `ROBOT_RADIUS_M` | 0.17 | radius of the robot's round body, meters |
| `LIDAR_SELF_RETURN_RANGE_M` | 0.18 | LiDAR points closer than this are reflections from the robot itself and are ignored, meters |
| `BATTERY_FULL_V` | 16.8 | battery voltage shown as 100 % by `print_status` |
| `BATTERY_EMPTY_V` | 13.0 | battery voltage shown as 0 % |
| `BATTERY_CRITICAL_V` | 12.8 | below this the battery is critical |
| `HOKUYO_MIN_RANGE_M`, `HOKUYO_MAX_RANGE_M` | 0.02, 5.6 | LiDAR range limits, meters |
| `HOKUYO_STEPS_PER_REV` | 1024 | LiDAR angular steps per full turn |
| `HOKUYO_FRONT_STEP` | 384 | the step that points straight ahead |
| `HOKUYO_FIRST_STEP`, `HOKUYO_LAST_STEP` | 44, 725 | the steps the LiDAR measures (682 beams, about 240°) |
| `HOKUYO_STEP_ANGLES_RAD` | tuple | the direction of each measured step in the robot frame, radians |

These are the values assumed by the firmware. The physical robot deviates from them because of
wheel wear and slip; where an exact speed is required, it should be measured.

## Converting a DriveVector

| Field | Physical value |
| --- | --- |
| `velocity` | `velocity * FULL_SPEED_RIM_M_S` meters per second |
| `omega` | `omega * FULL_SPEED_YAW_RATE_RAD_S` radians per second, counter-clockwise |

```python
from omni_drive import Angle, DriveVector
from omni_drive.hardware import FULL_SPEED_RIM_M_S, FULL_SPEED_YAW_RATE_RAD_S

forward_15_cm_s = DriveVector(0.15 / FULL_SPEED_RIM_M_S, Angle.deg(0))         # velocity 0.386
turn_45_deg_s = DriveVector(0.0, Angle.deg(0), omega=0.785 / FULL_SPEED_YAW_RATE_RAD_S)  # omega 0.202
```

This conversion holds as long as no wheel is commanded above full speed. Beyond that, all three
wheel speeds are scaled down by the same factor.
