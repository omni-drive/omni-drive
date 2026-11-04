# omni_drive.sensors

The time-of-flight (IR) and ultrasonic distance sensors. They are not created directly but
obtained from the robot: `robot.ir_left`, `robot.ir_front`, `robot.ir_right`, `robot.us_left`,
`robot.us_front`, `robot.us_right`.

## Sensor

```python
Sensor[SensorIdT: (IrSensorId, UltrasonicSensorId)](
    index: SensorIdT,
    esp_sub: ZmqSubscriber[TelemetryResponse] | None = None,
    config: SensorConfig | None = None,
    get_yaw: Callable[[], float] | None = None,
)
```

One distance sensor that reads the latest ESP32 telemetry message. `esp_sub` is the telemetry
subscriber; a sensor without one always reads `Distance.inf`. `config` defaults to
`SensorConfig.IR(...)` for an `IrSensorId` and `SensorConfig.US(...)` for an
`UltrasonicSensorId`, with the angle from `SENSOR_ANGLES`. `get_yaw`, if given, returns a
heading in radians used to correct the reading.

| Member | Description |
| --- | --- |
| `distance: Distance` | the current reading (below) |
| `await wait_for(condition: Callable[[Distance], bool], polling_interval: Time = Time.ms(20)) -> Distance` | reads `distance` every `polling_interval` until `condition(reading)` is true, then returns that reading |
| `index: SensorIdT` | the sensor's `IrSensorId` or `UltrasonicSensorId` |
| `config: SensorConfig` | mounting and range, described below |

`distance` is computed as follows:

1. If there is no telemetry yet, or the raw reading is 0 or less, it returns `Distance.inf`.
2. If `get_yaw` was given, the raw reading in meters is multiplied by `cos(get_yaw())`. The robot
   passes `imu_yaw` to the three IR (ToF) sensors, so their readings are multiplied by
   `cos(robot.imu_yaw.radians)`; ultrasonic readings are not.
3. `config.offset` is subtracted and the result is clamped at 0.

`min_range` and `max_range` from the config are not applied to the reading.

```python
d = await robot.ir_front.wait_for(lambda d: d.meters < 0.5)
```

`wait_for` does not skip `Distance.inf`: a condition such as `d.meters > 1.0` is satisfied by a
missing reading.

## IrSensor, UltrasonicSensor

```python
type IrSensor = Sensor[IrSensorId]
type UltrasonicSensor = Sensor[UltrasonicSensorId]
```

Type aliases for annotations. They are `type` statements, so they cannot be called or used with
`isinstance`; use `Sensor` for that.

## SensorConfig

```python
SensorConfig(angle: float, offset: float, cone: float, min_range: float, max_range: float)
```

Mounting and range of a sensor. The dataclass is frozen.

| Field | Description |
| --- | --- |
| `angle` | direction the sensor faces in the robot frame, radians; 0 is forward |
| `offset` | subtracted from every reading, meters |
| `cone` | full beam angle, radians |
| `min_range` | shortest range, meters |
| `max_range` | longest range, meters; used as the limit when ray-casting expected readings |

| Member | Description |
| --- | --- |
| `SensorConfig.IR(angle: float = 0.0)` | ToF sensor: `offset=0.0`, `cone=radians(27)`, `min_range=0.04`, `max_range=4.0` |
| `SensorConfig.US(angle: float = 0.0)` | ultrasonic sensor: `offset=0.0132`, `cone=radians(55)`, `min_range=0.01`, `max_range=4.0` |
| `face: tuple[float, float]` | position of the sensor face in the robot frame: 0.15 m from the center in the direction `angle`, meters |
| `beam_angles(rays: int = 7) -> np.ndarray` | `rays` directions spread evenly across the cone, robot frame, radians |

## Constants

| Name | Value |
| --- | --- |
| `SENSOR_ANGLES` | `LEFT` π/2, `FRONT` 0, `RIGHT` −π/2, for both `IrSensorId` and `UltrasonicSensorId` |
| `BEAM_RAYS` | 7, the default number of rays for `beam_angles` |
| `POLL_INTERVAL` | `Time.ms(20)`, the default `wait_for` interval |
