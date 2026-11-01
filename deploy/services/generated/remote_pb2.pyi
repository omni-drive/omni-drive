from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class SetSpeedRequest(_message.Message):
    __slots__ = ("x", "y", "omega", "reset_pose")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    OMEGA_FIELD_NUMBER: _ClassVar[int]
    RESET_POSE_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    omega: float
    reset_pose: bool
    def __init__(self, x: _Optional[float] = ..., y: _Optional[float] = ..., omega: _Optional[float] = ..., reset_pose: _Optional[bool] = ...) -> None: ...

class DistanceSensors(_message.Message):
    __slots__ = ("left_m", "front_m", "right_m")
    LEFT_M_FIELD_NUMBER: _ClassVar[int]
    FRONT_M_FIELD_NUMBER: _ClassVar[int]
    RIGHT_M_FIELD_NUMBER: _ClassVar[int]
    left_m: float
    front_m: float
    right_m: float
    def __init__(self, left_m: _Optional[float] = ..., front_m: _Optional[float] = ..., right_m: _Optional[float] = ...) -> None: ...

class RobotTelemetry(_message.Message):
    __slots__ = ("battery_voltage", "battery_percentage", "is_battery_critical", "x", "y", "yaw_rad", "yaw_deg", "tof_sensors", "ultrasonic_sensors")
    BATTERY_VOLTAGE_FIELD_NUMBER: _ClassVar[int]
    BATTERY_PERCENTAGE_FIELD_NUMBER: _ClassVar[int]
    IS_BATTERY_CRITICAL_FIELD_NUMBER: _ClassVar[int]
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    YAW_RAD_FIELD_NUMBER: _ClassVar[int]
    YAW_DEG_FIELD_NUMBER: _ClassVar[int]
    TOF_SENSORS_FIELD_NUMBER: _ClassVar[int]
    ULTRASONIC_SENSORS_FIELD_NUMBER: _ClassVar[int]
    battery_voltage: float
    battery_percentage: float
    is_battery_critical: bool
    x: float
    y: float
    yaw_rad: float
    yaw_deg: float
    tof_sensors: DistanceSensors
    ultrasonic_sensors: DistanceSensors
    def __init__(self, battery_voltage: _Optional[float] = ..., battery_percentage: _Optional[float] = ..., is_battery_critical: _Optional[bool] = ..., x: _Optional[float] = ..., y: _Optional[float] = ..., yaw_rad: _Optional[float] = ..., yaw_deg: _Optional[float] = ..., tof_sensors: _Optional[_Union[DistanceSensors, _Mapping]] = ..., ultrasonic_sensors: _Optional[_Union[DistanceSensors, _Mapping]] = ...) -> None: ...
