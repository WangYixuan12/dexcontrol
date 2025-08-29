from collections.abc import Iterable as _Iterable
from typing import ClassVar as _ClassVar
from typing import Optional as _Optional

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf.internal import containers as _containers

DESCRIPTOR: _descriptor.FileDescriptor

class MotorStateWithTorque(_message.Message):
    __slots__ = ("pos", "vel", "torque", "error", "timestamp_ns")
    POS_FIELD_NUMBER: _ClassVar[int]
    VEL_FIELD_NUMBER: _ClassVar[int]
    TORQUE_FIELD_NUMBER: _ClassVar[int]
    ERROR_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_NS_FIELD_NUMBER: _ClassVar[int]
    pos: _containers.RepeatedScalarFieldContainer[float]
    vel: _containers.RepeatedScalarFieldContainer[float]
    torque: _containers.RepeatedScalarFieldContainer[float]
    error: _containers.RepeatedScalarFieldContainer[int]
    timestamp_ns: int
    def __init__(self, pos: _Optional[_Iterable[float]] = ..., vel: _Optional[_Iterable[float]] = ..., torque: _Optional[_Iterable[float]] = ..., error: _Optional[_Iterable[int]] = ..., timestamp_ns: _Optional[int] = ...) -> None: ...

class MotorStateWithCurrent(_message.Message):
    __slots__ = ("pos", "vel", "cur", "error", "timestamp_ns")
    POS_FIELD_NUMBER: _ClassVar[int]
    VEL_FIELD_NUMBER: _ClassVar[int]
    CUR_FIELD_NUMBER: _ClassVar[int]
    ERROR_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_NS_FIELD_NUMBER: _ClassVar[int]
    pos: _containers.RepeatedScalarFieldContainer[float]
    vel: _containers.RepeatedScalarFieldContainer[float]
    cur: _containers.RepeatedScalarFieldContainer[float]
    error: _containers.RepeatedScalarFieldContainer[int]
    timestamp_ns: int
    def __init__(self, pos: _Optional[_Iterable[float]] = ..., vel: _Optional[_Iterable[float]] = ..., cur: _Optional[_Iterable[float]] = ..., error: _Optional[_Iterable[int]] = ..., timestamp_ns: _Optional[int] = ...) -> None: ...

class MotorPosCommand(_message.Message):
    __slots__ = ("pos", "timestamp_ns")
    POS_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_NS_FIELD_NUMBER: _ClassVar[int]
    pos: _containers.RepeatedScalarFieldContainer[float]
    timestamp_ns: int
    def __init__(self, pos: _Optional[_Iterable[float]] = ..., timestamp_ns: _Optional[int] = ...) -> None: ...

class MotorVelCommand(_message.Message):
    __slots__ = ("vel", "timestamp_ns")
    VEL_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_NS_FIELD_NUMBER: _ClassVar[int]
    vel: _containers.RepeatedScalarFieldContainer[float]
    timestamp_ns: int
    def __init__(self, vel: _Optional[_Iterable[float]] = ..., timestamp_ns: _Optional[int] = ...) -> None: ...

class MotorPosVelCommand(_message.Message):
    __slots__ = ("pos", "vel", "timestamp_ns")
    POS_FIELD_NUMBER: _ClassVar[int]
    VEL_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_NS_FIELD_NUMBER: _ClassVar[int]
    pos: _containers.RepeatedScalarFieldContainer[float]
    vel: _containers.RepeatedScalarFieldContainer[float]
    timestamp_ns: int
    def __init__(self, pos: _Optional[_Iterable[float]] = ..., vel: _Optional[_Iterable[float]] = ..., timestamp_ns: _Optional[int] = ...) -> None: ...

class MotorPosVelCurrentCommand(_message.Message):
    __slots__ = ("pos", "vel", "cur", "timestamp_ns")
    POS_FIELD_NUMBER: _ClassVar[int]
    VEL_FIELD_NUMBER: _ClassVar[int]
    CUR_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_NS_FIELD_NUMBER: _ClassVar[int]
    pos: _containers.RepeatedScalarFieldContainer[float]
    vel: _containers.RepeatedScalarFieldContainer[float]
    cur: _containers.RepeatedScalarFieldContainer[float]
    timestamp_ns: int
    def __init__(self, pos: _Optional[_Iterable[float]] = ..., vel: _Optional[_Iterable[float]] = ..., cur: _Optional[_Iterable[float]] = ..., timestamp_ns: _Optional[int] = ...) -> None: ...

class EndEffectorPassThroughCommand(_message.Message):
    __slots__ = ("data", "timestamp_ns")
    DATA_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_NS_FIELD_NUMBER: _ClassVar[int]
    data: bytes
    timestamp_ns: int
    def __init__(self, data: _Optional[bytes] = ..., timestamp_ns: _Optional[int] = ...) -> None: ...

class BMSState(_message.Message):
    __slots__ = ("voltage", "current", "temperature", "percentage", "is_charging", "error", "timestamp_ns")
    VOLTAGE_FIELD_NUMBER: _ClassVar[int]
    CURRENT_FIELD_NUMBER: _ClassVar[int]
    TEMPERATURE_FIELD_NUMBER: _ClassVar[int]
    PERCENTAGE_FIELD_NUMBER: _ClassVar[int]
    IS_CHARGING_FIELD_NUMBER: _ClassVar[int]
    ERROR_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_NS_FIELD_NUMBER: _ClassVar[int]
    voltage: float
    current: float
    temperature: float
    percentage: int
    is_charging: bool
    error: int
    timestamp_ns: int
    def __init__(self, voltage: _Optional[float] = ..., current: _Optional[float] = ..., temperature: _Optional[float] = ..., percentage: _Optional[int] = ..., is_charging: bool = ..., error: _Optional[int] = ..., timestamp_ns: _Optional[int] = ...) -> None: ...

class WrenchState(_message.Message):
    __slots__ = ("wrench", "blue_button", "green_button", "timestamp_ns")
    WRENCH_FIELD_NUMBER: _ClassVar[int]
    BLUE_BUTTON_FIELD_NUMBER: _ClassVar[int]
    GREEN_BUTTON_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_NS_FIELD_NUMBER: _ClassVar[int]
    wrench: _containers.RepeatedScalarFieldContainer[float]
    blue_button: bool
    green_button: bool
    timestamp_ns: int
    def __init__(self, wrench: _Optional[_Iterable[float]] = ..., blue_button: bool = ..., green_button: bool = ..., timestamp_ns: _Optional[int] = ...) -> None: ...

class EStopState(_message.Message):
    __slots__ = ("software_estop_enabled", "left_button_pressed", "right_button_pressed", "waist_button_pressed", "wireless_button_pressed", "timestamp_ns")
    SOFTWARE_ESTOP_ENABLED_FIELD_NUMBER: _ClassVar[int]
    LEFT_BUTTON_PRESSED_FIELD_NUMBER: _ClassVar[int]
    RIGHT_BUTTON_PRESSED_FIELD_NUMBER: _ClassVar[int]
    WAIST_BUTTON_PRESSED_FIELD_NUMBER: _ClassVar[int]
    WIRELESS_BUTTON_PRESSED_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_NS_FIELD_NUMBER: _ClassVar[int]
    software_estop_enabled: bool
    left_button_pressed: bool
    right_button_pressed: bool
    waist_button_pressed: bool
    wireless_button_pressed: bool
    timestamp_ns: int
    def __init__(self, software_estop_enabled: bool = ..., left_button_pressed: bool = ..., right_button_pressed: bool = ..., waist_button_pressed: bool = ..., wireless_button_pressed: bool = ..., timestamp_ns: _Optional[int] = ...) -> None: ...

class UltrasonicState(_message.Message):
    __slots__ = ("front_left", "front_right", "back_left", "back_right", "timestamp_ns")
    FRONT_LEFT_FIELD_NUMBER: _ClassVar[int]
    FRONT_RIGHT_FIELD_NUMBER: _ClassVar[int]
    BACK_LEFT_FIELD_NUMBER: _ClassVar[int]
    BACK_RIGHT_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_NS_FIELD_NUMBER: _ClassVar[int]
    front_left: float
    front_right: float
    back_left: float
    back_right: float
    timestamp_ns: int
    def __init__(self, front_left: _Optional[float] = ..., front_right: _Optional[float] = ..., back_left: _Optional[float] = ..., back_right: _Optional[float] = ..., timestamp_ns: _Optional[int] = ...) -> None: ...

class IMUState(_message.Message):
    __slots__ = ("acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z", "quat_w", "quat_x", "quat_y", "quat_z", "timestamp_ns")
    ACC_X_FIELD_NUMBER: _ClassVar[int]
    ACC_Y_FIELD_NUMBER: _ClassVar[int]
    ACC_Z_FIELD_NUMBER: _ClassVar[int]
    GYRO_X_FIELD_NUMBER: _ClassVar[int]
    GYRO_Y_FIELD_NUMBER: _ClassVar[int]
    GYRO_Z_FIELD_NUMBER: _ClassVar[int]
    QUAT_W_FIELD_NUMBER: _ClassVar[int]
    QUAT_X_FIELD_NUMBER: _ClassVar[int]
    QUAT_Y_FIELD_NUMBER: _ClassVar[int]
    QUAT_Z_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_NS_FIELD_NUMBER: _ClassVar[int]
    acc_x: float
    acc_y: float
    acc_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    quat_w: float
    quat_x: float
    quat_y: float
    quat_z: float
    timestamp_ns: int
    def __init__(self, acc_x: _Optional[float] = ..., acc_y: _Optional[float] = ..., acc_z: _Optional[float] = ..., gyro_x: _Optional[float] = ..., gyro_y: _Optional[float] = ..., gyro_z: _Optional[float] = ..., quat_w: _Optional[float] = ..., quat_x: _Optional[float] = ..., quat_y: _Optional[float] = ..., quat_z: _Optional[float] = ..., timestamp_ns: _Optional[int] = ...) -> None: ...

class HandTouchSensorState(_message.Message):
    __slots__ = ("force", "timestamp_ns")
    FORCE_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_NS_FIELD_NUMBER: _ClassVar[int]
    force: _containers.RepeatedScalarFieldContainer[float]
    timestamp_ns: int
    def __init__(self, force: _Optional[_Iterable[float]] = ..., timestamp_ns: _Optional[int] = ...) -> None: ...
