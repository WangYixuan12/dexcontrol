from collections.abc import Iterable as _Iterable
from collections.abc import Mapping as _Mapping
from typing import ClassVar as _ClassVar
from typing import Optional as _Optional
from typing import Union as _Union

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper

DESCRIPTOR: _descriptor.FileDescriptor

class ArmState(_message.Message):
    __slots__ = ("joint_pos", "joint_vel", "joint_cur", "joint_err", "timestamp_ns")
    JOINT_POS_FIELD_NUMBER: _ClassVar[int]
    JOINT_VEL_FIELD_NUMBER: _ClassVar[int]
    JOINT_CUR_FIELD_NUMBER: _ClassVar[int]
    JOINT_ERR_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_NS_FIELD_NUMBER: _ClassVar[int]
    joint_pos: _containers.RepeatedScalarFieldContainer[float]
    joint_vel: _containers.RepeatedScalarFieldContainer[float]
    joint_cur: _containers.RepeatedScalarFieldContainer[float]
    joint_err: _containers.RepeatedScalarFieldContainer[int]
    timestamp_ns: int
    def __init__(self, joint_pos: _Optional[_Iterable[float]] = ..., joint_vel: _Optional[_Iterable[float]] = ..., joint_cur: _Optional[_Iterable[float]] = ..., joint_err: _Optional[_Iterable[int]] = ..., timestamp_ns: _Optional[int] = ...) -> None: ...

class HandState(_message.Message):
    __slots__ = ("joint_pos", "joint_vel", "joint_statu", "timestamp_ns")
    JOINT_POS_FIELD_NUMBER: _ClassVar[int]
    JOINT_VEL_FIELD_NUMBER: _ClassVar[int]
    JOINT_STATU_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_NS_FIELD_NUMBER: _ClassVar[int]
    joint_pos: _containers.RepeatedScalarFieldContainer[float]
    joint_vel: _containers.RepeatedScalarFieldContainer[float]
    joint_statu: _containers.RepeatedScalarFieldContainer[int]
    timestamp_ns: int
    def __init__(self, joint_pos: _Optional[_Iterable[float]] = ..., joint_vel: _Optional[_Iterable[float]] = ..., joint_statu: _Optional[_Iterable[int]] = ..., timestamp_ns: _Optional[int] = ...) -> None: ...

class HeadState(_message.Message):
    __slots__ = ("joint_pos", "joint_vel", "timestamp_ns")
    JOINT_POS_FIELD_NUMBER: _ClassVar[int]
    JOINT_VEL_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_NS_FIELD_NUMBER: _ClassVar[int]
    joint_pos: _containers.RepeatedScalarFieldContainer[float]
    joint_vel: _containers.RepeatedScalarFieldContainer[float]
    timestamp_ns: int
    def __init__(self, joint_pos: _Optional[_Iterable[float]] = ..., joint_vel: _Optional[_Iterable[float]] = ..., timestamp_ns: _Optional[int] = ...) -> None: ...

class TorsoState(_message.Message):
    __slots__ = ("joint_pos", "joint_vel", "timestamp_ns")
    JOINT_POS_FIELD_NUMBER: _ClassVar[int]
    JOINT_VEL_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_NS_FIELD_NUMBER: _ClassVar[int]
    joint_pos: _containers.RepeatedScalarFieldContainer[float]
    joint_vel: _containers.RepeatedScalarFieldContainer[float]
    timestamp_ns: int
    def __init__(self, joint_pos: _Optional[_Iterable[float]] = ..., joint_vel: _Optional[_Iterable[float]] = ..., timestamp_ns: _Optional[int] = ...) -> None: ...

class SingleWheelState(_message.Message):
    __slots__ = ("steering_pos", "wheel_pos", "wheel_vel", "wheel_cur")
    STEERING_POS_FIELD_NUMBER: _ClassVar[int]
    WHEEL_POS_FIELD_NUMBER: _ClassVar[int]
    WHEEL_VEL_FIELD_NUMBER: _ClassVar[int]
    WHEEL_CUR_FIELD_NUMBER: _ClassVar[int]
    steering_pos: float
    wheel_pos: float
    wheel_vel: float
    wheel_cur: float
    def __init__(self, steering_pos: _Optional[float] = ..., wheel_pos: _Optional[float] = ..., wheel_vel: _Optional[float] = ..., wheel_cur: _Optional[float] = ...) -> None: ...

class ChassisState(_message.Message):
    __slots__ = ("left", "right", "timestamp_ns")
    LEFT_FIELD_NUMBER: _ClassVar[int]
    RIGHT_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_NS_FIELD_NUMBER: _ClassVar[int]
    left: SingleWheelState
    right: SingleWheelState
    timestamp_ns: int
    def __init__(self, left: _Optional[_Union[SingleWheelState, _Mapping]] = ..., right: _Optional[_Union[SingleWheelState, _Mapping]] = ..., timestamp_ns: _Optional[int] = ...) -> None: ...

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
    __slots__ = ("button_pressed", "software_estop_enabled", "timestamp_ns")
    BUTTON_PRESSED_FIELD_NUMBER: _ClassVar[int]
    SOFTWARE_ESTOP_ENABLED_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_NS_FIELD_NUMBER: _ClassVar[int]
    button_pressed: bool
    software_estop_enabled: bool
    timestamp_ns: int
    def __init__(self, button_pressed: bool = ..., software_estop_enabled: bool = ..., timestamp_ns: _Optional[int] = ...) -> None: ...

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

class ArmCommand(_message.Message):
    __slots__ = ("command_type", "joint_pos", "joint_vel")
    class CommandType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        POSITION: _ClassVar[ArmCommand.CommandType]
        VELOCITY_FEEDFORWARD: _ClassVar[ArmCommand.CommandType]
    POSITION: ArmCommand.CommandType
    VELOCITY_FEEDFORWARD: ArmCommand.CommandType
    COMMAND_TYPE_FIELD_NUMBER: _ClassVar[int]
    JOINT_POS_FIELD_NUMBER: _ClassVar[int]
    JOINT_VEL_FIELD_NUMBER: _ClassVar[int]
    command_type: ArmCommand.CommandType
    joint_pos: _containers.RepeatedScalarFieldContainer[float]
    joint_vel: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, command_type: _Optional[_Union[ArmCommand.CommandType, str]] = ..., joint_pos: _Optional[_Iterable[float]] = ..., joint_vel: _Optional[_Iterable[float]] = ...) -> None: ...

class HandCommand(_message.Message):
    __slots__ = ("joint_pos",)
    JOINT_POS_FIELD_NUMBER: _ClassVar[int]
    joint_pos: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, joint_pos: _Optional[_Iterable[float]] = ...) -> None: ...

class HeadCommand(_message.Message):
    __slots__ = ("joint_pos", "joint_vel")
    JOINT_POS_FIELD_NUMBER: _ClassVar[int]
    JOINT_VEL_FIELD_NUMBER: _ClassVar[int]
    joint_pos: _containers.RepeatedScalarFieldContainer[float]
    joint_vel: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, joint_pos: _Optional[_Iterable[float]] = ..., joint_vel: _Optional[_Iterable[float]] = ...) -> None: ...

class TorsoCommand(_message.Message):
    __slots__ = ("joint_pos", "joint_vel")
    JOINT_POS_FIELD_NUMBER: _ClassVar[int]
    JOINT_VEL_FIELD_NUMBER: _ClassVar[int]
    joint_pos: _containers.RepeatedScalarFieldContainer[float]
    joint_vel: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, joint_pos: _Optional[_Iterable[float]] = ..., joint_vel: _Optional[_Iterable[float]] = ...) -> None: ...

class SingleWheelCommand(_message.Message):
    __slots__ = ("steering_pos", "wheel_vel")
    STEERING_POS_FIELD_NUMBER: _ClassVar[int]
    WHEEL_VEL_FIELD_NUMBER: _ClassVar[int]
    steering_pos: float
    wheel_vel: float
    def __init__(self, steering_pos: _Optional[float] = ..., wheel_vel: _Optional[float] = ...) -> None: ...

class ChassisCommand(_message.Message):
    __slots__ = ("left", "right")
    LEFT_FIELD_NUMBER: _ClassVar[int]
    RIGHT_FIELD_NUMBER: _ClassVar[int]
    left: SingleWheelCommand
    right: SingleWheelCommand
    def __init__(self, left: _Optional[_Union[SingleWheelCommand, _Mapping]] = ..., right: _Optional[_Union[SingleWheelCommand, _Mapping]] = ...) -> None: ...

class EndEffectorPassThroughCommand(_message.Message):
    __slots__ = ("data",)
    DATA_FIELD_NUMBER: _ClassVar[int]
    data: bytes
    def __init__(self, data: _Optional[bytes] = ...) -> None: ...
