from collections.abc import Mapping as _Mapping
from typing import ClassVar as _ClassVar
from typing import Optional as _Optional
from typing import Union as _Union

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper

DESCRIPTOR: _descriptor.FileDescriptor

class ComponentStatus(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    NORMAL: _ClassVar[ComponentStatus]
    NA: _ClassVar[ComponentStatus]
    ERROR: _ClassVar[ComponentStatus]
NORMAL: ComponentStatus
NA: ComponentStatus
ERROR: ComponentStatus

class SetArmMode(_message.Message):
    __slots__ = ("mode",)
    class Mode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        POSITION: _ClassVar[SetArmMode.Mode]
        DISABLE: _ClassVar[SetArmMode.Mode]
    POSITION: SetArmMode.Mode
    DISABLE: SetArmMode.Mode
    MODE_FIELD_NUMBER: _ClassVar[int]
    mode: SetArmMode.Mode
    def __init__(self, mode: _Optional[_Union[SetArmMode.Mode, str]] = ...) -> None: ...

class SetHeadMode(_message.Message):
    __slots__ = ("mode",)
    class Mode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        ENABLE: _ClassVar[SetHeadMode.Mode]
        DISABLE: _ClassVar[SetHeadMode.Mode]
    ENABLE: SetHeadMode.Mode
    DISABLE: SetHeadMode.Mode
    MODE_FIELD_NUMBER: _ClassVar[int]
    mode: SetHeadMode.Mode
    def __init__(self, mode: _Optional[_Union[SetHeadMode.Mode, str]] = ...) -> None: ...

class SetEstop(_message.Message):
    __slots__ = ("enable",)
    ENABLE_FIELD_NUMBER: _ClassVar[int]
    enable: bool
    def __init__(self, enable: bool = ...) -> None: ...

class SetLed(_message.Message):
    __slots__ = ("enable",)
    ENABLE_FIELD_NUMBER: _ClassVar[int]
    enable: bool
    def __init__(self, enable: bool = ...) -> None: ...

class ClearError(_message.Message):
    __slots__ = ("component",)
    class Component(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        LEFT_ARM: _ClassVar[ClearError.Component]
        RIGHT_ARM: _ClassVar[ClearError.Component]
        HEAD: _ClassVar[ClearError.Component]
        CHASSIS: _ClassVar[ClearError.Component]
    LEFT_ARM: ClearError.Component
    RIGHT_ARM: ClearError.Component
    HEAD: ClearError.Component
    CHASSIS: ClearError.Component
    COMPONENT_FIELD_NUMBER: _ClassVar[int]
    component: ClearError.Component
    def __init__(self, component: _Optional[_Union[ClearError.Component, str]] = ...) -> None: ...

class RebootComponent(_message.Message):
    __slots__ = ("component",)
    class Component(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        ARM: _ClassVar[RebootComponent.Component]
        TORSO: _ClassVar[RebootComponent.Component]
        CHASSIS: _ClassVar[RebootComponent.Component]
    ARM: RebootComponent.Component
    TORSO: RebootComponent.Component
    CHASSIS: RebootComponent.Component
    COMPONENT_FIELD_NUMBER: _ClassVar[int]
    component: RebootComponent.Component
    def __init__(self, component: _Optional[_Union[RebootComponent.Component, str]] = ...) -> None: ...

class FirmwareVersion(_message.Message):
    __slots__ = ("hardware_version", "software_version", "compile_time", "main_hash", "sub_hash")
    HARDWARE_VERSION_FIELD_NUMBER: _ClassVar[int]
    SOFTWARE_VERSION_FIELD_NUMBER: _ClassVar[int]
    COMPILE_TIME_FIELD_NUMBER: _ClassVar[int]
    MAIN_HASH_FIELD_NUMBER: _ClassVar[int]
    SUB_HASH_FIELD_NUMBER: _ClassVar[int]
    hardware_version: int
    software_version: int
    compile_time: int
    main_hash: str
    sub_hash: str
    def __init__(self, hardware_version: _Optional[int] = ..., software_version: _Optional[int] = ..., compile_time: _Optional[int] = ..., main_hash: _Optional[str] = ..., sub_hash: _Optional[str] = ...) -> None: ...

class SoftwareVersion(_message.Message):
    __slots__ = ("firmware_version",)
    class FirmwareVersionEntry(_message.Message):
        __slots__ = ("key", "value")
        KEY_FIELD_NUMBER: _ClassVar[int]
        VALUE_FIELD_NUMBER: _ClassVar[int]
        key: str
        value: FirmwareVersion
        def __init__(self, key: _Optional[str] = ..., value: _Optional[_Union[FirmwareVersion, _Mapping]] = ...) -> None: ...
    FIRMWARE_VERSION_FIELD_NUMBER: _ClassVar[int]
    firmware_version: _containers.MessageMap[str, FirmwareVersion]
    def __init__(self, firmware_version: _Optional[_Mapping[str, FirmwareVersion]] = ...) -> None: ...

class SingleComponentState(_message.Message):
    __slots__ = ("connected", "enabled", "error_state", "error_code")
    CONNECTED_FIELD_NUMBER: _ClassVar[int]
    ENABLED_FIELD_NUMBER: _ClassVar[int]
    ERROR_STATE_FIELD_NUMBER: _ClassVar[int]
    ERROR_CODE_FIELD_NUMBER: _ClassVar[int]
    connected: bool
    enabled: ComponentStatus
    error_state: ComponentStatus
    error_code: str
    def __init__(self, connected: bool = ..., enabled: _Optional[_Union[ComponentStatus, str]] = ..., error_state: _Optional[_Union[ComponentStatus, str]] = ..., error_code: _Optional[str] = ...) -> None: ...

class ComponentStates(_message.Message):
    __slots__ = ("states",)
    class StatesEntry(_message.Message):
        __slots__ = ("key", "value")
        KEY_FIELD_NUMBER: _ClassVar[int]
        VALUE_FIELD_NUMBER: _ClassVar[int]
        key: str
        value: SingleComponentState
        def __init__(self, key: _Optional[str] = ..., value: _Optional[_Union[SingleComponentState, _Mapping]] = ...) -> None: ...
    STATES_FIELD_NUMBER: _ClassVar[int]
    states: _containers.MessageMap[str, SingleComponentState]
    def __init__(self, states: _Optional[_Mapping[str, SingleComponentState]] = ...) -> None: ...
