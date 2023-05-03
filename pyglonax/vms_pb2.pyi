from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Empty(_message.Message):
    __slots__ = []
    def __init__(self) -> None: ...

class Motion(_message.Message):
    __slots__ = ["changes", "type"]
    class MotionType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = []
    class ChangeSet(_message.Message):
        __slots__ = ["actuator", "value"]
        ACTUATOR_FIELD_NUMBER: _ClassVar[int]
        VALUE_FIELD_NUMBER: _ClassVar[int]
        actuator: int
        value: int
        def __init__(self, actuator: _Optional[int] = ..., value: _Optional[int] = ...) -> None: ...
    CHANGE: Motion.MotionType
    CHANGES_FIELD_NUMBER: _ClassVar[int]
    NONE: Motion.MotionType
    RESUME_ALL: Motion.MotionType
    STOP_ALL: Motion.MotionType
    TYPE_FIELD_NUMBER: _ClassVar[int]
    changes: _containers.RepeatedCompositeFieldContainer[Motion.ChangeSet]
    type: Motion.MotionType
    def __init__(self, type: _Optional[_Union[Motion.MotionType, str]] = ..., changes: _Optional[_Iterable[_Union[Motion.ChangeSet, _Mapping]]] = ...) -> None: ...

class Signal(_message.Message):
    __slots__ = ["acceleration", "address", "angle", "function", "percent", "rpm", "speed", "temperature"]
    class vector3(_message.Message):
        __slots__ = ["x", "y", "z"]
        X_FIELD_NUMBER: _ClassVar[int]
        Y_FIELD_NUMBER: _ClassVar[int]
        Z_FIELD_NUMBER: _ClassVar[int]
        x: float
        y: float
        z: float
        def __init__(self, x: _Optional[float] = ..., y: _Optional[float] = ..., z: _Optional[float] = ...) -> None: ...
    ACCELERATION_FIELD_NUMBER: _ClassVar[int]
    ADDRESS_FIELD_NUMBER: _ClassVar[int]
    ANGLE_FIELD_NUMBER: _ClassVar[int]
    FUNCTION_FIELD_NUMBER: _ClassVar[int]
    PERCENT_FIELD_NUMBER: _ClassVar[int]
    RPM_FIELD_NUMBER: _ClassVar[int]
    SPEED_FIELD_NUMBER: _ClassVar[int]
    TEMPERATURE_FIELD_NUMBER: _ClassVar[int]
    acceleration: Signal.vector3
    address: int
    angle: float
    function: int
    percent: int
    rpm: int
    speed: float
    temperature: float
    def __init__(self, address: _Optional[int] = ..., function: _Optional[int] = ..., temperature: _Optional[float] = ..., angle: _Optional[float] = ..., speed: _Optional[float] = ..., rpm: _Optional[int] = ..., acceleration: _Optional[_Union[Signal.vector3, _Mapping]] = ..., percent: _Optional[int] = ...) -> None: ...
