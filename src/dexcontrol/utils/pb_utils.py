# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Utility functions for Python data structures and protobuf messages."""

from enum import Enum
from typing import Any, Literal

from dexcontrol.proto import dexcontrol_query_pb2

TYPE_SOFTWARE_VERSION = dict[
    Literal["hardware_version", "software_version", "main_hash", "compile_time"], Any
]


def software_version_to_dict(
    version_msg: dexcontrol_query_pb2.SoftwareVersion,
) -> dict[str, TYPE_SOFTWARE_VERSION]:
    """Convert a SoftwareVersion protobuf message to a dictionary.

    Args:
        version_msg: SoftwareVersion protobuf message.

    Returns:
        Dictionary containing version information with component names as keys.
    """
    return {
        key: {
            "hardware_version": value.hardware_version,
            "software_version": value.software_version,
            "main_hash": value.main_hash,
            "compile_time": value.compile_time,
        }
        for key, value in version_msg.firmware_version.items()
    }


class ComponentStatus(Enum):
    """Enum representing the status of a component."""

    NORMAL = 0
    NA = 1
    ERROR = 2


def status_to_enum(status: dexcontrol_query_pb2.ComponentStatus) -> ComponentStatus:
    """Convert a ComponentStatus protobuf message to a ComponentStatus enum.

    Args:
        status: ComponentStatus protobuf message.

    Returns:
        ComponentStatus enum value.

    Raises:
        ValueError: If the status value is not recognized.
    """
    status_map = {
        dexcontrol_query_pb2.ComponentStatus.NORMAL: ComponentStatus.NORMAL,
        dexcontrol_query_pb2.ComponentStatus.NA: ComponentStatus.NA,
        dexcontrol_query_pb2.ComponentStatus.ERROR: ComponentStatus.ERROR,
    }

    if status not in status_map:
        raise ValueError(f"Invalid status: {status}")

    return status_map[status]


def status_to_dict(
    status_msg: dexcontrol_query_pb2.ComponentStates,
) -> dict[str, dict[str, Any]]:
    """Convert a ComponentStates protobuf message to a dictionary.

    Args:
        status_msg: ComponentStates protobuf message.

    Returns:
        Dictionary containing status information for each component.
    """
    return {
        name: {
            "connected": state.connected,
            "enabled": status_to_enum(state.enabled),
            "error_state": status_to_enum(state.error_state),
            "error_code": state.error_code,
        }
        for name, state in status_msg.states.items()
    }


def convert_enum_to_str(d: dict) -> dict:
    """Convert enum values to their string representations in a dictionary.

    Args:
        d: Dictionary containing enum values.

    Returns:
        Dictionary with enum values converted to their string representations.
    """
    for key, value in d.items():
        if isinstance(value, dict):
            d[key] = convert_enum_to_str(value)
        elif isinstance(value, Enum):
            d[key] = value.name
    return d
