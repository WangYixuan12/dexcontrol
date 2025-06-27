# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Decoder functions for Zenoh subscribers.

This module provides common decoder functions that can be used with the
GenericZenohSubscriber to transform raw Zenoh bytes into specific data formats.
"""

import json
from collections.abc import Callable
from typing import Any, Type, TypeVar

import zenoh
from google.protobuf.message import Message

M = TypeVar("M", bound=Message)

# Type alias for decoder functions
DecoderFunction = Callable[[zenoh.ZBytes], Any]


def protobuf_decoder(message_type: Type[M]) -> DecoderFunction:
    """Create a decoder function for a specific protobuf message type.

    Args:
        message_type: Protobuf message class to decode to.

    Returns:
        Decoder function that parses bytes into the specified protobuf message.
    """

    def decode(data: zenoh.ZBytes) -> M:
        message = message_type()
        message.ParseFromString(data.to_bytes())
        return message

    return decode


def raw_bytes_decoder(data: zenoh.ZBytes) -> bytes:
    """Decoder that returns raw bytes.

    Args:
        data: Zenoh bytes data.

    Returns:
        Raw bytes data.
    """
    return data.to_bytes()


def json_decoder(data: zenoh.ZBytes) -> Any:
    """Decoder that parses JSON from bytes.

    Args:
        data: Zenoh bytes data containing JSON.

    Returns:
        Parsed JSON data.

    Raises:
        json.JSONDecodeError: If the data is not valid JSON.
    """
    return json.loads(data.to_bytes().decode("utf-8"))


def string_decoder(data: zenoh.ZBytes, encoding: str = "utf-8") -> str:
    """Decoder that converts bytes to string.

    Args:
        data: Zenoh bytes data.
        encoding: Text encoding to use for decoding.

    Returns:
        Decoded string.

    Raises:
        UnicodeDecodeError: If the data cannot be decoded with the specified encoding.
    """
    return data.to_bytes().decode(encoding)
