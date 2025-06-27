# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Zenoh subscriber utilities for dexcontrol.

This module provides a collection of subscriber classes and utilities for handling
Zenoh communication in a flexible and reusable way.
"""

from .base import BaseZenohSubscriber, CustomDataHandler
from .camera import DepthCameraSubscriber, RGBCameraSubscriber, RGBDCameraSubscriber
from .decoders import (
    DecoderFunction,
    json_decoder,
    protobuf_decoder,
    raw_bytes_decoder,
    string_decoder,
)
from .generic import GenericZenohSubscriber
from .imu import IMUSubscriber
from .lidar import LidarSubscriber
from .protobuf import ProtobufZenohSubscriber
from .rtc import RTCSubscriber

__all__ = [
    "BaseZenohSubscriber",
    "CustomDataHandler",
    "GenericZenohSubscriber",
    "ProtobufZenohSubscriber",
    "DecoderFunction",
    "protobuf_decoder",
    "raw_bytes_decoder",
    "json_decoder",
    "string_decoder",
    # Camera subscribers
    "RGBCameraSubscriber",
    "DepthCameraSubscriber",
    "RGBDCameraSubscriber",
    # Lidar subscriber
    "LidarSubscriber",
    # IMU subscriber
    "IMUSubscriber",
    # RTC subscriber
    "RTCSubscriber",
]
