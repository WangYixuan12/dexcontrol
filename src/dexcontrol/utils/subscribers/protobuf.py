# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Protobuf-specific Zenoh subscriber.

This module provides a subscriber specifically designed for handling protobuf messages
with automatic parsing and type safety. Uses lazy decoding for efficiency.
"""

from typing import TypeVar, cast

import zenoh
from google.protobuf.message import Message
from loguru import logger

from .base import BaseZenohSubscriber, CustomDataHandler

M = TypeVar("M", bound=Message)


class ProtobufZenohSubscriber(BaseZenohSubscriber):
    """Zenoh subscriber specifically for protobuf messages.

    This subscriber automatically handles protobuf message parsing and provides
    type-safe access to the latest message data. Uses lazy decoding - protobuf
    messages are only parsed when requested.
    """

    def __init__(
        self,
        topic: str,
        zenoh_session: zenoh.Session,
        message_type: type[M],
        name: str = "protobuf_subscriber",
        enable_fps_tracking: bool = False,
        fps_log_interval: int = 100,
        custom_data_handler: CustomDataHandler | None = None,
    ) -> None:
        """Initialize the protobuf Zenoh subscriber.

        Args:
            topic: Zenoh topic to subscribe to for protobuf messages.
            zenoh_session: Active Zenoh session for communication.
            message_type: Protobuf message class to parse incoming data.
            name: Name for logging purposes.
            enable_fps_tracking: Whether to track and log FPS metrics.
            fps_log_interval: Number of frames between FPS calculations.
            custom_data_handler: Optional custom function to handle incoming data.
                                If provided, this will replace the default data
                                handling logic entirely.
        """
        super().__init__(
            topic,
            zenoh_session,
            name,
            enable_fps_tracking,
            fps_log_interval,
            custom_data_handler,
        )
        self._message_type = message_type
        self._latest_raw_data: zenoh.ZBytes = zenoh.ZBytes("")

    def _data_handler(self, sample: zenoh.Sample) -> None:
        """Handle incoming protobuf data.

        Args:
            sample: Zenoh sample containing protobuf data.
        """
        with self._data_lock:
            self._latest_raw_data = sample.payload
            self._active = True

        self._update_fps_metrics()

    def get_latest_data(self) -> M | None:
        """Get the latest protobuf message.

        Returns:
            Latest parsed protobuf message if available and parsing succeeded,
            None otherwise.
        """
        with self._data_lock:
            if not self._active:
                return None

            # Parse protobuf message on demand
            try:
                message = self._message_type()
                message.ParseFromString(self._latest_raw_data.to_bytes())
                return cast(M, message)
            except Exception as e:
                logger.error(f"Failed to parse protobuf message for {self._name}: {e}")
                return None

    def get_latest_raw_data(self) -> bytes | None:
        """Get the latest raw protobuf data bytes.

        Returns:
            Latest raw protobuf data bytes if available, None otherwise.
        """
        with self._data_lock:
            if not self._active:
                return None
            return self._latest_raw_data.to_bytes()
