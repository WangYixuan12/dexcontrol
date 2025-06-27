# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Generic Zenoh subscriber with configurable decoder functions.

This module provides a flexible subscriber that can handle any type of data
by accepting decoder functions that transform raw Zenoh bytes into the desired format.
"""

from typing import Any

import zenoh
from loguru import logger

from .base import BaseZenohSubscriber, CustomDataHandler
from .decoders import DecoderFunction


class GenericZenohSubscriber(BaseZenohSubscriber):
    """Generic Zenoh subscriber with configurable decoder function.

    This subscriber can handle any type of data by accepting a decoder function
    that transforms the raw Zenoh bytes into the desired data format.
    Uses lazy decoding - data is only decoded when requested.
    """

    def __init__(
        self,
        topic: str,
        zenoh_session: zenoh.Session,
        decoder: DecoderFunction | None = None,
        name: str = "generic_subscriber",
        enable_fps_tracking: bool = False,
        fps_log_interval: int = 100,
        custom_data_handler: CustomDataHandler | None = None,
    ) -> None:
        """Initialize the generic Zenoh subscriber.

        Args:
            topic: Zenoh topic to subscribe to for data.
            zenoh_session: Active Zenoh session for communication.
            decoder: Optional function to decode raw bytes into desired format.
                     If None, raw bytes are returned.
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
        self._decoder = decoder
        self._latest_raw_data: zenoh.ZBytes = zenoh.ZBytes("")

    def _data_handler(self, sample: zenoh.Sample) -> None:
        """Handle incoming data.

        Args:
            sample: Zenoh sample containing data.
        """
        with self._data_lock:
            self._latest_raw_data = sample.payload
            self._active = True

        self._update_fps_metrics()

    def get_latest_data(self) -> Any | None:
        """Get the latest data.

        Returns:
            Latest decoded data if decoder is provided and decoding succeeded,
            otherwise raw bytes. None if no data received.
        """
        with self._data_lock:
            if not self._active:
                return None

            if self._decoder is not None:
                try:
                    return self._decoder(self._latest_raw_data)
                except Exception as e:
                    logger.error(f"Failed to decode data for {self._name}: {e}")
                    return None
            else:
                return self._latest_raw_data.to_bytes()

    def get_latest_raw_data(self) -> bytes | None:
        """Get the latest raw data bytes.

        Returns:
            Latest raw data bytes if available, None otherwise.
        """
        with self._data_lock:
            if not self._active:
                return None
            return self._latest_raw_data.to_bytes()
