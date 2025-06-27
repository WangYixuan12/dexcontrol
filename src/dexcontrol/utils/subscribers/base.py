# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Base Zenoh subscriber utilities.

This module provides the abstract base class for all Zenoh subscribers
and common utilities used across different subscriber implementations.
"""

import threading
import time
from abc import ABC, abstractmethod
from collections.abc import Callable
from typing import Any, Final, TypeVar

import zenoh
from google.protobuf.message import Message
from loguru import logger

from dexcontrol.utils.os_utils import resolve_key_name

# Type variable for Message subclasses
M = TypeVar("M", bound=Message)

# Type alias for custom data handler functions
CustomDataHandler = Callable[[zenoh.Sample], None]


class BaseZenohSubscriber(ABC):
    """Base class for Zenoh subscribers with configurable data handling.

    This class provides a common interface for subscribing to Zenoh topics
    and processing incoming data through configurable decoder functions.
    It handles the Zenoh communication setup and provides thread-safe access
    to the latest data.

    Attributes:
        _active: Whether subscriber is receiving data updates.
        _data_lock: Lock for thread-safe data access.
        _zenoh_session: Active Zenoh session for communication.
        _subscriber: Zenoh subscriber for data.
        _topic: The resolved Zenoh topic name.
        _enable_fps_tracking: Whether to track and log FPS metrics.
        _frame_count: Counter for frames processed since last FPS calculation.
        _fps: Most recently calculated frames per second value.
        _last_fps_time: Timestamp of last FPS calculation.
        _fps_log_interval: Number of frames between FPS calculations.
        _name: Name for logging purposes.
        _custom_data_handler: Optional custom data handler function.
        _last_data_time: Timestamp of last data received.
    """

    def __init__(
        self,
        topic: str,
        zenoh_session: zenoh.Session,
        name: str = "subscriber",
        enable_fps_tracking: bool = False,
        fps_log_interval: int = 100,
        custom_data_handler: CustomDataHandler | None = None,
    ) -> None:
        """Initialize the base Zenoh subscriber.

        Args:
            topic: Zenoh topic to subscribe to for data.
            zenoh_session: Active Zenoh session for communication.
            name: Name for logging purposes.
            enable_fps_tracking: Whether to track and log FPS metrics.
            fps_log_interval: Number of frames between FPS calculations.
            custom_data_handler: Optional custom function to handle incoming data.
                                If provided, this will replace the default data
                                handling logic entirely.
        """
        self._active: bool = False
        self._data_lock: Final[threading.RLock] = threading.RLock()
        self._zenoh_session: Final[zenoh.Session] = zenoh_session
        self._name = name
        self._custom_data_handler = custom_data_handler

        # Data freshness tracking
        self._last_data_time: float | None = None

        # FPS tracking
        self._enable_fps_tracking = enable_fps_tracking
        self._fps_log_interval = fps_log_interval
        self._frame_count = 0
        self._fps = 0.0
        self._last_fps_time = time.time()

        # Setup Zenoh subscriber
        self._topic: Final[str] = resolve_key_name(topic)
        self._subscriber: Final[zenoh.Subscriber] = zenoh_session.declare_subscriber(
            self._topic, self._data_handler_wrapper
        )

        logger.info(f"Initialized {self._name} subscriber for topic: {self._topic}")

    def _data_handler_wrapper(self, sample: zenoh.Sample) -> None:
        """Wrapper for data handling that calls either custom or default handler.

        Args:
            sample: Zenoh sample containing data.
        """
        # Update data freshness timestamp
        with self._data_lock:
            self._last_data_time = time.monotonic()

        # Call custom data handler if provided, otherwise call default handler
        if self._custom_data_handler is not None:
            try:
                self._custom_data_handler(sample)
            except Exception as e:
                logger.error(f"Custom data handler failed for {self._name}: {e}")
        else:
            # Call the default data handler
            self._data_handler(sample)

    @abstractmethod
    def _data_handler(self, sample: zenoh.Sample) -> None:
        """Handle incoming data.

        This method must be implemented by subclasses to process the specific
        type of data they handle.

        Args:
            sample: Zenoh sample containing data.
        """
        pass

    @abstractmethod
    def get_latest_data(self) -> Any | None:
        """Get the latest data.

        Returns:
            Latest data if available, None otherwise.
        """
        pass

    def _update_fps_metrics(self) -> None:
        """Update FPS tracking metrics.

        Increments frame counter and recalculates FPS at specified intervals.
        Only has an effect if fps_tracking was enabled during initialization.
        """
        if not self._enable_fps_tracking:
            return

        self._frame_count += 1
        if self._frame_count >= self._fps_log_interval:
            current_time = time.time()
            elapsed = current_time - self._last_fps_time
            self._fps = self._frame_count / elapsed
            logger.info(f"{self._name} {self._topic} frequency: {self._fps:.2f} Hz")
            self._frame_count = 0
            self._last_fps_time = current_time

    def wait_for_active(self, timeout: float = 5.0) -> bool:
        """Wait for the subscriber to start receiving data.

        Args:
            timeout: Maximum time to wait in seconds.

        Returns:
            True if subscriber becomes active, False if timeout is reached.
        """
        start_time = time.monotonic()
        check_interval = min(0.05, timeout / 10)  # Check every 10ms or less

        while True:
            with self._data_lock:
                if self._active:
                    return True

            elapsed = time.monotonic() - start_time
            if elapsed >= timeout:
                logger.error(f"No data received from {self._topic} after {timeout}s")
                return False

            # Sleep for the shorter of: remaining time or check interval
            sleep_time = min(check_interval, timeout - elapsed)
            time.sleep(sleep_time)

    def is_active(self) -> bool:
        """Check if the subscriber is actively receiving data.

        Returns:
            True if subscriber is active, False otherwise.
        """
        with self._data_lock:
            return self._active

    def shutdown(self) -> None:
        """Stop the subscriber and release resources."""
        # Mark as inactive first to prevent further data processing
        with self._data_lock:
            self._active = False

        # Small delay to allow any ongoing data processing to complete
        time.sleep(0.05)

        try:
            if hasattr(self, "_subscriber") and self._subscriber:
                self._subscriber.undeclare()
        except Exception as e:
            # Don't log "Undeclared subscriber" errors as warnings - they're expected during shutdown
            error_msg = str(e).lower()
            if not ("undeclared" in error_msg or "closed" in error_msg):
                logger.warning(f"Error undeclaring subscriber for {self._topic}: {e}")

        # Additional delay to allow Zenoh to process the undeclare operation
        time.sleep(0.02)

    @property
    def topic(self) -> str:
        """Get the Zenoh topic name.

        Returns:
            The resolved Zenoh topic name.
        """
        return self._topic

    @property
    def fps(self) -> float:
        """Get the current FPS measurement.

        Returns:
            Current frames per second measurement.
        """
        return self._fps

    @property
    def name(self) -> str:
        """Get the subscriber name.

        Returns:
            The subscriber name.
        """
        return self._name

    def is_data_fresh(self, max_age_seconds: float) -> bool:
        """Check if the most recent data is fresh (received within the specified time).

        This method checks if data has been received within the specified time window,
        regardless of whether the data content has changed. This is useful for
        detecting communication failures or stale data streams.

        Args:
            max_age_seconds: Maximum age in seconds for data to be considered fresh.

        Returns:
            True if fresh data was received within the time window, False otherwise.
            Returns False if no data has ever been received.
        """
        with self._data_lock:
            if self._last_data_time is None:
                return False

            current_time = time.monotonic()
            age = current_time - self._last_data_time
            return age <= max_age_seconds

    def get_time_since_last_data(self) -> float | None:
        """Get the time elapsed since the last data was received.

        Returns:
            Time in seconds since last data was received, or None if no data
            has ever been received.
        """
        with self._data_lock:
            if self._last_data_time is None:
                return None

            current_time = time.monotonic()
            return current_time - self._last_data_time
