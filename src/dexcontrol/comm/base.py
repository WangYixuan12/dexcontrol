# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Base classes for DexControl communication using DexComm.

Provides abstract base classes and common functionality for all
communication components in DexControl.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Dict, Generic, List, Optional, TypeVar

from dexcomm import (
    BufferedSubscriber,
    Publisher,
    Subscriber,
    SubscriberManager,
    ZenohConfig,
)
from loguru import logger

from dexcontrol.utils.os_utils import resolve_key_name

T = TypeVar("T")


class DataFormat(Enum):
    """Supported data formats for communication."""

    RAW = "raw"
    JSON = "json"
    NUMPY = "numpy"
    IMAGE_RGB = "image_rgb"
    IMAGE_DEPTH = "image_depth"
    IMU = "imu"
    LIDAR_2D = "lidar_2d"
    PROTOBUF = "protobuf"
    CUSTOM = "custom"


@dataclass
class SubscriberConfig:
    """Configuration for a subscriber."""

    topic: str
    format: DataFormat = DataFormat.RAW
    buffer_size: int = 1
    enable_buffering: bool = False
    callback: Optional[Callable[[Any], None]] = None
    namespace: Optional[str] = None
    config_path: Optional[Path] = None
    qos: Dict[str, Any] = field(default_factory=dict)


class SensorSubscriber(ABC, Generic[T]):
    """Abstract base class for sensor subscribers.

    Provides a clean interface for subscribing to sensor data with
    automatic deserialization and type safety.
    """

    def __init__(self, config: SubscriberConfig):
        """Initialize the sensor subscriber.

        Args:
            config: Subscriber configuration
        """
        self.config = config
        self._subscriber: Optional[Subscriber] = None
        self._setup_subscriber()

    def _setup_subscriber(self) -> None:
        """Setup the DexComm subscriber."""
        topic = self._resolve_topic()

        # Choose subscriber type based on configuration
        subscriber_class = (
            BufferedSubscriber if self.config.enable_buffering else Subscriber
        )

        self._subscriber = subscriber_class(
            topic=topic,
            callback=self._wrapped_callback,
            deserializer=self._get_deserializer(),
            config=self._get_zenoh_config(),
            buffer_size=self.config.buffer_size,
            qos=self.config.qos,
        )

        logger.info(f"Created {self.__class__.__name__} for topic: {topic}")

    def _resolve_topic(self) -> str:
        """Resolve the full topic name including namespace."""
        topic = self.config.topic
        if self.config.namespace:
            topic = f"{self.config.namespace}/{topic}"
        return resolve_key_name(topic)

    def _wrapped_callback(self, data: Any) -> None:
        """Wrapper for the user callback with error handling."""
        try:
            # Process data through subclass implementation
            processed = self.process_data(data)

            # Call user callback if provided
            if self.config.callback:
                self.config.callback(processed)
        except Exception as e:
            logger.error(f"Error in {self.__class__.__name__} callback: {e}")

    def _get_zenoh_config(self) -> Optional[ZenohConfig]:
        """Get Zenoh configuration."""
        if self.config.config_path:
            return ZenohConfig.from_file(self.config.config_path)
        return None

    @abstractmethod
    def _get_deserializer(self) -> Optional[Callable[[bytes], Any]]:
        """Get the appropriate deserializer for this sensor type.

        Returns:
            Deserializer function or None for raw bytes
        """
        pass

    @abstractmethod
    def process_data(self, data: Any) -> T:
        """Process raw data into the appropriate type.

        Args:
            data: Deserialized data from subscriber

        Returns:
            Processed data of type T
        """
        pass

    def get_latest(self) -> Optional[T]:
        """Get the latest data from the sensor.

        Returns:
            Latest processed data or None if no data available
        """
        if not self._subscriber:
            return None

        raw_data = self._subscriber.get_latest()
        if raw_data is not None:
            return self.process_data(raw_data)
        return None

    def wait_for_data(self, timeout: float = 5.0) -> Optional[T]:
        """Wait for data from the sensor.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            Received data or None if timeout
        """
        if not self._subscriber:
            return None

        raw_data = self._subscriber.wait_for_message(timeout)
        if raw_data is not None:
            return self.process_data(raw_data)
        return None

    def is_active(self) -> bool:
        """Check if the sensor is actively receiving data.

        Returns:
            True if receiving data, False otherwise
        """
        return self.get_latest() is not None

    def get_stats(self) -> Dict[str, Any]:
        """Get subscriber statistics.

        Returns:
            Dictionary with subscriber statistics
        """
        if not self._subscriber:
            return {}
        return self._subscriber.get_stats()

    def shutdown(self) -> None:
        """Shutdown the subscriber and release resources."""
        if self._subscriber:
            self._subscriber.shutdown()
            logger.debug(
                f"{self.__class__.__name__} shutdown for topic: {self.config.topic}"
            )


class StreamSubscriber(SensorSubscriber[T]):
    """Base class for high-frequency streaming data subscribers.

    Adds rate monitoring and buffering capabilities for streams.
    """

    def __init__(self, config: SubscriberConfig):
        """Initialize the stream subscriber."""
        super().__init__(config)
        self._rate_monitor = RateMonitor(window_size=100)

    def _wrapped_callback(self, data: Any) -> None:
        """Extended callback with rate monitoring."""
        self._rate_monitor.update()
        super()._wrapped_callback(data)

    def get_rate(self) -> float:
        """Get the current data rate in Hz.

        Returns:
            Current rate in Hz
        """
        return self._rate_monitor.get_rate()

    def get_buffer(self) -> List[T]:
        """Get buffered data (only if buffering is enabled).

        Returns:
            List of buffered data
        """
        if not isinstance(self._subscriber, BufferedSubscriber):
            return []

        raw_buffer = self._subscriber.get_buffer()
        return [self.process_data(data) for data in raw_buffer]

    def clear_buffer(self) -> None:
        """Clear the data buffer."""
        if isinstance(self._subscriber, BufferedSubscriber):
            self._subscriber.clear_buffer()


class TopicManager:
    """Manages multiple topics dynamically using DexComm's manager pattern.

    Useful for applications that need to subscribe/unsubscribe to topics
    at runtime, such as data loggers or monitoring systems.
    """

    def __init__(self, namespace: Optional[str] = None):
        """Initialize the topic manager.

        Args:
            namespace: Optional namespace for all topics
        """
        self.namespace = namespace
        self._subscribers = SubscriberManager()
        self._publishers = {}
        self._callbacks: Dict[str, List[Callable]] = {}

    def add_subscriber(
        self,
        topic: str,
        callback: Optional[Callable[[Any], None]] = None,
        format: DataFormat = DataFormat.RAW,
        buffer_size: int = 1,
    ) -> None:
        """Add a new subscriber dynamically.

        Args:
            topic: Topic to subscribe to
            callback: Optional callback function
            format: Data format for deserialization
            buffer_size: Number of messages to buffer
        """
        full_topic = self._resolve_topic(topic)

        # Store callback
        if callback:
            if topic not in self._callbacks:
                self._callbacks[topic] = []
            self._callbacks[topic].append(callback)

        # Create wrapper callback
        def wrapper(msg):
            for cb in self._callbacks.get(topic, []):
                try:
                    cb(msg)
                except Exception as e:
                    logger.error(f"Error in callback for {topic}: {e}")

        # Add to manager
        self._subscribers.add(
            full_topic,
            callback=wrapper if callback else None,
            buffer_size=buffer_size,
        )

        logger.info(f"Added subscriber for topic: {full_topic}")

    def remove_subscriber(self, topic: str) -> None:
        """Remove a subscriber.

        Args:
            topic: Topic to unsubscribe from
        """
        full_topic = self._resolve_topic(topic)
        self._subscribers.remove(full_topic)

        # Clear callbacks
        if topic in self._callbacks:
            del self._callbacks[topic]

        logger.info(f"Removed subscriber for topic: {full_topic}")

    def add_publisher(
        self, topic: str, format: DataFormat = DataFormat.RAW
    ) -> Publisher:
        """Add a new publisher dynamically.

        Args:
            topic: Topic to publish to
            format: Data format for serialization

        Returns:
            Publisher instance
        """
        full_topic = self._resolve_topic(topic)

        if topic not in self._publishers:
            self._publishers[topic] = Publisher(full_topic)
            logger.info(f"Added publisher for topic: {full_topic}")

        return self._publishers[topic]

    def publish(self, topic: str, data: Any) -> None:
        """Publish data to a topic.

        Args:
            topic: Topic to publish to
            data: Data to publish
        """
        if topic not in self._publishers:
            self.add_publisher(topic)

        self._publishers[topic].publish(data)

    def get_latest(self, topic: str) -> Optional[Any]:
        """Get latest message from a topic.

        Args:
            topic: Topic to get data from

        Returns:
            Latest message or None
        """
        full_topic = self._resolve_topic(topic)
        return self._subscribers.get_latest(full_topic)

    def get_all_latest(self) -> Dict[str, Any]:
        """Get latest messages from all subscribed topics.

        Returns:
            Dictionary mapping topics to their latest messages
        """
        return self._subscribers.get_all_latest()

    def _resolve_topic(self, topic: str) -> str:
        """Resolve topic with namespace."""
        if self.namespace:
            topic = f"{self.namespace}/{topic}"
        return resolve_key_name(topic)

    def shutdown(self) -> None:
        """Shutdown all subscribers and publishers."""
        self._subscribers.shutdown_all()
        for pub in self._publishers.values():
            pub.shutdown()
        logger.info("TopicManager shutdown complete")


class RateMonitor:
    """Monitor data rate for streaming subscribers."""

    def __init__(self, window_size: int = 100):
        """Initialize rate monitor.

        Args:
            window_size: Number of samples for rate calculation
        """
        self.window_size = window_size
        self.timestamps: List[float] = []

    def update(self) -> None:
        """Update with new data point."""
        import time

        self.timestamps.append(time.time())

        # Keep only window_size samples
        if len(self.timestamps) > self.window_size:
            self.timestamps.pop(0)

    def get_rate(self) -> float:
        """Calculate current rate in Hz.

        Returns:
            Rate in Hz or 0 if insufficient data
        """
        if len(self.timestamps) < 2:
            return 0.0

        time_span = self.timestamps[-1] - self.timestamps[0]
        if time_span > 0:
            return (len(self.timestamps) - 1) / time_span
        return 0.0
