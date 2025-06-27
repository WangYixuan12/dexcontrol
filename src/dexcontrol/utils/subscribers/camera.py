# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Camera Zenoh subscribers for RGB and depth data.

This module provides specialized subscribers for camera data including RGB images
and depth images, using the serialization formats from dexsensor.
"""

import numpy as np
import zenoh
from loguru import logger

from .base import BaseZenohSubscriber, CustomDataHandler

# Import camera serialization functions from dexsensor
try:
    from dexsensor.serialization.camera import decode_depth, decode_image
except ImportError:
    logger.error(
        "Failed to import dexsensor camera serialization functions. Please install dexsensor."
    )
    decode_image = None
    decode_depth = None


class RGBCameraSubscriber(BaseZenohSubscriber):
    """Zenoh subscriber for RGB camera data.

    This subscriber handles RGB image data encoded using the dexsensor
    camera serialization format with JPEG compression.
    Uses lazy decoding - data is only decoded when requested.
    """

    def __init__(
        self,
        topic: str,
        zenoh_session: zenoh.Session,
        name: str = "rgb_camera_subscriber",
        enable_fps_tracking: bool = True,
        fps_log_interval: int = 30,
        custom_data_handler: CustomDataHandler | None = None,
    ) -> None:
        """Initialize the RGB camera subscriber.

        Args:
            topic: Zenoh topic to subscribe to for RGB data.
            zenoh_session: Active Zenoh session for communication.
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
        self._latest_raw_data: bytes | None = None

    def _data_handler(self, sample: zenoh.Sample) -> None:
        """Handle incoming RGB image data.

        Args:
            sample: Zenoh sample containing encoded RGB image data.
        """
        with self._data_lock:
            self._latest_raw_data = sample.payload.to_bytes()
            self._active = True

        self._update_fps_metrics()

    def get_latest_data(self) -> np.ndarray | None:
        """Get the latest RGB image.

        Returns:
            Latest RGB image as numpy array (HxWxC) if available, None otherwise.
        """
        with self._data_lock:
            if self._latest_raw_data is None:
                return None

            if decode_image is None:
                logger.error(
                    f"Cannot decode RGB image for {self._name}: dexsensor not available"
                )
                return None

            try:
                # Decode the RGB image
                image = decode_image(self._latest_raw_data)
                return image
            except Exception as e:
                logger.error(f"Failed to decode RGB image for {self._name}: {e}")
                return None

    def get_latest_image(self) -> np.ndarray | None:
        """Get the latest RGB image.

        Alias for get_latest_data() for clarity.

        Returns:
            Latest RGB image as numpy array (HxWxC) if available, None otherwise.
        """
        return self.get_latest_data()


class DepthCameraSubscriber(BaseZenohSubscriber):
    """Zenoh subscriber for depth camera data.

    This subscriber handles depth image data encoded using the dexsensor
    camera serialization format with compression.
    Uses lazy decoding - data is only decoded when requested.
    """

    def __init__(
        self,
        topic: str,
        zenoh_session: zenoh.Session,
        name: str = "depth_camera_subscriber",
        enable_fps_tracking: bool = True,
        fps_log_interval: int = 30,
        custom_data_handler: CustomDataHandler | None = None,
    ) -> None:
        """Initialize the depth camera subscriber.

        Args:
            topic: Zenoh topic to subscribe to for depth data.
            zenoh_session: Active Zenoh session for communication.
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
        self._latest_raw_data: bytes | None = None

    def _data_handler(self, sample: zenoh.Sample) -> None:
        """Handle incoming depth image data.

        Args:
            sample: Zenoh sample containing encoded depth image data.
        """
        with self._data_lock:
            self._latest_raw_data = sample.payload.to_bytes()
            self._active = True

        self._update_fps_metrics()

    def get_latest_data(self) -> np.ndarray | None:
        """Get the latest depth data.

        Returns:
            depth_image if available, None otherwise. Depth values in meters as numpy array (HxW)
        """
        with self._data_lock:
            if self._latest_raw_data is None:
                return None

            if decode_depth is None:
                logger.error(
                    f"Cannot decode depth image for {self._name}: dexsensor not available"
                )
                return None

            try:
                # Decode the depth image
                depth = decode_depth(self._latest_raw_data)
                return depth
            except Exception as e:
                logger.error(f"Failed to decode depth image for {self._name}: {e}")
                return None


class RGBDCameraSubscriber(BaseZenohSubscriber):
    """Zenoh subscriber for RGBD camera data.

    This subscriber handles both RGB and depth data from an RGBD camera,
    subscribing to separate topics for RGB and depth streams.
    """

    def __init__(
        self,
        rgb_topic: str,
        depth_topic: str,
        zenoh_session: zenoh.Session,
        name: str = "rgbd_camera_subscriber",
        enable_fps_tracking: bool = True,
        fps_log_interval: int = 30,
        custom_data_handler: CustomDataHandler | None = None,
    ) -> None:
        """Initialize the RGBD camera subscriber.

        Args:
            rgb_topic: Zenoh topic for RGB data.
            depth_topic: Zenoh topic for depth data.
            zenoh_session: Active Zenoh session for communication.
            name: Name for logging purposes.
            enable_fps_tracking: Whether to track and log FPS metrics.
            fps_log_interval: Number of frames between FPS calculations.
            custom_data_handler: Optional custom function to handle incoming data.
                                If provided, this will replace the default data
                                handling logic entirely.
        """
        # Initialize with RGB topic as primary
        super().__init__(
            rgb_topic,
            zenoh_session,
            name,
            enable_fps_tracking,
            fps_log_interval,
            custom_data_handler,
        )

        # Create separate subscribers for RGB and depth
        self._rgb_subscriber = RGBCameraSubscriber(
            rgb_topic,
            zenoh_session,
            f"{name}_rgb",
            enable_fps_tracking,
            fps_log_interval,
            custom_data_handler,
        )
        self._depth_subscriber = DepthCameraSubscriber(
            depth_topic,
            zenoh_session,
            f"{name}_depth",
            enable_fps_tracking,
            fps_log_interval,
            custom_data_handler,
        )

    def _data_handler(self, sample: zenoh.Sample) -> None:
        """Handle incoming data - not used as we use separate subscribers."""
        pass

    def get_latest_data(self) -> tuple[np.ndarray, np.ndarray] | None:
        """Get the latest RGBD data.

        Returns:
            Tuple of (rgb_image, depth_image, depth_min, depth_max) if both available, None otherwise.
            rgb_image: RGB image as numpy array (HxWxC)
            depth_image: Depth values in meters as numpy array (HxW)
        """
        rgb_image = self._rgb_subscriber.get_latest_data()
        depth_image = self._depth_subscriber.get_latest_data()

        if rgb_image is not None and depth_image is not None:
            return rgb_image, depth_image
        return None

    def get_latest_rgb(self) -> np.ndarray | None:
        """Get the latest RGB image.

        Returns:
            Latest RGB image as numpy array (HxWxC) if available, None otherwise.
        """
        return self._rgb_subscriber.get_latest_image()

    def get_latest_depth(self) -> np.ndarray | None:
        """Get the latest depth image.

        Returns:
            Latest depth image as numpy array (HxW) with values in meters if available, None otherwise.
        """
        return self._depth_subscriber.get_latest_data()

    def wait_for_active(self, timeout: float = 5.0) -> bool:
        """Wait for both RGB and depth subscribers to start receiving data.

        Args:
            timeout: Maximum time to wait in seconds.

        Returns:
            True if both subscribers become active, False if timeout is reached.
        """
        rgb_active = self._rgb_subscriber.wait_for_active(timeout)
        depth_active = self._depth_subscriber.wait_for_active(timeout)
        return rgb_active and depth_active

    def is_active(self) -> bool:
        """Check if both RGB and depth subscribers are actively receiving data.

        Returns:
            True if both subscribers are active, False otherwise.
        """
        return self._rgb_subscriber.is_active() and self._depth_subscriber.is_active()

    def shutdown(self) -> None:
        """Stop both subscribers and release resources."""
        self._rgb_subscriber.shutdown()
        self._depth_subscriber.shutdown()
        super().shutdown()

    @property
    def rgb_fps(self) -> float:
        """Get the RGB stream FPS measurement.

        Returns:
            Current RGB frames per second measurement.
        """
        return self._rgb_subscriber.fps

    @property
    def depth_fps(self) -> float:
        """Get the depth stream FPS measurement.

        Returns:
            Current depth frames per second measurement.
        """
        return self._depth_subscriber.fps
