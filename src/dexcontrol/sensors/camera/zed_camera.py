# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""ZED camera sensor implementation using RTC subscribers for RGB and Zenoh subscriber for depth."""

import logging
import time
from typing import Any, Dict, Optional, Union

import numpy as np
import zenoh

from dexcontrol.config.sensors.cameras import ZedCameraConfig
from dexcontrol.utils.os_utils import resolve_key_name
from dexcontrol.utils.rtc_utils import create_rtc_subscriber_from_zenoh
from dexcontrol.utils.subscribers.camera import (
    DepthCameraSubscriber,
    RGBCameraSubscriber,
)
from dexcontrol.utils.subscribers.rtc import RTCSubscriber
from dexcontrol.utils.zenoh_utils import query_zenoh_json

logger = logging.getLogger(__name__)

# Optional import for depth processing
try:
    from dexsensor.serialization.camera import decode_depth
    DEXSENSOR_AVAILABLE = True
except ImportError:
    logger.warning("dexsensor not available. Depth data will be returned without decoding.")
    decode_depth = None
    DEXSENSOR_AVAILABLE = False


class ZedCameraSensor:
    """ZED camera sensor for multi-stream (RGB, Depth) data acquisition.

    This sensor manages left RGB, right RGB, and depth data streams from a ZED
    camera. It can be configured to use high-performance RTC subscribers for RGB
    streams (`use_rtc=True`) or fall back to standard Zenoh subscribers
    (`use_rtc=False`). The depth stream always uses a standard Zenoh subscriber.
    """

    SubscriberType = Union[RTCSubscriber, DepthCameraSubscriber, RGBCameraSubscriber]

    def __init__(
        self,
        configs: ZedCameraConfig,
        zenoh_session: zenoh.Session,
    ) -> None:
        """Initialize the ZED camera sensor and its subscribers.

        Args:
            configs: Configuration object for the ZED camera.
            zenoh_session: Active Zenoh session for communication.
        """
        self._name = configs.name
        self._zenoh_session = zenoh_session
        self._configs = configs
        self._subscribers: Dict[str, Optional[ZedCameraSensor.SubscriberType]] = {}
        self._camera_info: Optional[Dict[str, Any]] = None

        self._create_subscribers()
        self._query_camera_info()

    def _create_subscriber(
        self, stream_name: str, stream_config: Dict[str, Any]
    ) -> Optional[SubscriberType]:
        """Factory method to create a subscriber based on stream type and config."""
        try:
            if not stream_config.get("enable", False):
                logger.info(f"'{self._name}': Stream '{stream_name}' is disabled.")
                return None

            # Create Depth subscriber
            if stream_name == "depth":
                topic = stream_config.get("topic")
                if not topic:
                    logger.warning(f"'{self._name}': No 'topic' for depth stream.")
                    return None
                logger.info(f"'{self._name}': Creating Zenoh depth subscriber.")
                return DepthCameraSubscriber(
                    topic=topic,
                    zenoh_session=self._zenoh_session,
                    name=f"{self._name}_{stream_name}_subscriber",
                    enable_fps_tracking=self._configs.enable_fps_tracking,
                    fps_log_interval=self._configs.fps_log_interval,
                )

            # Create RGB subscriber (RTC or Zenoh)
            if self._configs.use_rtc:
                info_key = stream_config.get("info_key")
                if not info_key:
                    logger.warning(f"'{self._name}': No 'info_key' for RTC stream '{stream_name}'.")
                    return None
                logger.info(f"'{self._name}': Creating RTC subscriber for '{stream_name}'.")
                return create_rtc_subscriber_from_zenoh(
                    zenoh_session=self._zenoh_session,
                    info_topic=info_key,
                    name=f"{self._name}_{stream_name}_subscriber",
                    enable_fps_tracking=self._configs.enable_fps_tracking,
                    fps_log_interval=self._configs.fps_log_interval,
                )
            else:
                topic = stream_config.get("topic")
                if not topic:
                    logger.warning(f"'{self._name}': No 'topic' for Zenoh stream '{stream_name}'.")
                    return None
                logger.info(f"'{self._name}': Creating Zenoh RGB subscriber for '{stream_name}'.")
                return RGBCameraSubscriber(
                    topic=topic,
                    zenoh_session=self._zenoh_session,
                    name=f"{self._name}_{stream_name}_subscriber",
                    enable_fps_tracking=self._configs.enable_fps_tracking,
                    fps_log_interval=self._configs.fps_log_interval,
                )

        except Exception as e:
            logger.error(f"Error creating subscriber for '{self._name}/{stream_name}': {e}")
            return None

    def _create_subscribers(self) -> None:
        """Create subscribers for all configured camera streams."""
        subscriber_config = self._configs.subscriber_config
        stream_definitions = {
            "left_rgb": subscriber_config.get("left_rgb", {}),
            "right_rgb": subscriber_config.get("right_rgb", {}),
            "depth": subscriber_config.get("depth", {}),
        }

        for name, config in stream_definitions.items():
            self._subscribers[name] = self._create_subscriber(name, config)

    def _query_camera_info(self) -> None:
        """Query Zenoh for camera metadata if using RTC."""

        enabled_rgb_streams = [
            s
            for s_name, s in self._subscribers.items()
            if "rgb" in s_name and s is not None
        ]

        if not enabled_rgb_streams:
            logger.warning(f"'{self._name}': No enabled RGB streams to query for camera info.")
            return

        # Use the info_key from the first available RGB subscriber's config
        first_stream_name = "left_rgb" if self._subscribers.get("left_rgb") else "right_rgb"
        stream_config = self._configs.subscriber_config.get(first_stream_name, {})
        info_key = stream_config.get("info_key")

        if not info_key:
            logger.warning(f"'{self._name}': Could not find info_key for camera info query.")
            return

        try:
            # Construct the root info key (e.g., 'camera/head/info')
            resolved_key = resolve_key_name(info_key).rstrip("/")
            info_key_root = "/".join(resolved_key.split("/")[:-2])
            final_info_key = f"{info_key_root}/info"

            logger.info(f"'{self._name}': Querying for camera info at '{final_info_key}'.")
            self._camera_info = query_zenoh_json(self._zenoh_session, final_info_key)
            if self._camera_info:
                logger.info(f"'{self._name}': Successfully received camera info.")
            else:
                logger.warning(f"'{self._name}': No camera info found at '{final_info_key}'.")
        except Exception as e:
            logger.error(f"'{self._name}': Failed to query camera info: {e}")

    def shutdown(self) -> None:
        """Shutdown all active subscribers for the camera sensor."""
        logger.info(f"Shutting down all subscribers for '{self._name}'.")
        for stream_name, subscriber in self._subscribers.items():
            if subscriber:
                try:
                    subscriber.shutdown()
                    logger.debug(f"'{self._name}': Subscriber '{stream_name}' shut down.")
                except Exception as e:
                    logger.error(
                        f"Error shutting down '{stream_name}' subscriber for '{self._name}': {e}"
                    )
        logger.info(f"'{self._name}' sensor shut down.")

    def is_active(self) -> bool:
        """Check if any of the camera's subscribers are actively receiving data.

        Returns:
            True if at least one subscriber is active, False otherwise.
        """
        return any(
            sub.is_active() for sub in self._subscribers.values() if sub is not None
        )

    def is_stream_active(self, stream_name: str) -> bool:
        """Check if a specific camera stream is actively receiving data.

        Args:
            stream_name: The name of the stream (e.g., 'left_rgb', 'depth').

        Returns:
            True if the specified stream's subscriber is active, False otherwise.
        """
        subscriber = self._subscribers.get(stream_name)
        return subscriber.is_active() if subscriber else False

    def wait_for_active(self, timeout: float = 5.0, require_all: bool = False) -> bool:
        """Wait for camera streams to become active.

        Args:
            timeout: Maximum time to wait in seconds for each subscriber.
            require_all: If True, waits for all enabled streams to become active.
                         If False, waits for at least one stream to become active.

        Returns:
            True if the condition is met within the timeout, False otherwise.
        """
        enabled_subscribers = [s for s in self._subscribers.values() if s is not None]
        if not enabled_subscribers:
            logger.warning(f"'{self._name}': No subscribers enabled, cannot wait.")
            return True  # No subscribers to wait for

        if require_all:
            for sub in enabled_subscribers:
                if not sub.wait_for_active(timeout):
                    logger.warning(f"'{self._name}': Timed out waiting for subscriber '{sub.name}'.")
                    return False
            logger.info(f"'{self._name}': All enabled streams are active.")
            return True
        else:
            start_time = time.time()
            while time.time() - start_time < timeout:
                if self.is_active():
                    logger.info(f"'{self._name}': At least one stream is active.")
                    return True
                time.sleep(0.1)
            logger.warning(f"'{self._name}': Timed out waiting for any stream to become active.")
            return False

    def get_obs(
        self, obs_keys: Optional[list[str]] = None,
        include_timestamp: bool = False
    ) -> Dict[str, Optional[np.ndarray]]:
        """Get the latest observation data from specified camera streams.

        Args:
            obs_keys: A list of stream names to retrieve data from (e.g.,
                      ['left_rgb', 'depth']). If None, retrieves data from all
                      enabled streams.
            include_timestamp: If True, includes the timestamp in the observation data.
                                The timestamp data is not available for RTC streams.

        Returns:
            A dictionary mapping stream names to their latest image data with timestamp. The
            image is a numpy array (HxWxC for RGB, HxW for depth) or None if
            no data is available for that stream. If include_timestamp is True,
            the value in the dictionary is a tuple with the image and timestamp.
        """
        keys_to_fetch = obs_keys or self.available_streams
        obs_out = {}
        for key in keys_to_fetch:
            subscriber = self._subscribers.get(key)
            data = subscriber.get_latest_data() if subscriber else None

            is_tuple_or_list = isinstance(data, (tuple, list))

            if include_timestamp:
                if not is_tuple_or_list:
                    logger.warning(f"Timestamp is not available yet for {key} stream.")
                obs_out[key] = data
            else:
                obs_out[key] = data[0] if is_tuple_or_list else data
        return obs_out

    def get_left_rgb(self) -> Optional[np.ndarray]:
        """Get the latest image from the left RGB stream.

        Returns:
            The latest left RGB image as a numpy array, or None if not available.
        """
        subscriber = self._subscribers.get("left_rgb")
        return subscriber.get_latest_data() if subscriber else None

    def get_right_rgb(self) -> Optional[np.ndarray]:
        """Get the latest image from the right RGB stream.

        Returns:
            The latest right RGB image as a numpy array, or None if not available.
        """
        subscriber = self._subscribers.get("right_rgb")
        return subscriber.get_latest_data() if subscriber else None

    def get_depth(self) -> Optional[np.ndarray]:
        """Get the latest image from the depth stream.

        The depth data is returned as a numpy array with values in meters.

        Returns:
            The latest depth image as a numpy array, or None if not available.
        """
        subscriber = self._subscribers.get("depth")
        return subscriber.get_latest_data() if subscriber else None

    @property
    def fps(self) -> Dict[str, float]:
        """Get the current FPS measurement for each active stream.

        Returns:
            A dictionary mapping stream names to their FPS measurements.
        """
        return {
            name: sub.fps
            for name, sub in self._subscribers.items()
            if sub is not None
        }

    @property
    def name(self) -> str:
        """Get the sensor name.

        Returns:
            Sensor name string.
        """
        return self._name

    @property
    def available_streams(self) -> list:
        """Get list of available stream names.

        Returns:
            List of stream names that have active subscribers.
        """
        return [name for name, sub in self._subscribers.items() if sub is not None]

    @property
    def active_streams(self) -> list:
        """Get list of currently active stream names.

        Returns:
            List of stream names that are currently receiving data.
        """
        return [name for name, sub in self._subscribers.items() if sub and sub.is_active()]

    @property
    def dexsensor_available(self) -> bool:
        """Check if dexsensor is available for depth decoding.

        Returns:
            True if dexsensor is available, False otherwise.
        """
        return DEXSENSOR_AVAILABLE

    @property
    def camera_info(self) -> dict | None:
        """Get the camera info.

        Returns:
            Camera info dictionary if available, None otherwise.
        """
        return self._camera_info

    @property
    def height(self) -> dict[str, int]:
        """Get the height of the camera image.

        Returns:
            Height of the camera image.
        """
        images = self.get_obs()
        return {name: image.shape[0] if image is not None else 0 for name, image in images.items()}

    @property
    def width(self) -> dict[str, int]:
        """Get the width of the camera image.

        Returns:
            Width of the camera image.
        """
        images = self.get_obs()
        return {name: image.shape[1] if image is not None else 0 for name, image in images.items()}


    @property
    def resolution(self) -> dict[str, tuple[int, int]]:
        """Get the resolution of the camera image.

        Returns:
            Resolution of the camera image.
        """
        images = self.get_obs()
        return {name: (image.shape[0], image.shape[1]) if image is not None else (0, 0) for name, image in images.items()}
