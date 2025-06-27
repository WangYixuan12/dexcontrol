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

import numpy as np
import zenoh

from dexcontrol.utils.os_utils import resolve_key_name
from dexcontrol.utils.rtc_utils import create_rtc_subscriber_from_zenoh
from dexcontrol.utils.subscribers.camera import DepthCameraSubscriber
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
    """ZED camera sensor using RTC subscribers for RGB and Zenoh subscriber for depth.

    This sensor provides left RGB, right RGB, and depth image data from a ZED camera.
    RGB streams use RTC subscribers for efficient data handling, while depth uses
    regular Zenoh subscriber.

    Note: For depth data decoding, dexsensor package is required.
    """

    def __init__(
        self,
        configs,
        zenoh_session: zenoh.Session,
        *args,
        **kwargs,
    ) -> None:
        """Initialize the ZED camera sensor.

        Args:
            configs: Configuration for the ZED camera sensor.
            zenoh_session: Active Zenoh session for communication.
        """
        self._name = configs.name
        self._zenoh_session = zenoh_session
        self._configs = configs

        # Initialize subscribers dictionary - RGB uses RTC, depth uses Zenoh
        self._subscribers: dict[str, RTCSubscriber | DepthCameraSubscriber | None] = {}

        # Create subscribers for each enabled stream
        self._create_subscribers()

    def _create_subscribers(self) -> None:
        """Create subscribers for each enabled stream - RTC for RGB, Zenoh for depth."""
        subscriber_config = self._configs.subscriber_config

        # Define stream types and their configurations
        streams = {
            'left_rgb': subscriber_config.get('left_rgb', {}),
            'right_rgb': subscriber_config.get('right_rgb', {}),
            'depth': subscriber_config.get('depth', {})
        }

        for stream_name, stream_config in streams.items():
            if stream_config.get('enable', False):
                try:
                    if stream_name == 'depth':
                        # Use regular Zenoh subscriber for depth
                        topic = stream_config.get('topic')
                        if topic:
                            subscriber = DepthCameraSubscriber(
                                topic=topic,
                                zenoh_session=self._zenoh_session,
                                name=f"{self._name}_{stream_name}_subscriber",
                                enable_fps_tracking=self._configs.enable_fps_tracking,
                                fps_log_interval=self._configs.fps_log_interval,
                            )
                            logger.info(f"Created Zenoh depth subscriber for {self._name} {stream_name}")
                            self._subscribers[stream_name] = subscriber
                        else:
                            logger.warning(f"No topic found for {self._name} {stream_name}")
                            self._subscribers[stream_name] = None
                    else:
                        # Use RTC subscriber for RGB streams
                        info_key = stream_config.get('info_key')
                        if info_key:
                            subscriber = create_rtc_subscriber_from_zenoh(
                                zenoh_session=self._zenoh_session,
                                info_topic=info_key,
                                name=f"{self._name}_{stream_name}_subscriber",
                                enable_fps_tracking=self._configs.enable_fps_tracking,
                                fps_log_interval=self._configs.fps_log_interval,
                            )

                            if subscriber is None:
                                logger.warning(f"Failed to create RTC subscriber for {self._name} {stream_name}")
                            else:
                                logger.info(f"Created RTC subscriber for {self._name} {stream_name}")

                            self._subscribers[stream_name] = subscriber
                        else:
                            logger.warning(f"No info_key found for {self._name} {stream_name}")
                            self._subscribers[stream_name] = None
                except Exception as e:
                    logger.error(f"Error creating subscriber for {self._name} {stream_name}: {e}")
                    self._subscribers[stream_name] = None
            else:
                logger.info(f"Stream {stream_name} disabled for {self._name}")
                self._subscribers[stream_name] = None

        # Query for camera info - use info_key from one of the RGB streams
        enabled_rgb_configs = [config for config in [subscriber_config.get('left_rgb'), subscriber_config.get('right_rgb')] if config and config.get('enable')]
        if enabled_rgb_configs:
            info_key = resolve_key_name(enabled_rgb_configs[0].get('info_key')).rstrip('/')
            info_key_root = '/'.join(info_key.split('/')[:-2])
            info_key = f"{info_key_root}/info"
            info = query_zenoh_json(self._zenoh_session, info_key)
            self._camera_info = info
            if info is not None:
                self._depth_min = info.get('depth_min')
                self._depth_max = info.get('depth_max')
            else:
                logger.warning(f"No camera info found for {self._name}")
                self._depth_min = None
                self._depth_max = None
        else:
            logger.warning(f"No enabled RGB streams found for camera info query for {self._name}")
            self._camera_info = None
            self._depth_min = None
            self._depth_max = None

    def _decode_depth_data(self, encoded_depth_data: bytes | None) -> np.ndarray | None:
        """Decode depth data from encoded bytes to actual depth values.

        Args:
            encoded_depth_data: Raw depth data as bytes.

        Returns:
            Decoded depth data as numpy array (HxW).

        Raises:
            RuntimeError: If dexsensor is not available for depth decoding.
        """
        if encoded_depth_data is None:
            return None

        if not DEXSENSOR_AVAILABLE or decode_depth is None:
            raise RuntimeError(
                f"dexsensor is required for depth decoding in {self._name}. "
                "Please install dexsensor: pip install dexsensor"
            )

        try:
            # Decode the depth data from bytes - this returns (depth, depth_min, depth_max)
            depth_decoded = decode_depth(encoded_depth_data)
            return depth_decoded
        except Exception as e:
            raise RuntimeError(f"Failed to decode depth data for {self._name}: {e}")

    def shutdown(self) -> None:
        """Shutdown the camera sensor."""
        for stream_name, subscriber in self._subscribers.items():
            if subscriber:
                try:
                    subscriber.shutdown()
                    logger.info(f"Shut down {stream_name} subscriber for {self._name}")
                except Exception as e:
                    logger.error(f"Error shutting down {stream_name} subscriber for {self._name}: {e}")

    def is_active(self) -> bool:
        """Check if any camera stream is actively receiving data.

        Returns:
            True if at least one stream is receiving data, False otherwise.
        """
        for subscriber in self._subscribers.values():
            if subscriber and subscriber.is_active():
                return True
        return False

    def is_stream_active(self, stream_name: str) -> bool:
        """Check if a specific stream is actively receiving data.

        Args:
            stream_name: Name of the stream ('left_rgb', 'right_rgb', 'depth').

        Returns:
            True if the stream is receiving data, False otherwise.
        """
        subscriber = self._subscribers.get(stream_name)
        return subscriber.is_active() if subscriber else False

    def wait_for_active(self, timeout: float = 5.0, require_all: bool = False) -> bool:
        """Wait for camera streams to start receiving data.

        Args:
            timeout: Maximum time to wait in seconds.
            require_all: If True, wait for all enabled streams. If False, wait for any stream.

        Returns:
            True if condition is met, False if timeout is reached.
        """
        active_subscribers = [sub for sub in self._subscribers.values() if sub is not None]

        if not active_subscribers:
            logger.warning(f"No active subscribers for {self._name}")
            return False

        if require_all:
            # Wait for all subscribers to become active
            for subscriber in active_subscribers:
                if not subscriber.wait_for_active(timeout):
                    return False
            return True
        else:
            # Wait for any subscriber to become active
            import time
            start_time = time.time()
            while time.time() - start_time < timeout:
                if self.is_active():
                    return True
                time.sleep(0.1)
            return False

    def get_obs(self, obs_keys: list[str] | None = None) -> dict[str, np.ndarray]:
        """Get the latest image data from specified streams.

        Args:
            obs_keys: List of stream names to get data from.
                     If None, gets data from all enabled streams.

        Returns:
            Dictionary mapping stream names to image arrays (HxWxC for RGB, HxW for depth)
            if available, None otherwise.

        Raises:
            RuntimeError: If depth data is requested but dexsensor is not available.
        """
        if obs_keys is None:
            obs_keys = list(self._subscribers.keys())

        obs_out = {}
        for key in obs_keys:
            if key in self._subscribers:
                subscriber = self._subscribers[key]
                if subscriber:
                    raw_data = subscriber.get_latest_data()
                    obs_out[key] = raw_data
                else:
                    obs_out[key] = None
            else:
                logger.warning(f"Unknown stream key: {key} for {self._name}")

        return obs_out

    def get_left_rgb(self) -> np.ndarray | None:
        """Get the latest left RGB image.

        Returns:
            Latest left RGB image as numpy array (HxWxC) if available, None otherwise.
        """
        subscriber = self._subscribers.get('left_rgb')
        return subscriber.get_latest_data() if subscriber else None

    def get_right_rgb(self) -> np.ndarray | None:
        """Get the latest right RGB image.

        Returns:
            Latest right RGB image as numpy array (HxWxC) if available, None otherwise.
        """
        subscriber = self._subscribers.get('right_rgb')
        return subscriber.get_latest_data() if subscriber else None

    def get_depth(self) -> np.ndarray | None:
        """Get the latest depth image.

        Returns:
            Latest depth image as numpy array (HxW) if available, None otherwise.

        Raises:
            RuntimeError: If dexsensor is not available for depth decoding.
        """
        subscriber = self._subscribers.get('depth')
        if not subscriber:
            return None

        # DepthCameraSubscriber already handles decoding
        return subscriber.get_latest_data()

    @property
    def fps(self) -> dict[str, float]:
        """Get the current FPS measurement for each stream.

        Returns:
            Dictionary mapping stream names to their FPS measurements.
        """
        fps_dict = {}
        for stream_name, subscriber in self._subscribers.items():
            if subscriber:
                fps_dict[stream_name] = subscriber.fps
            else:
                fps_dict[stream_name] = 0.0
        return fps_dict

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
