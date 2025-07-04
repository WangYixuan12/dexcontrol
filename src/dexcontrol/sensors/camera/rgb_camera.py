# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""RGB camera sensor implementation using RTC or Zenoh subscriber."""

import logging
from typing import Optional, Union

import numpy as np
import zenoh

from dexcontrol.config.sensors.cameras import RGBCameraConfig
from dexcontrol.utils.rtc_utils import create_rtc_subscriber_with_config
from dexcontrol.utils.subscribers.camera import RGBCameraSubscriber
from dexcontrol.utils.subscribers.rtc import RTCSubscriber

logger = logging.getLogger(__name__)


class RGBCameraSensor:
    """RGB camera sensor that supports both RTC and standard Zenoh subscribers.

    This sensor provides RGB image data from a camera. It can be configured to use
    either a high-performance RTC subscriber for real-time video streams or a
    standard Zenoh subscriber for raw image topics. The mode is controlled by the
    `use_rtc` flag in the `RGBCameraConfig`.
    """

    def __init__(
        self,
        configs: RGBCameraConfig,
        zenoh_session: zenoh.Session,
    ) -> None:
        """Initialize the RGB camera sensor based on the provided configuration.

        Args:
            configs: Configuration object for the RGB camera sensor.
            zenoh_session: Active Zenoh session for communication.
        """
        self._name = configs.name
        self._subscriber: Optional[Union[RTCSubscriber, RGBCameraSubscriber]] = None

        subscriber_config = configs.subscriber_config.get("rgb", {})
        if not subscriber_config or not subscriber_config.get("enable", False):
            logger.info(f"RGBCameraSensor '{self._name}' is disabled in config.")
            return

        try:
            if configs.use_rtc:
                logger.info(f"'{self._name}': Using RTC subscriber.")
                self._subscriber = create_rtc_subscriber_with_config(
                    zenoh_session=zenoh_session,
                    config=subscriber_config,
                    name=f"{self._name}_subscriber",
                    enable_fps_tracking=configs.enable_fps_tracking,
                    fps_log_interval=configs.fps_log_interval,
                )
            else:
                logger.info(f"'{self._name}': Using standard Zenoh subscriber.")
                topic = subscriber_config.get("topic")
                if topic:
                    self._subscriber = RGBCameraSubscriber(
                        topic=topic,
                        zenoh_session=zenoh_session,
                        name=f"{self._name}_subscriber",
                        enable_fps_tracking=configs.enable_fps_tracking,
                        fps_log_interval=configs.fps_log_interval,
                    )
                else:
                    logger.warning(
                        f"No 'topic' specified for '{self._name}' in non-RTC mode."
                    )

            if self._subscriber is None:
                logger.warning(f"Failed to create subscriber for '{self._name}'.")

        except Exception as e:
            logger.error(f"Error creating subscriber for '{self._name}': {e}")
            self._subscriber = None

    def shutdown(self) -> None:
        """Shutdown the camera sensor and release all resources."""
        if self._subscriber:
            self._subscriber.shutdown()
            logger.info(f"'{self._name}' sensor shut down.")

    def is_active(self) -> bool:
        """Check if the camera sensor is actively receiving data.

        Returns:
            True if the subscriber exists and is receiving data, False otherwise.
        """
        return self._subscriber.is_active() if self._subscriber else False

    def wait_for_active(self, timeout: float = 5.0) -> bool:
        """Wait for the camera sensor to start receiving data.

        Args:
            timeout: Maximum time to wait in seconds.

        Returns:
            True if the sensor becomes active within the timeout, False otherwise.
        """
        if not self._subscriber:
            logger.warning(f"'{self._name}': Cannot wait, no subscriber initialized.")
            return False
        return self._subscriber.wait_for_active(timeout)

    def get_obs(self) -> np.ndarray | None:
        """Get the latest observation (RGB image) from the sensor.

        Returns:
            The latest RGB image as a numpy array (HxWxC) if available, otherwise None.
        """
        return self._subscriber.get_latest_data() if self._subscriber else None

    @property
    def fps(self) -> float:
        """Get the current FPS measurement.

        Returns:
            Current frames per second measurement.
        """
        return self._subscriber.fps if self._subscriber else 0.0

    @property
    def name(self) -> str:
        """Get the sensor name.

        Returns:
            Sensor name string.
        """
        return self._name

    @property
    def height(self) -> int:
        """Get the height of the camera image.

        Returns:
            Height of the camera image.
        """
        image = self.get_obs()
        if image is None:
            return 0
        return image.shape[0]

    @property
    def width(self) -> int:
        """Get the width of the camera image.

        Returns:
            Width of the camera image.
        """
        image = self.get_obs()
        if image is None:
            return 0
        return image.shape[1]

    @property
    def resolution(self) -> tuple[int, int]:
        """Get the resolution of the camera image.

        Returns:
            Resolution of the camera image.
        """
        image = self.get_obs()
        if image is None:
            return 0, 0
        return image.shape[0], image.shape[1]
