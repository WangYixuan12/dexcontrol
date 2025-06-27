# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""RGB camera sensor implementation using RTC subscriber with Zenoh query."""

import logging

import numpy as np
import zenoh

from dexcontrol.utils.rtc_utils import create_rtc_subscriber_with_config

logger = logging.getLogger(__name__)


class RGBCameraSensor:
    """RGB camera sensor using RTC subscriber.

    This sensor provides RGB image data from a camera using RTC.
    It first queries Zenoh for RTC connection information, then creates
    a RTC subscriber for efficient data handling.
    """

    def __init__(
        self,
        configs,
        zenoh_session: zenoh.Session,
        *args,
        **kwargs,
    ) -> None:
        """Initialize the RGB camera sensor.

        Args:
            configs: Configuration for the RGB camera sensor.
            zenoh_session: Active Zenoh session for communication.
        """
        self._name = configs.name

        # Create the RTC subscriber using the utility function
        self._subscriber = create_rtc_subscriber_with_config(
            zenoh_session=zenoh_session,
            config=configs.subscriber_config.rgb,
            name=f"{self._name}_subscriber",
            enable_fps_tracking=configs.enable_fps_tracking,
            fps_log_interval=configs.fps_log_interval,
        )

        if self._subscriber is None:
            logger.warning(f"Failed to create RTC subscriber for {self._name}")
            # Continue initialization without subscriber

    def shutdown(self) -> None:
        """Shutdown the camera sensor."""
        if self._subscriber:
            self._subscriber.shutdown()

    def is_active(self) -> bool:
        """Check if the camera sensor is actively receiving data.

        Returns:
            True if receiving data, False otherwise.
        """
        return self._subscriber.is_active() if self._subscriber else False

    def wait_for_active(self, timeout: float = 5.0) -> bool:
        """Wait for the camera sensor to start receiving data.

        Args:
            timeout: Maximum time to wait in seconds.

        Returns:
            True if sensor becomes active, False if timeout is reached.
        """
        return self._subscriber.wait_for_active(timeout) if self._subscriber else False

    def get_obs(self) -> np.ndarray | None:
        """Get the latest RGB image data.

        Returns:
            Latest RGB image as numpy array (HxWxC) if available, None otherwise.
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
