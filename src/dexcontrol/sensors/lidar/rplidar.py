# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""LIDAR sensor implementations using Zenoh subscribers.

This module provides LIDAR sensor classes that use the specialized LIDAR
subscriber for scan data.
"""

from typing import Any

import numpy as np
import zenoh

from dexcontrol.utils.subscribers.lidar import LidarSubscriber


class RPLidarSensor:
    """LIDAR sensor using Zenoh subscriber.

    This sensor provides LIDAR scan data using the LidarSubscriber
    for efficient data handling with lazy decoding.
    """

    def __init__(
        self,
        configs,
        zenoh_session: zenoh.Session,
    ) -> None:
        """Initialize the LIDAR sensor.

        Args:
            topic: Zenoh topic to subscribe to for LIDAR data.
            zenoh_session: Active Zenoh session for communication.
            name: Name for the sensor instance.
            enable_fps_tracking: Whether to track and log FPS metrics.
            fps_log_interval: Number of frames between FPS calculations.
        """
        self._name = configs.name

        # Create the LIDAR subscriber
        self._subscriber = LidarSubscriber(
            topic=configs.topic,
            zenoh_session=zenoh_session,
            name=f"{self._name}_subscriber",
            enable_fps_tracking=configs.enable_fps_tracking,
            fps_log_interval=configs.fps_log_interval,
        )


    def shutdown(self) -> None:
        """Shutdown the LIDAR sensor."""
        self._subscriber.shutdown()

    def is_active(self) -> bool:
        """Check if the LIDAR sensor is actively receiving data.

        Returns:
            True if receiving data, False otherwise.
        """
        return self._subscriber.is_active()

    def wait_for_active(self, timeout: float = 5.0) -> bool:
        """Wait for the LIDAR sensor to start receiving data.

        Args:
            timeout: Maximum time to wait in seconds.

        Returns:
            True if sensor becomes active, False if timeout is reached.
        """
        return self._subscriber.wait_for_active(timeout)

    def get_obs(self) -> dict[str, Any] | None:
        """Get the latest LIDAR scan data.

        Returns:
            Latest scan data dictionary if available, None otherwise.
            Dictionary contains:
                - ranges: Array of range measurements in meters
                - angles: Array of corresponding angles in radians
                - qualities: Array of quality values (0-255) if available, None otherwise
        """
        return self._subscriber.get_latest_data()

    def get_ranges(self) -> np.ndarray | None:
        """Get the latest range measurements.

        Returns:
            Array of range measurements in meters if available, None otherwise.
        """
        return self._subscriber.get_ranges()

    def get_angles(self) -> np.ndarray | None:
        """Get the latest angle measurements.

        Returns:
            Array of angle measurements in radians if available, None otherwise.
        """
        return self._subscriber.get_angles()

    def get_qualities(self) -> np.ndarray | None:
        """Get the latest quality measurements.

        Returns:
            Array of quality values (0-255) if available, None otherwise.
        """
        return self._subscriber.get_qualities()

    def get_point_count(self) -> int:
        """Get the number of points in the latest scan.

        Returns:
            Number of points in the scan, 0 if no data available.
        """
        ranges = self.get_ranges()
        if ranges is not None:
            return len(ranges)
        return 0

    def has_qualities(self) -> bool:
        """Check if the latest scan data includes quality information.

        Returns:
            True if quality data is available, False otherwise.
        """
        return self._subscriber.has_qualities()

    @property
    def fps(self) -> float:
        """Get the current FPS measurement.

        Returns:
            Current frames per second measurement.
        """
        return self._subscriber.fps

    @property
    def name(self) -> str:
        """Get the LIDAR name.

        Returns:
            LIDAR name string.
        """
        return self._name
