# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""LIDAR Zenoh subscriber for scan data.

This module provides a specialized subscriber for LIDAR scan data,
using the serialization format from dexsensor.
"""

from typing import Any

import numpy as np
import zenoh
from loguru import logger

from .base import BaseZenohSubscriber, CustomDataHandler

# Import lidar serialization functions from dexsensor
try:
    from dexsensor.serialization.lidar import decode_scan_data
except ImportError:
    logger.error(
        "Failed to import dexsensor lidar serialization functions. Please install dexsensor."
    )
    decode_scan_data = None


class LidarSubscriber(BaseZenohSubscriber):
    """Zenoh subscriber for LIDAR scan data.

    This subscriber handles LIDAR scan data encoded using the dexsensor
    lidar serialization format with compression.
    Uses lazy decoding - data is only decoded when requested.
    """

    def __init__(
        self,
        topic: str,
        zenoh_session: zenoh.Session,
        name: str = "lidar_subscriber",
        enable_fps_tracking: bool = True,
        fps_log_interval: int = 50,
        custom_data_handler: CustomDataHandler | None = None,
    ) -> None:
        """Initialize the LIDAR subscriber.

        Args:
            topic: Zenoh topic to subscribe to for LIDAR data.
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
        """Handle incoming LIDAR scan data.

        Args:
            sample: Zenoh sample containing encoded LIDAR scan data.
        """
        with self._data_lock:
            self._latest_raw_data = sample.payload.to_bytes()
            self._active = True

        self._update_fps_metrics()

    def get_latest_data(self) -> dict[str, Any] | None:
        """Get the latest LIDAR scan data.

        Returns:
            Latest scan data dictionary if available, None otherwise.
            Dictionary contains:
                - ranges: Array of range measurements
                - angles: Array of corresponding angles
                - intensities: Array of intensity values (if available)
                - angle_min: Minimum angle of the scan
                - angle_max: Maximum angle of the scan
                - angle_increment: Angular distance between measurements
                - scan_time: Time for a complete scan
                - time_increment: Time between measurements
                - range_min: Minimum range value
                - range_max: Maximum range value
        """
        with self._data_lock:
            if self._latest_raw_data is None:
                return None

            if decode_scan_data is None:
                logger.error(
                    f"Cannot decode LIDAR scan for {self._name}: dexsensor not available"
                )
                return None

            try:
                # Decode the LIDAR scan data
                scan_data = decode_scan_data(self._latest_raw_data)
                # Return a copy to avoid external modifications
                return {
                    key: value.copy() if isinstance(value, np.ndarray) else value
                    for key, value in scan_data.items()
                }
            except Exception as e:
                logger.error(f"Failed to decode LIDAR scan for {self._name}: {e}")
                return None

    def get_latest_scan(self) -> dict[str, Any] | None:
        """Get the latest LIDAR scan data.

        Alias for get_latest_data() for clarity.

        Returns:
            Latest scan data dictionary if available, None otherwise.
        """
        return self.get_latest_data()

    def get_ranges(self) -> np.ndarray | None:
        """Get the latest range measurements.

        Returns:
            Array of range measurements if available, None otherwise.
        """
        scan_data = self.get_latest_data()
        if scan_data is not None:
            return scan_data["ranges"]
        return None

    def get_angles(self) -> np.ndarray | None:
        """Get the latest angle measurements.

        Returns:
            Array of angle measurements if available, None otherwise.
        """
        scan_data = self.get_latest_data()
        if scan_data is not None:
            return scan_data["angles"]
        return None

    def get_intensities(self) -> np.ndarray | None:
        """Get the latest intensity measurements.

        Returns:
            Array of intensity measurements if available, None otherwise.
        """
        scan_data = self.get_latest_data()
        if scan_data is not None and "intensities" in scan_data:
            intensities = scan_data["intensities"]
            return intensities if intensities is not None else None
        return None

    def get_scan_info(self) -> dict[str, float] | None:
        """Get scan metadata information.

        Returns:
            Dictionary with scan metadata if available, None otherwise.
            Contains: angle_min, angle_max, angle_increment, scan_time,
                     time_increment, range_min, range_max
        """
        scan_data = self.get_latest_data()
        if scan_data is not None:
            return {
                "angle_min": scan_data["angle_min"],
                "angle_max": scan_data["angle_max"],
                "angle_increment": scan_data["angle_increment"],
                "scan_time": scan_data["scan_time"],
                "time_increment": scan_data["time_increment"],
                "range_min": scan_data["range_min"],
                "range_max": scan_data["range_max"],
            }
        return None

    def has_intensities(self) -> bool:
        """Check if the latest scan data includes intensity information.

        Returns:
            True if intensity data is available, False otherwise.
        """
        scan_data = self.get_latest_data()
        if scan_data is not None:
            intensities = scan_data.get("intensities")
            return intensities is not None and len(intensities) > 0
        return False
