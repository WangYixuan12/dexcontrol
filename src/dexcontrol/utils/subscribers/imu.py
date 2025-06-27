# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""IMU Zenoh subscriber for inertial measurement data.

This module provides a specialized subscriber for IMU data,
using the serialization format from dexsensor.
"""

from typing import Any

import numpy as np
import zenoh
from loguru import logger

from .base import BaseZenohSubscriber, CustomDataHandler

# Import IMU serialization functions from dexsensor
try:
    from dexsensor.serialization.imu import decode_imu_data
except ImportError:
    logger.error(
        "Failed to import dexsensor IMU serialization functions. Please install dexsensor."
    )
    decode_imu_data = None


class IMUSubscriber(BaseZenohSubscriber):
    """Zenoh subscriber for IMU data.

    This subscriber handles IMU data encoded using the dexsensor
    IMU serialization format with compression.
    Uses lazy decoding - data is only decoded when requested.
    """

    def __init__(
        self,
        topic: str,
        zenoh_session: zenoh.Session,
        name: str = "imu_subscriber",
        enable_fps_tracking: bool = True,
        fps_log_interval: int = 50,
        custom_data_handler: CustomDataHandler | None = None,
    ) -> None:
        """Initialize the IMU subscriber.

        Args:
            topic: Zenoh topic to subscribe to for IMU data.
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
        """Handle incoming IMU data.

        Args:
            sample: Zenoh sample containing encoded IMU data.
        """
        with self._data_lock:
            self._latest_raw_data = sample.payload.to_bytes()
            self._active = True

        self._update_fps_metrics()

    def get_latest_data(self) -> dict[str, Any] | None:
        """Get the latest IMU data.

        Returns:
            Latest IMU data dictionary if available, None otherwise.
            Dictionary contains:
                - acceleration: Linear acceleration [x, y, z] in m/s²
                - angular_velocity: Angular velocity [x, y, z] in rad/s
                - orientation: Orientation quaternion [x, y, z, w]
                - magnetometer: Magnetic field [x, y, z] in µT (if available)
                - timestamp: Timestamp of the measurement
        """
        with self._data_lock:
            if self._latest_raw_data is None:
                return None

            if decode_imu_data is None:
                logger.error(
                    f"Cannot decode IMU data for {self._name}: dexsensor not available"
                )
                return None

            try:
                # Decode the IMU data
                imu_data = decode_imu_data(self._latest_raw_data)
                # Return a copy to avoid external modifications
                return {
                    key: value.copy() if isinstance(value, np.ndarray) else value
                    for key, value in imu_data.items()
                }
            except Exception as e:
                logger.error(f"Failed to decode IMU data for {self._name}: {e}")
                return None

    def get_acceleration(self) -> np.ndarray | None:
        """Get the latest linear acceleration.

        Returns:
            Linear acceleration [x, y, z] in m/s² if available, None otherwise.
        """
        imu_data = self.get_latest_data()
        if imu_data is not None:
            return imu_data["acceleration"]
        return None

    def get_angular_velocity(self) -> np.ndarray | None:
        """Get the latest angular velocity.

        Returns:
            Angular velocity [x, y, z] in rad/s if available, None otherwise.
        """
        imu_data = self.get_latest_data()
        if imu_data is not None:
            return imu_data["angular_velocity"]
        return None

    def get_orientation(self) -> np.ndarray | None:
        """Get the latest orientation quaternion.

        Returns:
            Orientation quaternion [x, y, z, w] if available, None otherwise.
        """
        imu_data = self.get_latest_data()
        if imu_data is not None:
            return imu_data["orientation"]
        return None

    def get_magnetometer(self) -> np.ndarray | None:
        """Get the latest magnetometer reading.

        Returns:
            Magnetic field [x, y, z] in µT if available, None otherwise.
        """
        imu_data = self.get_latest_data()
        if imu_data is not None and "magnetometer" in imu_data:
            magnetometer = imu_data["magnetometer"]
            return magnetometer if magnetometer is not None else None
        return None

    def has_magnetometer(self) -> bool:
        """Check if the latest IMU data includes magnetometer information.

        Returns:
            True if magnetometer data is available, False otherwise.
        """
        imu_data = self.get_latest_data()
        if imu_data is not None:
            magnetometer = imu_data.get("magnetometer")
            return magnetometer is not None and len(magnetometer) > 0
        return False
