# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""ZED IMU sensor implementation using Zenoh subscriber."""

import numpy as np
import zenoh

from dexcontrol.utils.subscribers.imu import IMUSubscriber


class ZedIMUSensor:
    """ZED IMU sensor using Zenoh subscriber.

    This sensor provides IMU data from ZED cameras including acceleration, angular velocity,
    orientation quaternion, and magnetometer data using the IMUSubscriber for efficient data handling.
    The ZED IMU typically provides 9-axis data (accelerometer, gyroscope, magnetometer) with
    quaternion orientation.
    """

    def __init__(
        self,
        configs,
        zenoh_session: zenoh.Session,
    ) -> None:
        """Initialize the ZED IMU sensor.

        Args:
            configs: Configuration object containing topic, name, and other settings.
            zenoh_session: Active Zenoh session for communication.
        """
        self._name = configs.name

        # Create the IMU subscriber
        self._subscriber = IMUSubscriber(
            topic=configs.topic,
            zenoh_session=zenoh_session,
            name=f"{self._name}_subscriber",
            enable_fps_tracking=configs.enable_fps_tracking,
            fps_log_interval=configs.fps_log_interval,
        )

    def shutdown(self) -> None:
        """Shutdown the ZED IMU sensor."""
        self._subscriber.shutdown()

    def is_active(self) -> bool:
        """Check if the ZED IMU sensor is actively receiving data.

        Returns:
            True if receiving data, False otherwise.
        """
        return self._subscriber.is_active()

    def wait_for_active(self, timeout: float = 5.0) -> bool:
        """Wait for the ZED IMU sensor to start receiving data.

        Args:
            timeout: Maximum time to wait in seconds.

        Returns:
            True if sensor becomes active, False if timeout is reached.
        """
        return self._subscriber.wait_for_active(timeout)

    def get_obs(self, obs_keys: list[str] | None = None) -> dict[str, np.ndarray] | None:
        """Get observation data for the ZED IMU sensor.

        Args:
            obs_keys: List of observation keys to retrieve. If None, returns all available data.
                     Valid keys: ['ang_vel', 'acc', 'quat', 'timestamp']

        Returns:
            Dictionary with observation data including all IMU measurements.
            Keys are mapped as follows:
            - 'ang_vel': Angular velocity from 'angular_velocity'
            - 'acc': Linear acceleration from 'acceleration'
            - 'quat': Orientation quaternion from 'orientation'
            - 'timestamp_ns': Timestamp from 'timestamp'
        """
        if obs_keys is None:
            obs_keys = ['ang_vel', 'acc', 'quat']

        data = self._subscriber.get_latest_data()
        if data is None:
            return None

        obs_out = {'timestamp_ns': data['timestamp']}

        for key in obs_keys:
            if key == 'ang_vel':
                obs_out[key] = data['angular_velocity']
            elif key == 'acc':
                obs_out[key] = data['acceleration']
            elif key == 'quat':
                obs_out[key] = data['orientation']
            else:
                raise ValueError(f"Invalid observation key: {key}")

        return obs_out

    def get_acceleration(self) -> np.ndarray | None:
        """Get the latest linear acceleration from ZED IMU.

        Returns:
            Linear acceleration [x, y, z] in m/s² if available, None otherwise.
        """
        return self._subscriber.get_acceleration()

    def get_angular_velocity(self) -> np.ndarray | None:
        """Get the latest angular velocity from ZED IMU.

        Returns:
            Angular velocity [x, y, z] in rad/s if available, None otherwise.
        """
        return self._subscriber.get_angular_velocity()

    def get_orientation(self) -> np.ndarray | None:
        """Get the latest orientation quaternion from ZED IMU.

        Returns:
            Orientation quaternion [x, y, z, w] if available, None otherwise.
        """
        return self._subscriber.get_orientation()

    def get_magnetometer(self) -> np.ndarray | None:
        """Get the latest magnetometer reading from ZED IMU.

        Returns:
            Magnetic field [x, y, z] in µT if available, None otherwise.
        """
        return self._subscriber.get_magnetometer()

    def has_magnetometer(self) -> bool:
        """Check if the ZED IMU has magnetometer data available.

        Returns:
            True if magnetometer data is available, False otherwise.
        """
        return self._subscriber.has_magnetometer()

    @property
    def fps(self) -> float:
        """Get the current FPS measurement.

        Returns:
            Current frames per second measurement.
        """
        return self._subscriber.fps

    @property
    def name(self) -> str:
        """Get the ZED IMU name.

        Returns:
            IMU name string.
        """
        return self._name
