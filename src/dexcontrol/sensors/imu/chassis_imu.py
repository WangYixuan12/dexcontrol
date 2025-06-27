# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Ultrasonic sensor implementations using Zenoh subscribers.

This module provides ultrasonic sensor classes that use the generic
subscriber for distance measurements.
"""

from typing import Literal, cast

import numpy as np
import zenoh

from dexcontrol.proto import dexcontrol_msg_pb2
from dexcontrol.utils.subscribers import ProtobufZenohSubscriber


class ChassisIMUSensor:
    """Chassis IMU sensor using Zenoh subscriber.

    This sensor provides IMU data from the chassis
    """

    def __init__(
        self,
        configs,
        zenoh_session: zenoh.Session,
    ) -> None:
        """Initialize the ultrasonic sensor.

        Args:
            configs: Configuration for the ultrasonic sensor.
            zenoh_session: Active Zenoh session for communication.
        """
        self._name = configs.name

        # Create the generic subscriber with JSON decoder
        self._subscriber = ProtobufZenohSubscriber(
            topic=configs.topic,
            zenoh_session=zenoh_session,
            message_type=dexcontrol_msg_pb2.IMUState,
            name=f"{self._name}_subscriber",
            enable_fps_tracking=configs.enable_fps_tracking,
            fps_log_interval=configs.fps_log_interval,
        )


    def shutdown(self) -> None:
        """Shutdown the ultrasonic sensor."""
        self._subscriber.shutdown()

    def is_active(self) -> bool:
        """Check if the ultrasonic sensor is actively receiving data.

        Returns:
            True if receiving data, False otherwise.
        """
        return self._subscriber.is_active()

    def wait_for_active(self, timeout: float = 5.0) -> bool:
        """Wait for the ultrasonic sensor to start receiving data.

        Args:
            timeout: Maximum time to wait in seconds.

        Returns:
            True if sensor becomes active, False if timeout is reached.
        """
        return self._subscriber.wait_for_active(timeout)

    def get_obs(self, obs_keys: list[Literal['ang_vel', 'acc', 'quat']] | None = None) -> dict[str, np.ndarray]:
        """Get observation data for the ZED IMU sensor.

        Args:
            obs_keys: List of observation keys to retrieve. If None, returns all available data.
                     Valid keys: ['ang_vel', 'acc', 'quat']

        Returns:
            Dictionary with observation data including all IMU measurements.
            Keys are mapped as follows:
            - 'ang_vel': Angular velocity from 'angular_velocity'
            - 'acc': Linear acceleration from 'acceleration'
            - 'quat': Orientation quaternion from 'orientation', in wxyz convention
        """
        if obs_keys is None:
            obs_keys = ['ang_vel', 'acc', 'quat']

        data = self._subscriber.get_latest_data()
        data = cast(dexcontrol_msg_pb2.IMUState, data)
        if data is None:
            raise RuntimeError("No IMU data available")

        obs_out = {}

        for key in obs_keys:
            if key == 'ang_vel':
                obs_out[key] = np.array([data.gyro_x, data.gyro_y, data.gyro_z])
            elif key == 'acc':
                obs_out[key] = np.array([data.acc_x, data.acc_y, data.acc_z])
            elif key == 'quat':
                obs_out[key] = np.array([data.quat_w, data.quat_x, data.quat_y, data.quat_z])
            else:
                raise ValueError(f"Invalid observation key: {key}")

        return obs_out

    def get_acceleration(self) -> np.ndarray:
        """Get the latest linear acceleration from ZED IMU.

        Returns:
            Linear acceleration [x, y, z] in m/s² if available, None otherwise.
        """
        return self.get_obs(obs_keys=['acc'])['acc']

    def get_angular_velocity(self) -> np.ndarray:
        """Get the latest angular velocity from ZED IMU.

        Returns:
            Angular velocity [x, y, z] in rad/s if available, None otherwise.
        """
        return self.get_obs(obs_keys=['ang_vel'])['ang_vel']

    def get_orientation(self) -> np.ndarray:
        """Get the latest orientation quaternion from ZED IMU.

        Returns:
            Orientation quaternion [x, y, z, w] if available, None otherwise.
        """
        return self.get_obs(obs_keys=['quat'])['quat']

    @property
    def fps(self) -> float:
        """Get the current FPS measurement.

        Returns:
            Current frames per second measurement.
        """
        return self._subscriber.fps

    @property
    def name(self) -> str:
        """Get the sensor name.

        Returns:
            Sensor name string.
        """
        return self._name
