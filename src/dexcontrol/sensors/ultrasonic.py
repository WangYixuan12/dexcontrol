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

import numpy as np
import zenoh

from dexcontrol.proto import dexcontrol_msg_pb2
from dexcontrol.utils.subscribers import ProtobufZenohSubscriber


class UltrasonicSensor:
    """Ultrasonic sensor using Zenoh subscriber.

    This sensor provides distance measurements from ultrasonic sensors
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
            message_type=dexcontrol_msg_pb2.UltrasonicState,
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

    def get_obs(self) -> np.ndarray | None:
        """Get observation data for the ultrasonic sensor.

        This method provides a standardized observation format
        that can be used in robotics applications.

        Returns:
            Numpy array of distances in meters with shape (4,) in the order:
            [front_left, front_right, back_left, back_right].
        """
        data = self._subscriber.get_latest_data()
        if data is not None:
            obs = [
                data.front_left,
                data.front_right,
                data.back_left,
                data.back_right,
            ]
            return np.array(obs, dtype=np.float32)

        return None

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
