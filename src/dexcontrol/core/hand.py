# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Robot hand control module.

This module provides the Hand class for controlling a robotic hand through Zenoh
communication. It handles joint position control and state monitoring.
"""

import numpy as np
import zenoh
from jaxtyping import Float

from dexcontrol.config.core.hand import HandConfig
from dexcontrol.core.component import RobotJointComponent
from dexcontrol.proto import dexcontrol_msg_pb2


class Hand(RobotJointComponent):
    """Robot hand control class.

    This class provides methods to control a robotic hand by publishing commands and
    receiving state information through Zenoh communication.
    """

    def __init__(
        self,
        configs: HandConfig,
        zenoh_session: zenoh.Session,
    ) -> None:
        """Initialize the hand controller.

        Args:
            configs: Hand configuration parameters containing communication topics
                and predefined hand positions.
            zenoh_session: Active Zenoh communication session for message passing.
        """
        super().__init__(
            state_sub_topic=configs.state_sub_topic,
            control_pub_topic=configs.control_pub_topic,
            state_message_type=dexcontrol_msg_pb2.HandState,
            zenoh_session=zenoh_session,
            joint_name=configs.joint_name,
        )

        # Store predefined hand positions as private attributes
        self._joint_pos_open = np.array(configs.joint_pos_open, dtype=np.float32)
        self._joint_pos_close = np.array(configs.joint_pos_close, dtype=np.float32)

    def _send_position_command(
        self, joint_pos: Float[np.ndarray, " N"] | list[float]
    ) -> None:
        """Send joint position control commands to the hand.

        Args:
            joint_pos: Joint positions as list or numpy array.
        """
        control_msg = dexcontrol_msg_pb2.HandCommand()
        joint_pos_array = self._convert_joint_cmd_to_array(joint_pos)
        control_msg.joint_pos.extend(joint_pos_array.tolist())
        self._publish_control(control_msg)

    def set_joint_pos(
        self,
        joint_pos: Float[np.ndarray, " N"] | list[float] | dict[str, float],
        relative: bool = False,
        wait_time: float = 0.0,
        wait_kwargs: dict[str, float] | None = None,
    ) -> None:
        """Send joint position control commands to the hand.

        Args:
            joint_pos: Joint positions as either:
                - List of joint values [j1, j2, ..., jN]
                - Numpy array with shape (N,)
                - Dictionary mapping joint names to position values
            relative: If True, the joint positions are relative to the current position.
            wait_time: Time to wait after setting the joint positions.
            wait_kwargs: Optional parameters for trajectory generation (not used in Hand).

        Raises:
            ValueError: If joint_pos dictionary contains invalid joint names.
        """
        super().set_joint_pos(joint_pos, relative, wait_time, wait_kwargs)

    def open_hand(self, wait_time: float = 0.0) -> None:
        """Open the hand to the predefined open position.

        Args:
            wait_time: Time to wait after opening the hand.
        """
        self.set_joint_pos(self._joint_pos_open, wait_time=wait_time)

    def close_hand(self, wait_time: float = 0.0) -> None:
        """Close the hand to the predefined closed position.

        Args:
            wait_time: Time to wait after closing the hand.
        """
        self.set_joint_pos(self._joint_pos_close, wait_time=wait_time)


class HandF5D6(Hand):
    """Specialized hand class for the F5D6 hand model.

    Extends the basic Hand class with additional control methods specific to
    the F5D6 hand model.
    """

    def close_hand(self, wait_time: float = 0.0) -> None:
        """Close the hand fully using a two-step approach for better control."""
        if self.is_joint_pos_reached(self._joint_pos_close, tolerance=0.1):
            return

        # First step: Move to intermediate position
        intermediate_pos = self._get_intermediate_close_position()
        first_step_wait_time = 0.8
        self.set_joint_pos(intermediate_pos, wait_time=first_step_wait_time)

        # Second step: Move to final closed position
        remaining_wait_time = max(0.0, wait_time - first_step_wait_time)
        self.set_joint_pos(self._joint_pos_close, wait_time=remaining_wait_time)

    def open_hand(self, wait_time: float = 0.0) -> None:
        """Open the hand fully using a two-step approach for better control."""
        if self.is_joint_pos_reached(self._joint_pos_open, tolerance=0.1):
            return

        # First step: Move to intermediate position
        intermediate_pos = self._get_intermediate_open_position()
        first_step_wait_time = 0.3
        self.set_joint_pos(intermediate_pos, wait_time=first_step_wait_time)

        # Second step: Move to final open position
        remaining_wait_time = max(0.0, wait_time - first_step_wait_time)
        self.set_joint_pos(self._joint_pos_open, wait_time=remaining_wait_time)

    def _get_intermediate_close_position(self) -> np.ndarray:
        """Get intermediate position for closing hand.

        Returns:
            Intermediate joint positions for smooth closing motion.
        """
        intermediate_pos = self._joint_pos_close.copy()
        ratio = 0.2
        # Adjust thumb opposition joint (last joint)
        intermediate_pos[-1] = self._joint_pos_close[-1] * ratio + self._joint_pos_open[
            -1
        ] * (1 - ratio)
        return intermediate_pos

    def _get_intermediate_open_position(self) -> np.ndarray:
        """Get intermediate position for opening hand.

        Returns:
            Intermediate joint positions for smooth opening motion.
        """
        intermediate_pos = self._joint_pos_close.copy()
        ratio = 0.2
        # Adjust thumb opposition joint (last joint)
        intermediate_pos[-1] = self._joint_pos_close[-1] * ratio + self._joint_pos_open[
            -1
        ] * (1 - ratio)
        # Adjust thumb flexion joint (first joint)
        intermediate_pos[0] = self._joint_pos_close[0] * ratio + self._joint_pos_open[
            0
        ] * (1 - ratio)
        return intermediate_pos
