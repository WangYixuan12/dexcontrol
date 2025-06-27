# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Robot head control module.

This module provides the Head class for controlling a robot head through Zenoh
communication. It handles joint position and velocity control, mode setting, and
state monitoring.
"""

import time
from typing import Final, Literal

import numpy as np
import zenoh
from jaxtyping import Float
from loguru import logger

from dexcontrol.config.core import HeadConfig
from dexcontrol.core.component import RobotJointComponent
from dexcontrol.proto import dexcontrol_msg_pb2, dexcontrol_query_pb2
from dexcontrol.utils.os_utils import resolve_key_name


class Head(RobotJointComponent):
    """Robot head control class.

    This class provides methods to control a robot head by publishing commands and
    receiving state information through Zenoh communication.

    Attributes:
        mode_querier: Zenoh querier for setting head mode.
        default_vel: Default joint velocities for all joints.
        max_vel: Maximum allowed joint velocities for all joints.
    """

    def __init__(
        self,
        configs: HeadConfig,
        zenoh_session: zenoh.Session,
    ) -> None:
        """Initialize the head controller.

        Args:
            configs: Configuration parameters for the head including communication topics.
            zenoh_session: Active Zenoh communication session for message passing.
        """
        super().__init__(
            state_sub_topic=configs.state_sub_topic,
            control_pub_topic=configs.control_pub_topic,
            state_message_type=dexcontrol_msg_pb2.HeadState,
            zenoh_session=zenoh_session,
            joint_name=configs.joint_name,
            pose_pool=configs.pose_pool,
        )

        self.mode_querier: Final[zenoh.Querier] = zenoh_session.declare_querier(
            resolve_key_name(configs.set_mode_query), timeout=2.0
        )
        self.default_vel: Final[float] = configs.default_vel
        self.max_vel: Final[float] = configs.max_vel
        assert self.max_vel > self.default_vel, (
            "max_vel must be greater than default_vel"
        )
        self._joint_limit: Float[np.ndarray, "3 2"] = np.stack(
            [configs.joint_limit_lower, configs.joint_limit_upper], axis=1
        )

    def set_joint_pos(
        self,
        joint_pos: Float[np.ndarray, "2"] | list[float] | dict[str, float],
        relative: bool = False,
        wait_time: float = 0.0,
        wait_kwargs: dict[str, float] | None = None,
    ) -> None:
        """Send joint position control commands to the head.

        Args:
            joint_pos: Joint positions as either:
                - List of joint values [j1, j2]
                - Numpy array with shape (2,), in radians
                - Dictionary mapping joint names to position values
            relative: If True, the joint positions are relative to the current position.
            wait_time: Time to wait after sending command in seconds. If 0, returns
                immediately after sending command.
            wait_kwargs: Optional parameters for trajectory generation (not used in Head).

        Raises:
            ValueError: If joint_pos dictionary contains invalid joint names.
        """
        self.set_joint_pos_vel(
            joint_pos, joint_vel=None, relative=relative, wait_time=wait_time
        )

    def set_joint_pos_vel(
        self,
        joint_pos: Float[np.ndarray, "2"] | list[float] | dict[str, float],
        joint_vel: Float[np.ndarray, "2"]
        | list[float]
        | dict[str, float]
        | float
        | None = None,
        relative: bool = False,
        wait_time: float = 0.0,
    ) -> None:
        """Send control commands to the head.

        Args:
            joint_pos: Joint positions as either:
                - List of joint values [j1, j2]
                - Numpy array with shape (2,), in radians
                - Dictionary mapping joint names to position values
            joint_vel: Optional joint velocities as either:
                - List of joint values [v1, v2]
                - Numpy array with shape (2,), in rad/s
                - Dictionary mapping joint names to velocity values
                - Single float value to be applied to all joints
                If None, velocities are calculated based on default velocity setting.
            relative: If True, the joint positions are relative to the current position.
            wait_time: Time to wait after sending command in seconds. If 0, returns
                immediately after sending command.

        Raises:
            ValueError: If wait_time is negative or joint_pos dictionary contains
                invalid joint names.
        """
        if wait_time < 0.0:
            raise ValueError("wait_time must be greater than or equal to 0")

        # Handle relative positioning
        if relative:
            joint_pos = self._resolve_relative_joint_cmd(joint_pos)

        # Convert inputs to numpy arrays
        joint_pos = self._convert_joint_cmd_to_array(joint_pos)
        joint_vel = self._process_joint_velocities(joint_vel, joint_pos)

        # Create and send control message
        control_msg = dexcontrol_msg_pb2.HeadCommand()
        control_msg.joint_pos.extend(joint_pos.tolist())
        control_msg.joint_vel.extend(joint_vel.tolist())
        self._publish_control(control_msg)

        # Wait if specified
        if wait_time > 0.0:
            time.sleep(wait_time)

    def set_mode(self, mode: Literal["enable", "disable"]) -> None:
        """Set the operating mode of the head.

        Args:
            mode: Operating mode for the head. Must be either "enable" or "disable".

        Raises:
            ValueError: If an invalid mode is specified.
        """
        mode_map = {
            "enable": dexcontrol_query_pb2.SetHeadMode.Mode.ENABLE,
            "disable": dexcontrol_query_pb2.SetHeadMode.Mode.DISABLE,
        }

        if mode not in mode_map:
            raise ValueError(
                f"Invalid mode: {mode}. Must be one of {list(mode_map.keys())}"
            )

        query_msg = dexcontrol_query_pb2.SetHeadMode()
        query_msg.mode = mode_map[mode]
        replies = self.mode_querier.get(payload=query_msg.SerializeToString())

        for reply in replies:
            if reply.ok is not None and reply.ok.payload is not None:
                logger.info(reply.ok.payload.to_string())
        time.sleep(0.5)

    def get_joint_limit(self) -> Float[np.ndarray, "3 2"]:
        """Get the joint limits of the head.

        Returns:
            Array of joint limits with shape (3, 2), where the first column contains
            lower limits and the second column contains upper limits.
        """
        return self._joint_limit

    def stop(self) -> None:
        """Stop the head by setting target position to current position with zero velocity."""
        current_pos = self.get_joint_pos()
        zero_vel = np.zeros(3, dtype=np.float32)
        self.set_joint_pos_vel(current_pos, zero_vel, relative=False, wait_time=0.0)

    def shutdown(self) -> None:
        """Clean up Zenoh resources for the head component."""
        self.stop()
        super().shutdown()
        try:
            if hasattr(self, "mode_querier") and self.mode_querier:
                self.mode_querier.undeclare()
        except Exception as e:
            # Don't log "Undeclared querier" errors as warnings - they're expected during shutdown
            error_msg = str(e).lower()
            if not ("undeclared" in error_msg or "closed" in error_msg):
                logger.warning(
                    f"Error undeclaring mode querier for {self.__class__.__name__}: {e}"
                )

    def _process_joint_velocities(
        self,
        joint_vel: Float[np.ndarray, "2"]
        | list[float]
        | dict[str, float]
        | float
        | None,
        joint_pos: np.ndarray,
    ) -> np.ndarray:
        """Process and validate joint velocities.

        Args:
            joint_vel: Joint velocities in various formats or None.
            joint_pos: Target joint positions for velocity calculation.

        Returns:
            Processed joint velocities as numpy array.
        """
        if joint_vel is None:
            # Calculate velocities based on motion direction and default velocity
            joint_motion = joint_pos - self.get_joint_pos()
            motion_norm = np.linalg.norm(joint_motion)

            if motion_norm < 1e-6:  # Avoid division by zero
                return np.zeros(2, dtype=np.float32)

            # Scale velocities by default velocity
            return (joint_motion / motion_norm) * self.default_vel

        if isinstance(joint_vel, (int, float)):
            # Single value - apply to all joints
            return np.full(2, joint_vel, dtype=np.float32)

        # Convert to array and clip to max velocity
        return self._convert_joint_cmd_to_array(joint_vel, clip_value=self.max_vel)
