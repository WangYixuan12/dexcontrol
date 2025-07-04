# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Robot arm control module.

This module provides the Arm class for controlling a robot arm through Zenoh
communication and the ArmWrenchSensor class for reading wrench sensor data.
"""

import time
from typing import Any, Final, Literal

import numpy as np
import zenoh
from jaxtyping import Float
from loguru import logger

from dexcontrol.config.core.arm import ArmConfig
from dexcontrol.core.component import RobotComponent, RobotJointComponent
from dexcontrol.proto import dexcontrol_msg_pb2, dexcontrol_query_pb2
from dexcontrol.utils.os_utils import resolve_key_name
from dexcontrol.utils.rate_limiter import RateLimiter
from dexcontrol.utils.trajectory_utils import generate_linear_trajectory


class Arm(RobotJointComponent):
    """Robot arm control class.

    This class provides methods to control a robot arm by publishing commands and
    receiving state information through Zenoh communication.

    Attributes:
        mode_querier: Zenoh querier for setting arm mode.
        wrench_sensor: Optional ArmWrenchSensor instance for wrench sensor data.
    """

    def __init__(
        self,
        configs: ArmConfig,
        zenoh_session: zenoh.Session,
    ) -> None:
        """Initialize the arm controller.

        Args:
            configs: Configuration parameters for the arm including communication topics.
            zenoh_session: Active Zenoh communication session for message passing.
        """
        super().__init__(
            state_sub_topic=configs.state_sub_topic,
            control_pub_topic=configs.control_pub_topic,
            state_message_type=dexcontrol_msg_pb2.ArmState,
            zenoh_session=zenoh_session,
            joint_name=configs.joint_name,
            pose_pool=configs.pose_pool,
        )

        self.mode_querier: Final[zenoh.Querier] = zenoh_session.declare_querier(
            resolve_key_name(configs.set_mode_query), timeout=2.0
        )

        # Initialize wrench sensor if configured
        self.wrench_sensor: ArmWrenchSensor | None = None
        if configs.wrench_sub_topic:
            self.wrench_sensor = ArmWrenchSensor(
                configs.wrench_sub_topic, zenoh_session
            )

        self._default_max_vel = configs.default_max_vel
        self._default_control_hz = configs.default_control_hz
        if self._default_max_vel > 3.0:
            logger.warning(
                f"Max velocity is set to {self._default_max_vel}, which is greater than 3.0. This is not recommended."
            )
            self._default_max_vel = 3.0
            logger.warning("Max velocity is clamped to 3.0")

    def set_mode(self, mode: Literal["position", "disable"]) -> None:
        """Sets the operating mode of the arm.

        Args:
            mode: Operating mode for the arm. Must be either "position" or "disable".
                "position": Enable position control
                "disable": Disable control

        Raises:
            ValueError: If an invalid mode is specified.
        """
        mode_map = {
            "position": dexcontrol_query_pb2.SetArmMode.Mode.POSITION,
            "disable": dexcontrol_query_pb2.SetArmMode.Mode.DISABLE,
        }

        if mode not in mode_map:
            raise ValueError(
                f"Invalid mode: {mode}. Must be one of {list(mode_map.keys())}"
            )

        query_msg = dexcontrol_query_pb2.SetArmMode(mode=mode_map[mode])
        self.mode_querier.get(payload=query_msg.SerializeToString())

    def _send_position_command(self, joint_pos: np.ndarray) -> None:
        """Send joint position command.

        Args:
            joint_pos: Joint positions as numpy array.
        """
        control_msg = dexcontrol_msg_pb2.ArmCommand()
        control_msg.joint_pos.extend(joint_pos.tolist())
        self._publish_control(control_msg)

    def set_joint_pos(
        self,
        joint_pos: Float[np.ndarray, " N"] | list[float] | dict[str, float],
        relative: bool = False,
        wait_time: float = 0.0,
        wait_kwargs: dict[str, float] | None = None,
        exit_on_reach: bool = False,
        exit_on_reach_kwargs: dict[str, Any] | None = None,
    ) -> None:
        """Controls the arm in joint position mode.

        Args:
            joint_pos: Joint positions as either:
                - List of joint values [j1, j2, ..., j7]
                - Numpy array with shape (7,), in radians
                - Dictionary mapping joint names to position values
            relative: If True, the joint positions are relative to the current position.
            wait_time: Time to wait between movements in seconds. If wait_time is 0,
                the joint positions will be sent, and the function call will return
                immediately. If wait_time is greater than 0, the joint positions will
                be interpolated between the current position and the target position,
                and the function will wait for the specified time between each movement.
            wait_kwargs: Keyword arguments for the interpolation (only used if
                wait_time > 0). Supported keys:
                - control_hz: Control frequency in Hz (default: 100).
                - max_vel: Maximum velocity in rad/s (default: 0.5).
            exit_on_reach: If True, the function will exit when the joint positions are reached.
            exit_on_reach_kwargs: Optional parameters for exit when the joint positions are reached.
        Raises:
            ValueError: If joint_pos dictionary contains invalid joint names.
        """
        if wait_kwargs is None:
            wait_kwargs = {}

        resolved_joint_pos = (
            self._resolve_relative_joint_cmd(joint_pos) if relative else joint_pos
        )

        if wait_time > 0.0:
            self._execute_trajectory_motion(
                resolved_joint_pos,
                wait_time,
                wait_kwargs,
                exit_on_reach,
                exit_on_reach_kwargs,
            )
        else:
            # Convert to array format
            if isinstance(resolved_joint_pos, (list, dict)):
                resolved_joint_pos = self._convert_joint_cmd_to_array(
                    resolved_joint_pos
                )
            self._send_position_command(resolved_joint_pos)

    def _execute_trajectory_motion(
        self,
        joint_pos: Float[np.ndarray, " N"] | list[float] | dict[str, float],
        wait_time: float,
        wait_kwargs: dict[str, float],
        exit_on_reach: bool = False,
        exit_on_reach_kwargs: dict[str, Any] | None = None,
    ) -> None:
        """Execute trajectory-based motion to target position.

        Args:
            joint_pos: Target joint positions as list, numpy array, or dictionary.
            wait_time: Total time for the motion.
            wait_kwargs: Parameters for trajectory generation.
            exit_on_reach: If True, the function will exit when the joint positions are reached.
            exit_on_reach_kwargs: Optional parameters for exit when the joint positions are reached.
        """
        # Set default parameters
        control_hz = wait_kwargs.get("control_hz", self._default_control_hz)
        max_vel = wait_kwargs.get("max_vel", self._default_max_vel)
        exit_on_reach_kwargs = exit_on_reach_kwargs or {}
        exit_on_reach_kwargs.setdefault("tolerance", 0.05)

        # Create rate limiter and get current position
        rate_limiter = RateLimiter(control_hz)
        current_joint_pos = self.get_joint_pos().copy()

        # Convert input to numpy array for trajectory generation
        if isinstance(joint_pos, (list, dict)):
            target_pos = (
                self._convert_dict_to_array(joint_pos)
                if isinstance(joint_pos, dict)
                else np.array(joint_pos, dtype=np.float32)
            )
        else:
            target_pos = joint_pos

        # Generate trajectory using utility function
        trajectory, _ = generate_linear_trajectory(
            current_joint_pos, target_pos, max_vel, control_hz
        )
        # Execute trajectory with time limit
        start_time = time.time()
        for pos in trajectory:
            if time.time() - start_time > wait_time:
                break
            self._send_position_command(pos)
            rate_limiter.sleep()

        # Hold final position for remaining time
        while time.time() - start_time < wait_time:
            self._send_position_command(target_pos)
            rate_limiter.sleep()
            if exit_on_reach and self.is_joint_pos_reached(
                target_pos, **exit_on_reach_kwargs
            ):
                break

    def set_joint_pos_vel(
        self,
        joint_pos: Float[np.ndarray, " N"] | list[float] | dict[str, float],
        joint_vel: Float[np.ndarray, " N"] | list[float] | dict[str, float],
        relative: bool = False,
    ) -> None:
        """Controls the arm in joint position mode with a velocity feedforward term.

        Args:
            joint_pos: Joint positions as either:
                - List of joint values [j1, j2, ..., j7]
                - Numpy array with shape (7,), in radians
                - Dictionary of joint names and position values
            joint_vel: Joint velocities as either:
                - List of joint values [v1, v2, ..., v7]
                - Numpy array with shape (7,), in radians/sec
                - Dictionary of joint names and velocity values
            relative: If True, the joint positions are relative to the current position.

        Raises:
            ValueError: If joint_pos dictionary contains invalid joint names.
        """
        resolved_joint_pos = (
            self._resolve_relative_joint_cmd(joint_pos) if relative else joint_pos
        )

        # Convert inputs to numpy arrays
        if isinstance(resolved_joint_pos, (list, dict)):
            target_pos = (
                self._convert_dict_to_array(resolved_joint_pos)
                if isinstance(resolved_joint_pos, dict)
                else np.array(resolved_joint_pos, dtype=np.float32)
            )
        else:
            target_pos = resolved_joint_pos

        if isinstance(joint_vel, (list, dict)):
            target_vel = (
                self._convert_dict_to_array(joint_vel)
                if isinstance(joint_vel, dict)
                else np.array(joint_vel, dtype=np.float32)
            )
        else:
            target_vel = joint_vel

        control_msg = dexcontrol_msg_pb2.ArmCommand(
            command_type=dexcontrol_msg_pb2.ArmCommand.CommandType.VELOCITY_FEEDFORWARD,
            joint_pos=list(target_pos),
            joint_vel=list(target_vel),
        )
        self._publish_control(control_msg)

    def shutdown(self) -> None:
        """Cleans up all Zenoh resources."""
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

        if self.wrench_sensor:
            self.wrench_sensor.shutdown()


class ArmWrenchSensor(RobotComponent):
    """Wrench sensor reader for the robot arm.

    This class provides methods to read wrench sensor data through Zenoh communication.
    """

    def __init__(self, state_sub_topic: str, zenoh_session: zenoh.Session) -> None:
        """Initialize the wrench sensor reader.

        Args:
            state_sub_topic: Topic to subscribe to for wrench sensor data.
            zenoh_session: Active Zenoh communication session for message passing.
        """
        super().__init__(
            state_sub_topic=state_sub_topic,
            zenoh_session=zenoh_session,
            state_message_type=dexcontrol_msg_pb2.WrenchState,
        )

    def get_wrench_state(self) -> Float[np.ndarray, "6"]:
        """Get the current wrench sensor reading.

        Returns:
            Array of wrench values [fx, fy, fz, tx, ty, tz].
        """
        state = self._get_state()
        return np.array(state.wrench, dtype=np.float32)

    def get_button_state(self) -> tuple[bool, bool]:
        """Get the state of the wrench sensor buttons.

        Returns:
            Tuple of (blue_button_state, green_button_state).
        """
        state = self._get_state()
        return state.blue_button, state.green_button

    def get_state(self) -> dict[str, Float[np.ndarray, "6"] | bool]:
        """Get the complete wrench sensor state.

        Returns:
            Dictionary containing wrench values and button states.
        """
        state = self._get_state()
        return {
            "wrench": np.array(state.wrench, dtype=np.float32),
            "blue_button": state.blue_button,
            "green_button": state.green_button,
        }

    def get_blue_button_state(self) -> bool:
        """Get the state of the blue button.

        Returns:
            True if the blue button is pressed, False otherwise.
        """
        state = self._get_state()
        return state.blue_button

    def get_green_button_state(self) -> bool:
        """Get the state of the green button.

        Returns:
            True if the green button is pressed, False otherwise.
        """
        state = self._get_state()
        return state.green_button
