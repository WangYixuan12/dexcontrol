# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Example script demonstrating robot arm dance sequence execution.

This script loads a pre-recorded dance trajectory and executes synchronized
movements of both robot arms while respecting motion constraints.
"""

from pathlib import Path

import numpy as np
import ruckig
import tyro
from loguru import logger

from dexcontrol.robot import Robot


class TrajectorySmoother:
    """Generates smooth trajectories respecting motion constraints."""

    def __init__(
        self, dt: float = 0.01, safety_factor: float = 2.0, dof: int = 7
    ) -> None:
        """Initializes trajectory smoother with motion constraints.

        Args:
            dt: Control cycle time in seconds. Must be positive.
            safety_factor: Factor to scale down motion limits. Must be positive.

        Raises:
            ValueError: If dt or safety_factor is not positive.
        """
        if dt <= 0 or safety_factor <= 0:
            raise ValueError("dt and safety_factor must be positive")

        self.arm_dof = dof
        self.dt = dt

        # Initialize Ruckig OTG
        self.otg = ruckig.Ruckig(self.arm_dof, dt)
        self.inp = ruckig.InputParameter(self.arm_dof)
        self.out = ruckig.OutputParameter(self.arm_dof)

        # Set joint motion limits (deg/s, deg/s^2, deg/s^3)
        velocities = [180] * dof
        accelerations = [600] * dof
        jerks = [6000] * dof

        # Convert to radians and apply safety factor
        self.inp.max_velocity = np.deg2rad(velocities) / safety_factor
        self.inp.max_acceleration = np.deg2rad(accelerations) / safety_factor
        self.inp.max_jerk = np.deg2rad(jerks) / safety_factor

    def smooth_trajectory(
        self, waypoints: np.ndarray, trajectory_dt: float
    ) -> tuple[np.ndarray, np.ndarray]:
        """Generates time-optimal smooth trajectory through waypoints.

        Args:
            waypoints: Joint positions array of shape (N, 7).
            trajectory_dt: Desired time duration between waypoints.

        Returns:
            Tuple of position and velocity trajectories.

        Raises:
            ValueError: If waypoints are invalid.
        """
        if len(waypoints) < 2 or waypoints.shape[1] != self.arm_dof:
            raise ValueError(
                f"Waypoints must have shape (N, {self.arm_dof}) where N >= 2"
            )

        pos_traj = []
        vel_traj = []
        n_points = int(trajectory_dt / self.dt)
        n_points = max(1, n_points)

        self.inp.current_position = waypoints[0]

        # Generate trajectory segments
        for target_pos in waypoints[1:]:
            self.inp.target_position = target_pos
            self.inp.target_velocity = [0.0] * self.arm_dof

            for _ in range(n_points):
                self.otg.update(self.inp, self.out)
                pos_traj.append(self.out.new_position)
                vel_traj.append(self.out.new_velocity)
                self.out.pass_to_input(self.inp)

        return np.array(pos_traj), np.array(vel_traj)


def main(control_hz: int = 250, speed_factor: float = 2.0) -> None:
    """Executes pre-recorded dance routine with both robot arms.

    Args:
        control_hz: Control frequency in Hz.
        speed_factor: Factor to scale the speed of the dance. 1 is the original speed. Do not set it over 3.
    """

    logger.warning(
        "Warning: Be ready to press e-stop if needed! "
        "This example does not check for self-collisions."
    )
    logger.warning(
        "Please ensure the arms and the torso have sufficient space to move."
    )
    if input("Continue? [y/N]: ").lower() != "y":
        return

    speed_factor = np.clip(speed_factor, 0.5, 3)

    bot = Robot()
    assert bot.robot_model in ["vega-rc2", "vega-1"], "Invalid robot name"
    if not bot.left_arm.is_pose_reached("folded") and not bot.right_arm.is_pose_reached(
        "folded"
    ):
        left_arm_l_shape = bot.left_arm.is_pose_reached(
            "L_shape", joint_id=list(range(1, 7)), tolerance=0.1
        )
        right_arm_l_shape = bot.right_arm.is_pose_reached(
            "L_shape", joint_id=list(range(1, 7)), tolerance=0.1
        )
        if not left_arm_l_shape and not right_arm_l_shape:
            raise RuntimeError(
                "Please first run init_arm_safe.py to initialize the arms to safe positions!"
            )

    bot.set_joint_pos(
        {
            "left_arm": bot.left_arm.get_predefined_pose("folded"),
            "right_arm": bot.right_arm.get_predefined_pose("folded"),
            "head": bot.head.get_predefined_pose("home"),
        },
        wait_time=5.0,
    )
    if not bot.left_arm.is_pose_reached("folded") or not bot.right_arm.is_pose_reached(
        "folded"
    ):
        raise RuntimeError("Arms did not reach the target position! Exiting...")
    if not bot.torso.is_pose_reached("crouch20_medium", tolerance=0.1):
        bot.torso.set_joint_pos(
            bot.torso.get_predefined_pose("crouch20_medium"), wait_time=8.0
        )
    logger.info("Getting torso state")
    logger.info(bot.torso.get_joint_pos())

    # Compute desired control frequency
    desired_control_hz = int(control_hz * speed_factor)

    # Load trajectory data
    trajectory_filename = f"{bot.robot_model}_dance.npz"
    data_path = Path(__file__).parent / "data" / trajectory_filename
    data = np.load(data_path)

    # Extract joint trajectories
    left_qs = data["left_arm"][::2]
    right_qs = data["right_arm"][::2]
    torso_qs = data["torso"][::2]
    head_qs = data["head"][::2]

    # Move to initial positions
    bot.left_hand.close_hand()
    bot.right_hand.close_hand()
    bot.set_joint_pos(
        {
            "left_arm": left_qs[0],
            "right_arm": right_qs[0],
            "head": head_qs[0],
            "torso": torso_qs[0],
        },
        wait_time=3.0,
    )
    logger.info("Initial movement completed")

    input("Enter to start dancing ...")

    # Initialize smoothers for each body part
    smoothers = {
        "left": TrajectorySmoother(dt=1 / desired_control_hz),
        "right": TrajectorySmoother(dt=1 / desired_control_hz),
        "torso": TrajectorySmoother(dt=1 / desired_control_hz, dof=3),
        "head": TrajectorySmoother(dt=1 / desired_control_hz, dof=3),
    }

    # Calculate segment duration based on speed factor
    segment_duration = 0.00001 / speed_factor

    logger.info(f"Starting dance sequence with {len(left_qs)} waypoints")
    logger.info("Using smooth Ruckig trajectories to reach each goal point")

    # Execute trajectory by generating smooth segments to each goal point
    for i in range(1, len(left_qs)):  # Skip first point as we're already there
        logger.info(f"Smooth trajectory to waypoint {i}/{len(left_qs) - 1}")

        # Generate smooth trajectory segments from current to next waypoint
        left_segment, _ = smoothers["left"].smooth_trajectory(
            np.array([left_qs[i - 1], left_qs[i]]), segment_duration
        )
        right_segment, _ = smoothers["right"].smooth_trajectory(
            np.array([right_qs[i - 1], right_qs[i]]), segment_duration
        )
        torso_segment, _ = smoothers["torso"].smooth_trajectory(
            np.array([torso_qs[i - 1], torso_qs[i]]), segment_duration
        )
        head_segment, _ = smoothers["head"].smooth_trajectory(
            np.array([head_qs[i - 1], head_qs[i]]), segment_duration
        )

        # Execute the smooth trajectory segment to reach this goal point
        bot.execute_trajectory(
            trajectory=dict(
                left_arm=left_segment,
                right_arm=right_segment,
                torso=torso_segment,
                head=head_segment,
            ),
            control_hz=desired_control_hz,
            relative=False,
        )

    # Finish sequence
    bot.left_hand.open_hand()
    bot.right_hand.open_hand()

    logger.info("Movement sequence completed")
    bot.shutdown()


if __name__ == "__main__":
    tyro.cli(main)
