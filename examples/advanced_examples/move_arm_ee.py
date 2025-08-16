# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Example script to move the end-effector by a relative pose.

This script demonstrates how to move both robot arms to a position and orientation
relative to their current pose.
"""

import numpy as np
import tyro
from dexmotion.tasks.move_to_relative_pose_task import MoveToRelativePoseTask
from dexmotion.utils import robot_utils
from loguru import logger

from dexcontrol.robot import Robot


def main() -> None:
    """Run the move to relative pose example.

    Creates an instance of MoveToRelativePoseTask and executes the motion planning
    and control sequence to move both end-effectors to a relative pose.
    """
    # Define relative pose changes for each end-effector
    # Format: (relative_position, relative_rpy)
    # relative_position: 3x1 array for x,y,z changes in meters
    # relative_rpy: 3x1 array for roll,pitch,yaw changes in radians
    relative_poses_dict: dict[str, tuple[np.ndarray, np.ndarray]] = {
        "L_ee": (
            np.array([0.0, 0.0, 0.1]),  # Move 10cm in z direction
            np.array([np.pi / 4, 0.0, 0.0]),  # Rotate 45 degrees around x-axis
        ),
        "R_ee": (
            np.array([0.0, 0.0, 0.1]),  # Move 10cm in z direction
            np.array([np.pi / 4, 0.0, 0.0]),  # Rotate 45 degrees around x-axis
        ),
    }

    # Initialize robot and get current joint positions
    bot = Robot()
    control_hz = 250
    initial_joint_pos = bot.get_joint_pos_dict(
        component=["left_arm", "right_arm", "torso", "head"]
    )

    # Create task instance with initial joint configuration
    move_to_relative_pose_task = MoveToRelativePoseTask(
        initial_joint_configuration=initial_joint_pos,
    )
    if move_to_relative_pose_task.motion_manager is None:
        raise ValueError("Motion manager is None")

    start_configuration_dict = (
        move_to_relative_pose_task.motion_manager.get_joint_pos_dict()
    )

    # Run the task and get trajectory data
    ts_sample, qs_sample, qds_sample, qdds_sample, duration = (
        move_to_relative_pose_task.run(
            relative_poses_dict,
            start_configuration_dict,
            control_frequency=control_hz,
            planner_type="interpolation_planner",
            generate_trajectory=True,
        )
    )

    if move_to_relative_pose_task.motion_manager.pin_robot is None:
        raise ValueError("Pin robot is None")

    if move_to_relative_pose_task.motion_manager.visualizer is None:
        raise ValueError("Visualizer is None")

    # Update visualization with the motion plan
    move_to_relative_pose_task.motion_manager.visualizer.update_motion_plan(
        qs_sample,
        robot_utils.get_joint_names(
            move_to_relative_pose_task.motion_manager.pin_robot
        ),
        duration=duration if duration is not None else 1.0,
    )

    logger.info(f"Final configuration: {qs_sample[-1]}")
    logger.info("Press enter to run the trajectory on the real robot...")
    input()

    # Extract arm trajectories and execute on the robot
    trajectory = {"left_arm": qs_sample[:, :7], "right_arm": qs_sample[:, 7:14]}
    bot.execute_trajectory(trajectory, control_hz=control_hz, relative=False)  # type: ignore

    move_to_relative_pose_task.motion_manager.set_joint_pos(qs_sample[-1])
    move_to_relative_pose_task.motion_manager.visualizer.clear_motion_plan()


if __name__ == "__main__":
    tyro.cli(main)
