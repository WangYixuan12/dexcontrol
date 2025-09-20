# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Base teleop class with shared functionality for different teleop modes.

This module provides base classes for robot teleoperation with inverse kinematics
control support for multiple teleoperation modes (joint space, Cartesian space).
"""

import os
import copy
import threading
import time

import tyro
import numpy as np
import pytransform3d.rotations as pr
from pytransform3d.rotations import check_matrix
from dexmotion.motion_manager import MotionManager
from dexmotion.utils import robot_utils
from loguru import logger

from dexcontrol.robot import Robot


def is_running_remote() -> bool:
    """Check if the script is running in a remote environment.

    Returns:
        bool: True if running in a remote environment (no display), False otherwise.
    """
    return "DISPLAY" not in os.environ

def interpolate_poses(start_pose: np.ndarray, end_pose: np.ndarray) -> np.ndarray:
    """Interpolate between two poses.
    
    Args:
        start_pose (np.ndarray): Start pose.
        end_pose (np.ndarray): End pose.

    Returns:
        np.ndarray: Interpolated pose.
    """
    step_pos = 0.002
    step_rot = np.pi / 180
    
    start_R = start_pose[:3, :3]
    end_R = end_pose[:3, :3]
    check_matrix(start_R, strict_check=True)
    check_matrix(end_R, strict_check=True)
    
    start_t = start_pose[:3, 3]
    end_t = end_pose[:3, 3]
    
    rel_pos = float(np.linalg.norm(end_t - start_t))
    R_rel = np.dot(start_R, end_R.T)
    
    axis_angle_rel = pr.axis_angle_from_matrix(R_rel)
    rel_rot = float(axis_angle_rel[3])
    
    num_steps = max(int(rel_pos / step_pos), int(rel_rot / step_rot))
    
    interp_poses = []
    for i in range(num_steps):
        pos_t = min(step_pos * i / rel_pos, 1.0)
        rot_t = min(step_rot * i / rel_rot, 1.0)
        interp_R = pr.matrix_slerp(start_R, end_R, rot_t)
        interp_t = start_t + (end_t - start_t) * pos_t
        interp_pose = np.concatenate([interp_R, interp_t[:, None]], axis=1)
        interp_pose = np.concatenate([interp_pose, np.array([[0.0, 0.0, 0.0, 1.0]])], axis=0)
        interp_poses.append(interp_pose)
    interp_poses = np.stack(interp_poses)
    return interp_poses

class BaseIKController:
    """Unified controller for arm inverse kinematics operations.

    This class supports both joint-space and Cartesian-space control of robot arms
    using the MotionManager for kinematics calculations.

    Attributes:
        bot (Robot): Robot instance to control.
        motion_manager (MotionManager): MotionManager instance for IK solving.
        arm_dof (int): Number of degrees of freedom in the robot arms.
        visualize (bool): Whether to enable visualization.
    """

    def __init__(self, bot: Robot | None = None, visualize: bool = True):
        """Initialize the arm IK controller.

        Args:
            bot (Robot | None): Robot instance to control. If None, a new Robot
                instance will be created.
            visualize (bool): Whether to enable visualization. Defaults to True.

        Raises:
            RuntimeError: If config path can't be found or Motion Manager
                initialization fails.
        """
        # Initialize robot if not provided
        if bot is None:
            self.bot = Robot()
        else:
            self.bot = bot

        self.visualize = visualize

        self.arm_dof = 7  # Number of degrees of freedom in the arms

        # Get initial joint positions
        try:
            joint_pos_dict = self.bot.get_joint_pos_dict(
                component=["left_arm", "right_arm", "torso"]
            )
            logger.info("Initial joint positions retrieved successfully")
        except Exception as e:
            logger.error(f"Error getting joint positions: {e}")
            raise

        # Initialize MotionManager for IK solving

        try:
            self.bot.heartbeat.pause()

            self.motion_manager = MotionManager(
                init_visualizer=self.visualize,
                initial_joint_configuration_dict=joint_pos_dict,
            )

            logger.success("Motion Manager setup complete")
        except Exception as e:
            logger.error(f"Error initializing Motion Manager: {e}")
            raise

    def move_delta_joint_space(
        self, new_joint_pos_dict: dict[str, float]
    ) -> dict[str, float]:
        """Apply a delta movement to specified joints.

        Args:
            new_joint_pos_dict (dict[str, float]): Dictionary of joint names and
                target positions.

        Returns:
            dict[str, float]: Dictionary of updated joint positions.

        Raises:
            RuntimeError: If robot or IK solver is not initialized.
        """
        if self.motion_manager.pin_robot is None:
            raise RuntimeError("Robot is not initialized")
        if self.motion_manager.local_ik_solver is None:
            raise RuntimeError("Local IK solver is not initialized")

        current_qpos_dict = self.motion_manager.get_joint_pos_dict()

        # Create a new dictionary with updated values
        updated_new_joint_pos_dict = dict(current_qpos_dict)
        updated_new_joint_pos_dict.update(new_joint_pos_dict)

        updated_new_joint_pos = robot_utils.get_qpos_from_joint_dict(
            self.motion_manager.pin_robot, updated_new_joint_pos_dict
        )

        qpos_dict, is_in_collision, is_within_joint_limits = (
            self.motion_manager.local_ik_solver.solve_ik(
                target_configuration=updated_new_joint_pos
            )
        )
        self.motion_manager.set_joint_pos(qpos_dict)

        # Update visualizer if available
        if self.motion_manager.visualizer is not None:
            qpos_array = robot_utils.get_qpos_from_joint_dict(
                self.motion_manager.pin_robot, qpos_dict
            )
            self.motion_manager.visualizer.update_motion_plan(
                qpos_array.reshape(1, -1),
                robot_utils.get_joint_names(self.motion_manager.pin_robot),
                duration=0.4,
            )

        return qpos_dict

    def move_delta_cartesian(
        self, delta_xyz: np.ndarray, delta_rpy: np.ndarray, arm_side: str
    ) -> np.ndarray:
        """Move the specified arm by a delta in Cartesian space.

        Args:
            delta_xyz (np.ndarray): Translation delta in x, y, z (meters).
            delta_rpy (np.ndarray): Rotation delta in roll, pitch, yaw (radians).
            arm_side (str): Which arm to move ("left" or "right").

        Returns:
            np.ndarray: Target joint positions for the specified arm.

        Raises:
            ValueError: If an invalid arm side is specified.
            RuntimeError: If robot or IK solver is not initialized.
        """
        if self.motion_manager.pin_robot is None:
            raise RuntimeError("Robot is not initialized")
        if self.motion_manager.local_ik_solver is None:
            raise RuntimeError("Local IK solver is not initialized")

        current_qpos = self.motion_manager.get_joint_pos()
        assert isinstance(current_qpos, np.ndarray)

        ee_pose = self.motion_manager.fk(
            frame_names=self.motion_manager.target_frames,
            qpos=current_qpos,
            update_robot_state=False,
        )

        left_ee_pose = ee_pose["L_ee"]
        right_ee_pose = ee_pose["R_ee"]

        if arm_side == "left":
            ee_pose_np = left_ee_pose.np.copy()  # type: ignore
            ee_pose_np[:3, :3] = (
                pr.matrix_from_euler(delta_rpy, 0, 1, 2, True) @ ee_pose_np[:3, :3]
            )
            ee_pose_np[:3, 3] += delta_xyz
            target_poses_dict = {
                "L_ee": ee_pose_np,
                "R_ee": right_ee_pose.np,
            }  # type: ignore
        elif arm_side == "right":
            ee_pose_np = right_ee_pose.np.copy()  # type: ignore
            ee_pose_np[:3, 3] += delta_xyz
            ee_pose_np[:3, :3] = (
                pr.matrix_from_euler(delta_rpy, 0, 1, 2, True) @ ee_pose_np[:3, :3]
            )
            target_poses_dict = {
                "L_ee": left_ee_pose.np,
                "R_ee": ee_pose_np,
            }  # type: ignore
        else:
            raise ValueError(
                f"Invalid arm side: {arm_side}. Must be 'left' or 'right'."
            )

        qpos_dict, is_in_collision, is_within_joint_limits = (
            self.motion_manager.local_ik_solver.solve_ik(target_poses_dict)
        )
        self.motion_manager.set_joint_pos(qpos_dict)

        # Update visualizer if available
        if self.motion_manager.visualizer is not None:
            qpos_array = robot_utils.get_qpos_from_joint_dict(
                self.motion_manager.pin_robot, qpos_dict
            )
            self.motion_manager.visualizer.update_motion_plan(
                qpos_array.reshape(1, -1),
                robot_utils.get_joint_names(self.motion_manager.pin_robot),
                duration=0.4,
            )

        # Extract joint positions for the requested arm
        arm_prefix = arm_side[0].upper()
        arm_joint_indices = [f"{arm_prefix}_arm_j{i + 1}" for i in range(self.arm_dof)]
        return np.array([qpos_dict[joint_name] for joint_name in arm_joint_indices])
    
    def move_to_pose(self, target_pose: np.ndarray, arm_side: str) -> None:
        """Move the specified arm to a pose.

        Args:
            pose (np.ndarray): Pose to move to.
            arm_side (str): Which arm to move ("left" or "right").

        Raises:
            ValueError: If an invalid arm side is specified.
            RuntimeError: If robot or IK solver is not initialized.
        """
        if self.motion_manager.pin_robot is None:
            raise RuntimeError("Robot is not initialized")
        if self.motion_manager.local_ik_solver is None:
            raise RuntimeError("Local IK solver is not initialized")

        current_qpos = self.motion_manager.get_joint_pos()
        assert isinstance(current_qpos, np.ndarray)

        curr_poses_dict = self.motion_manager.fk(
            frame_names=self.motion_manager.target_frames,
            qpos=current_qpos,
            update_robot_state=False,
        )
        base_t_curr_pose = curr_poses_dict[arm_side + "_ee"].np
        
        trajs = interpolate_poses(base_t_curr_pose, target_pose)
        for pose in trajs:
            if arm_side == "L":
                target_poses_dict = {
                    "L_ee": pose,
                    "R_ee": curr_poses_dict["R_ee"].np,
                }
            elif arm_side == "R":
                target_poses_dict = {
                    "R_ee": pose,
                    "L_ee": curr_poses_dict["L_ee"].np,
                }
            
            qpos_dict, is_in_collision, is_within_joint_limits = (
                self.motion_manager.local_ik_solver.solve_ik(target_poses_dict)
            )
            qval = np.zeros(7)
            for i in range(7):
                qval[i] = qpos_dict[f'{arm_side}_arm_j{i+1}']
            self.bot.right_arm.set_joint_pos(qval)
            self.motion_manager.set_joint_pos(qpos_dict)
            time.sleep(0.01)


def main() -> None:
    """Move the right arm to a pose."""
    check_board_size = 0.05
    base_t_right_eef = np.array(
        [[-1.0, 0.0, 0.0, 0.35],
         [0.0, 1.0, 0.0, -0.13],
         [0.0, 0.0, -1.0, 0.93],
         [0.0, 0.0, 0.0, 1.0]]
    )
    ik_controller = BaseIKController(visualize=False)
    ik_controller.move_to_pose(base_t_right_eef, "R")
    # width = 5
    # height = 5
    # for i in range(width):
    #     for j in range(height):
    #         base_t_right_eef = np.array(
    #             [[-1.0, 0.0, 0.0, 0.35 + j * check_board_size],
    #              [0.0, 1.0, 0.0, -0.28 + i * check_board_size],
    #              [0.0, 0.0, -1.0, 0.93],
    #              [0.0, 0.0, 0.0, 1.0]]
    #         )
    #         ik_controller.move_to_pose(base_t_right_eef, "R")
    #         time.sleep(5)


if __name__ == "__main__":
    tyro.cli(main)