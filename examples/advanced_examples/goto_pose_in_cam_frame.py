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
import warnings
import threading
import cv2
import time

import tyro
import numpy as np
import pytransform3d.rotations as pr
from pytransform3d.rotations import check_matrix
from dexmotion.motion_manager import MotionManager
from dexmotion.utils import robot_utils
from loguru import logger

import pyzed.sl as sl
from dexcontrol.robot import Robot
# from yixuan_utilities.kinematics_helper import KinHelper


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
        pos_t = min(step_pos * i / (rel_pos + 1e-6), 1.0)
        rot_t = min(step_rot * i / (rel_rot + 1e-6), 1.0)
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

    def __init__(self, bot: Robot | None = None, visualize: bool = True, use_custom_kin_helper: bool = True):
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
        
        head_pos = np.array([-0.01, -0.02078687, 0.00514872])
        self.bot.head.set_joint_pos(head_pos, wait_time=2.0, exit_on_reach=True)

        self.visualize = visualize

        self.arm_dof = 7  # Number of degrees of freedom in the arms
        self.use_custom_kin_helper = use_custom_kin_helper
        if use_custom_kin_helper:
            from yixuan_utilities.kinematics_helper import KinHelper
            self.kin_helper = KinHelper("vega_with_robotiq")
        else:
            self.kin_helper = None

        # Get initial joint positions
        try:
            joint_pos_dict = self.bot.get_joint_pos_dict(
                component=["left_arm", "right_arm", "torso", "head"]
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
            self.fk_solver = MotionManager(
                robot_type="vega-1",
                init_visualizer=self.visualize,
                initial_joint_configuration_dict=joint_pos_dict,
                joints_to_lock=[],
                apply_locked_joints=False,
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
    
    def get_sapien_qpos(self) -> np.ndarray:
        """Get joint positions compatiable with sapien convention
        
        Returns:
            np.ndarray: joint positions compatiable with sapien convention
        """
        left_arm_qpos = self.bot.left_arm.get_joint_pos()
        right_arm_qpos = self.bot.right_arm.get_joint_pos()
        torso_qpos = self.bot.torso.get_joint_pos()
        head_qpos = self.bot.head.get_joint_pos()
        wheel_qpos = np.zeros(6)
        left_joint_names = [f"L_arm_j{i+1}" for i in range(7)]
        right_joint_names = [f"R_arm_j{i+1}" for i in range(7)]
        torso_joint_names = [f"torso_j{i+1}" for i in range(3)]
        head_joint_names = [f"head_j{i+1}" for i in range(3)]
        wheel_joint_names = [f"wheel_j{i+1}" for i in range(6)]
        all_qpos = np.concatenate([left_arm_qpos, right_arm_qpos, torso_qpos, head_qpos, wheel_qpos])
        all_joint_names = left_joint_names + right_joint_names + torso_joint_names + head_joint_names + wheel_joint_names
        sapien_qpos = self.kin_helper.convert_to_sapien_joint_order(all_qpos, all_joint_names)
        return sapien_qpos

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


        # get the current pose
        if self.use_custom_kin_helper:
            sapien_qpos = self.get_sapien_qpos()
            curr_poses_dict = self.kin_helper.compute_fk_from_link_names(sapien_qpos, [f"{arm_side}_robotiq"])
            base_t_curr_pose = curr_poses_dict[f"{arm_side}_robotiq"]
        else:
            current_qpos = self.motion_manager.get_joint_pos()
            assert isinstance(current_qpos, np.ndarray)

            curr_poses_dict = self.motion_manager.fk(
                frame_names=self.motion_manager.target_frames,
                qpos=current_qpos,
                update_robot_state=False,
            )
            base_t_curr_pose = curr_poses_dict[arm_side + "_ee"].np

        # interpolate the poses
        trajs = interpolate_poses(base_t_curr_pose, target_pose)
        # hack to make the final alignment work
        trajs = np.concatenate([trajs, np.tile(target_pose[None, :, :], (10, 1, 1))])
        for pose in trajs:
            if self.use_custom_kin_helper:
                active_qmask = np.zeros(self.kin_helper.sapien_robot.dof, dtype=bool)
                for j_idx, joint in enumerate(self.kin_helper.sapien_robot.get_active_joints()):
                    if f"{arm_side}_arm_" in joint.name:
                        active_qmask[j_idx] = True
                eef_idx = -1
                for l_idx, link in enumerate(self.kin_helper.sapien_robot.get_links()):
                    if link.name == f"{arm_side}_robotiq":
                        eef_idx = l_idx
                        break
                curr_qpos = self.get_sapien_qpos()
                qpos = self.kin_helper.compute_ik_from_mat(curr_qpos, pose, active_qmask=active_qmask, eef_idx=eef_idx)
                qval = qpos[active_qmask]
            else:
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
                
                qpos_dict, is_in_collision, is_within_joint_limits = self.motion_manager.ik(target_pose=target_poses_dict)
                qval = np.zeros(7)
                for i in range(7):
                    qval[i] = qpos_dict[f'{arm_side}_arm_j{i+1}']
                self.motion_manager.set_joint_pos(qpos_dict)
            
            if arm_side == "R":
                self.bot.right_arm.set_joint_pos(qval)
            elif arm_side == "L":
                self.bot.left_arm.set_joint_pos(qval)
            time.sleep(0.01)


def find_charuco_board_corners(img: np.ndarray) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(
        image=img,
        dictionary=dictionary,
        parameters=None,
    )
    return corners, ids

class Camera:
    def __init__(self):
        self.zed = sl.Camera()

    def open(self) -> None:
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.AUTO # Use HD720 or HD1200 video mode (default fps: 60)
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP # Use a right-handed Y-up coordinate system
        init_params.coordinate_units = sl.UNIT.METER  # Set units in meters
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL

        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print("Camera Open : "+repr(err)+". Exit program.")
            exit()
    
    def get_obs(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        runtime_parameters = sl.RuntimeParameters()
        cam_info = self.zed.get_camera_information()
        calib = cam_info.camera_configuration.calibration_parameters  # rectified pair

        left_K = np.array([[calib.left_cam.fx, 0, calib.left_cam.cx],
                           [0, calib.left_cam.fy, calib.left_cam.cy],
                           [0, 0, 1]], dtype=np.float32)

        left_rgb_sl = sl.Mat()
        depth_sl = sl.Mat()
        confidence_sl = sl.Mat()
        if self.zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Copy the newly arrived RGBD frame
            self.zed.retrieve_image(left_rgb_sl, sl.VIEW.LEFT)   # BGRA by default
            self.zed.retrieve_measure(depth_sl, sl.MEASURE.DEPTH)  # float32 meters
            self.zed.retrieve_measure(confidence_sl, sl.MEASURE.CONFIDENCE)  # float32 meters
            left_rgba = left_rgb_sl.get_data()     # HxWx4 uint8, LEFT
            depth   = depth_sl.get_data()        # HxW float32, aligned to LEFT
            confidence = confidence_sl.get_data().astype(np.uint8)
            left_rgb_np  = left_rgba[:, :, :3][:, :, ::-1]
            return left_rgb_np.copy(), depth.copy(), confidence.copy(), left_K.copy()


def draw_corners(img: np.ndarray, corners: np.ndarray) -> np.ndarray:
    for corner in corners:
        corner = corner.astype(np.int32)
        cv2.circle(img, corner, 5, (0, 255, 0), -1)
    return img

def main() -> None:
    # Initialize Robot
    ik_controller = BaseIKController(visualize=False, use_custom_kin_helper=False)
    
    # open camera and detect corners
    cam = Camera()
    cam.open()
    while True:
        rgb, depth, confidence, K = cam.get_obs()
        if depth[~np.isnan(depth) & ~np.isinf(depth)].max() > 0:
            break
        cv2.imwrite('tmp.png', rgb)
    corners, ids = find_charuco_board_corners(rgb)  # (col, row)
    if len(corners) == 0:
        warnings.warn('no markers detected')
        return
    
    # get the positions of all corners in the camera frame
    corners = np.concatenate(corners, axis=0)  # (B, N, 2)
    corners = corners.reshape(-1, 2)  # (N, 2)
    
    vis_img = draw_corners(rgb.copy(), corners)
    cv2.imwrite('tmp_vis.png', vis_img)
    
    tgt_img = draw_corners(rgb.copy(), corners[0:1])
    cv2.imwrite('tmp_tgt.png', tgt_img)
    
    corners_z = depth[corners[:, 1].astype(int), corners[:, 0].astype(int)]  # (N,)
    corners_x = (corners[:, 0] - K[0, 2]) * corners_z / K[0, 0]
    corners_y = (corners[:, 1] - K[1, 2]) * corners_z / K[1, 1]
    cam_t_corners = np.concatenate([corners_x[:, None], corners_y[:, None], corners_z[:, None]], axis=1)
    
    # FK
    joint_pos_dict = ik_controller.bot.get_joint_pos_dict(["left_arm", "right_arm", "torso", "head"])
    ik_controller.fk_solver.set_joint_pos(joint_pos_dict)
    base_t_cam = ik_controller.fk_solver.fk(["zed_depth_frame"])["zed_depth_frame"].np

    base_t_corners = base_t_cam @ np.concatenate(
        [cam_t_corners.T, np.ones((1, cam_t_corners.shape[0]))], axis=0
    )  # (4, N)
    base_t_corners = base_t_corners[:3, :].T  # (N, 3)
    
    base_t_right_eef = np.array(
        [[-1.0, 0.0, 0.0, 0.0],
         [0.0, 1.0, 0.0, 0.0],
         [0.0, 0.0, -1.0, 0.0],
         [0.0, 0.0, 0.0, 1.0]]
    )
    base_t_right_eef[:3, 3] = base_t_corners[0]
    base_t_right_eef[2, 3] += 0.25  # lift up 30cm
    
    print("Target Position: ", base_t_right_eef)
    
    reset_pose = base_t_right_eef.copy()
    reset_pose[2, 3] = 0.99
    
    # Resets height if target height is too low.
    MIN_HEIGHT = 0.933
    if base_t_right_eef[2, 3] < MIN_HEIGHT:
        logger.warning(f"Target height {base_t_right_eef[2, 3]:.3f}m is below minimum {MIN_HEIGHT}m. Clamping to minimum.")
        ik_controller.move_to_pose(reset_pose, "R")
        
    print("Target Position adjusted: ", base_t_right_eef)
    
    ik_controller.move_to_pose(base_t_right_eef, "R")


if __name__ == "__main__":
    tyro.cli(main)
