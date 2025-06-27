# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai


import numpy as np
import ruckig
from jaxtyping import Float


class ArmOfflineTrajectorySmoother:
    """Class to generate smooth trajectories that respect joint limits."""

    def __init__(
        self,
        dt: float = 0.01,
        safety_factor: float = 2.0,
    ) -> None:
        """Initialize trajectory smoother with motion constraints.

        Args:
            dt: Control cycle time in seconds. Must be positive.
            safety_factor: Factor to scale down motion limits for additional safety.
                         Must be positive. Larger values mean more conservative motion.

        Raises:
            ValueError: If dt or safety_factor is not positive.
        """
        if dt <= 0:
            raise ValueError("Control cycle must be positive")
        if safety_factor <= 0:
            raise ValueError("Safety factor must be positive")

        self.arm_dof = 7  # 7 degrees of freedom for each arm
        self.dt = dt

        # Initialize Ruckig Online Trajectory Generation
        self.otg = ruckig.Ruckig(self.arm_dof, dt)
        self.inp = ruckig.InputParameter(self.arm_dof)
        self.out = ruckig.OutputParameter(self.arm_dof)

        # Set motion limits for each joint (in radians)
        # Convert degrees to radians and apply safety factor
        self.inp.max_velocity = (
            np.deg2rad([180, 180, 220, 220, 220, 220, 220]) / safety_factor
        )
        self.inp.max_acceleration = (
            np.deg2rad([600, 600, 600, 600, 600, 600, 600]) / safety_factor
        )
        self.inp.max_jerk = (
            np.deg2rad([6000, 6000, 6000, 6000, 6000, 6000, 6000]) / safety_factor
        )

    def smooth_trajectory(
        self, waypoints: Float[np.ndarray, "N 7"], trajectory_dt: float
    ) -> tuple[Float[np.ndarray, "M 7"], Float[np.ndarray, "M 7"]]:
        """Generate time-optimal smooth trajectory through waypoints.

        Args:
            waypoints: Array of joint positions with shape (N, 7) where N is the number
                      of waypoints and 7 is the number of joints.
            trajectory_dt: Desired time duration between waypoints (unit: s).

        Returns:
            A tuple containing:
                - Position trajectory with shape (M, 7)
                - Velocity trajectory with shape (M, 7)

        Raises:
            ValueError: If fewer than 2 waypoints are provided or if they don't match
                       the expected 7-DOF format.
        """
        if len(waypoints) < 2:
            raise ValueError("At least two waypoints are required")
        if waypoints.shape[1] != self.arm_dof:
            raise ValueError(f"Waypoints must have shape (N, {self.arm_dof})")

        pos_traj = []
        vel_traj = []
        n_intermediate_points = int(trajectory_dt / self.dt)
        self.inp.current_position = waypoints[0]

        # Generate smooth trajectory segments between waypoints
        for i in range(1, len(waypoints)):
            self.inp.target_position = waypoints[i]
            self.inp.target_velocity = [0.0] * self.arm_dof

            for _ in range(n_intermediate_points):
                _ = self.otg.update(self.inp, self.out)
                pos_traj.append(self.out.new_position)
                vel_traj.append(self.out.new_velocity)
                self.out.pass_to_input(self.inp)

        return np.array(pos_traj), np.array(vel_traj)


class ArmOnlineTrajectorySmoother:
    def __init__(
        self,
        init_qpos: np.ndarray,
        control_cycle: float = 0.005,
        safety_factor: float = 2.0,
    ) -> None:
        self.dof = len(init_qpos)

        # Initialize Ruckig
        self.otg = ruckig.Ruckig(self.dof, control_cycle)
        self.inp = ruckig.InputParameter(self.dof)
        self.out = ruckig.OutputParameter(self.dof)
        self.inp.current_position = init_qpos

        # Set motion limits
        self.inp.max_velocity = (
            np.deg2rad([180, 180, 220, 220, 220, 220, 220]) / safety_factor
        )
        self.inp.max_acceleration = (
            np.deg2rad([600, 600, 600, 600, 600, 600, 600]) / safety_factor
        )
        self.inp.max_jerk = (
            np.deg2rad([6000, 6000, 6000, 6000, 6000, 6000, 6000]) / safety_factor
        )

    def update(
        self, target_position: np.ndarray | None = None
    ) -> tuple[np.ndarray, np.ndarray]:
        """Update trajectory with new target position."""
        if target_position is not None:
            self.inp.target_position = target_position
            self.inp.target_velocity = [0.0] * self.dof

        _ = self.otg.update(self.inp, self.out)
        self.out.pass_to_input(self.inp)

        return np.array(self.out.new_position), np.array(self.out.new_velocity)

    def reset(self, init_qpos: np.ndarray):
        self.inp.current_position = init_qpos
        self.inp.current_velocity = [0.0] * self.dof


def linear_interpolation(
    start: Float[np.ndarray, " DoF"],
    end: Float[np.ndarray, " DoF"],
    steps: int,
) -> Float[np.ndarray, "steps DoF"]:
    """Generate linear interpolation between start and end joint positions.

    Args:
        start: Starting joint positions array of shape (DoF,)
        end: Ending joint positions array of shape (DoF,)
        steps: Number of interpolation steps (including start and end points)

    Returns:
        Array of interpolated joint positions with shape (steps, DoF)
        where DoF is the degrees of freedom (matches input dimensions)
    """
    t = np.linspace(0, 1, steps)[:, None]
    return (1 - t) * start + t * end


def plan_arm_move_to_target(
    init_qpos: Float[np.ndarray, "7"],
    target_qpos: Float[np.ndarray, "7"],
    max_qv: float,
    dt: float = 0.01,
) -> tuple[Float[np.ndarray, "steps 7"], Float[np.ndarray, "steps 7"]]:
    """Plan a smooth arm movement from initial to target joint positions.

    Args:
        init_qpos: Initial joint positions array of shape (7,)
        target_qpos: Target joint positions array of shape (7,)
        max_qv: Maximum allowed joint velocity magnitude (radians/second)
        dt: Control cycle time in seconds (default: 0.01)

    Returns:
        A tuple containing:
            - Position trajectory with shape (steps, 7)
            - Velocity trajectory with shape (steps, 7)
            Both trajectories include the final target state.
    """
    duration = np.linalg.norm(target_qpos - init_qpos) / max_qv
    steps = int(duration / dt)
    trajectory = linear_interpolation(init_qpos, target_qpos, steps)

    # Smooth the trajectory while respecting motion constraints
    smoother = ArmOfflineTrajectorySmoother(dt=dt)
    pos_traj, vel_traj = smoother.smooth_trajectory(trajectory, dt)

    # Append final target state with zero velocity
    pos_traj = np.concatenate([pos_traj, target_qpos[None, :]])
    vel_traj = np.concatenate([vel_traj, np.zeros((1, 7))])

    return pos_traj, vel_traj
