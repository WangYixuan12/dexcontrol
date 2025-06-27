# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Trajectory utility functions for smooth motion generation."""

import numpy as np


def generate_linear_trajectory(
    current_pos: np.ndarray,
    target_pos: np.ndarray,
    max_vel: float = 0.5,
    control_hz: float = 100,
) -> tuple[np.ndarray, int]:
    """Generate a linear trajectory between current and target positions.

    Args:
        current_pos: Current position array.
        target_pos: Target position array.
        max_vel: Maximum velocity in units per second.
        control_hz: Control frequency in Hz.

    Returns:
        Tuple containing:
        - trajectory: Array of waypoints from current to target position.
        - num_steps: Number of steps in the trajectory.
    """
    # Calculate linear interpolation between current and target positions
    max_diff = np.max(np.abs(target_pos - current_pos))
    num_steps = int(max_diff / max_vel * control_hz)

    # Ensure at least one step
    num_steps = max(1, num_steps)

    # Generate trajectory with endpoints (exclude the starting point in the return)
    trajectory = np.linspace(current_pos, target_pos, num_steps + 1, endpoint=True)[1:]

    return trajectory, num_steps
