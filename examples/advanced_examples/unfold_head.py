# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Example script demonstrating folding and unfolding robot poses.

This script shows how to safely move the robot between folded and unfolded
configurations using predefined poses. It includes safety checks and proper
error handling.
"""

import os

import tyro
from loguru import logger

from dexcontrol.robot import Robot


def is_running_remote() -> bool:
    """Check if the script is running in a remote environment.

    Returns:
        True if running in a remote environment (no display), False otherwise.
    """
    return "DISPLAY" not in os.environ


def unfold_robot() -> None:
    """Unfold the robot from folded position to operational position.

    Raises:
        RuntimeError: If torso or arms fail to reach target positions.
    """
    with Robot() as bot:
        # Set head to home position
        head_home_pose = bot.compensate_torso_pitch(
            bot.head.get_predefined_pose("home"), "head"
        )
        bot.head.set_joint_pos(head_home_pose, wait_time=2.0, exit_on_reach=True)
        logger.info("Robot is unfolded!")


def main() -> None:
    """Execute robot folding or unfolding sequence.
    """
    unfold_robot()

if __name__ == "__main__":
    tyro.cli(main)
