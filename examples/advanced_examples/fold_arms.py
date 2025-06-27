# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Example script for folding robot arms to a predefined pose.

This script demonstrates how to move both robot arms to a folded position
with an option to compensate for torso pitch. It includes safety prompts
and proper shutdown procedures.
"""

import tyro
from loguru import logger

from dexcontrol.robot import Robot


def main(comp_pitch: bool = False) -> None:
    """Moves both robot arms to the folded position.

    Args:
        comp_pitch: Whether to compensate for torso pitch angle.
    """
    # Safety confirmation
    logger.warning(
        "Warning: Be ready to press e-stop if needed! "
        "This example does not check for self-collisions. "
        "Highly recommend to first run init_arm_safe.py to make sure the arms start from a safe pose."
    )
    logger.warning("Please ensure the arms have sufficient space to move.")
    if input("Continue? [y/N]: ").lower() != "y":
        return

    target_pose = "folded"

    with Robot() as bot:
        logger.info("Moving both arms to folded position")

        if comp_pitch:
            # Compensate for torso pitch and move arms
            torso_pitch = bot.torso.pitch_angle
            logger.debug(f"Current torso pitch: {torso_pitch:.4f} rad")

            left_arm_target_pose = bot.compensate_torso_pitch(
                bot.left_arm.get_predefined_pose(target_pose),
                "left_arm",
            )
            right_arm_target_pose = bot.compensate_torso_pitch(
                bot.right_arm.get_predefined_pose(target_pose),
                "right_arm",
            )

            # Move both arms simultaneously
            bot.set_joint_pos(
                {
                    "left_arm": left_arm_target_pose,
                    "right_arm": right_arm_target_pose,
                },
                wait_time=5.0,
            )
        else:
            # Move both arms to folded position without pitch compensation
            bot.set_joint_pos(
                {
                    "left_arm": bot.left_arm.get_predefined_pose(target_pose),
                    "right_arm": bot.right_arm.get_predefined_pose(target_pose),
                },
                wait_time=5.0,
            )

        logger.info("Arms successfully moved to folded position")


if __name__ == "__main__":
    tyro.cli(main)
