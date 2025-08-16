# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Example script to display robot system information.

This script demonstrates how to query and display various robot status information
including software version, component status, and battery level.
"""

from typing import Literal

import tyro
from loguru import logger

from dexcontrol.robot import Robot


def main(part: Literal["arm", "chassis", "torso"]) -> None:
    """Reboot part of the robot.

    Args:
        part: Part of the robot to reboot.
    """
    # Initialize robot with default configuration
    bot = Robot()
    logger.info(f"Rebooting {part}...")
    bot.reboot_component(part)

    # Display robot system information
    bot.shutdown()


if __name__ == "__main__":
    tyro.cli(main)
