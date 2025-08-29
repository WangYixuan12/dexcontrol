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

from dexcontrol.config.vega import get_vega_config
from dexcontrol.robot import Robot


def main() -> None:
    """Display robot system information including version, status and battery.

    Queries the robot for software version, component status, and battery
    information and displays the results.
    """
    # Initialize robot with default configuration
    configs = get_vega_config()
    # temporarily disable estop checking so that even if the robot is in estop,
    # we can still get the information
    # DO NOT DO THIS IN OTHER CASES
    configs.estop.enabled = False
    bot = Robot(configs=configs)

    # Display robot system information
    bot.get_version_info(show=True)
    bot.get_component_status(show=True)
    bot.estop.show()
    bot.heartbeat.show()
    bot.battery.show()
    bot.shutdown()


if __name__ == "__main__":
    main()
