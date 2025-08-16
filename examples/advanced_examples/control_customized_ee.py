# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Example script demonstrating customized end effector control via RS485 pass-through.

This script shows how to control a custom end effector (e.g., gripper, tool) by sending
raw RS485 commands through the robot arm's pass-through mode.
"""

from dexcontrol.robot import Robot


def main():
    bot = Robot()
    left_arm = bot.left_arm
    right_arm = bot.right_arm

    # Example: RS485 command to send (replace with your actual protocol)
    # For example, to open the default hand installed on vega robot
    left_hand_data = bytes.fromhex(
        "34 10 04 6F 00 06 0C 00 00 00 00 FF FF FF FF FF FF FF FF E7 86"
    )
    right_hand_data = bytes.fromhex(
        "AB 10 04 6F 00 06 0C 00 00 00 00 FF FF FF FF FF FF FF FF 5D 8E"
    )

    # Send the command via pass-through
    left_arm.send_ee_pass_through_message(left_hand_data)
    right_arm.send_ee_pass_through_message(right_hand_data)

    # Optionally, shutdown the robot after use
    bot.shutdown()


if __name__ == "__main__":
    main()
