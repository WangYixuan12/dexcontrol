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

import time
from dexcontrol.robot import Robot


def main():
    bot = Robot()
    right_arm = bot.right_arm

    # activate hand
    right_hand_data = bytes.fromhex(
        "09 10 03 E8 00 03 06 00 00 00 00 00 00 73 30"
    )

    # Send the command via pass-through
    right_arm.send_ee_pass_through_message(right_hand_data)
    time.sleep(0.05)
    
    right_hand_data = bytes.fromhex(
        "09 10 03 E8 00 03 06 01 00 00 00 00 00 72 E1"
    )
    right_arm.send_ee_pass_through_message(right_hand_data)
    time.sleep(0.5)
    
    while True:
        print("closing hand")
        right_hand_data = bytes.fromhex(
            "09 10 03 E8 00 03 06 09 00 00 FF FF FF 42 29"
        )
        right_arm.send_ee_pass_through_message(right_hand_data)
        time.sleep(1.0)
        
        print("opening hand")
        right_hand_data = bytes.fromhex(
            "09 10 03 E8 00 03 06 09 00 00 00 FF FF 72 19"
        )
        right_arm.send_ee_pass_through_message(right_hand_data)
        time.sleep(1.0)

    # Optionally, shutdown the robot after use
    bot.shutdown()


if __name__ == "__main__":
    main()
