# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Example script to display force/torque sensor readings."""

import time

import numpy as np
import tyro

from dexcontrol.robot import Robot


def print_wrench_data(bot, arm_side: str) -> None:
    """Print current force/torque sensor values."""
    arm = bot.left_arm if arm_side == "left" else bot.right_arm

    if arm.wrench_sensor is None:
        return

    wrench = arm.wrench_sensor.get_wrench_state()
    components = ["fx", "fy", "fz", "mx", "my", "mz"]
    units = ["N"] * 3 + ["Nm"] * 3

    print(f"\n{arm_side.upper()} ARM:")
    for val, comp, unit in zip(wrench, components, units):
        print(f"{comp}: {val:.4f} {unit}")
    print("\n" + "-" * 50)


def main(arm_side: str = "left") -> None:
    """Display force/torque sensor information."""
    bot = Robot()
    np.set_printoptions(precision=3)

    try:
        while True:
            print_wrench_data(bot, arm_side)
            time.sleep(0.05)
    except KeyboardInterrupt:
        bot.shutdown()


if __name__ == "__main__":
    tyro.cli(main)
