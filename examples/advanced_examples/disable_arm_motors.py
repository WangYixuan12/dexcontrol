# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Example script to disable arm motors with brake released or not.

This script demonstrates how to disable arm motors with brake released or not.
"""

from typing import Literal, cast

import tyro

from dexcontrol.robot import Robot


def main(
    side: Literal["left", "right", "both"],
    joint_idx: list[int],
    release_brake: bool = False,
) -> None:
    """Disable arm motors with brake released or not.

    Args:
        side: Side of the arm to disable.
        joint_idx: Joint indices to disable.
        release_brake: Whether to release the brake.
    """
    bot = Robot()
    mode_name = "release" if release_brake else "disable"
    modes = ["position"] * 7
    for idx in joint_idx:
        modes[idx] = mode_name

    modes = cast(list[Literal["position", "disable", "release"]], modes)
    if side == "left":
        bot.left_arm.set_modes(modes)
    elif side == "right":
        bot.right_arm.set_modes(modes)
    elif side == "both":
        bot.left_arm.set_modes(modes)
        bot.right_arm.set_modes(modes)


if __name__ == "__main__":
    tyro.cli(main)
