# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Operating system utility functions."""

import os
import re
from typing import Final

from dexcontrol.utils.constants import ROBOT_NAME_ENV_VAR


def resolve_key_name(key: str) -> str:
    """Resolves a key name for zenoh topic by prepending robot name.

    Args:
        key: Original key name (e.g. 'lidar' or '/lidar')

    Returns:
        Resolved key with robot name prepended (e.g. 'robot/lidar')
    """
    # Get robot name from env var or use default
    robot_name: Final[str] = os.getenv(ROBOT_NAME_ENV_VAR, "robot")

    # Remove leading slash if present
    key = key.lstrip("/")

    # Check if robot name is already present at the beginning
    if key.startswith(f"{robot_name}/"):
        return key

    # Combine robot name and key with single slash
    return f"{robot_name}/{key}"


def get_robot_model() -> str:
    """Get the robot model from the environment variable."""
    robot_model_abb_mapping = dict(vg="vega")
    robot_name = os.getenv(ROBOT_NAME_ENV_VAR)
    if robot_name is None:
        raise ValueError(
            f"Robot name is not set, please set the environment variable {ROBOT_NAME_ENV_VAR}"
        )
    if not re.match(r"^dm/[a-zA-Z0-9]{12}-(?:\d+|rc\d+)$", robot_name):
        raise ValueError(f"Robot name is not in the correct format: {robot_name}")

    robot_model_abb = robot_name.split("/")[-1].split("-")[0][:2]
    if robot_model_abb not in robot_model_abb_mapping:
        raise ValueError(f"Unknown robot model: {robot_model_abb}")
    model = robot_model_abb_mapping[robot_model_abb] + "-" + robot_name.split("-")[-1]
    return model
