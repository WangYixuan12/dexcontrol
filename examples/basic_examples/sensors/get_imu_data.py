# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Example script to read IMU data.

This script demonstrates how to retrieve inertial measurement data from the robot's
base IMU sensor. It initializes a robot instance, reads the IMU data including
acceleration, angular velocity, magnetometer, and orientation, logs the information,
and performs a clean shutdown.
"""

import numpy as np
import tyro
from loguru import logger

from dexcontrol.config.vega import get_vega_config
from dexcontrol.robot import Robot


def main():
    """Gets and logs IMU data from the robot's base IMU sensor.

    Creates a robot instance and retrieves inertial measurement data including
    linear acceleration, angular velocity, magnetometer readings, and orientation.

    Returns:
        Dict containing IMU data if successful, None otherwise.
    """
    # Initialize robot
    configs = get_vega_config()
    configs.sensors.head_imu.enable = True
    configs.sensors.base_imu.enable = True
    with Robot(configs=configs) as robot:
        # Get IMU data
        head_imu_data = robot.sensors.head_imu.get_obs()

        if head_imu_data is not None:
            logger.info("IMU Data Retrieved:")
            for key, value in head_imu_data.items():
                logger.info(f"  {key}: {np.round(value, 3)}")

        else:
            logger.warning("No IMU data available")

        base_imu_data = robot.sensors.base_imu.get_obs()
        if base_imu_data is not None:
            logger.info("Base IMU Data Retrieved:")
            for key, value in base_imu_data.items():
                logger.info(f"  {key}: {np.round(value, 3)}")

        else:
            logger.warning("No Base IMU data available")


if __name__ == "__main__":
    tyro.cli(main)
