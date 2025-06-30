# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""This example demonstrates how to retrieve and display chassis camera data.

The script initializes the robot with all chassis cameras enabled, captures an
image from each one, and displays them all in a single tiled window.
"""

import math
import time
from typing import List

import matplotlib.pyplot as plt
import numpy as np
import tyro
from loguru import logger

from dexcontrol.config.vega import get_vega_config
from dexcontrol.robot import Robot


def display_tiled_cameras(images: List[np.ndarray], titles: List[str]) -> None:
    """Displays multiple images in a tiled matplotlib figure.

    Args:
        images: A list of NumPy arrays, where each array is an image.
        titles: A list of titles corresponding to each image.

    Raises:
        ValueError: If the list of images is empty.
    """
    if not images:
        raise ValueError("No images were provided for display.")

    num_images = len(images)
    cols = math.ceil(math.sqrt(num_images))
    rows = math.ceil(num_images / cols)

    fig, axes = plt.subplots(rows, cols, figsize=(15, 10), squeeze=False)
    fig.suptitle("Chassis Camera Feeds", fontsize=16)

    for i, (img, title) in enumerate(zip(images, titles)):
        ax = axes[i // cols, i % cols]
        ax.imshow(img)
        ax.set_title(title)
        ax.axis("off")

    # Hide any unused subplots
    for i in range(num_images, rows * cols):
        axes[i // cols, i % cols].axis("off")

    plt.tight_layout()
    plt.show()


def main() -> None:
    """Initializes the robot, retrieves chassis camera data, and displays it."""
    configs = get_vega_config()

    # Define chassis cameras to be used in this example.
    camera_definitions = {
        "base_left_camera": "Left Camera",
        "base_right_camera": "Right Camera",
        "base_front_camera": "Front Camera",
        "base_back_camera": "Back Camera",
    }

    # Enable all defined cameras in the configuration.
    for camera_name in camera_definitions:
        if hasattr(configs.sensors, camera_name):
            getattr(configs.sensors, camera_name).enable = True
            logger.info(f"Enabled '{camera_name}'.")
        else:
            logger.warning(f"Camera '{camera_name}' not found in configs.")

    robot = Robot(configs=configs)

    # Wait a moment for camera streams to stabilize.
    time.sleep(3)

    images, titles = [], []
    for camera_name, display_name in camera_definitions.items():
        try:
            camera_sensor = getattr(robot.sensors, camera_name, None)
            if camera_sensor and camera_sensor.is_active():
                image = camera_sensor.get_obs()
                if image is not None:
                    images.append(image)
                    titles.append(display_name)
                    logger.info(f"Retrieved '{display_name}' with shape: {image.shape}")
                else:
                    logger.warning(f"Failed to get image from '{display_name}'.")
            else:
                logger.warning(
                    f"Camera sensor '{display_name}' is not available or not active."
                )
        except Exception as e:
            logger.error(
                f"Error getting data from '{display_name}': {e}",
                exc_info=True,
            )

    try:
        if images:
            logger.info(f"Displaying {len(images)} camera feeds...")
            display_tiled_cameras(images, titles)
        else:
            logger.error("No camera data was retrieved. Nothing to display.")
    except ValueError as e:
        logger.error(f"Error displaying images: {e}")
    finally:
        logger.info("Shutting down the robot.")
        robot.shutdown()


if __name__ == "__main__":
    tyro.cli(main)
