# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Example script to read head camera data.

This script demonstrates how to retrieve RGB and depth images along with their
intrinsic parameters from the robot's head camera. It initializes a robot instance,
reads the camera data, logs the information, and displays all camera feeds in a
single tiled window using matplotlib.
"""

import math
import time

import matplotlib.pyplot as plt
import tyro
from loguru import logger

from dexcontrol.config.vega import get_vega_config
from dexcontrol.robot import Robot


def display_tiled_cameras(images, titles):
    """Display multiple camera images in a tiled matplotlib figure.

    Args:
        images: List of numpy arrays representing images
        titles: List of strings for image titles
    """
    if not images:
        logger.error("No images to display")
        return

    num_images = len(images)

    # Calculate grid dimensions (try to make it as square as possible)
    cols = math.ceil(math.sqrt(num_images))
    rows = math.ceil(num_images / cols)

    # Create matplotlib figure
    fig = plt.figure(figsize=(15, 10))
    fig.suptitle("Camera Feeds", fontsize=16)

    # Display each image using subplot
    for i, (img, title) in enumerate(zip(images, titles)):
        ax = fig.add_subplot(rows, cols, i + 1)

        # Assume the image is in RGB format from the robot
        ax.imshow(img)
        ax.set_title(title)
        ax.axis("off")  # Remove axis ticks and labels

    plt.tight_layout()
    plt.show()


def main():
    """Gets and displays camera data in a tiled window.

    Creates a robot instance and retrieves images from all available cameras,
    then displays them in a single tiled window using matplotlib.
    """
    # Initialize robot with cameras enabled
    configs = get_vega_config()
    configs.sensors.base_left_camera.enable = True
    configs.sensors.base_right_camera.enable = True
    configs.sensors.base_front_camera.enable = True
    configs.sensors.base_back_camera.enable = True
    robot = Robot(configs=configs)
    time.sleep(3)

    # Collect camera data from all available cameras
    camera_data = []
    camera_names = []

    # Try to get data from each camera
    cameras = [
        ("base_left_camera", "Left Camera"),
        ("base_right_camera", "Right Camera"),
        ("base_front_camera", "Front Camera"),
        ("base_back_camera", "Back Camera"),
    ]

    for camera_attr, display_name in cameras:
        try:
            camera = getattr(robot.sensors, camera_attr, None)
            if camera is not None:
                data = camera.get_obs()
                if data is not None:
                    # Keep data in RGB format for matplotlib
                    camera_data.append(data)
                    camera_names.append(display_name)
                    logger.info(f"{display_name} data shape: {data.shape}")
        except Exception as e:
            logger.warning(f"Could not get data from {display_name}: {e}")

    if not camera_data:
        logger.error("No camera data available")
        robot.shutdown()
        return

    logger.info(f"Displaying {len(camera_data)} camera feeds")

    # Display the camera feeds using matplotlib
    display_tiled_cameras(camera_data, camera_names)

    robot.shutdown()


if __name__ == "__main__":
    tyro.cli(main)
