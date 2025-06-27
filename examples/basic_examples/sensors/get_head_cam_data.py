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

import json
import math
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import tyro
from loguru import logger

from dexcontrol.config.vega import get_vega_config
from dexcontrol.robot import Robot


def display_tiled_cameras(images: list[np.ndarray], titles: list[str]) -> None:
    """Display multiple camera images in a tiled matplotlib figure.

    Args:
        images: List of numpy arrays representing images.
        titles: List of strings for image titles.

    Raises:
        ValueError: If no images are provided for display.
    """
    if not images:
        raise ValueError("No images provided for display")

    num_images = len(images)

    # Calculate grid dimensions (try to make it as square as possible)
    cols = math.ceil(math.sqrt(num_images))
    rows = math.ceil(num_images / cols)

    # Create matplotlib figure
    fig = plt.figure(figsize=(15, 10))
    fig.suptitle("Head Camera Feeds", fontsize=16)

    # Display each image using subplot
    for i, (img, title) in enumerate(zip(images, titles)):
        ax = fig.add_subplot(rows, cols, i + 1)

        # Handle different image types (RGB vs depth)
        if "depth" in title.lower():
            # Display depth images with a colormap
            ax.imshow(img, cmap="viridis")
        else:
            # Assume RGB format for color images
            ax.imshow(img)

        ax.set_title(title)
        ax.axis("off")  # Remove axis ticks and labels

    plt.tight_layout()
    plt.show()


def _print_camera_info(camera_info: dict[str, Any] | None) -> None:
    """Print camera information in a formatted way.

    Args:
        camera_info: Dictionary containing camera information, or None.
    """
    if camera_info:
        print("\n" + "=" * 50)
        print("CAMERA INFORMATION")
        print("=" * 50)
        print(json.dumps(camera_info, indent=2))
        print("=" * 50 + "\n")
    else:
        logger.warning("No camera info available")


def _extract_camera_images(
    camera_data: dict[str, np.ndarray],
) -> tuple[list[np.ndarray], list[str]]:
    """Extract valid images and titles from camera data.

    Args:
        camera_data: Dictionary containing camera data with keys as identifiers
            and values as numpy arrays.

    Returns:
        A tuple containing:
            - List of valid image arrays
            - List of corresponding titles

    Raises:
        ValueError: If no valid camera images are available.
    """
    images = []
    titles = []

    for key, data in camera_data.items():
        if data is not None:
            images.append(data)
            titles.append(key.replace("_", " ").title())
            logger.info(f"{key} data shape: {data.shape}")

    if not images:
        raise ValueError("No valid camera images available")

    return images, titles


def get_head_camera_data(robot: Robot) -> dict[str, np.ndarray] | None:
    """Retrieve head camera data from the robot.

    Args:
        robot: Robot instance to get camera data from.

    Returns:
        Dictionary containing camera data, or None if retrieval failed.
    """
    try:
        camera_data = robot.sensors.head_camera.get_obs(
            obs_keys=[
                "left_rgb",
                "right_rgb",
                "depth",
            ]
        )
        return camera_data
    except Exception as e:
        logger.error(f"Failed to retrieve camera data: {e}")
        return None


def main() -> None:
    """Get and display head camera data in a tiled window.

    Creates a robot instance and retrieves RGB/depth images from the head camera,
    then displays them in a single tiled window using matplotlib.

    Raises:
        SystemExit: If camera data retrieval or processing fails.
    """
    configs = get_vega_config()
    configs.sensors.head_camera.enable = True
    robot = Robot(configs=configs)

    try:
        # Get camera information and print it
        camera_info = robot.sensors.head_camera.camera_info
        _print_camera_info(camera_info)

        # Get camera data
        camera_data = get_head_camera_data(robot)
        if camera_data is None:
            logger.error("Failed to retrieve camera data")
            return

        # Extract images and titles
        images, titles = _extract_camera_images(camera_data)

        logger.info(f"Displaying {len(images)} head camera feeds")

        # Display the camera feeds using matplotlib
        display_tiled_cameras(images, titles)

    except ValueError as e:
        logger.error(f"Camera processing error: {e}")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    finally:
        robot.shutdown()


if __name__ == "__main__":
    tyro.cli(main)
