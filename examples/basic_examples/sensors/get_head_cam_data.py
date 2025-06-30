# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""This example demonstrates how to retrieve and display head camera data.

The script initializes the robot, fetches RGB and depth images from the head
camera, and displays them in a tiled window. It also prints the camera's
intrinsic parameters.
"""

import json
import math
from typing import Any, Dict, List, Optional, Tuple

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
    fig.suptitle("Head Camera Feeds", fontsize=16)

    for i, (img, title) in enumerate(zip(images, titles)):
        ax = axes[i // cols, i % cols]
        if "depth" in title.lower():
            ax.imshow(img, cmap="viridis")
        else:
            ax.imshow(img)
        ax.set_title(title)
        ax.axis("off")

    # Hide any unused subplots
    for i in range(num_images, rows * cols):
        axes[i // cols, i % cols].axis("off")

    plt.tight_layout()
    plt.show()


def print_camera_info(camera_info: Optional[Dict[str, Any]]) -> None:
    """Prints camera information in a formatted JSON block.

    Args:
        camera_info: A dictionary containing camera metadata, or None.
    """
    if camera_info:
        print("\n" + "=" * 50)
        print("CAMERA INFORMATION")
        print("=" * 50)
        print(json.dumps(camera_info, indent=2))
        print("=" * 50 + "\n")
    else:
        logger.warning("No camera info available.")


def extract_camera_data(
    camera_data: Dict[str, Optional[np.ndarray]],
) -> Tuple[List[np.ndarray], List[str]]:
    """Extracts valid images and their titles from the camera data dictionary.

    Args:
        camera_data: A dictionary where keys are stream identifiers (e.g.,
            'left_rgb') and values are either a NumPy array (the image) or
            None.

    Returns:
        A tuple containing two lists: one for valid images and one for their
        corresponding titles.

    Raises:
        ValueError: If no valid images are found in the camera data.
    """
    images, titles = [], []
    for key, data in camera_data.items():
        if data is not None:
            images.append(data)
            titles.append(key.replace("_", " ").title())
            logger.info(f"Retrieved '{key}' with shape: {data.shape}")

    if not images:
        raise ValueError("No valid camera images were retrieved.")

    return images, titles


def main() -> None:
    """Initializes the robot, retrieves head camera data, and displays it."""
    configs = get_vega_config()
    configs.sensors.head_camera.enable = True
    robot = Robot(configs=configs)

    try:
        # Print camera intrinsics and other metadata.
        print_camera_info(robot.sensors.head_camera.camera_info)

        # Retrieve all available observations from the head camera.
        obs_keys = ["left_rgb", "right_rgb", "depth"]
        camera_data = robot.sensors.head_camera.get_obs(obs_keys=obs_keys)

        if not camera_data:
            logger.error("Failed to retrieve camera data.")
            return

        # Extract images and titles from the raw data.
        images, titles = extract_camera_data(camera_data)

        # Display the camera feeds in a single window.
        logger.info(f"Displaying {len(images)} head camera feeds...")
        display_tiled_cameras(images, titles)

    except ValueError as e:
        logger.error(f"Error processing camera data: {e}")
    except Exception as e:
        logger.error(f"An unexpected error occurred: {e}", exc_info=True)
    finally:
        logger.info("Shutting down the robot.")
        robot.shutdown()


if __name__ == "__main__":
    tyro.cli(main)
