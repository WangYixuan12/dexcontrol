# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""This example demonstrates how to retrieve and display chassis camera data in real-time.

The script initializes the robot with all chassis cameras enabled, captures images
from each camera continuously, and displays them in a single tiled window that updates live.
Press Ctrl+C to exit the live feed.
"""

import math
import time

import matplotlib.pyplot as plt
import numpy as np
import tyro
from loguru import logger

from dexcontrol.config.vega import get_vega_config
from dexcontrol.robot import Robot


class LiveCameraDisplay:
    """Handles live display of multiple camera feeds in a tiled layout."""

    def __init__(self, camera_definitions: dict):
        """Initialize the live camera display.

        Args:
            camera_definitions: Dictionary mapping camera names to display names
        """
        self.camera_definitions = camera_definitions
        self.num_cameras = len(camera_definitions)

        # Calculate grid layout
        self.cols = math.ceil(math.sqrt(self.num_cameras))
        self.rows = math.ceil(self.num_cameras / self.cols)

        # Setup matplotlib figure and axes
        plt.ion()  # Turn on interactive mode
        self.fig, self.axes = plt.subplots(
            self.rows, self.cols, figsize=(15, 10), squeeze=False
        )
        self.fig.suptitle(
            "Live Chassis Camera Feeds (Press Ctrl+C to exit)", fontsize=16
        )

        # Initialize image plots
        self.image_plots = {}
        for i, (camera_name, display_name) in enumerate(camera_definitions.items()):
            ax = self.axes[i // self.cols, i % self.cols]
            ax.set_title(display_name)
            ax.axis("off")
            # Create empty image plot
            self.image_plots[camera_name] = ax.imshow(
                np.zeros((480, 640, 3), dtype=np.uint8)
            )

        # Hide unused subplots
        for i in range(self.num_cameras, self.rows * self.cols):
            self.axes[i // self.cols, i % self.cols].axis("off")

        plt.tight_layout()
        plt.show(block=False)

    def update_images(self, robot) -> None:
        """Update all camera images in the display.

        Args:
            robot: The robot instance to get camera data from
        """
        updated = False
        for camera_name, display_name in self.camera_definitions.items():
            try:
                camera_sensor = getattr(robot.sensors, camera_name, None)
                if camera_sensor and camera_sensor.is_active():
                    image = camera_sensor.get_obs()
                    if image is not None:
                        self.image_plots[camera_name].set_array(image)
                        updated = True
                    else:
                        logger.debug(f"No image data from '{display_name}'.")
                else:
                    logger.debug(
                        f"Camera sensor '{display_name}' is not available or not active."
                    )
            except Exception as e:
                logger.error(
                    f"Error getting data from '{display_name}': {e}",
                    exc_info=True,
                )

        if updated:
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

    def close(self):
        """Close the display window."""
        plt.ioff()
        plt.close(self.fig)


def main() -> None:
    """Initializes the robot, retrieves chassis camera data, and displays it in real-time."""
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
    logger.info("Waiting for camera streams to stabilize...")
    time.sleep(3)

    # Initialize live display
    display = LiveCameraDisplay(camera_definitions)

    logger.info("Starting live camera feed. Press Ctrl+C to exit...")

    try:
        while True:
            display.update_images(robot)
            time.sleep(0.1)  # Update at ~10 FPS

    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received. Stopping live feed...")

    except Exception as e:
        logger.error(f"Unexpected error in live feed: {e}", exc_info=True)

    finally:
        logger.info("Shutting down...")
        display.close()
        robot.shutdown()
        logger.info("Shutdown complete.")


if __name__ == "__main__":
    tyro.cli(main)
