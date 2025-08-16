# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Example script to show head camera data live.

This script demonstrates how to retrieve head camera data (RGB and depth images)
from the robot and display them live using matplotlib animation. It showcases the
simple API for getting camera data and provides live visualization.
"""

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import tyro

from dexcontrol.config.vega import get_vega_config
from dexcontrol.robot import Robot


def get_camera_data(robot):
    """Simple function to get camera data from robot sensors.

    This demonstrates how easy it is to get camera data using our API.
    """
    return robot.sensors.head_camera.get_obs(
        obs_keys=["left_rgb", "right_rgb", "depth"], include_timestamp=True
    )


def print_camera_info(camera_info):
    """Nicely format and print any nested dictionary."""

    def print_dict(d, indent=0):
        """Recursively print dictionary with proper indentation."""
        for key, value in d.items():
            # Create indentation
            spaces = "  " * indent

            if isinstance(value, dict):
                print(f"{spaces}{key}:")
                print_dict(value, indent + 1)
            elif isinstance(value, (list, tuple)):
                print(f"{spaces}{key}: {value}")
            else:
                print(f"{spaces}{key}: {value}")

    if not camera_info:
        print("No camera information available")
        return

    print("\n" + "=" * 50)
    print("HEAD CAMERA INFORMATION")
    print("=" * 50)
    print_dict(camera_info)
    print("=" * 50)
    print()


def visualize_camera_data(robot, fps: float = 30.0):
    """Visualize camera data using matplotlib."""
    from matplotlib import cm

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle("Live Head Camera Feeds", fontsize=16)

    # Setup display
    displays = [ax.imshow(np.zeros((480, 640, 3))) for ax in axes]
    for ax, title in zip(axes, ["Left RGB", "Right RGB", "Depth"]):
        ax.set_title(title)
        ax.axis("off")

    plt.tight_layout()

    def update(frame):
        # Get camera data - simple API call
        camera_data = get_camera_data(robot)

        # Update displays
        titles = ["Left RGB", "Right RGB", "Depth"]
        for i, key in enumerate(["left_rgb", "right_rgb", "depth"]):
            if key in camera_data and camera_data[key] is not None:
                data = camera_data[key]
                timestamp = None

                if isinstance(data, tuple):
                    img = data[0]  # Extract image from tuple
                    timestamp = data[1]
                else:
                    img = data

                # Update title with timestamp if available
                if timestamp is not None:
                    axes[i].set_title(f"{titles[i]}\n{img.shape}\n{timestamp} (ns)")
                else:
                    axes[i].set_title(f"{titles[i]}\n{img.shape}")

                # Process depth image for visualization
                if "depth" in key:
                    # Normalize depth to 0-255
                    img = (
                        (img - img.min()) / (img.max() - img.min() + 1e-8) * 255
                    ).astype(np.uint8)
                    # Apply colormap
                    img = (cm.viridis(img / 255.0)[:, :, :3] * 255).astype(np.uint8)

                displays[i].set_array(img)

    # Start animation
    _ = animation.FuncAnimation(fig, update, interval=int(1000 / fps), blit=False)
    plt.show()


def main(fps: float = 30.0) -> None:
    """Main function to initialize robot and display camera feeds.

    Args:
        fps: Display refresh rate in Hz (default: 30.0)
    """
    configs = get_vega_config()
    configs.sensors.head_camera.enable = True

    with Robot(configs=configs) as robot:
        # Print camera information nicely
        camera_info = robot.sensors.head_camera.camera_info
        print_camera_info(camera_info)

        # Start live camera visualization
        visualize_camera_data(robot, fps)


if __name__ == "__main__":
    tyro.cli(main)
