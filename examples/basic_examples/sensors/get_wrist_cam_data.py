# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Example script to show wrist camera (Luxonis) data live.

This script demonstrates how to retrieve wrist camera data (RGB and depth images)
from the robot and display them live using matplotlib animation. It mirrors the
head camera example structure and ensures timestamps are available.
"""

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import tyro

from dexcontrol.config.vega import get_vega_config
from dexcontrol.robot import Robot


def get_camera_data(robot):
    """Get wrist camera data (Left RGB, Right RGB, and Depth) with timestamps."""
    return robot.sensors.left_wrist_camera.get_obs(  # type: ignore[attr-defined]
        obs_keys=["left_rgb", "right_rgb", "depth"], include_timestamp=True
    )


def visualize_camera_data(robot, fps: float = 30.0):
    """Visualize wrist camera data using matplotlib (RGB + Depth)."""
    from matplotlib import cm

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    fig.suptitle("Live Wrist Camera (Luxonis)", fontsize=16)

    displays = [ax.imshow(np.zeros((480, 640, 3), dtype=np.uint8)) for ax in axes]
    for ax, title in zip(axes, ["Left RGB", "Right RGB", "Depth"]):
        ax.set_title(title)
        ax.axis("off")

    plt.tight_layout()

    def update(frame):
        camera_data = get_camera_data(robot)

        titles = ["Left RGB", "Right RGB", "Depth"]
        for i, key in enumerate(["left_rgb", "right_rgb", "depth"]):
            if key in camera_data and camera_data[key] is not None:
                data = camera_data[key]
                timestamp = None

                if isinstance(data, (tuple, list)):
                    img = data[0]
                    timestamp = data[1]
                else:
                    img = data

                if timestamp is not None:
                    axes[i].set_title(f"{titles[i]}\n{img.shape}\n{timestamp} (ns)")
                else:
                    axes[i].set_title(f"{titles[i]}\n{img.shape}")

                # Depth visualization: normalize and apply colormap
                if key == "depth":
                    img = (
                        (img - img.min()) / (img.max() - img.min() + 1e-8) * 255
                    ).astype(np.uint8)
                    img = (cm.viridis(img / 255.0)[:, :, :3] * 255).astype(np.uint8)

                displays[i].set_array(img)

    _ = animation.FuncAnimation(fig, update, interval=int(1000 / fps), blit=False)
    plt.show()


def main(fps: float = 30.0) -> None:
    configs = get_vega_config()

    configs.sensors.left_wrist_camera.enable = True
    configs.sensors.right_wrist_camera.enable = True

    with Robot(configs=configs) as robot:
        visualize_camera_data(robot, fps)


if __name__ == "__main__":
    tyro.cli(main)
