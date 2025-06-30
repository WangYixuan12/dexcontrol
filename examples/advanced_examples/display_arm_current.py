# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Example script to display robot arm joint currents in real-time.

Visualizes joint currents of robot arms using matplotlib. Supports plotting either
or both arms and allows selecting specific joints to monitor.
"""

import argparse
import time
from collections import deque
from typing import Sequence

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D

from dexcontrol.robot import Robot


class CurrentPlotter:
    """Real-time plotter for robot arm joint currents.

    Handles real-time visualization of joint currents for robot arms. Can display
    currents for left arm, right arm, or both simultaneously.

    Attributes:
        bot: Robot instance for getting joint states
        max_points: Maximum number of data points to display
        times: Deque storing timestamps
        plot_left: Whether to plot left arm
        plot_right: Whether to plot right arm
        joints: Joint indices to plot
        left_currents: Current values for left arm joints
        right_currents: Current values for right arm joints
        fig: Matplotlib figure
        axes: List of plot axes
        left_ax: Left arm plot axis
        right_ax: Right arm plot axis
        left_lines: Plot lines for left arm
        right_lines: Plot lines for right arm
        start_time: Start time of plotting
    """

    def __init__(
        self,
        plot_left: bool = True,
        plot_right: bool = True,
        max_points: int = 100,
        joints: Sequence[int] | None = None,
    ) -> None:
        """Initialize the current plotter.

        Args:
            plot_left: Whether to plot left arm currents
            plot_right: Whether to plot right arm currents
            max_points: Maximum number of points in history
            joints: Joint indices to plot (0-based). If None, plots all joints
        """
        self.bot = Robot()
        self.max_points = max_points
        self.times = deque(maxlen=max_points)
        self.plot_left = plot_left
        self.plot_right = plot_right
        self.joints = range(7) if joints is None else joints

        # Initialize data storage
        self.left_currents = (
            {i: deque(maxlen=max_points) for i in self.joints} if plot_left else {}
        )
        self.right_currents = (
            {i: deque(maxlen=max_points) for i in self.joints} if plot_right else {}
        )

        # Setup plot
        if plot_left and plot_right:
            self.fig, (self.left_ax, self.right_ax) = plt.subplots(
                2, 1, figsize=(10, 8)
            )
            self.axes = [self.left_ax, self.right_ax]
        else:
            self.fig, ax = plt.subplots(1, 1, figsize=(10, 5))
            self.axes = [ax]
            if plot_left:
                self.left_ax = ax
            else:
                self.right_ax = ax

        self._setup_axes()
        self._initialize_plot_lines()
        self.start_time = time.time()

    def _initialize_plot_lines(self) -> None:
        """Initialize plot lines for each joint."""
        self.left_lines = []
        self.right_lines = []

        if self.plot_left:
            self.left_lines = [
                self.left_ax.plot(
                    [],
                    [],
                    label=f"{self.bot.left_arm.joint_name[i]} (A)",
                    linewidth=2.0,
                )[0]
                for i in self.joints
            ]
        if self.plot_right:
            self.right_lines = [
                self.right_ax.plot(
                    [],
                    [],
                    label=f"{self.bot.right_arm.joint_name[i]} (A)",
                    linewidth=2.0,
                )[0]
                for i in self.joints
            ]

    def _setup_axes(self) -> None:
        """Configure plot axes appearance."""
        if self.plot_left:
            self.left_ax.set_title("Left Arm Joint Currents")
        if self.plot_right:
            self.right_ax.set_title("Right Arm Joint Currents")

        for ax in self.axes:
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Current (Amperes)")
            ax.grid(True)
            ax.set_xticks(np.linspace(0, 2, 11))
            ax.set_xlim(0, 2)

    def create_legends(self) -> None:
        """Create plot legends."""
        for ax in self.axes:
            ax.legend(loc="upper left", bbox_to_anchor=(1, 1))

    def update(self, _) -> list[Line2D]:
        """Update plot data.

        Args:
            _: Frame number (unused but required by FuncAnimation)

        Returns:
            List of updated plot lines
        """
        current_time = time.time() - self.start_time
        self.times.append(current_time)
        shifted_times = np.array(list(self.times)) - (current_time - 2)

        if self.plot_left:
            left_current = self.bot.left_arm.get_joint_current()
            for i, joint_idx in enumerate(self.joints):
                self.left_currents[joint_idx].append(left_current[joint_idx])
                self.left_lines[i].set_data(
                    shifted_times, list(self.left_currents[joint_idx])
                )

        if self.plot_right:
            right_current = self.bot.right_arm.get_joint_current()
            for i, joint_idx in enumerate(self.joints):
                self.right_currents[joint_idx].append(right_current[joint_idx])
                self.right_lines[i].set_data(
                    shifted_times, list(self.right_currents[joint_idx])
                )

        # Update y-axis limits
        for ax in self.axes:
            ax.relim()
            ax.autoscale_view(scalex=False)

        return self.left_lines + self.right_lines


def parse_args() -> argparse.Namespace:
    """Parse command line arguments.

    Returns:
        Parsed command line arguments
    """
    parser = argparse.ArgumentParser(description="Plot robot arm joint currents.")
    parser.add_argument("--left", action="store_true", help="Plot left arm currents")
    parser.add_argument("--right", action="store_true", help="Plot right arm currents")
    parser.add_argument(
        "--joints",
        type=int,
        nargs="+",
        help="Joint indices to plot (1-7). If not specified, plots all joints",
    )
    parser.add_argument("--robot_model", type=str, default="vega-1", help="Robot name")
    args = parser.parse_args()

    # Default to plotting both arms if none specified
    if not args.left and not args.right:
        args.left = args.right = True

    # Validate and convert joint indices to 0-based
    if args.joints:
        invalid_joints = [j for j in args.joints if not 1 <= j <= 7]
        if invalid_joints:
            parser.error(
                f"Invalid joint indices: {invalid_joints}. Must be between 1 and 7."
            )
        args.joints = [j - 1 for j in args.joints]

    return args


def main() -> None:
    """Run the current plotting visualization."""
    args = parse_args()
    plotter = CurrentPlotter(
        plot_left=args.left, plot_right=args.right, joints=args.joints
    )
    plotter.create_legends()

    try:
        _ = FuncAnimation(
            plotter.fig,
            plotter.update,
            interval=100,  # 10 Hz update rate
            blit=True,
            cache_frame_data=False,
            save_count=100,
        )
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        plotter.bot.shutdown()
        plt.close()


if __name__ == "__main__":
    main()
