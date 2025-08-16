# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Example script to demonstrate heartbeat monitoring functionality.

This script shows how the heartbeat monitor works to ensure the low-level
controller is functioning properly. The robot will automatically exit if
no heartbeat is received within the configured timeout period (0.8 seconds).

To disable heartbeat monitoring for testing purposes, set the environment variable:
    export DEXCONTROL_DISABLE_HEARTBEAT=1
    # or
    DEXCONTROL_DISABLE_HEARTBEAT=1 python heartbeat_monitor_demo.py
"""

import os
import time

import typer
from loguru import logger
from rich.console import Console
from rich.live import Live
from rich.table import Table

from dexcontrol.robot import Robot
from dexcontrol.utils.constants import DISABLE_HEARTBEAT_ENV_VAR


def _format_uptime(seconds: float) -> str:
    """Convert seconds to human-readable uptime format with high resolution.

    Args:
        seconds: Total seconds of uptime.

    Returns:
        Human-readable string like "1mo 2d 3h 45m 12s 345ms".
    """
    # Calculate months (assuming 30 days per month)
    months = int(seconds // (86400 * 30))
    remaining = seconds % (86400 * 30)

    # Calculate days
    days = int(remaining // 86400)
    remaining = remaining % 86400

    # Calculate hours
    hours = int(remaining // 3600)
    remaining = remaining % 3600

    # Calculate minutes
    minutes = int(remaining // 60)
    remaining = remaining % 60

    # Calculate seconds and milliseconds
    secs = int(remaining)
    milliseconds = int((remaining - secs) * 1000)

    parts = []
    if months > 0:
        parts.append(f"{months}mo")
    if days > 0:
        parts.append(f"{days}d")
    if hours > 0:
        parts.append(f"{hours}h")
    if minutes > 0:
        parts.append(f"{minutes}m")
    if secs > 0:
        parts.append(f"{secs}s")
    if milliseconds > 0 or not parts:
        parts.append(f"{milliseconds}ms")

    return " ".join(parts)


def main(
    monitor_duration: float = 30.0,
    disable_heartbeat: bool = False,
) -> None:
    """Demonstrate heartbeat monitoring functionality.

    Args:
        monitor_duration: How long to monitor the heartbeat in seconds.
        disable_heartbeat: If True, disable heartbeat monitoring for this demo.
    """
    # Set environment variable if requested
    if disable_heartbeat:
        os.environ[DISABLE_HEARTBEAT_ENV_VAR] = "1"
        logger.warning("Heartbeat monitoring has been DISABLED for this demo")

    logger.info("Starting heartbeat monitoring demonstration")
    logger.info(f"Will monitor for {monitor_duration} seconds")

    if not disable_heartbeat:
        logger.info(
            "The robot will automatically exit if no heartbeat is received for 0.8 seconds"
        )
    else:
        logger.info(
            "Heartbeat monitoring is DISABLED - robot will not exit on heartbeat timeout"
        )

    # Initialize robot with default configuration
    bot = Robot()

    try:
        # Display initial heartbeat status
        logger.info("Initial heartbeat status:")
        bot.heartbeat.show()
        logger.info(f"Starting live monitoring for {monitor_duration} seconds...")

        console = Console()

        def generate_status_table(elapsed_time: float, status: dict) -> Table:
            """Generate a status table for the live display."""
            table = Table(
                title=f"Live Heartbeat Monitor - Elapsed: {elapsed_time:.1f}s / {monitor_duration:.1f}s"
            )
            table.add_column("Parameter", style="cyan")
            table.add_column("Value", style="white")

            # Enabled status
            enabled = status.get("enabled", True)
            if not enabled:
                table.add_row("Status", "[yellow]DISABLED[/]")
                table.add_row(
                    "Reason", f"[yellow]Disabled via {DISABLE_HEARTBEAT_ENV_VAR}[/]"
                )
                return table

            # Paused status
            paused = status.get("paused", False)
            if paused:
                table.add_row("Status", "[yellow]PAUSED[/]")
                return table

            # Active status
            is_active = status.get("is_active", False)
            active_style = "bold green" if is_active else "bold red"
            table.add_row("Signal Active", f"[{active_style}]{is_active}[/]")

            # Last heartbeat value (robot uptime)
            last_value = status.get("last_value")
            if last_value is not None:
                uptime_str = _format_uptime(last_value)
                table.add_row("Robot Uptime", f"[blue]{uptime_str}[/]")
            else:
                table.add_row("Robot Uptime", "[red]No data[/]")

            # Time since last heartbeat
            time_since = status.get("time_since_last")
            timeout_seconds = status.get("timeout_seconds", 0.8)
            if time_since is not None and isinstance(timeout_seconds, (int, float)):
                time_style = (
                    "bold red" if time_since > timeout_seconds * 0.8 else "bold green"
                )
                table.add_row("Time Since Last", f"[{time_style}]{time_since:.3f}s[/]")
            else:
                table.add_row("Time Since Last", "[yellow]N/A[/]")

            # Timeout setting
            table.add_row("Timeout", f"[blue]{timeout_seconds}s[/]")

            # Progress bar
            progress = min(elapsed_time / monitor_duration, 1.0)
            progress_bar = "█" * int(progress * 30) + "░" * (30 - int(progress * 30))
            table.add_row("Progress", f"[green]{progress_bar}[/] {progress * 100:.1f}%")

            return table

        # Monitor heartbeat with live updating display
        start_time = time.time()
        with Live(console=console, refresh_per_second=4) as live:
            while time.time() - start_time < monitor_duration:
                elapsed_time = time.time() - start_time
                status = bot.heartbeat.get_status()

                # Update the live display
                live.update(generate_status_table(elapsed_time, status))

                time.sleep(0.25)

        logger.info("Heartbeat monitoring demonstration completed successfully")

    except KeyboardInterrupt:
        logger.info("Demonstration interrupted by user")
    except Exception as e:
        logger.error(f"Error during demonstration: {e}")
    finally:
        logger.info("Shutting down robot...")
        bot.shutdown()


if __name__ == "__main__":
    typer.run(main)
