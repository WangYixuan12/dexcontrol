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

from dexcontrol.robot import Robot
from dexcontrol.utils.constants import DISABLE_HEARTBEAT_ENV_VAR


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

        # Monitor heartbeat for the specified duration
        start_time = time.time()
        while time.time() - start_time < monitor_duration:
            # Display heartbeat status every 5 seconds
            if int(time.time() - start_time) % 5 == 0:
                logger.info(
                    f"Heartbeat status after {int(time.time() - start_time)} seconds:"
                )
                bot.heartbeat.show()

                # Get detailed status
                status = bot.heartbeat.get_status()
                if status["enabled"]:
                    if status["is_active"]:
                        logger.info(
                            f"✓ Heartbeat active - Last value: {status['last_value']}"
                        )
                        if status["time_since_last"] is not None:
                            logger.info(
                                f"  Time since last: {status['time_since_last']:.3f}s"
                            )
                    else:
                        logger.warning("⚠ No heartbeat signal detected yet")
                else:
                    logger.info("ℹ Heartbeat monitoring is disabled")

            time.sleep(0.2)

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
