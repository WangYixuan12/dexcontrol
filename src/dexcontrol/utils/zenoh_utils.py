# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Zenoh utilities for dexcontrol.

This module provides comprehensive utility functions for working with Zenoh
communication framework, including session management, configuration loading,
JSON queries, and statistics computation.
"""

import gc
import json
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import zenoh
from loguru import logger
from omegaconf import DictConfig, OmegaConf

import dexcontrol
from dexcontrol.config.vega import get_vega_config
from dexcontrol.utils.os_utils import resolve_key_name

if TYPE_CHECKING:
    from dexcontrol.config.vega import VegaConfig


# =============================================================================
# Session Management Functions
# =============================================================================


def get_default_zenoh_config() -> str | None:
    """Gets the default zenoh configuration file path.

    Returns:
        Path to default config file if it exists, None otherwise.
    """
    default_path = dexcontrol.COMM_CFG_PATH
    if not default_path.exists():
        logger.warning(f"Zenoh config file not found at {default_path}")
        logger.warning("Please use dextop to set up the zenoh config file")
        return None
    return str(default_path)


def create_zenoh_session(zenoh_config_file: str | None = None) -> zenoh.Session:
    """Creates and initializes a Zenoh communication session.

    Args:
        zenoh_config_file: Path to zenoh configuration file. If None,
                          uses the default configuration path.

    Returns:
        Initialized zenoh session.

    Raises:
        RuntimeError: If zenoh session initialization fails.
    """
    try:
        config_path = zenoh_config_file or get_default_zenoh_config()
        if config_path is None:
            logger.warning("Using default zenoh config settings")
            return zenoh.open(zenoh.Config())
        return zenoh.open(zenoh.Config.from_file(config_path))
    except Exception as e:
        raise RuntimeError(f"Failed to initialize zenoh session: {e}") from e


def load_robot_config(
    robot_config_path: str | None = None,
) -> "VegaConfig":
    """Load robot configuration from file or use default variant.

    Args:
        robot_config_path: Path to robot configuration file. If None,
                          uses default configuration for detected robot model.

    Returns:
        Robot configuration as OmegaConf object.

    Raises:
        ValueError: If configuration cannot be loaded or parsed.
    """
    try:
        if robot_config_path is not None:
            # Load custom configuration from file
            config_path = Path(robot_config_path)
            if not config_path.exists():
                raise ValueError(f"Configuration file not found: {config_path}")

            # Load YAML configuration and merge with default
            base_config = DictConfig, get_vega_config()
            custom_config = OmegaConf.load(config_path)
            return OmegaConf.merge(base_config, custom_config)
        else:
            # Use default configuration for detected robot model
            try:
                return get_vega_config()
            except ValueError as e:
                # If robot model detection fails, use default vega-1 config
                if "Robot name is not set" in str(e):
                    logger.warning(
                        "Robot model not detected, using default vega-1 configuration"
                    )
                    return get_vega_config("vega-1")
                raise

    except Exception as e:
        raise ValueError(f"Failed to load robot configuration: {e}") from e


def create_standalone_robot_interface(
    zenoh_config_file: str | None = None,
    robot_config_path: str | None = None,
) -> tuple[zenoh.Session, "VegaConfig"]:
    """Create standalone zenoh session and robot configuration.

    This function provides a convenient way to create both a zenoh session
    and robot configuration for use with RobotQueryInterface without
    requiring the full Robot class initialization.

    Args:
        zenoh_config_file: Path to zenoh configuration file. If None,
                          uses the default configuration path.
        robot_config_path: Path to robot configuration file. If None,
                          uses default configuration for detected robot model.

    Returns:
        Tuple of (zenoh_session, robot_config) ready for use with
        RobotQueryInterface.

    Raises:
        RuntimeError: If zenoh session initialization fails.
        ValueError: If robot configuration cannot be loaded.

    Example:
        >>> session, config = create_standalone_robot_interface()
        >>> query_interface = RobotQueryInterface(session, config)
        >>> version_info = query_interface.get_version_info()
        >>> session.close()
    """
    # Create zenoh session
    session = create_zenoh_session(zenoh_config_file)

    # Load robot configuration
    config = load_robot_config(robot_config_path)

    return session, config


# =============================================================================
# Query and Communication Functions
# =============================================================================
def query_zenoh_json(
    zenoh_session: zenoh.Session,
    topic: str,
    timeout: float = 2.0,
    max_retries: int = 1,
    retry_delay: float = 0.5,
) -> dict | None:
    """Query Zenoh for JSON information with retry logic.

    Args:
        zenoh_session: Active Zenoh session for communication.
        topic: Zenoh topic to query.
        timeout: Maximum time to wait for a response in seconds.
        max_retries: Maximum number of retry attempts.
        retry_delay: Initial delay between retries (doubles each retry).

    Returns:
        Dictionary containing the parsed JSON response if successful, None otherwise.
    """
    resolved_topic = resolve_key_name(topic)
    logger.debug(f"Querying Zenoh topic: {resolved_topic}")

    for attempt in range(max_retries + 1):
        try:
            # Add delay before retry (except first attempt)
            if attempt > 0:
                delay = retry_delay * (2 ** (attempt - 1))  # Exponential backoff
                logger.debug(f"Retry {attempt}/{max_retries} after {delay}s delay...")
                time.sleep(delay)

            # Try to get the info
            for reply in zenoh_session.get(resolved_topic, timeout=timeout):
                if reply.ok:
                    response = json.loads(reply.ok.payload.to_bytes())
                    return response
            else:
                # No valid reply received
                if attempt < max_retries:
                    logger.debug(f"No reply on attempt {attempt + 1}, will retry...")
                else:
                    logger.error(
                        f"No valid reply received on topic '{resolved_topic}' after {max_retries + 1} attempts."
                    )

        except StopIteration:
            if attempt < max_retries:
                logger.debug(f"Query timed out on attempt {attempt + 1}, will retry...")
            else:
                logger.error(f"Query timed out after {max_retries + 1} attempts.")
        except Exception as e:
            if attempt < max_retries:
                logger.debug(
                    f"Query failed on attempt {attempt + 1}: {e}, will retry..."
                )
            else:
                logger.error(f"Query failed after {max_retries + 1} attempts: {e}")

    return None


# =============================================================================
# Cleanup and Exit Handling Functions
# =============================================================================
def close_zenoh_session_with_timeout(
    session: zenoh.Session, timeout: float = 2.0
) -> tuple[bool, Exception | None]:
    """Close a Zenoh session with timeout handling.

    This function attempts to close a Zenoh session gracefully with a timeout.
    If the close operation takes too long, it returns with a timeout indication.

    Args:
        session: The Zenoh session to close.
        timeout: Maximum time to wait for session close (default 2.0 seconds).

    Returns:
        Tuple of (success, exception):
        - success: True if session closed successfully, False otherwise
        - exception: Any exception that occurred during close, or None
    """

    close_success = False
    close_exception = None

    def _close_session():
        """Inner function to close the session."""
        nonlocal close_success, close_exception
        try:
            session.close()
            close_success = True
        except Exception as e:  # pylint: disable=broad-except
            close_exception = e
            logger.debug(f"Zenoh session close attempt failed: {e}")
            # Try to trigger garbage collection as fallback
            try:
                gc.collect()
            except Exception:  # pylint: disable=broad-except
                pass

    # Try to close zenoh session with timeout
    close_thread = threading.Thread(target=_close_session, daemon=True)
    close_thread.start()

    # Use progressive timeout strategy
    timeouts = [timeout / 2, timeout / 2]  # Split timeout into two attempts
    for i, wait_time in enumerate(timeouts):
        close_thread.join(timeout=wait_time)
        if not close_thread.is_alive():
            break

    if close_thread.is_alive():
        return False, Exception("Close operation timed out")
    elif close_success:
        return True, None
    else:
        logger.debug(f"Zenoh session closed with error: {close_exception}")
        return False, close_exception


def wait_for_zenoh_cleanup(cleanup_delays: list[float] | None = None) -> list[str]:
    """Wait for Zenoh internal threads to clean up.

    This function waits for Zenoh's internal pyo3 threads to clean up after
    session closure, using progressive delays to balance responsiveness and
    thoroughness.

    Args:
        cleanup_delays: List of delays in seconds to wait between checks.
                       Defaults to [0.1, 0.2, 0.3] if not provided.

    Returns:
        List of thread names that are still active after cleanup attempts.
    """
    if cleanup_delays is None:
        cleanup_delays = [0.1, 0.2, 0.3]  # Progressive delays totaling 0.6s

    for delay in cleanup_delays:
        time.sleep(delay)
        # Check if threads are still active
        active_threads = get_active_zenoh_threads()
        if not active_threads:
            return []

    # Return any remaining threads
    lingering_threads = get_active_zenoh_threads()
    if lingering_threads:
        logger.debug(
            f"Note: {len(lingering_threads)} Zenoh internal thread(s) still active. "
            "These typically clean up after script exit."
        )
    return lingering_threads


def get_active_zenoh_threads() -> list[str]:
    """Get list of active Zenoh (pyo3) threads.

    Returns:
        List of thread names that are pyo3-related and still active.
    """
    return [
        t.name
        for t in threading.enumerate()
        if "pyo3" in t.name and t.is_alive() and not t.daemon
    ]


# =============================================================================
# Statistics and Analysis Functions
# =============================================================================
def compute_ntp_stats(offsets: list[float], rtts: list[float]) -> dict[str, float]:
    """Compute NTP statistics, removing outliers based on RTT median and std.

    Args:
        offsets: List of offset values (seconds).
        rtts: List of round-trip time values (seconds).

    Returns:
        Dictionary with computed statistics (mean, std, min, max, sample_count) for offset and rtt.
    """
    offsets_np = np.array(offsets)
    rtts_np = np.array(rtts)
    if len(rtts_np) < 3:
        mask = np.ones_like(rtts_np, dtype=bool)
    else:
        median = np.median(rtts_np)
        std = np.std(rtts_np)
        mask = np.abs(rtts_np - median) <= 2 * std
    offsets_filtered = offsets_np[mask]
    rtts_filtered = rtts_np[mask]

    def safe_stat(arr, func):
        return float(func(arr)) if len(arr) > 0 else 0.0

    stats = {
        "offset (mean)": safe_stat(offsets_filtered, np.mean),
        "offset (std)": safe_stat(offsets_filtered, np.std),
        "offset (min)": safe_stat(offsets_filtered, np.min),
        "offset (max)": safe_stat(offsets_filtered, np.max),
        "round_trip_time (mean)": safe_stat(rtts_filtered, np.mean),
        "round_trip_time (std)": safe_stat(rtts_filtered, np.std),
        "round_trip_time (min)": safe_stat(rtts_filtered, np.min),
        "round_trip_time (max)": safe_stat(rtts_filtered, np.max),
        "sample_count": int(len(offsets_filtered)),
    }
    return stats
