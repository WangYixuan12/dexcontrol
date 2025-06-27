# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""RTC utilities for dexcontrol.

This module provides utility functions for creating RTC subscribers
that first query Zenoh for connection information.
"""

import zenoh
from loguru import logger

from dexcontrol.utils.subscribers.rtc import RTCSubscriber
from dexcontrol.utils.zenoh_utils import query_zenoh_json


def query_rtc_info(
    zenoh_session: zenoh.Session,
    info_topic: str,
    timeout: float = 2.0,
    max_retries: int = 1,
    retry_delay: float = 0.5,
) -> dict | None:
    """Query Zenoh for RTC connection information.

    Args:
        zenoh_session: Active Zenoh session for communication.
        info_topic: Zenoh topic to query for RTC info.
        timeout: Maximum time to wait for a response in seconds.
        max_retries: Maximum number of retry attempts.
        retry_delay: Initial delay between retries (doubles each retry).

    Returns:
        Dictionary containing host and port information if successful, None otherwise.
    """

    # Use the general Zenoh query function
    info = query_zenoh_json(
        zenoh_session=zenoh_session,
        topic=info_topic,
        timeout=timeout,
        max_retries=max_retries,
        retry_delay=retry_delay,
    )

    return info


def create_rtc_subscriber_from_zenoh(
    zenoh_session: zenoh.Session,
    info_topic: str,
    name: str = "rtc_subscriber",
    enable_fps_tracking: bool = True,
    fps_log_interval: int = 100,
    query_timeout: float = 2.0,
    max_retries: int = 1,
) -> RTCSubscriber | None:
    """Create a RTC subscriber by first querying Zenoh for connection info.

    Args:
        zenoh_session: Active Zenoh session for communication.
        info_topic: Zenoh topic to query for RTC connection info.
        name: Name for logging purposes.
        enable_fps_tracking: Whether to track and log FPS metrics.
        fps_log_interval: Number of frames between FPS calculations.
        query_timeout: Maximum time to wait for Zenoh query response.
        max_retries: Maximum number of retry attempts for Zenoh query.

    Returns:
        RTCSubscriber instance if successful, None otherwise.
    """
    # Query Zenoh for RTC connection information
    rtc_info = query_rtc_info(zenoh_session, info_topic, query_timeout, max_retries)

    if rtc_info is None:
        logger.error("Failed to get RTC connection info from Zenoh")
        return None

    url = rtc_info.get("signaling_url")

    if not url:
        logger.error(f"Invalid RTC info: url={url}")
        return None

    # Construct WebSocket URL
    ws_url = url
    logger.info(f"Creating RTC subscriber with URL: {ws_url}")

    try:
        # Create and return the RTC subscriber
        subscriber = RTCSubscriber(
            url=ws_url,
            name=name,
            enable_fps_tracking=enable_fps_tracking,
            fps_log_interval=fps_log_interval,
        )
        return subscriber
    except Exception as e:
        logger.error(f"Failed to create RTC subscriber: {e}")
        return None


def create_rtc_subscriber_with_config(
    zenoh_session: zenoh.Session,
    config,
    name: str = "rtc_subscriber",
    enable_fps_tracking: bool = True,
    fps_log_interval: int = 100,
) -> RTCSubscriber | None:
    """Create a RTC subscriber using configuration object.

    Args:
        zenoh_session: Active Zenoh session for communication.
        config: Configuration object containing info_key.
        name: Name for logging purposes.
        enable_fps_tracking: Whether to track and log FPS metrics.
        fps_log_interval: Number of frames between FPS calculations.
    Returns:
        RTCSubscriber instance if successful, None otherwise.
    """
    if "info_key" not in config:
        logger.error("Config subscriber_config missing info_key")
        return None

    if not config["enable"]:
        logger.info(f"Skipping {name} because it is disabled")
        return None

    info_topic = config["info_key"]

    return create_rtc_subscriber_from_zenoh(
        zenoh_session=zenoh_session,
        info_topic=info_topic,
        name=name,
        enable_fps_tracking=enable_fps_tracking,
        fps_log_interval=fps_log_interval,
    )
