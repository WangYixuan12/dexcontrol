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

This module provides general utility functions for working with Zenoh
communication framework.
"""

import json
import time

import numpy as np
import zenoh
from loguru import logger

from dexcontrol.utils.os_utils import resolve_key_name


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
