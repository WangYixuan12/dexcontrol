# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Luxonis wrist camera sensor using RGB and depth Zenoh subscribers.

This sensor mirrors the high-level API of other camera sensors. It subscribes to
RGB (JPEG over Zenoh) and depth streams published by dexsensor's Luxonis camera
pipeline and exposes a simple interface for getting images.
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional, Union

import numpy as np
import zenoh

from dexcontrol.config.sensors.cameras.luxonis_camera import LuxonisCameraConfig
from dexcontrol.utils.subscribers.camera import (
    DepthCameraSubscriber,
    RGBCameraSubscriber,
)

logger = logging.getLogger(__name__)


class LuxonisCameraSensor:
    """RGBD camera sensor wrapper for Luxonis/OAK wrist camera.

    Provides access to RGB and depth frames via dedicated Zenoh subscribers.
    """

    def __init__(
        self,
        configs: LuxonisCameraConfig,
        zenoh_session: zenoh.Session,
    ) -> None:
        self._name = configs.name
        self._configs = configs
        self._zenoh_session = zenoh_session

        # Support left and right RGB streams + depth, mirroring ZED structure
        self._subscribers: Dict[str, Optional[Union[RGBCameraSubscriber, DepthCameraSubscriber]]] = {}

        subscriber_config = configs.subscriber_config

        try:
            # Create subscribers for left_rgb, right_rgb, depth (configurable enables)
            stream_defs: Dict[str, Dict[str, Any]] = {
                "left_rgb": subscriber_config.get("left_rgb", {}),
                "right_rgb": subscriber_config.get("right_rgb", {}),
                "depth": subscriber_config.get("depth", {}),
            }

            for stream_name, cfg in stream_defs.items():
                if not cfg or not cfg.get("enable", False):
                    self._subscribers[stream_name] = None
                    continue
                topic = cfg.get("topic")
                if not topic:
                    logger.warning(f"'{self._name}': No topic configured for '{stream_name}'")
                    self._subscribers[stream_name] = None
                    continue

                if stream_name == "depth":
                    self._subscribers[stream_name] = DepthCameraSubscriber(
                        topic=topic,
                        zenoh_session=self._zenoh_session,
                        name=f"{self._name}_{stream_name}_subscriber",
                        enable_fps_tracking=configs.enable_fps_tracking,
                        fps_log_interval=configs.fps_log_interval,
                    )
                else:
                    self._subscribers[stream_name] = RGBCameraSubscriber(
                        topic=topic,
                        zenoh_session=self._zenoh_session,
                        name=f"{self._name}_{stream_name}_subscriber",
                        enable_fps_tracking=configs.enable_fps_tracking,
                        fps_log_interval=configs.fps_log_interval,
                    )

        except Exception as e:
            logger.error(f"Error creating Luxonis wrist camera subscribers: {e}")

    # Lifecycle
    def shutdown(self) -> None:
        for sub in self._subscribers.values():
            if sub:
                sub.shutdown()
        logger.info(f"'{self._name}' sensor shut down.")

    # Status
    def is_active(self) -> bool:
        return any(sub.is_active() for sub in self._subscribers.values() if sub is not None)

    def wait_for_active(self, timeout: float = 5.0, require_both: bool = False) -> bool:
        subs = [s for s in self._subscribers.values() if s is not None]
        if not subs:
            return False
        if require_both:
            return all(s.wait_for_active(timeout) for s in subs)
        return any(s.wait_for_active(timeout) for s in subs)

    # Data access
    def get_obs(
        self, obs_keys: Optional[list[str]] = None, include_timestamp: bool = False
    ) -> Dict[str, Optional[np.ndarray]]:
        """Get latest images.

        obs_keys can include any of: ["left_rgb", "right_rgb", "depth"]. If None, returns
        all available. If include_timestamp is True and the underlying subscriber returns
        (image, timestamp), that tuple is forwarded; otherwise only the image is returned.
        """
        keys_to_fetch = obs_keys or self.available_streams

        out: Dict[str, Optional[np.ndarray]] = {}
        for key in keys_to_fetch:
            sub = self._subscribers.get(key)
            data = sub.get_latest_data() if sub else None
            is_tuple_or_list = isinstance(data, (tuple, list))
            if include_timestamp:
                if not is_tuple_or_list and data is not None:
                    logger.warning(f"Timestamp is not available yet for {key} stream.")
                out[key] = data
            else:
                out[key] = data[0] if is_tuple_or_list else data
        return out

    def get_rgb(self) -> Optional[np.ndarray]:
        # Backward-compat: return left_rgb if available else right_rgb
        for key in ("left_rgb", "right_rgb"):
            sub = self._subscribers.get(key)
            if sub:
                data = sub.get_latest_data()
                return data[0] if isinstance(data, (tuple, list)) else data
        return None

    def get_depth(self) -> Optional[np.ndarray]:
        sub = self._subscribers.get("depth")
        if not sub:
            return None
        data = sub.get_latest_data()
        return data[0] if isinstance(data, (tuple, list)) else data

    # Properties
    @property
    def fps(self) -> Dict[str, float]:
        return {name: sub.fps for name, sub in self._subscribers.items() if sub is not None}

    @property
    def name(self) -> str:
        return self._name

    @property
    def available_streams(self) -> list:
        return [name for name, sub in self._subscribers.items() if sub is not None]

    @property
    def active_streams(self) -> list:
        return [name for name, sub in self._subscribers.items() if sub and sub.is_active()]

