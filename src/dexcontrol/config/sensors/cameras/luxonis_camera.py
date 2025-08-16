# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

from dataclasses import dataclass, field


@dataclass
class LuxonisCameraConfig:
    """Configuration for LuxonisCameraSensor (wrist camera).

    Attributes:
        _target_: Target sensor class for Hydra instantiation.
        name: Logical name of the camera.
        enable: Whether this sensor is enabled.
        enable_fps_tracking: Enable FPS tracking logs.
        fps_log_interval: Frames between FPS logs.
        subscriber_config: Topics for RGB and Depth Zenoh subscribers.
    """

    _target_: str = "dexcontrol.sensors.camera.luxonis_camera.LuxonisCameraSensor"
    name: str = "wrist_camera"
    enable: bool = False
    enable_fps_tracking: bool = False
    fps_log_interval: int = 30

    # Note: Resolution is set by the publisher (dexsensor). The publisher now defaults
    # to 720p to match common Luxonis sensor capabilities (e.g., OV9782 supports 720p/800p).
    # These subscribers only define topics to consume.
    subscriber_config: dict = field(
        default_factory=lambda: {
            "left_rgb": {
                "enable": True,
                "topic": "camera/wrist/left_rgb",
            },
            "right_rgb": {
                "enable": True,
                "topic": "camera/wrist/right_rgb",
            },
            "depth": {
                "enable": False,
                "topic": "camera/wrist/depth",
            },
        }
    )
