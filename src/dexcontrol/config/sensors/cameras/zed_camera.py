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
class ZedCameraConfig:
    _target_: str = "dexcontrol.sensors.camera.zed_camera.ZedCameraSensor"
    name: str = "zed_camera"
    enable_fps_tracking: bool = False
    fps_log_interval: int = 30
    enable: bool = False
    subscriber_config: dict = field(
        default_factory=lambda: dict(
            left_rgb=dict(
                enable=True,
                info_key="camera/head/left_rgb/info",
            ),
            right_rgb=dict(
                enable=True,
                info_key="camera/head/right_rgb/info",
            ),
            depth=dict(
                enable=True,
                topic="camera/head/depth",
            ),
        )
    )
