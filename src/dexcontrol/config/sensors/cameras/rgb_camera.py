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
class RGBCameraConfig:
    _target_: str = "dexcontrol.sensors.camera.rgb_camera.RGBCameraSensor"
    name: str = "rgb_camera"
    enable_fps_tracking: bool = False
    fps_log_interval: int = 30
    enable: bool = False
    subscriber_config: dict = field(
        default_factory=lambda: dict(
            rgb=dict(
                enable=True,
                info_key="camera/rgb/info",
            )
        )
    )
