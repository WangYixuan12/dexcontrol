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
from pathlib import Path

from dexmate_urdf import robots
from dexmotion.configs.core.motion_manager_config import (
    JointRegion,
    JointsToLockConfig,
    MotionManagerConfig,
    Vega1MotionManagerConfig,
)

try:
    from dexmotion.configs.core.motion_manager_config_internal import (
        VegaRC2MotionManagerConfig,
    )

    _HAS_VEGA_RC2 = True
except ImportError:
    _HAS_VEGA_RC2 = False


@dataclass
class CustomVega1MotionManagerConfig(Vega1MotionManagerConfig):
    """Custom configuration for Vega 1 motion manager."""

    urdf_path: Path = Path(str(robots.humanoid.vega_1.vega.collision_spheres_urdf))
    srdf_path: Path = Path(str(robots.humanoid.vega_1.vega.srdf))
    joints_to_lock: JointsToLockConfig = field(
        default_factory=lambda: JointsToLockConfig(
            active_groups=[
                JointRegion.HEAD,  # Lock head joints
                JointRegion.LEFT_HAND,  # Lock left hand joints
                JointRegion.RIGHT_HAND,  # Lock right hand joints
                JointRegion.TORSO,  # Lock torso joints
                JointRegion.BASE,  # Lock base joints
            ]
        )
    )


ManagerConfigDict: dict[str, type[MotionManagerConfig]] = {
    "vega-1": CustomVega1MotionManagerConfig,
}
try:
    from dexmotion.configs.core import VegaRC2MotionManagerConfig

    _HAS_VEGA_RC2 = True

    @dataclass
    class CustomVegaRC2MotionManagerConfig(VegaRC2MotionManagerConfig):
        """Custom configuration for Vega RC2 motion manager."""

        urdf_path: Path = Path(
            str(robots.humanoid.vega_rc2.vega.collision_spheres_urdf)
        )
        srdf_path: Path = Path(str(robots.humanoid.vega_rc2.vega.srdf))
        joints_to_lock: JointsToLockConfig = field(
            default_factory=lambda: JointsToLockConfig(
                active_groups=[
                    JointRegion.HEAD,  # Lock head joints
                    JointRegion.LEFT_HAND,  # Lock left hand joints
                    JointRegion.RIGHT_HAND,  # Lock right hand joints
                    JointRegion.TORSO,  # Lock torso joints
                    JointRegion.BASE,  # Lock base joints
                ]
            )
        )

    ManagerConfigDict["vega-rc2"] = CustomVegaRC2MotionManagerConfig

except ImportError:
    _HAS_VEGA_RC2 = False
