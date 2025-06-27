# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

from dataclasses import dataclass

from dexmotion.configs.planning import OMPLPlannerConfig


@dataclass
class CustomOMPLPlannerConfig(OMPLPlannerConfig):
    """Configuration for OMPL-based planning."""

    # Whether to check for self-collisions during planning
    enable_self_collision_avoidance: bool = True
