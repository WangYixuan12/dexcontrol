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

from dexmotion.configs.ik import LocalPinkIKConfig


@dataclass
class CustomLocalPinkIKConfig(LocalPinkIKConfig):
    """Custom configuration for Local Pink IK solver with overridden defaults."""

    qp_solver: str = "quadprog"  # Override default dt
    # Add any other parameter overrides here
