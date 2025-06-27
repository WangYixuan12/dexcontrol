# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

from hydra.core.config_store import ConfigStore

from .vega import Vega1Config, VegaConfig

# Register the configs
cs = ConfigStore.instance()
cs.store(name="vega", node=VegaConfig)
cs.store(name="vega-rc2", node=VegaConfig)
cs.store(name="vega-1", node=Vega1Config)
