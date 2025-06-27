# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

from typing import Any

from loguru import logger


def print_dict(d: dict[str, Any], indent: int = 0) -> None:
    """Print a nested dictionary with nice formatting.

    Recursively prints dictionary contents with proper indentation.

    Args:
        d: Dictionary to print.
        indent: Current indentation level in spaces.
    """
    indent_str = " " * indent
    for key, value in d.items():
        if isinstance(value, dict):
            logger.info(f"{indent_str}{key}:")
            print_dict(value, indent + 4)
        else:
            logger.info(f"{indent_str}{key}: {value}")
