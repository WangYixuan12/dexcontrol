# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Utility functions for displaying information in a Rich table format."""

from typing import Literal

from rich.console import Console
from rich.table import Table

TYPE_VERSION = dict[Literal["hardware_version", "software_version", "main_hash"], str]


def show_software_version(version_info: dict[str, TYPE_VERSION]) -> None:
    """Create a Rich table for displaying firmware version information.

    Args:
        version_info: Dictionary containing version info for each component.
    """
    table = Table(title="Firmware Version")
    table.add_column("Component", style="cyan")
    table.add_column("Hardware Version")
    table.add_column("Software Version")
    table.add_column("Main Hash")

    for component, version in sorted(version_info.items()):
        table.add_row(
            component,
            str(version["hardware_version"]),
            str(version["software_version"]),
            str(version["main_hash"]),
        )

    console = Console()
    console.print(table)


def show_component_status(status_info: dict[str, dict]) -> None:
    """Create a Rich table for displaying component status information.

    Args:
        status_info: Dictionary containing status info for each component.
    """
    from dexcontrol.utils.pb_utils import ComponentStatus

    table = Table(title="Component Status")
    table.add_column("Component", style="cyan")
    table.add_column("Connected", justify="center")
    table.add_column("Enabled", justify="center")
    table.add_column("Error", justify="center")

    status_icons = {
        True: ":white_check_mark:",
        False: ":x:",
        ComponentStatus.NORMAL: ":white_check_mark:",
        ComponentStatus.NA: "N/A",
    }

    # Sort components by name to ensure consistent order
    for component in sorted(status_info.keys()):
        status = status_info[component]
        # Format connection status
        connected = status_icons[status["connected"]]

        # Format enabled status
        enabled = status_icons.get(status["enabled"], ":x:")

        # Format error status
        if status["error_state"] == ComponentStatus.NORMAL:
            error = ":white_check_mark:"
        elif status["error_state"] == ComponentStatus.NA:
            error = "N/A"
        else:
            error = status["error_code"]

        table.add_row(
            component,
            connected,
            enabled,
            error,
        )

    console = Console()
    console.print(table)
