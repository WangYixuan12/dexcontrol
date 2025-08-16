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

from loguru import logger
from rich.console import Console
from rich.table import Table

from dexcontrol.utils.pb_utils import TYPE_SOFTWARE_VERSION


def show_software_version(version_info: dict[str, TYPE_SOFTWARE_VERSION]):
    """Create a Rich table for displaying firmware version information.

    Args:
        version_info: Dictionary containing version info for each component.
    """
    table = Table(title="Firmware Version")
    table.add_column("Component", style="cyan")
    table.add_column("Hardware Version")
    table.add_column("Software Version")
    table.add_column("Main Hash")
    table.add_column("Compile Time")

    for component, version in sorted(version_info.items()):
        table.add_row(
            component,
            str(version["hardware_version"]),
            str(version["software_version"]),
            str(version["main_hash"]),
            str(version["compile_time"]),
        )

    console = Console()
    console.print(table)


def show_component_status(status_info: dict[str, dict]):
    """Create a Rich table for displaying component status information.

    Args:
        status_info: Dictionary containing status info for each component.
    """
    from dexcontrol.utils.error_code import get_error_description
    from dexcontrol.utils.pb_utils import ComponentStatus

    table = Table(title="Component Status")
    table.add_column("Component", style="cyan")
    table.add_column("Connected", justify="center")
    table.add_column("Enabled", justify="center")
    table.add_column("Error", justify="left")

    status_icons = {
        True: "[green]:white_check_mark:[/green]",
        False: "[red]:x:[/red]",
        ComponentStatus.NORMAL: "[green]:white_check_mark:[/green]",
        ComponentStatus.NA: "[dim]N/A[/dim]",
    }

    # Sort components by name to ensure consistent order
    for component in sorted(status_info.keys()):
        status = status_info[component]
        # Format connection status
        connected = status_icons[status["connected"]]

        # Format enabled status
        enabled = status_icons.get(status["enabled"], "[red]:x:[/red]")

        # Format error status
        if status["error_state"] == ComponentStatus.NORMAL:
            error = "[green]:white_check_mark:[/green]"
        elif status["error_state"] == ComponentStatus.NA:
            error = "[dim]N/A[/dim]"
        else:
            # Convert error code to human-readable text
            error_code = status["error_code"]
            if isinstance(error_code, int):
                error_desc = get_error_description(component, error_code)
                # Show both raw code and description with formatting
                if error_code == 0:
                    error = "[green]:white_check_mark:[/green]"
                else:
                    # Check if this is an unknown error
                    if "Unknown" in error_desc:
                        # For unknown errors, show the hex code prominently
                        error = f"[bold red]:warning: 0x{error_code:08X}[/bold red]\n[dim italic]Unknown error code[/dim italic]"
                    else:
                        # For known errors, show both code and description
                        error = f"[bold red]:warning: 0x{error_code:08X}[/bold red]\n[yellow]{error_desc}[/yellow]"
            else:
                # Handle hex string format if provided
                try:
                    error_code_int = (
                        int(error_code, 16)
                        if isinstance(error_code, str)
                        else error_code
                    )
                    error_desc = get_error_description(component, error_code_int)
                    # Show both raw code and description with formatting
                    if error_code_int == 0:
                        error = "[green]:white_check_mark:[/green]"
                    else:
                        # Check if this is an unknown error
                        if "Unknown" in error_desc:
                            # For unknown errors, show the hex code prominently
                            error = f"[bold red]:warning: 0x{error_code_int:08X}[/bold red]\n[dim italic]Unknown error code[/dim italic]"
                        else:
                            # For known errors, show both code and description
                            error = f"[bold red]:warning: 0x{error_code_int:08X}[/bold red]\n[yellow]{error_desc}[/yellow]"
                except (ValueError, TypeError):
                    error = f"[red]{str(error_code)}[/red]"

        table.add_row(
            component,
            connected,
            enabled,
            error,
        )

    console = Console()
    console.print(table)


def show_ntp_stats(stats: dict[str, float]):
    """Display NTP statistics in a Rich table format.

    Args:
        stats: Dictionary containing NTP statistics (e.g., mean_offset, mean_rtt, etc.).
    """
    table = Table()
    table.add_column("Time Statistic", style="cyan")
    table.add_column("Value (Unit: second)", justify="right")

    for key, value in stats.items():
        # Format floats to 6 decimal places, lists as comma-separated, others as str
        if isinstance(value, float):
            value_str = f"{value:.6f}"
        elif isinstance(value, list):
            value_str = ", ".join(
                f"{v:.6f}" if isinstance(v, float) else str(v) for v in value
            )
        else:
            value_str = str(value)
        table.add_row(key, value_str)

    console = Console()
    console.print(table)

    if "offset (mean)" in stats:
        offset = stats["offset (mean)"]
        if offset > 0:
            logger.info(
                f"To synchronize: server_time ≈ local_time + {offset:.3f} second"
            )
        else:
            logger.info(
                f"To synchronize: server_time ≈ local_time - {abs(offset):.3f} second"
            )
