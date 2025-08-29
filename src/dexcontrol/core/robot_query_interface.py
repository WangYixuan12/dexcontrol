# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Zenoh query utilities for robot communication.

This module provides the ZenohQueryable class that encapsulates all zenoh-based
queries and communication with the robot server. It handles various query types
including hand type detection, version information, status queries, and control operations.
"""

import json
import time
from typing import TYPE_CHECKING, Any, Literal, cast

import zenoh
from loguru import logger

from dexcontrol.config.vega import VegaConfig, get_vega_config
from dexcontrol.core.hand import HandType
from dexcontrol.proto import dexcontrol_query_pb2
from dexcontrol.utils.os_utils import resolve_key_name
from dexcontrol.utils.pb_utils import (
    ComponentStatus,
    status_to_dict,
)
from dexcontrol.utils.viz_utils import show_component_status
from dexcontrol.utils.zenoh_utils import compute_ntp_stats, create_zenoh_session

if TYPE_CHECKING:
    from dexcontrol.config.vega import VegaConfig


class RobotQueryInterface:
    """Base class for zenoh query operations.

    This class provides a clean interface for all zenoh-based queries and
    communication operations. It maintains references to the zenoh session
    and configuration needed for queries.

    Can be used as a context manager for automatic resource cleanup:
        >>> with RobotQueryInterface.create() as interface:
        ...     version_info = interface.get_version_info()
    """

    def __init__(self, zenoh_session: zenoh.Session, configs: "VegaConfig"):
        """Initialize the RobotQueryInterface with session and config.

        Args:
            zenoh_session: Active zenoh session for communication.
            configs: Robot configuration containing query names.
        """
        self._zenoh_session: zenoh.Session = zenoh_session
        self._configs = configs
        self._owns_session = False

    @classmethod
    def create(
        cls,
        zenoh_config_file: str | None = None,
    ) -> "RobotQueryInterface":
        """Create a standalone RobotQueryInterface with its own zenoh session.

        This class method provides a convenient way to create a RobotQueryInterface
        without requiring the full Robot class. The created interface will manage
        its own zenoh session and configuration.

        Args:
            zenoh_config_file: Path to zenoh configuration file. If None,
                              uses the default configuration path.
            robot_config_path: Path to robot configuration file. If None,
                              uses default configuration for detected robot model.

        Returns:
            RobotQueryInterface instance ready for use.

        Raises:
            RuntimeError: If zenoh session initialization fails.
            ValueError: If robot configuration cannot be loaded.

        Example:
            >>> query_interface = RobotQueryInterface.create_standalone()
            >>> version_info = query_interface.get_version_info()
            >>> query_interface.close()
        """

        session = create_zenoh_session(zenoh_config_file)
        config: VegaConfig = get_vega_config()

        instance = cls(session, config)
        instance._owns_session = True
        return instance

    def close(self) -> None:
        """Close the zenoh session if owned by this instance.

        This method should be called when done using a standalone
        RobotQueryInterface to properly clean up resources.
        """
        if self._owns_session and self._zenoh_session:
            logger.debug("Closing zenoh session")
            self._zenoh_session.close()

    def __enter__(self) -> "RobotQueryInterface":
        """Enter context manager."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Exit context manager and clean up resources."""
        self.close()

    def query_hand_type(self) -> dict[str, HandType]:
        """Query the hand type information from the server.

        Returns:
            Dictionary containing hand type information for left and right hands.
            Format: {"left": hand_type_name, "right": hand_type_name}
            Possible hand types: "UNKNOWN", "HandF5D6_V1", "HandF5D6_V2"
            UNKNOWN means not connected or unknown end effector connected.

        Raises:
            RuntimeError: If hand type information cannot be retrieved.
        """
        try:
            # Sleep more time to avoice zenoh query timeout
            time.sleep(2)
            replies = self._zenoh_session.get(
                resolve_key_name(self._configs.hand_info_query_name), timeout=5.0
            )

            for reply in replies:
                if reply.ok and reply.ok.payload:
                    # Parse JSON response (not protobuf)
                    payload_bytes = reply.ok.payload.to_bytes()
                    payload_str = payload_bytes.decode("utf-8")
                    hand_info = json.loads(payload_str)

                    # Validate the expected format
                    if isinstance(hand_info, dict):
                        logger.info(f"End effector hand types: {hand_info}")
                        return {
                            "left": HandType(hand_info["left"]),
                            "right": HandType(hand_info["right"]),
                        }
                    else:
                        logger.warning(
                            f"Invalid hand type format received: {hand_info}"
                        )

            # If no valid response received, assume v1 for backward compatibility
            return {"left": HandType.HandF5D6_V1, "right": HandType.HandF5D6_V1}

        except Exception as e:
            logger.warning(f"Failed to query hand type: {e}. Assuming v1 hand types.")
            return {"left": HandType.HandF5D6_V1, "right": HandType.HandF5D6_V1}

    def query_ntp(
        self,
        sample_count: int = 30,
        show: bool = False,
        timeout: float = 1.0,
        device: Literal["soc", "jetson"] = "soc",
    ) -> dict[Literal["success", "offset", "rtt"], bool | float]:
        """Query the NTP server via zenoh for time synchronization and compute robust statistics.

        Args:
            sample_count: Number of NTP samples to request (default: 50).
            show: Whether to print summary statistics using a rich table.
            timeout: Timeout for the zenoh querier in seconds (default: 2.0).
            device: Which device to query for NTP ("soc" or "jetson").

        Returns:
            Dictionary with keys:
                - "success": True if any replies were received, False otherwise.
                - "offset": Mean offset (in seconds) after removing RTT outliers.
                - "rtt": Mean round-trip time (in seconds) after removing RTT outliers.
        """
        if device == "soc":
            ntp_key = resolve_key_name(self._configs.soc_ntp_query_name)
        elif device == "jetson":
            raise NotImplementedError("Jetson NTP query is not implemented yet")

        time_offset = []
        time_rtt = []

        querier = self._zenoh_session.declare_querier(ntp_key, timeout=timeout)
        time.sleep(0.1)

        reply_count = 0
        for i in range(sample_count):
            request = dexcontrol_query_pb2.NTPRequest()
            request.client_send_time_ns = time.time_ns()
            request.sample_count = sample_count
            request.sample_index = i
            replies = querier.get(payload=request.SerializeToString())

            for reply in replies:
                reply_count += 1
                if reply.ok and reply.ok.payload:
                    client_receive_time_ns = time.time_ns()
                    response = dexcontrol_query_pb2.NTPResponse()
                    response.ParseFromString(reply.ok.payload.to_bytes())
                    t0 = request.client_send_time_ns
                    t1 = response.server_receive_time_ns
                    t2 = response.server_send_time_ns
                    t3 = client_receive_time_ns
                    offset = ((t1 - t0) + (t2 - t3)) // 2 / 1e9
                    rtt = (t3 - t0) / 1e9
                    time_offset.append(offset)
                    time_rtt.append(rtt)
            if i < sample_count - 1:
                time.sleep(0.01)

        querier.undeclare()
        if reply_count == 0:
            return {"success": False, "offset": 0, "rtt": 0}

        stats = compute_ntp_stats(time_offset, time_rtt)
        offset = stats["offset (mean)"]
        rtt = stats["round_trip_time (mean)"]
        if show:
            from dexcontrol.utils.viz_utils import show_ntp_stats

            show_ntp_stats(stats)

        return {"success": True, "offset": offset, "rtt": rtt}

    def get_version_info(self, show: bool = True) -> dict[str, Any]:
        """Retrieve comprehensive version information using JSON interface.

        This method queries the new JSON-based version endpoint that provides:
        - Server component versions (hardware, software, compile_time, hashes)
        - Minimum required client version
        - Version compatibility information

        Args:
            show: Whether to display the version information.

        Returns:
            Dictionary containing comprehensive version information with structure:
            {
                "server": {
                    "component_name": {
                        "hardware_version": int,
                        "software_version": int,
                        "compile_time": str,
                        "main_hash": str,
                        "sub_hash": str
                    }
                },
                "client": {
                    "minimal_version": str
                }
            }

        Raises:
            RuntimeError: If version information cannot be retrieved.
        """
        try:
            replies = self._zenoh_session.get(
                resolve_key_name(self._configs.version_info_name), timeout=5.0
            )

            for reply in replies:
                if reply.ok and reply.ok.payload:
                    try:
                        # Parse JSON response
                        payload_bytes = reply.ok.payload.to_bytes()
                        payload_str = payload_bytes.decode("utf-8")
                        version_info = json.loads(payload_str)

                        # Validate expected structure
                        if (
                            isinstance(version_info, dict)
                            and "server" in version_info
                            and "client" in version_info
                        ):
                            if show:
                                self._show_version_info(version_info)
                            return version_info
                        else:
                            logger.warning(
                                f"Invalid version info format received: {version_info}"
                            )
                    except (json.JSONDecodeError, UnicodeDecodeError) as e:
                        logger.warning(f"Failed to parse version info response: {e}")
                        continue

            raise RuntimeError("No valid version information received from server")

        except Exception as e:
            raise RuntimeError(f"Failed to retrieve version information: {e}") from e

    def get_component_status(
        self, show: bool = True
    ) -> dict[str, dict[str, bool | ComponentStatus]]:
        """Retrieve status information for all components.

        Args:
            show: Whether to display the status information.

        Returns:
            Dictionary containing status information for all components.

        Raises:
            RuntimeError: If status information cannot be retrieved.
        """
        try:
            replies = self._zenoh_session.get(
                resolve_key_name(self._configs.status_info_name)
            )
            status_dict = {}
            for reply in replies:
                if reply.ok and reply.ok.payload:
                    status_bytes = reply.ok.payload.to_bytes()
                    status_msg = cast(
                        dexcontrol_query_pb2.ComponentStates,
                        dexcontrol_query_pb2.ComponentStates.FromString(status_bytes),
                    )
                    status_dict = status_to_dict(status_msg)
                    break

            if show:
                show_component_status(status_dict)
            return status_dict
        except Exception as e:
            raise RuntimeError(f"Failed to retrieve component status: {e}") from e

    def reboot_component(
        self,
        part: Literal["arm", "chassis", "torso"],
    ) -> None:
        """Reboot a specific robot component.

        Args:
            part: Component to reboot ("arm", "chassis", or "torso").

        Raises:
            ValueError: If the specified component is invalid.
            RuntimeError: If the reboot operation fails.
        """
        component_map = {
            "arm": dexcontrol_query_pb2.RebootComponent.Component.ARM,
            "chassis": dexcontrol_query_pb2.RebootComponent.Component.CHASSIS,
            "torso": dexcontrol_query_pb2.RebootComponent.Component.TORSO,
        }

        if part not in component_map:
            raise ValueError(f"Invalid component: {part}")

        try:
            query_msg = dexcontrol_query_pb2.RebootComponent(
                component=component_map[part]
            )
            self._zenoh_session.get(
                resolve_key_name(self._configs.reboot_query_name),
                payload=query_msg.SerializeToString(),
            )
            logger.info(f"Rebooting component: {part}")
        except Exception as e:
            raise RuntimeError(f"Failed to reboot component {part}: {e}") from e

    def clear_error(
        self,
        part: Literal["left_arm", "right_arm", "chassis", "head"] | str,
    ) -> None:
        """Clear error state for a specific component.

        Args:
            part: Component to clear error state for.

        Raises:
            ValueError: If the specified component is invalid.
            RuntimeError: If the error clearing operation fails.
        """
        component_map = {
            "left_arm": dexcontrol_query_pb2.ClearError.Component.LEFT_ARM,
            "right_arm": dexcontrol_query_pb2.ClearError.Component.RIGHT_ARM,
            "chassis": dexcontrol_query_pb2.ClearError.Component.CHASSIS,
            "head": dexcontrol_query_pb2.ClearError.Component.HEAD,
        }

        if part not in component_map:
            raise ValueError(f"Invalid component: {part}")

        try:
            query_msg = dexcontrol_query_pb2.ClearError(component=component_map[part])
            self._zenoh_session.get(
                resolve_key_name(self._configs.clear_error_query_name),
                handler=lambda reply: logger.info(f"Cleared error of {part}"),
                payload=query_msg.SerializeToString(),
            )
        except Exception as e:
            raise RuntimeError(
                f"Failed to clear error for component {part}: {e}"
            ) from e

    def _show_version_info(self, version_info: dict[str, Any]) -> None:
        """Display comprehensive version information in a formatted table.

        Args:
            version_info: Dictionary containing server and client version information.
        """
        from rich.console import Console
        from rich.table import Table

        console = Console()
        table = Table(title="🤖 Robot System Version Information")

        table.add_column("Component", justify="left", style="cyan", no_wrap=True)
        table.add_column("Hardware Ver.", justify="center", style="magenta")
        table.add_column("Software Ver.", justify="center", style="green")
        table.add_column("Compile Time", justify="center", style="yellow")
        table.add_column("Main Hash", justify="center", style="blue")
        table.add_column("Sub Hash", justify="center", style="red")

        # Display server components
        server_info = version_info.get("server", {})
        for component, info in server_info.items():
            if isinstance(info, dict):
                table.add_row(
                    component,
                    str(info.get("hardware_version", "N/A")),
                    str(info.get("software_version", "N/A")),
                    str(info.get("compile_time", "N/A")),
                    str(info.get("main_hash", "N/A")[:8])
                    if info.get("main_hash")
                    else "N/A",
                    str(info.get("sub_hash", "N/A")[:8])
                    if info.get("sub_hash")
                    else "N/A",
                )

        console.print(table)
