# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Miscellaneous robot components module.

This module provides classes for various auxiliary robot components such as Battery,
EStop (emergency stop), and UltraSonicSensor.
"""

import os
import threading
import time
from typing import TypeVar

import zenoh
from google.protobuf.message import Message
from loguru import logger
from rich.console import Console
from rich.table import Table

from dexcontrol.config.core import BatteryConfig, EStopConfig, HeartbeatConfig
from dexcontrol.core.component import RobotComponent
from dexcontrol.proto import dexcontrol_msg_pb2, dexcontrol_query_pb2
from dexcontrol.utils.constants import DISABLE_HEARTBEAT_ENV_VAR
from dexcontrol.utils.os_utils import resolve_key_name
from dexcontrol.utils.subscribers.generic import GenericZenohSubscriber

# Type variable for Message subclasses
M = TypeVar("M", bound=Message)


class Battery(RobotComponent):
    """Battery component that monitors and displays battery status information.

    This class provides methods to monitor battery state including voltage, current,
    temperature and power consumption. It can display the information in either a
    formatted rich table or plain text format.

    Attributes:
        _console: Rich console instance for formatted output.
        _monitor_thread: Background thread for battery monitoring.
        _shutdown_event: Event to signal thread shutdown.
    """

    def __init__(self, configs: BatteryConfig, zenoh_session: zenoh.Session) -> None:
        """Initialize the Battery component.

        Args:
            configs: Battery configuration containing subscription topics.
            zenoh_session: Active Zenoh session for communication.
        """
        super().__init__(
            configs.state_sub_topic, zenoh_session, dexcontrol_msg_pb2.BMSState
        )
        self._console = Console()
        self._shutdown_event = threading.Event()
        self._monitor_thread = threading.Thread(
            target=self._battery_monitor, daemon=True
        )
        self._monitor_thread.start()

    def _battery_monitor(self) -> None:
        """Background thread that periodically checks battery level and warns if low."""
        while not self._shutdown_event.is_set():
            try:
                if self.is_active():
                    battery_level = self.get_status()["percentage"]
                    if battery_level < 20:
                        logger.warning(
                            f"Battery level is low ({battery_level:.1f}%). "
                            "Please charge the battery."
                        )
            except Exception as e:
                logger.debug(f"Battery monitor error: {e}")

            # Check every 30 seconds (low frequency)
            self._shutdown_event.wait(30.0)

    def get_status(self) -> dict[str, float]:
        """Gets the current battery state information.

        Returns:
            Dictionary containing battery metrics including:
                - percentage: Battery charge level (0-100)
                - temperature: Battery temperature in Celsius
                - current: Current draw in Amperes
                - voltage: Battery voltage
                - power: Power consumption in Watts
        """
        state = self._get_state()
        return {
            "percentage": float(state.percentage),
            "temperature": float(state.temperature),
            "current": float(state.current),
            "voltage": float(state.voltage),
            "power": float(state.current * state.voltage),
        }

    def show(self) -> None:
        """Displays the current battery status as a formatted table with color indicators."""
        state = self._get_state()

        table = Table(title="Battery Status")
        table.add_column("Parameter", style="cyan")
        table.add_column("Value")

        battery_style = self._get_battery_level_style(state.percentage)
        table.add_row("Battery Level", f"[{battery_style}]{state.percentage:.1f}%[/]")

        temp_style = self._get_temperature_style(state.temperature)
        table.add_row("Temperature", f"[{temp_style}]{state.temperature:.1f}°C[/]")

        power = state.current * state.voltage
        power_style = self._get_power_style(power)
        table.add_row(
            "Power",
            f"[{power_style}]{power:.2f}W[/] ([blue]{state.current:.2f}A[/] "
            f"× [blue]{state.voltage:.2f}V[/])",
        )

        self._console.print(table)

    def shutdown(self) -> None:
        """Shuts down the battery component and stops monitoring thread."""
        self._shutdown_event.set()
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=2.0)  # Extended timeout
            if self._monitor_thread.is_alive():
                logger.warning("Battery monitor thread did not terminate cleanly")
        super().shutdown()

    @staticmethod
    def _get_battery_level_style(percentage: float) -> str:
        """Returns the appropriate style based on battery percentage.

        Args:
            percentage: Battery charge level (0-100).

        Returns:
            Rich text style string for color formatting.
        """
        if percentage < 30:
            return "bold red"
        elif percentage < 60:
            return "bold yellow"
        else:
            return "bold dark_green"

    @staticmethod
    def _get_temperature_style(temperature: float) -> str:
        """Returns the appropriate style based on temperature value.

        Args:
            temperature: Battery temperature in Celsius.

        Returns:
            Rich text style string for color formatting.
        """
        if temperature < -1:
            return "bold red"  # Too cold
        elif temperature <= 30:
            return "bold dark_green"  # Normal range
        elif temperature <= 38:
            return "bold orange"  # Getting warm
        else:
            return "bold red"  # Too hot

    @staticmethod
    def _get_power_style(power: float) -> str:
        """Returns the appropriate style based on power consumption.

        Args:
            power: Power consumption in Watts.

        Returns:
            Rich text style string for color formatting.
        """
        if power < 200:
            return "bold dark_green"
        elif power <= 500:
            return "bold orange"
        else:
            return "bold red"


class EStop(RobotComponent):
    """EStop component that monitors and controls emergency stop functionality.

    This class provides methods to monitor EStop state and activate/deactivate
    the software emergency stop.

    Attributes:
        _estop_query_name: Zenoh query name for setting EStop state.
        _monitor_thread: Background thread for EStop monitoring.
        _shutdown_event: Event to signal thread shutdown.
    """

    def __init__(
        self,
        configs: EStopConfig,
        zenoh_session: zenoh.Session,
    ) -> None:
        """Initialize the EStop component.

        Args:
            configs: EStop configuration containing subscription topics.
            zenoh_session: Active Zenoh session for communication.
        """
        super().__init__(
            configs.state_sub_topic, zenoh_session, dexcontrol_msg_pb2.EStopState
        )
        self._estop_query_name = configs.estop_query_name
        self._shutdown_event = threading.Event()
        self._monitor_thread = threading.Thread(target=self._estop_monitor, daemon=True)
        self._monitor_thread.start()

    def _estop_monitor(self) -> None:
        """Background thread that continuously monitors EStop button state."""
        while not self._shutdown_event.is_set():
            try:
                if self.is_active() and self.is_button_pressed():
                    logger.critical(
                        "E-STOP BUTTON PRESSED! Exiting program immediately."
                    )
                    # Don't call self.shutdown() here as it would try to join the current thread
                    # os._exit(1) will terminate the entire process immediately
                    os._exit(1)
            except Exception as e:
                logger.debug(f"EStop monitor error: {e}")

            # Check every 100ms for responsive emergency stop
            self._shutdown_event.wait(0.1)

    def _set_estop(self, enable: bool) -> None:
        """Sets the software emergency stop (E-Stop) state of the robot.

        This controls the software E-Stop, which is separate from the physical button
        on the robot. The robot will stop if either the software or hardware E-Stop is
        activated.

        Args:
            enable: If True, activates the software E-Stop. If False, deactivates it.
        """
        query_msg = dexcontrol_query_pb2.SetEstop(enable=enable)
        self._zenoh_session.get(
            resolve_key_name(self._estop_query_name),
            handler=lambda reply: logger.info(f"Set E-Stop to {enable}"),
            payload=query_msg.SerializeToString(),
        )

    def get_status(self) -> dict[str, bool]:
        """Gets the current EStop state information.

        Returns:
            Dictionary containing EStop metrics including:
                - button_pressed: EStop button pressed
                - software_estop_enabled: Software EStop enabled
        """
        state = self._get_state()
        return {
            "button_pressed": state.button_pressed,
            "software_estop_enabled": state.software_estop_enabled,
        }

    def is_button_pressed(self) -> bool:
        """Checks if the EStop button is pressed."""
        return self._get_state().button_pressed

    def is_software_estop_enabled(self) -> bool:
        """Checks if the software EStop is enabled."""
        return self._get_state().software_estop_enabled

    def activate(self) -> None:
        """Activates the software emergency stop (E-Stop)."""
        self._set_estop(True)

    def deactivate(self) -> None:
        """Deactivates the software emergency stop (E-Stop)."""
        self._set_estop(False)

    def toggle(self) -> None:
        """Toggles the software emergency stop (E-Stop) state of the robot."""
        self._set_estop(not self.is_software_estop_enabled())

    def shutdown(self) -> None:
        """Shuts down the EStop component and stops monitoring thread."""
        self._shutdown_event.set()
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=2.0)  # Extended timeout
            if self._monitor_thread.is_alive():
                logger.warning("EStop monitor thread did not terminate cleanly")
        super().shutdown()

    def show(self) -> None:
        """Displays the current EStop status as a formatted table with color indicators."""
        state = self._get_state()

        table = Table(title="E-Stop Status")
        table.add_column("Parameter", style="cyan")
        table.add_column("Value")

        button_style = "bold red" if state.button_pressed else "bold dark_green"
        table.add_row("Button Pressed", f"[{button_style}]{state.button_pressed}[/]")

        software_style = (
            "bold red" if state.software_estop_enabled else "bold dark_green"
        )
        table.add_row(
            "Software E-Stop Enabled",
            f"[{software_style}]{state.software_estop_enabled}[/]",
        )

        console = Console()
        console.print(table)


class Heartbeat:
    """Heartbeat monitor that ensures the low-level controller is functioning properly.

    This class monitors a heartbeat signal from the low-level controller and exits
    the program immediately if no heartbeat is received within the specified timeout.
    This provides a critical safety mechanism to prevent the robot from operating
    when the low-level controller is not functioning properly.

    Attributes:
        _subscriber: Zenoh subscriber for heartbeat data.
        _monitor_thread: Background thread for heartbeat monitoring.
        _shutdown_event: Event to signal thread shutdown.
        _timeout_seconds: Timeout in seconds before triggering emergency exit.
        _enabled: Whether heartbeat monitoring is enabled.
        _paused: Whether heartbeat monitoring is temporarily paused.
    """

    def __init__(
        self,
        configs: HeartbeatConfig,
        zenoh_session: zenoh.Session,
    ) -> None:
        """Initialize the Heartbeat monitor.

        Args:
            configs: Heartbeat configuration containing topic and timeout settings.
            zenoh_session: Active Zenoh session for communication.
        """
        self._timeout_seconds = configs.timeout_seconds
        self._enabled = configs.enabled
        self._paused = False
        self._paused_lock = threading.Lock()
        self._shutdown_event = threading.Event()
        if not self._enabled:
            logger.info("Heartbeat monitoring is DISABLED via configuration")
            # Create a dummy subscriber that's never active
            self._subscriber = None
            self._monitor_thread = None
            return

        # Create a generic subscriber for the heartbeat topic
        self._subscriber = GenericZenohSubscriber(
            topic=configs.heartbeat_topic,
            zenoh_session=zenoh_session,
            decoder=self._decode_heartbeat,
            name="heartbeat_monitor",
            enable_fps_tracking=False,
        )

        # Start monitoring thread
        self._monitor_thread = threading.Thread(
            target=self._heartbeat_monitor, daemon=True
        )
        self._monitor_thread.start()

        logger.info(f"Heartbeat monitor started with {self._timeout_seconds}s timeout")

    def _decode_heartbeat(self, data: zenoh.ZBytes) -> float:
        """Decode heartbeat data from raw bytes.

        Args:
            data: Raw Zenoh bytes containing heartbeat value.

        Returns:
            Decoded heartbeat timestamp value in seconds.
        """
        timestamp = int.from_bytes(data.to_bytes(), byteorder="little")
        # convert it from microseconds to seconds
        return timestamp / 1000000.0

    def _heartbeat_monitor(self) -> None:
        """Background thread that continuously monitors heartbeat signal."""
        if not self._enabled or self._subscriber is None:
            return

        while not self._shutdown_event.is_set():
            try:
                # Skip monitoring if paused
                with self._paused_lock:
                    is_paused = self._paused
                if is_paused:
                    self._shutdown_event.wait(0.1)
                    continue

                # Check if fresh data is being received within the timeout period
                if self._subscriber.is_active():
                    if not self._subscriber.is_data_fresh(self._timeout_seconds):
                        time_since_fresh = self._subscriber.get_time_since_last_data()
                        if time_since_fresh is not None:
                            logger.critical(
                                f"HEARTBEAT TIMEOUT! No fresh heartbeat data received for {time_since_fresh:.2f}s "
                                f"(timeout: {self._timeout_seconds}s). Low-level controller may have failed. "
                                "Exiting program immediately for safety."
                            )
                        else:
                            logger.critical(
                                f"HEARTBEAT TIMEOUT! No heartbeat data ever received "
                                f"(timeout: {self._timeout_seconds}s). Low-level controller may have failed. "
                                "Exiting program immediately for safety."
                            )
                        # Exit immediately for safety
                        os._exit(1)

                # Check every 50ms for responsive monitoring
                self._shutdown_event.wait(0.05)

            except Exception as e:
                logger.error(f"Heartbeat monitor error: {e}")
                # Continue monitoring even if there's an error
                self._shutdown_event.wait(0.1)

    def pause(self) -> None:
        """Pause heartbeat monitoring temporarily.

        When paused, the heartbeat monitor will not check for timeouts or exit
        the program. This is useful for scenarios where you need to temporarily
        disable safety monitoring (e.g., during system maintenance or testing).

        Warning: Use with caution as this disables a critical safety mechanism.
        """
        if not self._enabled:
            logger.warning("Cannot pause heartbeat monitoring - it's already disabled")
            return

        with self._paused_lock:
            self._paused = True
        logger.warning(
            "Heartbeat monitoring PAUSED - safety mechanism temporarily disabled"
        )

    def resume(self) -> None:
        """Resume heartbeat monitoring after being paused.

        This re-enables the heartbeat timeout checking that was temporarily
        disabled by pause(). The monitor will immediately start checking
        for fresh heartbeat data again.
        """
        if not self._enabled:
            logger.warning("Cannot resume heartbeat monitoring - it's disabled")
            return

        with self._paused_lock:
            if not self._paused:
                logger.info("Heartbeat monitoring is already active")
                return
            self._paused = False
        # sleep for some time to make sure the heartbeat subscriber is resumed
        time.sleep(self._timeout_seconds)
        logger.info("Heartbeat monitoring RESUMED - safety mechanism re-enabled")

    def is_paused(self) -> bool:
        """Check if heartbeat monitoring is currently paused.

        Returns:
            True if monitoring is paused, False if active or disabled.
        """
        with self._paused_lock:
            return self._paused

    def get_status(self) -> dict[str, bool | float | float | None]:
        """Gets the current heartbeat status information.

        Returns:
            Dictionary containing heartbeat metrics including:
                - is_active: Whether heartbeat signal is being received (bool)
                - last_value: Last received heartbeat value (float | None)
                - time_since_last: Time since last fresh data in seconds (float | None)
                - timeout_seconds: Configured timeout value (float)
                - enabled: Whether heartbeat monitoring is enabled (bool)
                - paused: Whether heartbeat monitoring is paused (bool)
        """
        if not self._enabled or self._subscriber is None:
            return {
                "is_active": False,
                "last_value": None,
                "time_since_last": None,
                "timeout_seconds": self._timeout_seconds,
                "enabled": False,
                "paused": False,
            }

        last_value = self._subscriber.get_latest_data()
        time_since_last = self._subscriber.get_time_since_last_data()

        with self._paused_lock:
            paused = self._paused

        return {
            "is_active": self._subscriber.is_active(),
            "last_value": last_value,
            "time_since_last": time_since_last,
            "timeout_seconds": self._timeout_seconds,
            "enabled": True,
            "paused": paused,
        }

    def is_active(self) -> bool:
        """Check if heartbeat signal is being received.

        Returns:
            True if heartbeat is active, False otherwise.
        """
        if not self._enabled or self._subscriber is None:
            return False
        return self._subscriber.is_active()

    def shutdown(self) -> None:
        """Shuts down the heartbeat monitor and stops monitoring thread."""
        if not self._enabled:
            return
        self._shutdown_event.set()
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=2.0)  # Extended timeout
            if self._monitor_thread.is_alive():
                logger.warning("Heartbeat monitor thread did not terminate cleanly")
        if self._subscriber:
            self._subscriber.shutdown()

    def show(self) -> None:
        """Displays the current heartbeat status as a formatted table with color indicators."""
        status = self.get_status()

        table = Table(title="Heartbeat Monitor Status")
        table.add_column("Parameter", style="cyan")
        table.add_column("Value")

        # Enabled status
        enabled = status.get("enabled", True)
        if not enabled:
            table.add_row("Status", "[yellow]DISABLED[/]")
            table.add_row(
                "Reason", f"[yellow]Disabled via {DISABLE_HEARTBEAT_ENV_VAR}[/]"
            )
            console = Console()
            console.print(table)
            return

        # Paused status
        paused = status.get("paused", False)
        if paused:
            table.add_row("Status", "[yellow]PAUSED[/]")
        else:
            # Active status
            active_style = "bold dark_green" if status["is_active"] else "bold red"
            table.add_row("Signal Active", f"[{active_style}]{status['is_active']}[/]")

        # Last heartbeat value
        last_value = status["last_value"]
        if last_value is not None:
            table.add_row("Last Value", f"[blue]{last_value:.6f}s[/]")
        else:
            table.add_row("Last Value", "[red]No data[/]")

        # Time since last heartbeat
        time_since = status["time_since_last"]
        timeout_seconds = status["timeout_seconds"]
        if time_since is not None and isinstance(timeout_seconds, (int, float)):
            time_style = (
                "bold red" if time_since > timeout_seconds * 0.8 else "bold dark_green"
            )
            table.add_row("Time Since Last", f"[{time_style}]{time_since:.2f}s[/]")
        else:
            table.add_row("Time Since Last", "[yellow]N/A[/]")

        # Timeout setting
        table.add_row("Timeout", f"[blue]{timeout_seconds}s[/]")

        console = Console()
        console.print(table)
