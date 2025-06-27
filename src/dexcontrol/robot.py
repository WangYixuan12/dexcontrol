# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Main robot interface module.

This module provides the main Robot class that serves as the primary interface for
controlling and monitoring a robot system. It handles component initialization,
status monitoring, and system-wide operations.

The Robot class manages initialization and coordination of various robot components
including arms, hands, head, chassis, torso, and sensors. It provides methods for
system-wide operations like status monitoring, trajectory execution, and component
control.
"""

from __future__ import annotations

import os
import signal
import threading
import time
import weakref
from typing import (
    TYPE_CHECKING,
    Any,
    Final,
    Literal,
    cast,
)

import hydra.utils
import numpy as np
import omegaconf
import zenoh
from loguru import logger
from rich.console import Console
from rich.table import Table

import dexcontrol
from dexcontrol.config.vega import VegaConfig, get_vega_config
from dexcontrol.core.component import RobotComponent
from dexcontrol.proto import dexcontrol_query_pb2
from dexcontrol.sensors import Sensors
from dexcontrol.utils.constants import ROBOT_NAME_ENV_VAR
from dexcontrol.utils.os_utils import get_robot_model, resolve_key_name
from dexcontrol.utils.pb_utils import (
    ComponentStatus,
    software_version_to_dict,
    status_to_dict,
)
from dexcontrol.utils.rate_limiter import RateLimiter
from dexcontrol.utils.trajectory_utils import generate_linear_trajectory
from dexcontrol.utils.viz_utils import show_component_status, show_software_version

if TYPE_CHECKING:
    from dexcontrol.core.arm import Arm
    from dexcontrol.core.chassis import Chassis
    from dexcontrol.core.hand import Hand
    from dexcontrol.core.head import Head
    from dexcontrol.core.misc import Battery, EStop, Heartbeat
    from dexcontrol.core.torso import Torso


# Global registry to track active Robot instances for signal handling
_active_robots: weakref.WeakSet[Robot] = weakref.WeakSet()
_signal_handlers_registered: bool = False


def _signal_handler(signum: int, frame: Any) -> None:
    """Signal handler to shutdown all active Robot instances.

    Args:
        signum: Signal number received (e.g., SIGINT, SIGTERM).
        frame: Current stack frame (unused).
    """
    logger.info(f"Received signal {signum}, shutting down all active robots...")
    # Create a list copy to avoid modification during iteration
    robots_to_shutdown = list(_active_robots)
    for robot in robots_to_shutdown:
        logger.info(f"Shutting down robot: {robot}")
        try:
            robot.shutdown()
        except Exception as e:  # pylint: disable=broad-except
            logger.error(f"Error during robot shutdown: {e}", exc_info=True)
    logger.info("All robots shutdown complete")
    os._exit(1)


def _register_signal_handlers() -> None:
    """Register signal handlers for graceful shutdown.

    This function ensures signal handlers are registered only once and sets up
    handlers for SIGINT (Ctrl+C), SIGTERM, and SIGHUP (on Unix systems).
    """
    global _signal_handlers_registered
    if _signal_handlers_registered:
        return

    # Register handlers for common termination signals
    signal.signal(signal.SIGINT, _signal_handler)  # Ctrl+C
    signal.signal(signal.SIGTERM, _signal_handler)  # Termination signal

    # On Unix systems, also handle SIGHUP
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, _signal_handler)

    _signal_handlers_registered = True


class ComponentConfig(omegaconf.DictConfig):
    """Type hints for component configuration."""

    _target_: str
    configs: dict[str, Any]


class Robot:
    """Main interface class for robot control and monitoring.

    This class serves as the primary interface for interacting with a robot system.
    It manages initialization and coordination of various robot components including
    arms, hands, head, chassis, torso, and sensors. It provides methods for
    system-wide operations like status monitoring, trajectory execution, and component
    control.

    The Robot class supports context manager usage and automatic cleanup on program
    interruption through signal handlers.

    Example usage:
        # Using context manager (recommended)
        with Robot() as robot:
            robot.set_joint_pos({"left_arm": [0, 0, 0, 0, 0, 0, 0]})

        # Manual usage with explicit shutdown
        robot = Robot()
        try:
            robot.set_joint_pos({"left_arm": [0, 0, 0, 0, 0, 0, 0]})
        finally:
            robot.shutdown()

    Attributes:
        left_arm: Left arm component interface.
        right_arm: Right arm component interface.
        left_hand: Left hand component interface.
        right_hand: Right hand component interface.
        head: Head component interface.
        chassis: Chassis component interface.
        torso: Torso component interface.
        battery: Battery monitoring interface.
        estop: Emergency stop interface.
        heartbeat: Heartbeat monitoring interface.
        sensors: Sensor systems interface.
    """

    # Type annotations for dynamically created attributes
    left_arm: Arm
    right_arm: Arm
    left_hand: Hand
    right_hand: Hand
    head: Head
    chassis: Chassis
    torso: Torso
    battery: Battery
    estop: EStop
    heartbeat: Heartbeat
    sensors: Sensors
    _shutdown_called: bool = False

    def __init__(
        self,
        robot_model: str | None = None,
        configs: VegaConfig | None = None,
        zenoh_config_file: str | None = None,
        auto_shutdown: bool = True,
    ) -> None:
        """Initializes the Robot with the given configuration.

        Args:
            robot_model: Optional robot variant name (e.g., "vega-rc2", "vega-1").
                If configs is None, this will be used to get the appropriate config.
                Ignored if configs is provided.
            configs: Configuration parameters for all robot components.
                If None, will use the configuration specified by robot_model.
            zenoh_config_file: Optional path to the zenoh config file.
                Defaults to None to use system defaults.
            auto_shutdown: Whether to automatically register signal handlers for
                graceful shutdown on program interruption. Default is True.

        Raises:
            RuntimeError: If any critical component fails to become active within timeout.
            ValueError: If robot_model is invalid or configs cannot be loaded.
        """
        self._components: list[RobotComponent] = []

        if robot_model is None:
            robot_model = get_robot_model()
        self._robot_model: Final[str] = robot_model

        try:
            self._configs: Final[VegaConfig] = configs or get_vega_config(robot_model)
        except Exception as e:
            raise ValueError(f"Failed to load robot configuration: {e}") from e

        try:
            self._zenoh_session: Final[zenoh.Session] = self._init_zenoh_session(
                zenoh_config_file
            )
        except Exception as e:
            raise RuntimeError(f"Failed to initialize zenoh session: {e}") from e

        self._robot_name: Final[str] = os.getenv(ROBOT_NAME_ENV_VAR, "robot")
        self._pv_components: Final[list[str]] = [
            "left_hand",
            "right_hand",
            "head",
            "torso",
        ]

        # Register for automatic shutdown on signals if enabled
        if auto_shutdown:
            _register_signal_handlers()
            _active_robots.add(self)

        self._print_initialization_info(robot_model)

        # Initialize robot body components dynamically
        try:
            config_dict = omegaconf.OmegaConf.to_container(self._configs, resolve=True)
            if not isinstance(config_dict, dict):
                raise ValueError("Invalid configuration format")
            self._init_components(cast(dict[str, Any], config_dict))
        except Exception as e:
            self.shutdown()  # Clean up on initialization failure
            raise RuntimeError(f"Failed to initialize robot components: {e}") from e

        # Ensure all components are active
        try:
            self._wait_for_components()
        except Exception as e:
            self.shutdown()  # Clean up on initialization failure
            raise RuntimeError(f"Failed to activate components: {e}") from e

        try:
            self.sensors = Sensors(self._configs.sensors, self._zenoh_session)
            self.sensors.wait_for_all_active()
        except Exception as e:
            self.shutdown()  # Clean up on initialization failure
            raise RuntimeError(f"Failed to initialize sensors: {e}") from e

        # Set default modes
        try:
            self._set_default_modes()
        except Exception as e:
            self.shutdown()  # Clean up on initialization failure
            raise RuntimeError(f"Failed to set default modes: {e}") from e

    @property
    def robot_model(self) -> str:
        """Get the robot model.

        Returns:
            The robot model.
        """
        return self._robot_model

    @property
    def robot_name(self) -> str:
        """Get the robot name.

        Returns:
            The robot name.
        """
        return self._robot_name

    def __enter__(self) -> Robot:
        """Context manager entry.

        Returns:
            Self reference for context management.
        """
        return self

    def __exit__(
        self, exc_type: type | None, exc_val: Exception | None, exc_tb: Any
    ) -> None:
        """Context manager exit with automatic shutdown.

        Args:
            exc_type: Type of the exception that occurred, if any.
            exc_val: Exception instance that occurred, if any.
            exc_tb: Traceback of the exception that occurred, if any.
        """
        self.shutdown()

    def __del__(self) -> None:
        """Destructor to ensure cleanup."""
        if not self._shutdown_called:
            logger.warning(
                "Robot instance being destroyed without explicit shutdown call"
            )
            self.shutdown()

    def _print_initialization_info(self, robot_model: str | None) -> None:
        """Print initialization information.

        Args:
            robot_model: The robot model being initialized.
        """
        console = Console()
        table = Table(show_header=False)
        table.add_column(style="cyan", no_wrap=True)
        table.add_column(style="white")

        table.add_row("Robot Name", str(self._robot_name))
        if robot_model:
            table.add_row("Robot Model", str(robot_model))
        table.add_row("Communication Config", str(dexcontrol.COMM_CFG_PATH))

        console.print(table)

    def _init_components(self, config_dict: dict[str, Any]) -> None:
        """Initialize robot components from configuration.

        Args:
            config_dict: Configuration dictionary for components.

        Raises:
            RuntimeError: If component initialization fails.
        """
        for component_name, component_config in config_dict.items():
            if component_name == "sensors":
                continue

            try:
                component_config = getattr(self._configs, str(component_name))
                if (
                    not hasattr(component_config, "_target_")
                    or not component_config._target_
                ):
                    continue

                temp_config = omegaconf.OmegaConf.create(
                    {
                        "_target_": component_config._target_,
                        "configs": {
                            k: v for k, v in component_config.items() if k != "_target_"
                        },
                    }
                )
                component_instance = hydra.utils.instantiate(
                    temp_config, zenoh_session=self._zenoh_session
                )
                setattr(self, str(component_name), component_instance)
            except Exception as e:
                raise RuntimeError(
                    f"Failed to initialize component {component_name}: {e}"
                ) from e

    def _set_default_modes(self) -> None:
        """Set default control modes for robot components.

        Raises:
            RuntimeError: If setting default mode fails for any component.
        """
        for arm in ["left_arm", "right_arm"]:
            if component := getattr(self, arm, None):
                try:
                    component.set_mode("position")
                except Exception as e:
                    raise RuntimeError(
                        f"Failed to set default mode for {arm}: {e}"
                    ) from e

        if head := getattr(self, "head", None):
            try:
                head.set_mode("enable")
                home_pos = head.get_predefined_pose("home")
                home_pos = self.compensate_torso_pitch(home_pos, "head")
                head.set_joint_pos(home_pos)
            except Exception as e:
                raise RuntimeError(f"Failed to set default mode for head: {e}") from e

    def _wait_for_components(self) -> None:
        """Waits for all critical components to become active.

        This method monitors the activation status of essential robot components
        and ensures they are properly initialized before proceeding.

        Raises:
            RuntimeError: If any component fails to activate within the timeout period
                        or if shutdown is triggered during activation.
        """
        component_names: Final[list[str]] = [
            "left_arm",
            "right_arm",
            "left_hand",
            "right_hand",
            "head",
            "chassis",
            "torso",
            "battery",
            "estop",
        ]
        if self._configs.heartbeat.enabled:
            component_names.append("heartbeat")

        console = Console()
        actives: list[bool] = []
        timeout_sec: Final[int] = 8
        check_interval: Final[float] = 0.1  # Check every 100ms

        with console.status(
            "[bold green]Waiting for components to become active..."
        ) as status:
            for name in component_names:
                # Check if shutdown was triggered
                if self._shutdown_called:
                    raise RuntimeError("Shutdown triggered during component activation")

                status.update(f"Waiting for {name} to become active...")
                if component := getattr(self, name, None):
                    start_time = time.monotonic()
                    while True:
                        if self._shutdown_called:
                            raise RuntimeError(
                                "Shutdown triggered during component activation"
                            )

                        # Try a quick active check first
                        if component.is_active():
                            actives.append(True)
                            self._components.append(component)
                            break

                        # Check if we've exceeded timeout
                        if time.monotonic() - start_time >= timeout_sec:
                            console.log(f"{name} failed to become active")
                            actives.append(False)
                            break

                        # Wait a short interval before checking again
                        time.sleep(check_interval)

        if not all(actives):
            self.shutdown()
            inactive = [
                name for name, active in zip(component_names, actives) if not active
            ]
            raise RuntimeError(
                f"Components failed to activate within {timeout_sec}s: {', '.join(inactive)}"
            )

        logger.info("All components activated successfully")

    def _init_zenoh_session(self, zenoh_config_file: str | None) -> zenoh.Session:
        """Initializes Zenoh communication session.

        Args:
            zenoh_config_file: Path to zenoh configuration file.

        Returns:
            Initialized zenoh session.

        Raises:
            RuntimeError: If zenoh session initialization fails.
        """
        try:
            config_path = zenoh_config_file or self._get_default_zenoh_config()
            if config_path is None:
                logger.warning("Using default zenoh config settings")
                return zenoh.open(zenoh.Config())
            return zenoh.open(zenoh.Config.from_file(config_path))
        except Exception as e:
            raise RuntimeError(f"Failed to initialize zenoh session: {e}") from e

    @staticmethod
    def _get_default_zenoh_config() -> str | None:
        """Gets the default zenoh configuration file path.

        Returns:
            Path to default config file if it exists, None otherwise.
        """
        default_path = dexcontrol.COMM_CFG_PATH
        if not default_path.exists():
            logger.warning(f"Zenoh config file not found at {default_path}")
            logger.warning("Please use dextop to set up the zenoh config file")
            return None
        return str(default_path)

    def shutdown(self) -> None:
        """Cleans up and closes all component connections.

        This method ensures proper cleanup of all components and communication
        channels. It is automatically called when using the context manager
        or when the object is garbage collected.
        """
        if self._shutdown_called:
            logger.warning("Shutdown already called, skipping")
            return

        logger.info("Shutting down robot...")
        self._shutdown_called = True

        # Remove from active robots registry
        try:
            _active_robots.discard(self)
        except Exception:  # pylint: disable=broad-except
            pass  # WeakSet may already have removed it

        # First, stop all components that have stop methods to halt ongoing operations
        for component in self._components:
            if component is not None:
                try:
                    if hasattr(component, "stop"):
                        method = getattr(component, "stop")
                        if callable(method):
                            method()
                except Exception as e:  # pylint: disable=broad-except
                    logger.error(
                        f"Error stopping component {component.__class__.__name__}: {e}"
                    )

        # Longer delay to ensure all stop operations complete and background threads settle
        time.sleep(0.3)

        # Shutdown sensors first (they may have background threads)
        try:
            self.sensors.shutdown()
            # Give extra time for all sensor subscribers to undeclare cleanly
            time.sleep(0.5)
        except Exception as e:  # pylint: disable=broad-except
            logger.error(f"Error shutting down sensors: {e}")

        # Additional delay for sensor shutdown to complete
        time.sleep(0.2)

        # Shutdown components in reverse order
        for component in reversed(self._components):
            if component is not None:
                try:
                    component.shutdown()
                    # Small delay between each component shutdown to prevent race conditions
                    time.sleep(0.05)
                except Exception as e:  # pylint: disable=broad-except
                    logger.error(
                        f"Error shutting down component {component.__class__.__name__}: {e}"
                    )

        # Longer delay to allow all component shutdown to complete
        time.sleep(0.5)

        # Enhanced Zenoh session close with better synchronization
        zenoh_close_success = False

        def _close_zenoh_session():
            """Close zenoh session in a separate thread."""
            nonlocal zenoh_close_success
            try:
                # Give Zenoh even more time to clean up internal resources and settle threads
                time.sleep(0.5)
                self._zenoh_session.close()
                zenoh_close_success = True
            except Exception as e:  # pylint: disable=broad-except
                logger.warning(f"Zenoh session close error: {e}")

        # Use non-daemon thread for proper cleanup
        close_thread = threading.Thread(target=_close_zenoh_session, daemon=False)
        close_thread.start()

        # Wait for close with extended timeout to avoid race conditions
        close_thread.join(timeout=3.0)

        if close_thread.is_alive():
            logger.warning(
                "Zenoh session close timed out after 5s, continuing shutdown"
            )
            # Force daemon mode so it doesn't block process exit
            close_thread.daemon = True
        elif zenoh_close_success:
            logger.debug("Zenoh session closed cleanly")

        # Extended final delay to allow any remaining Zenoh background threads to finish
        time.sleep(0.8)

        logger.info("Robot shutdown complete")

    def is_shutdown(self) -> bool:
        """Check if the robot has been shutdown.

        Returns:
            True if the robot has been shutdown, False otherwise.
        """
        return self._shutdown_called

    def get_software_version(
        self, show: bool = True
    ) -> dict[
        str,
        dict[
            Literal["hardware_version", "software_version", "main_hash"],
            int | str,
        ],
    ]:
        """Retrieve software version information for all components.

        Args:
            show: Whether to display the version information.

        Returns:
            Dictionary containing version information for all components.

        Raises:
            RuntimeError: If version information cannot be retrieved.
        """
        try:
            replies = self._zenoh_session.get(
                resolve_key_name(self._configs.version_info_name)
            )
            version_dict = {}
            for reply in replies:
                if reply.ok and reply.ok.payload:
                    version_bytes = reply.ok.payload.to_bytes()
                    version_msg = cast(
                        dexcontrol_query_pb2.SoftwareVersion,
                        dexcontrol_query_pb2.SoftwareVersion.FromString(version_bytes),
                    )
                    version_dict = software_version_to_dict(version_msg)
                    break

            if show:
                show_software_version(version_dict)
            return version_dict
        except Exception as e:
            raise RuntimeError(f"Failed to retrieve software versions: {e}") from e

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

    def reboot_component(self, part: Literal["arm", "chassis", "torso"]) -> None:
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
        self, part: Literal["left_arm", "right_arm", "chassis", "head"] | str
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
                handler=lambda reply: logger.info(
                    f"Cleared error of {part}: {reply.ok}"
                ),
                payload=query_msg.SerializeToString(),
            )
        except Exception as e:
            raise RuntimeError(
                f"Failed to clear error for component {part}: {e}"
            ) from e

    def get_joint_pos_dict(
        self,
        component: Literal["left_arm", "right_arm", "torso", "head"]
        | list[Literal["left_arm", "right_arm", "torso", "head"]],
    ) -> dict[str, float]:
        """Get the joint positions of one or more robot components.

        Args:
            component: Component name or list of component names to get joint positions for.
                Valid components are "left_arm", "right_arm", "torso", and "head".

        Returns:
            Dictionary mapping joint names to joint positions.

        Raises:
            ValueError: If component is not a string or list.
            KeyError: If an invalid component name is provided.
            RuntimeError: If joint position retrieval fails.
        """
        component_map = {
            "left_arm": self.left_arm,
            "right_arm": self.right_arm,
            "torso": self.torso,
            "head": self.head,
        }

        try:
            if isinstance(component, str):
                if component not in component_map:
                    raise KeyError(f"Invalid component name: {component}")
                return component_map[component].get_joint_pos_dict()
            elif isinstance(component, list):
                joint_pos_dict = {}
                for c in component:
                    if c not in component_map:
                        raise KeyError(f"Invalid component name: {c}")
                    joint_pos_dict.update(component_map[c].get_joint_pos_dict())
                return joint_pos_dict
            else:
                raise ValueError("Component must be a string or list of strings")
        except (KeyError, ValueError) as e:
            raise e
        except Exception as e:
            raise RuntimeError(f"Failed to get joint positions: {e}") from e

    def execute_trajectory(
        self,
        trajectory: dict[str, np.ndarray | dict[str, np.ndarray]],
        control_hz: float = 100,
        relative: bool = False,
    ) -> None:
        """Execute a trajectory on the robot.

        Args:
            trajectory: Dictionary mapping component names to either:
                - numpy arrays of joint positions
                - dictionaries with 'position' and optional 'velocity' keys
            control_hz: Control frequency in Hz.
            relative: Whether positions are relative to current position.

        Raises:
            ValueError: If trajectory is empty or components have different trajectory lengths.
            ValueError: If trajectory format is invalid.
            RuntimeError: If trajectory execution fails.
        """
        if not trajectory:
            raise ValueError("Trajectory must be a non-empty dictionary")

        try:
            # Process trajectory to standardize format
            processed_trajectory = self._process_trajectory(trajectory)

            # Validate trajectory lengths
            self._validate_trajectory_lengths(processed_trajectory)

            # Execute trajectory
            self._execute_processed_trajectory(
                processed_trajectory, control_hz, relative
            )

        except Exception as e:
            raise RuntimeError(f"Failed to execute trajectory: {e}") from e

    def set_joint_pos(
        self,
        joint_pos: dict[str, list[float] | np.ndarray],
        relative: bool = False,
        wait_time: float = 0.0,
        wait_kwargs: dict[str, Any] | None = None,
    ) -> None:
        """Set the joint positions of the robot.

        Args:
            joint_pos: Dictionary mapping component names to joint positions.
                Values can be either lists of floats or numpy arrays.
            relative: Whether to set positions relative to current position.
            wait_time: Time to wait for movement completion in seconds.
            wait_kwargs: Additional parameters for trajectory generation.
                control_hz: Control frequency in Hz (default: 100).
                max_vel: Maximum velocity for trajectory (default: 0.5).

        Raises:
            ValueError: If any component name is invalid.
            RuntimeError: If joint position setting fails.
        """
        if wait_kwargs is None:
            wait_kwargs = {}

        try:
            component_map = self._get_component_map()

            # Validate component names
            self._validate_component_names(joint_pos, component_map)

            # Separate position-velocity controlled components from others
            pv_components = [c for c in joint_pos if c in self._pv_components]
            non_pv_components = [c for c in joint_pos if c not in self._pv_components]

            # Set PV components immediately
            self._set_pv_components(pv_components, joint_pos, component_map, relative)

            # Handle non-PV components based on wait_time
            if wait_time <= 0:
                self._set_non_pv_components_immediate(
                    non_pv_components, joint_pos, component_map, relative
                )
            else:
                self._set_non_pv_components_with_trajectory(
                    non_pv_components,
                    joint_pos,
                    component_map,
                    relative,
                    wait_time,
                    wait_kwargs,
                )

        except Exception as e:
            raise RuntimeError(f"Failed to set joint positions: {e}") from e

    def compensate_torso_pitch(self, joint_pos: np.ndarray, part: str) -> np.ndarray:
        """Compensate for torso pitch in joint positions.

        Args:
            joint_pos: Joint positions to compensate.
            robot: Robot instance.
            part: Component name for which joint positions are being compensated.

        Returns:
            Compensated joint positions.
        """
        # Supported robot models
        SUPPORTED_MODELS = {"vega-1", "vega-rc2", "vega-rc1"}

        if self.robot_model not in SUPPORTED_MODELS:
            raise ValueError(
                f"Unsupported robot model: {self.robot_model}. "
                f"Supported models: {SUPPORTED_MODELS}"
            )

        torso_pitch = self.torso.pitch_angle

        # Calculate pitch adjustment based on body part
        if part == "right_arm":
            pitch_adjustment = -torso_pitch
        elif part == "left_arm":
            pitch_adjustment = torso_pitch
        elif part == "head":
            pitch_adjustment = torso_pitch - np.pi / 2
        else:
            raise ValueError(
                f"Unsupported body part: {part}. "
                f"Supported parts: left_arm, right_arm, head"
            )

        # Create a copy to avoid modifying the original array
        adjusted_positions = joint_pos.copy()
        adjusted_positions[0] += pitch_adjustment

        return adjusted_positions

    def _process_trajectory(
        self, trajectory: dict[str, np.ndarray | dict[str, np.ndarray]]
    ) -> dict[str, dict[str, np.ndarray]]:
        """Process trajectory to standardize format.

        Args:
            trajectory: Raw trajectory data.

        Returns:
            Processed trajectory with standardized format.

        Raises:
            ValueError: If trajectory format is invalid.
        """
        processed_trajectory: dict[str, dict[str, np.ndarray]] = {}
        for component, data in trajectory.items():
            if isinstance(data, np.ndarray):
                processed_trajectory[component] = {"position": data}
            elif isinstance(data, dict) and "position" in data:
                processed_trajectory[component] = data
            else:
                raise ValueError(f"Invalid trajectory format for component {component}")
        return processed_trajectory

    def _validate_trajectory_lengths(
        self, processed_trajectory: dict[str, dict[str, np.ndarray]]
    ) -> None:
        """Validate that all trajectory components have consistent lengths.

        Args:
            processed_trajectory: Processed trajectory data.

        Raises:
            ValueError: If trajectory lengths are inconsistent.
        """
        first_component = next(iter(processed_trajectory))
        first_length = len(processed_trajectory[first_component]["position"])

        for component, data in processed_trajectory.items():
            if len(data["position"]) != first_length:
                raise ValueError(
                    f"Component {component} has different trajectory length"
                )
            if "velocity" in data and len(data["velocity"]) != first_length:
                raise ValueError(
                    f"Velocity length for {component} doesn't match position length"
                )

    def _execute_processed_trajectory(
        self,
        processed_trajectory: dict[str, dict[str, np.ndarray]],
        control_hz: float,
        relative: bool,
    ) -> None:
        """Execute the processed trajectory.

        Args:
            processed_trajectory: Processed trajectory data.
            control_hz: Control frequency in Hz.
            relative: Whether positions are relative to current position.

        Raises:
            ValueError: If invalid component is specified.
        """
        rate_limiter = RateLimiter(control_hz)
        component_map = {
            "left_arm": self.left_arm,
            "right_arm": self.right_arm,
            "torso": self.torso,
            "head": self.head,
            "left_hand": self.left_hand,
            "right_hand": self.right_hand,
        }

        first_component = next(iter(processed_trajectory))
        trajectory_length = len(processed_trajectory[first_component]["position"])

        for i in range(trajectory_length):
            for c, data in processed_trajectory.items():
                if c not in component_map:
                    raise ValueError(f"Invalid component: {c}")

                position = data["position"][i]
                if "velocity" in data:
                    velocity = data["velocity"][i]
                    component_map[c].set_joint_pos_vel(
                        position, velocity, relative=relative, wait_time=0.0
                    )
                else:
                    component_map[c].set_joint_pos(
                        position, relative=relative, wait_time=0.0
                    )
            rate_limiter.sleep()

    def _get_component_map(self) -> dict[str, Any]:
        """Get the component mapping dictionary.

        Returns:
            Dictionary mapping component names to component instances.
        """
        return {
            "left_arm": self.left_arm,
            "right_arm": self.right_arm,
            "torso": self.torso,
            "head": self.head,
            "left_hand": self.left_hand,
            "right_hand": self.right_hand,
        }

    def _validate_component_names(
        self,
        joint_pos: dict[str, list[float] | np.ndarray],
        component_map: dict[str, Any],
    ) -> None:
        """Validate that all component names are valid.

        Args:
            joint_pos: Joint position dictionary.
            component_map: Component mapping dictionary.

        Raises:
            ValueError: If invalid component names are found.
        """
        invalid_components = set(joint_pos.keys()) - set(component_map.keys())
        if invalid_components:
            raise ValueError(
                f"Invalid component names: {', '.join(invalid_components)}"
            )

    def _set_pv_components(
        self,
        pv_components: list[str],
        joint_pos: dict[str, list[float] | np.ndarray],
        component_map: dict[str, Any],
        relative: bool,
    ) -> None:
        """Set position-velocity controlled components immediately.

        Args:
            pv_components: List of PV component names.
            joint_pos: Joint position dictionary.
            component_map: Component mapping dictionary.
            relative: Whether positions are relative.
        """
        for c in pv_components:
            component_map[c].set_joint_pos(
                joint_pos[c], relative=relative, wait_time=0.0
            )

    def _set_non_pv_components_immediate(
        self,
        non_pv_components: list[str],
        joint_pos: dict[str, list[float] | np.ndarray],
        component_map: dict[str, Any],
        relative: bool,
    ) -> None:
        """Set non-PV components immediately without trajectory.

        Args:
            non_pv_components: List of non-PV component names.
            joint_pos: Joint position dictionary.
            component_map: Component mapping dictionary.
            relative: Whether positions are relative.
        """
        for c in non_pv_components:
            component_map[c].set_joint_pos(joint_pos[c], relative=relative)

    def _set_non_pv_components_with_trajectory(
        self,
        non_pv_components: list[str],
        joint_pos: dict[str, list[float] | np.ndarray],
        component_map: dict[str, Any],
        relative: bool,
        wait_time: float,
        wait_kwargs: dict[str, Any],
    ) -> None:
        """Set non-PV components with smooth trajectory over wait_time.

        Args:
            non_pv_components: List of non-PV component names.
            joint_pos: Joint position dictionary.
            component_map: Component mapping dictionary.
            relative: Whether positions are relative.
            wait_time: Time to wait for movement completion.
            wait_kwargs: Additional trajectory parameters.
        """
        control_hz = wait_kwargs.get("control_hz", 100)
        max_vel = wait_kwargs.get("max_vel", 0.5)

        # Generate trajectories for smooth motion during wait_time
        rate_limiter = RateLimiter(control_hz)
        non_pv_component_traj = {}
        max_traj_steps = 0

        # Calculate trajectories for each component
        for c in non_pv_components:
            current_joint_pos = component_map[c].get_joint_pos().copy()
            target_pos = joint_pos[c]
            # Convert to numpy array if it's a list
            if isinstance(target_pos, list):
                target_pos = np.array(target_pos)
            non_pv_component_traj[c], steps = generate_linear_trajectory(
                current_joint_pos, target_pos, max_vel, control_hz
            )
            max_traj_steps = max(max_traj_steps, steps)

        # Execute trajectories with timing
        start_time = time.time()
        for step in range(max_traj_steps):
            for c in non_pv_components:
                if step < len(non_pv_component_traj[c]):
                    component_map[c].set_joint_pos(
                        non_pv_component_traj[c][step], relative=relative, wait_time=0.0
                    )
            rate_limiter.sleep()
            if time.time() - start_time > wait_time:
                break

        # Wait for any remaining time
        while time.time() - start_time < wait_time:
            rate_limiter.sleep()
