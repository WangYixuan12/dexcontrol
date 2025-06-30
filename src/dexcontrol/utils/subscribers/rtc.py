# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

import asyncio
import json
import threading
import time

import numpy as np
import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription
from loguru import logger

# Try to import uvloop for better performance
try:
    import uvloop  # type: ignore[import-untyped]

    UVLOOP_AVAILABLE = True
except ImportError:
    uvloop = None  # type: ignore[assignment]
    UVLOOP_AVAILABLE = False


class RTCSubscriber:
    """
    Subscriber for receiving video data via RTC.

    This class connects to a RTC peer through a signaling server,
    receives a video stream, and makes the latest frame available.
    """

    def __init__(
        self,
        url: str,
        name: str = "rtc_subscriber",
        enable_fps_tracking: bool = True,
        fps_log_interval: int = 100,
    ):
        """
        Initialize the RTC subscriber.

        Args:
            url: WebSocket URL of the signaling server.
            name: Name for logging purposes.
            enable_fps_tracking: Whether to track and log FPS metrics.
            fps_log_interval: Number of frames between FPS calculations.
        """
        self._url = url
        self._name = name
        self._pc = RTCPeerConnection()
        self._latest_frame: np.ndarray | None = None
        self._active = False
        self._data_lock = threading.Lock()
        self._stop_event = (
            threading.Event()
        )  # Use threading.Event for cross-thread communication
        self._async_stop_event = None  # Will be created in the async context
        self._websocket = None  # Store websocket reference for clean shutdown

        # FPS tracking
        self._enable_fps_tracking = enable_fps_tracking
        self._fps_log_interval = fps_log_interval
        self._frame_count = 0
        self._fps = 0.0
        self._last_fps_time = time.time()

        self._thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self._thread.start()

    def _run_event_loop(self):
        """Run the asyncio event loop in a separate thread."""
        try:
            # Use uvloop if available for better performance
            if UVLOOP_AVAILABLE and uvloop is not None:
                # Create a new uvloop event loop for this thread
                loop = uvloop.new_event_loop()
                asyncio.set_event_loop(loop)
                logger.debug(f"Using uvloop for {self._name}")

                try:
                    loop.run_until_complete(self._run())
                finally:
                    loop.close()
            else:
                # Use default asyncio event loop
                asyncio.run(self._run())

        except Exception as e:
            logger.error(f"Event loop error for {self._name}: {e}")
        finally:
            with self._data_lock:
                self._active = False

    async def _run(self):
        """
        Connects to a RTC peer, receives video, and saves frames to disk.
        """
        # Create async stop event in the async context
        self._async_stop_event = asyncio.Event()

        # Start a task to monitor the threading stop event
        monitor_task = asyncio.create_task(self._monitor_stop_event())

        @self._pc.on("track")
        async def on_track(track):
            if track.kind == "video":
                while (
                    self._async_stop_event is not None
                    and not self._async_stop_event.is_set()
                ):
                    try:
                        frame = await asyncio.wait_for(
                            track.recv(), timeout=1.0
                        )  # Reduced timeout for faster shutdown response
                        img = frame.to_ndarray(format="rgb24")
                        with self._data_lock:
                            self._latest_frame = img
                            if not self._active:
                                self._active = True
                        self._update_fps_metrics()
                    except asyncio.TimeoutError:
                        # Check if we should stop before logging error
                        if (
                            self._async_stop_event is not None
                            and not self._async_stop_event.is_set()
                        ):
                            logger.warning(
                                f"Timeout: No frame received in 1 second from {self._url}"
                            )
                        continue
                    except Exception as e:
                        if (
                            self._async_stop_event is not None
                            and not self._async_stop_event.is_set()
                        ):
                            logger.error(f"Error receiving frame from {self._url}: {e}")
                        break

        @self._pc.on("connectionstatechange")
        async def on_connectionstatechange():
            if self._pc.connectionState == "failed":
                logger.warning(f"RTC connection failed for {self._url}")
                await self._pc.close()
                if self._async_stop_event is not None:
                    self._async_stop_event.set()

        try:
            async with websockets.connect(self._url) as websocket:
                self._websocket = websocket

                # Create an offer. The server's assertive codec control makes
                # client-side preferences redundant and potentially conflicting.
                self._pc.addTransceiver("video", direction="recvonly")
                offer = await self._pc.createOffer()
                await self._pc.setLocalDescription(offer)

                # Send the offer to the server
                await websocket.send(
                    json.dumps(
                        {
                            "sdp": self._pc.localDescription.sdp,
                            "type": self._pc.localDescription.type,
                        }
                    )
                )

                # Wait for the answer
                response = json.loads(await websocket.recv())
                if response["type"] == "answer":
                    await self._pc.setRemoteDescription(
                        RTCSessionDescription(
                            sdp=response["sdp"], type=response["type"]
                        )
                    )
                else:
                    logger.error(
                        f"Received unexpected message type: {response['type']} from {self._url}"
                    )
                    if self._async_stop_event is not None:
                        self._async_stop_event.set()

                # Wait until the stop event is set
                if self._async_stop_event is not None:
                    await self._async_stop_event.wait()

        except websockets.exceptions.ConnectionClosed:
            logger.info(f"WebSocket connection closed for {self._url}")
        except Exception as e:
            if not self._async_stop_event.is_set():
                logger.error(f"Operation failed for {self._url}: {e}")
        finally:
            # Cancel the monitor task
            monitor_task.cancel()
            try:
                await monitor_task
            except asyncio.CancelledError:
                pass

            # Close websocket if still open
            if self._websocket:
                try:
                    await self._websocket.close()
                except Exception as e:
                    logger.debug(f"Error closing websocket for {self._url}: {e}")

            # Close peer connection if not already closed
            if self._pc.connectionState != "closed":
                try:
                    await self._pc.close()
                except Exception as e:
                    logger.debug(f"Error closing peer connection for {self._url}: {e}")

            with self._data_lock:
                self._active = False

    async def _monitor_stop_event(self):
        """Monitor the threading stop event and set the async stop event when needed."""
        while not self._stop_event.is_set():
            await asyncio.sleep(0.1)  # Check every 100ms
        if self._async_stop_event is not None:
            self._async_stop_event.set()

    def _update_fps_metrics(self) -> None:
        """Update FPS tracking metrics.

        Increments frame counter and recalculates FPS at specified intervals.
        Only has an effect if fps_tracking was enabled during initialization.
        """
        if not self._enable_fps_tracking:
            return

        self._frame_count += 1
        if self._frame_count >= self._fps_log_interval:
            current_time = time.time()
            elapsed = current_time - self._last_fps_time
            self._fps = self._frame_count / elapsed
            logger.info(f"{self._name} frequency: {self._fps:.2f} Hz")
            self._frame_count = 0
            self._last_fps_time = current_time

    def get_latest_data(self) -> np.ndarray | None:
        """
        Get the latest video frame.

        Returns:
            Latest video frame as a numpy array (HxWxC RGB) if available, None otherwise.
        """
        with self._data_lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    def is_active(self) -> bool:
        """Check if the subscriber is actively receiving data."""
        with self._data_lock:
            return self._active

    def wait_for_active(self, timeout: float = 5.0) -> bool:
        """
        Wait for the subscriber to start receiving data.

        Args:
            timeout: Maximum time to wait in seconds.

        Returns:
            True if subscriber becomes active, False if timeout is reached.
        """
        start_time = time.time()
        while not self.is_active():
            if time.time() - start_time > timeout:
                logger.error(
                    f"No data received from {self._name} at {self._url} after {timeout}s"
                )
                return False
            time.sleep(0.1)
        return True

    def shutdown(self):
        """Stop the subscriber and release resources."""

        # Signal the async loop to stop
        self._stop_event.set()

        # Wait for the thread to finish with a reasonable timeout
        if self._thread.is_alive():
            self._thread.join(
                timeout=10.0
            )  # Increased timeout for more graceful shutdown

            if self._thread.is_alive():
                logger.warning(
                    f"{self._name} thread did not shut down gracefully within timeout."
                )

        # Ensure active state is set to False
        with self._data_lock:
            self._active = False

    @property
    def name(self) -> str:
        """Get the subscriber name."""
        return self._name

    @property
    def fps(self) -> float:
        """Get the current FPS measurement.

        Returns:
            Current frames per second measurement.
        """
        return self._fps
