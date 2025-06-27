# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Rate limiter utility for maintaining consistent execution rates."""

import sys
import time
from typing import Final

from loguru import logger


class RateLimiter:
    """Class for limiting execution rate to a target frequency.

    This class provides rate limiting functionality by sleeping between iterations to
    maintain a desired execution frequency. It also tracks statistics about the achieved
    rate and missed deadlines.

    Attributes:
        period_sec: Time period between iterations in seconds.
        target_rate_hz: Desired execution rate in Hz.
        window_size: Size of moving average window for rate calculations.
        last_time_sec: Timestamp of the last iteration.
        next_time_sec: Scheduled timestamp for the next iteration.
        start_time_sec: Timestamp when the rate limiter was initialized or reset.
        duration_buffer: List of recent iteration durations for rate calculation.
        missed_deadlines: Counter for iterations that missed their scheduled time.
        iterations: Total number of iterations since initialization or reset.
    """

    def __init__(self, rate_hz: float, window_size: int = 50) -> None:
        """Initializes rate limiter.

        Args:
            rate_hz: Desired rate in Hz.
            window_size: Size of moving average window for rate calculations.

        Raises:
            ValueError: If rate_hz is not positive.
        """
        if rate_hz <= 0:
            raise ValueError("Rate must be positive")

        self.period_sec: Final[float] = 1.0 / rate_hz
        self.target_rate_hz: Final[float] = rate_hz
        self.window_size: Final[int] = max(1, window_size)

        # Initialize timing variables
        now_sec = time.monotonic()
        self.last_time_sec: float = now_sec
        self.next_time_sec: float = now_sec + self.period_sec
        self.start_time_sec: float = now_sec

        # Initialize statistics
        self.duration_buffer: list[float] = []
        self.missed_deadlines: int = 0
        self.iterations: int = 0
        self._MAX_ITERATIONS: Final[int] = sys.maxsize - 1000  # Leave some buffer

    def sleep(self) -> None:
        """Sleeps to maintain desired rate.

        Sleeps for the appropriate duration to maintain the target rate. If the next
        scheduled time has already passed, increments the missed deadlines counter.
        Uses monotonic time for reliable timing regardless of system clock changes.
        """
        current_time_sec = time.monotonic()
        sleep_time_sec = self.next_time_sec - current_time_sec

        if sleep_time_sec > 0:
            time.sleep(sleep_time_sec)
        else:
            self.missed_deadlines += 1

        # Update timing and statistics
        now_sec = time.monotonic()

        # Reset iterations if approaching max value
        if self.iterations >= self._MAX_ITERATIONS:
            logger.warning(
                "Iteration counter approaching max value, resetting statistics"
            )
            self.reset()
            return

        self.iterations += 1

        if self.iterations > 1:  # Skip first iteration
            duration_sec = now_sec - self.last_time_sec
            if len(self.duration_buffer) >= self.window_size:
                self.duration_buffer.pop(0)
            self.duration_buffer.append(duration_sec)

        self.last_time_sec = now_sec

        # More efficient way to advance next_time_sec when multiple periods behind
        periods_behind = max(
            0, int((now_sec - self.next_time_sec) / self.period_sec) + 1
        )
        self.next_time_sec += periods_behind * self.period_sec

    def get_actual_rate(self) -> float:
        """Calculates actual achieved rate in Hz using moving average.

        Returns:
            Current execution rate based on recent iterations.
        """
        if not self.duration_buffer:
            return 0.0
        avg_duration_sec = sum(self.duration_buffer) / len(self.duration_buffer)
        return 0.0 if avg_duration_sec <= 0 else 1.0 / avg_duration_sec

    def get_average_rate(self) -> float:
        """Calculates average rate over entire run.

        Returns:
            Average execution rate since start or last reset.
        """
        if self.iterations < 2:
            return 0.0
        total_time_sec = time.monotonic() - self.start_time_sec
        return 0.0 if total_time_sec <= 0 else self.iterations / total_time_sec

    def reset(self) -> None:
        """Resets the rate limiter state and statistics."""
        now_sec = time.monotonic()
        self.last_time_sec = now_sec
        self.next_time_sec = now_sec + self.period_sec
        self.start_time_sec = now_sec
        self.duration_buffer.clear()
        self.missed_deadlines = 0
        self.iterations = 0

    def get_stats(self) -> dict[str, float | int]:
        """Gets runtime statistics.

        Returns:
            Dictionary containing execution statistics including actual rate,
            average rate, target rate, missed deadlines and iteration count.
        """
        return {
            "actual_rate": self.get_actual_rate(),
            "average_rate": self.get_average_rate(),
            "target_rate": self.target_rate_hz,
            "missed_deadlines": self.missed_deadlines,
            "iterations": self.iterations,
        }


if __name__ == "__main__":
    rate_limiter = RateLimiter(100.0)  # 100Hz

    try:
        while True:
            rate_limiter.sleep()
            actual_rate = rate_limiter.get_actual_rate()
            logger.info(f"Rate: {actual_rate:.2f} Hz")

    except KeyboardInterrupt:
        stats = rate_limiter.get_stats()
        logger.info("\nFinal stats:")
        logger.info(f"Average rate: {stats['average_rate']:.2f} Hz")
        logger.info(f"Final rate: {stats['actual_rate']:.2f} Hz")
        logger.info(f"Missed deadlines: {stats['missed_deadlines']}")
