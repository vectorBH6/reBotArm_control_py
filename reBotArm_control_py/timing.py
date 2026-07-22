"""Thread-safe timing diagnostics and deadline detection for periodic loops."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import threading
from typing import Callable, Deque, Optional


@dataclass(frozen=True)
class LoopTimingSnapshot:
    """Immutable timing counters for one periodic loop."""

    nominal_period_sec: float
    deadline_sec: float
    sample_count: int
    last_period_sec: float
    last_execution_sec: float
    max_jitter_sec: float
    p95_jitter_sec: float
    p99_jitter_sec: float
    deadline_misses: int
    lost_cycles: int
    consecutive_violations: int
    fault_latched: bool


@dataclass(frozen=True)
class DeadlineFault:
    """Description delivered once when a configured deadline is exceeded."""

    observed_sec: float
    snapshot: LoopTimingSnapshot


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    position = (len(ordered) - 1) * max(0.0, min(1.0, percentile))
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] + (ordered[upper] - ordered[lower]) * fraction


class LoopTimingMonitor:
    """Measure loop jitter and latch after repeated deadline violations.

    ``observe_cycle`` is intentionally allocation-light on the normal path. The
    bounded jitter window is only sorted when a diagnostics snapshot is read.
    Lifetime miss/lost-cycle counters survive loop restarts, while the previous
    timestamp, consecutive counter and fault latch can be reset independently.
    """

    def __init__(
        self,
        nominal_period_sec: float,
        *,
        deadline_sec: float = 0.0,
        violation_limit: int = 1,
        window_size: int = 2048,
        on_fault: Optional[Callable[[DeadlineFault], None]] = None,
    ) -> None:
        self._lock = threading.Lock()
        self._nominal_period_sec = max(float(nominal_period_sec), 1e-6)
        self._deadline_sec = max(float(deadline_sec), 0.0)
        self._violation_limit = max(int(violation_limit), 1)
        self._on_fault = on_fault
        self._jitters: Deque[float] = deque(maxlen=max(int(window_size), 32))
        self._previous_start_sec: float | None = None
        self._last_period_sec = 0.0
        self._last_execution_sec = 0.0
        self._max_jitter_sec = 0.0
        self._sample_count = 0
        self._deadline_misses = 0
        self._lost_cycles = 0
        self._consecutive_violations = 0
        self._fault_latched = False

    def configure(
        self,
        *,
        nominal_period_sec: float | None = None,
        deadline_sec: float | None = None,
        violation_limit: int | None = None,
        on_fault: Optional[Callable[[DeadlineFault], None]] = None,
    ) -> None:
        with self._lock:
            if nominal_period_sec is not None:
                self._nominal_period_sec = max(
                    float(nominal_period_sec), 1e-6
                )
            if deadline_sec is not None:
                self._deadline_sec = max(float(deadline_sec), 0.0)
            if violation_limit is not None:
                self._violation_limit = max(int(violation_limit), 1)
            if on_fault is not None:
                self._on_fault = on_fault

    def reset_runtime(self) -> None:
        """Start a fresh loop run without erasing lifetime diagnostics."""
        with self._lock:
            self._previous_start_sec = None
            self._last_period_sec = 0.0
            self._last_execution_sec = 0.0
            self._consecutive_violations = 0
            self._fault_latched = False

    def observe_cycle(
        self,
        start_sec: float,
        execution_sec: float,
    ) -> DeadlineFault | None:
        """Record one completed cycle and return a newly latched fault."""
        callback: Optional[Callable[[DeadlineFault], None]] = None
        fault: DeadlineFault | None = None
        with self._lock:
            start = float(start_sec)
            execution = max(float(execution_sec), 0.0)
            period = 0.0
            lost = 0
            if self._previous_start_sec is not None:
                period = max(start - self._previous_start_sec, 0.0)
                jitter = abs(period - self._nominal_period_sec)
                self._jitters.append(jitter)
                self._max_jitter_sec = max(self._max_jitter_sec, jitter)
                lost = max(
                    0,
                    int(
                        (period + self._nominal_period_sec * 1e-9)
                        / self._nominal_period_sec
                    )
                    - 1,
                )
                self._lost_cycles += lost
            self._previous_start_sec = start
            self._last_period_sec = period
            self._last_execution_sec = execution
            self._sample_count += 1

            observed = max(period, execution)
            missed = self._deadline_sec > 0.0 and observed > self._deadline_sec
            if missed:
                self._deadline_misses += 1
                # A long stall represents several consecutive absent cycles and
                # must not be diluted into a single violation.
                self._consecutive_violations += max(1, lost)
            else:
                self._consecutive_violations = 0

            if (
                missed
                and not self._fault_latched
                and self._consecutive_violations >= self._violation_limit
            ):
                self._fault_latched = True
                snapshot = self._snapshot_locked()
                fault = DeadlineFault(observed_sec=observed, snapshot=snapshot)
                callback = self._on_fault

        if fault is not None and callback is not None:
            try:
                callback(fault)
            except Exception:
                # Timing instrumentation must never crash the actuator loop.
                pass
        return fault

    def report_stall(self, elapsed_sec: float) -> DeadlineFault | None:
        """Latch an independently observed stall, e.g. a missing ROS timer."""
        callback: Optional[Callable[[DeadlineFault], None]] = None
        fault: DeadlineFault | None = None
        with self._lock:
            elapsed = max(float(elapsed_sec), 0.0)
            if (
                self._deadline_sec <= 0.0
                or elapsed <= self._deadline_sec
                or self._fault_latched
            ):
                return None
            lost = max(
                1,
                int(
                    (elapsed + self._nominal_period_sec * 1e-9)
                    / self._nominal_period_sec
                )
                - 1,
            )
            self._deadline_misses += 1
            self._lost_cycles += lost
            self._consecutive_violations = max(
                self._consecutive_violations,
                lost,
            )
            self._fault_latched = True
            snapshot = self._snapshot_locked()
            fault = DeadlineFault(observed_sec=elapsed, snapshot=snapshot)
            callback = self._on_fault

        if callback is not None:
            try:
                callback(fault)
            except Exception:
                pass
        return fault

    def snapshot(self) -> LoopTimingSnapshot:
        with self._lock:
            jitters = list(self._jitters)
            values = (
                self._nominal_period_sec,
                self._deadline_sec,
                self._sample_count,
                self._last_period_sec,
                self._last_execution_sec,
                self._max_jitter_sec,
                self._deadline_misses,
                self._lost_cycles,
                self._consecutive_violations,
                self._fault_latched,
            )
        return LoopTimingSnapshot(
            nominal_period_sec=values[0],
            deadline_sec=values[1],
            sample_count=values[2],
            last_period_sec=values[3],
            last_execution_sec=values[4],
            max_jitter_sec=values[5],
            p95_jitter_sec=_percentile(jitters, 0.95),
            p99_jitter_sec=_percentile(jitters, 0.99),
            deadline_misses=values[6],
            lost_cycles=values[7],
            consecutive_violations=values[8],
            fault_latched=values[9],
        )

    def _snapshot_locked(self) -> LoopTimingSnapshot:
        jitters = list(self._jitters)
        return LoopTimingSnapshot(
            nominal_period_sec=self._nominal_period_sec,
            deadline_sec=self._deadline_sec,
            sample_count=self._sample_count,
            last_period_sec=self._last_period_sec,
            last_execution_sec=self._last_execution_sec,
            max_jitter_sec=self._max_jitter_sec,
            p95_jitter_sec=_percentile(jitters, 0.95),
            p99_jitter_sec=_percentile(jitters, 0.99),
            deadline_misses=self._deadline_misses,
            lost_cycles=self._lost_cycles,
            consecutive_violations=self._consecutive_violations,
            fault_latched=self._fault_latched,
        )
