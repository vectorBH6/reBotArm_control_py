"""Synchronized joint-trajectory retiming.

The geometric path is never changed: only the common time axis is stretched.
Consequently, a limiting joint slows every joint by the same factor instead of
being clipped independently by the actuator.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class RetimeResult:
    """Result of a synchronized, path-preserving retiming operation."""

    times: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray
    scale_factor: float
    limiting_joint: int | None
    limiting_quantity: str | None

    @property
    def duration(self) -> float:
        return float(self.times[-1]) if self.times.size else 0.0


def tracking_speed_scale(
    commanded_positions: Sequence[float] | np.ndarray,
    actual_positions: Sequence[float] | np.ndarray,
    soft_error: float,
    hard_error: float,
) -> float:
    """Return a common virtual-time rate from the largest following error.

    The result is 1 below ``soft_error``, decreases linearly, and reaches zero
    at ``hard_error``.  No individual joint is advanced or clipped separately.
    """

    commanded = np.asarray(commanded_positions, dtype=np.float64).reshape(-1)
    actual = np.asarray(actual_positions, dtype=np.float64).reshape(-1)
    if commanded.shape != actual.shape or commanded.size == 0:
        raise ValueError("commanded and actual positions must have equal size")
    if not np.all(np.isfinite(commanded)) or not np.all(np.isfinite(actual)):
        return 0.0
    if not np.isfinite(soft_error) or not np.isfinite(hard_error):
        raise ValueError("tracking thresholds must be finite")
    if soft_error < 0.0 or hard_error <= soft_error:
        raise ValueError("tracking thresholds must satisfy 0 <= soft < hard")

    error = float(np.max(np.abs(commanded - actual)))
    if error <= soft_error:
        return 1.0
    if error >= hard_error:
        return 0.0
    return float((hard_error - error) / (hard_error - soft_error))


def retime_joint_trajectory(
    positions: Sequence[Sequence[float]] | np.ndarray,
    times: Sequence[float] | np.ndarray,
    max_velocities: Sequence[float] | np.ndarray,
    max_accelerations: Sequence[float] | np.ndarray,
    *,
    safety_factor: float = 1.0,
) -> RetimeResult:
    """Stretch a common time axis until every joint is within its limits.

    A single scale factor is deliberately used for the whole trajectory.  This
    is conservative, but it preserves every sampled joint position and thus the
    Cartesian path produced by IK.  Velocity scales with ``1 / k`` and
    acceleration with ``1 / k**2``.

    ``safety_factor`` reserves tracking margin below the configured limits.  It
    must be in ``(0, 1]``.
    """

    q = np.asarray(positions, dtype=np.float64)
    t = np.asarray(times, dtype=np.float64).reshape(-1)
    vmax = np.asarray(max_velocities, dtype=np.float64).reshape(-1)
    amax = np.asarray(max_accelerations, dtype=np.float64).reshape(-1)

    if q.ndim != 2 or q.shape[0] < 1 or q.shape[1] < 1:
        raise ValueError("positions must be a non-empty 2-D array")
    if t.size != q.shape[0]:
        raise ValueError("times length must match trajectory points")
    if vmax.size != q.shape[1] or amax.size != q.shape[1]:
        raise ValueError("joint limit vectors must match trajectory joints")
    if not np.all(np.isfinite(q)) or not np.all(np.isfinite(t)):
        raise ValueError("trajectory positions and times must be finite")
    if not np.all(np.isfinite(vmax)) or np.any(vmax <= 0.0):
        raise ValueError("max velocities must be finite and positive")
    if not np.all(np.isfinite(amax)) or np.any(amax <= 0.0):
        raise ValueError("max accelerations must be finite and positive")
    if not np.isfinite(safety_factor) or not 0.0 < safety_factor <= 1.0:
        raise ValueError("safety_factor must be in (0, 1]")

    t = t - t[0]
    if t.size > 1 and np.any(np.diff(t) <= 0.0):
        raise ValueError("trajectory times must be strictly increasing")

    effective_vmax = vmax * safety_factor
    effective_amax = amax * safety_factor
    velocities, accelerations = _finite_difference_kinematics(q, t)

    if t.size > 1:
        segment_velocities = np.diff(q, axis=0) / np.diff(t)[:, np.newaxis]
    else:
        segment_velocities = np.zeros((1, q.shape[1]), dtype=np.float64)
    velocity_ratios = (
        np.abs(segment_velocities) / effective_vmax[np.newaxis, :]
    )
    acceleration_ratios = (
        np.abs(accelerations) / effective_amax[np.newaxis, :]
    )
    velocity_flat = int(np.argmax(velocity_ratios))
    acceleration_flat = int(np.argmax(acceleration_ratios))
    max_velocity_ratio = float(velocity_ratios.flat[velocity_flat])
    max_acceleration_ratio = float(acceleration_ratios.flat[acceleration_flat])

    velocity_scale = max_velocity_ratio
    acceleration_scale = float(np.sqrt(max_acceleration_ratio))
    scale = max(1.0, velocity_scale, acceleration_scale)

    limiting_joint: int | None = None
    limiting_quantity: str | None = None
    if scale > 1.0 + 1e-12:
        if velocity_scale >= acceleration_scale:
            limiting_joint = velocity_flat % q.shape[1]
            limiting_quantity = "velocity"
        else:
            limiting_joint = acceleration_flat % q.shape[1]
            limiting_quantity = "acceleration"

    scaled_times = t * scale
    scaled_velocities, scaled_accelerations = _finite_difference_kinematics(
        q, scaled_times
    )
    return RetimeResult(
        times=scaled_times,
        velocities=scaled_velocities,
        accelerations=scaled_accelerations,
        scale_factor=scale,
        limiting_joint=limiting_joint,
        limiting_quantity=limiting_quantity,
    )


def _finite_difference_kinematics(
    positions: np.ndarray,
    times: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Estimate point velocities/accelerations without changing path samples."""

    point_count, joint_count = positions.shape
    velocities = np.zeros((point_count, joint_count), dtype=np.float64)
    accelerations = np.zeros_like(velocities)
    if point_count < 2:
        return velocities, accelerations

    dt = np.diff(times)
    segment_velocities = np.diff(positions, axis=0) / dt[:, np.newaxis]
    if point_count == 2:
        # The executor linearly interpolates this segment.  Treat its velocity
        # as the commanded velocity and leave acceleration to the drive ramp.
        velocities[0] = segment_velocities[0]
        velocities[1] = segment_velocities[0]
        return velocities, accelerations

    velocities[0] = segment_velocities[0]
    velocities[-1] = segment_velocities[-1]
    velocities[1:-1] = (
        segment_velocities[:-1] * dt[1:, np.newaxis]
        + segment_velocities[1:] * dt[:-1, np.newaxis]
    ) / (dt[:-1] + dt[1:])[:, np.newaxis]

    # Acceleration at an interior sample is the change in adjacent segment
    # velocities divided by the time between segment midpoints.
    accelerations[1:-1] = 2.0 * (
        segment_velocities[1:] - segment_velocities[:-1]
    ) / (dt[:-1] + dt[1:])[:, np.newaxis]
    accelerations[0] = accelerations[1]
    accelerations[-1] = accelerations[-2]
    return velocities, accelerations
