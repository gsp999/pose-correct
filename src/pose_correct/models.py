from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Observation:
    """One synchronized laser and odin pose sample.

    Distances are expected to be in the same unit as x/y, usually meters.
    Angles are radians.
    """

    m: float
    n: float
    s_x: float
    s_y: float
    s_yaw: float


@dataclass(frozen=True)
class PoseEstimate:
    """Corrected pose for point S."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class CalibrationResult:
    """Learned correction model parameters."""

    residual_rms_x: float
    residual_rms_y: float
    sample_count: int
    m_sensor_x: float
    m_sensor_y: float
    n_sensor_x: float
    n_sensor_y: float
