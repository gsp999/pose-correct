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
    p: float
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

    mn_distance: float
    yaw_offset: float
    x_coefficients: tuple[float, float, float]
    y_coefficients: tuple[float, float, float]
    yaw_sign: int
    residual_rms_x: float
    residual_rms_y: float
    residual_rms_yaw: float
    sample_count: int
    fit_method: str = "robust_geometric"
    position_scale: float = 0.02
    yaw_scale: float = 0.02
    optimizer_cost: float | None = None
    optimizer_iterations: int | None = None
    ab_sensor_x: float | None = None
    ab_sensor_mid_y: float | None = None
    p_sensor_x: float | None = None
    p_sensor_y: float | None = None
