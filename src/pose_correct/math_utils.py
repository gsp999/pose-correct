from __future__ import annotations

import math
from collections.abc import Iterable

import numpy as np


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""

    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def unwrap_angles(angles: Iterable[float]) -> np.ndarray:
    return np.unwrap(np.asarray(list(angles), dtype=float))


def circular_mean(angles: Iterable[float]) -> float:
    values = np.asarray(list(angles), dtype=float)
    if values.size == 0:
        raise ValueError("at least one angle is required")
    return math.atan2(float(np.sin(values).mean()), float(np.cos(values).mean()))


def ridge_lstsq(features: np.ndarray, target: np.ndarray, ridge: float) -> np.ndarray:
    if features.ndim != 2:
        raise ValueError("features must be a 2-D matrix")
    if target.ndim != 1:
        raise ValueError("target must be a 1-D vector")
    if features.shape[0] != target.shape[0]:
        raise ValueError("features and target sample counts do not match")

    lhs = features.T @ features
    if ridge > 0.0:
        penalty = np.eye(lhs.shape[0])
        penalty[0, 0] = 0.0
        lhs = lhs + ridge * penalty
    rhs = features.T @ target
    return np.linalg.solve(lhs, rhs)
