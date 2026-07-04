from __future__ import annotations

import math
from collections.abc import Iterable

import numpy as np
from scipy.optimize import least_squares

from .math_utils import circular_mean, ridge_lstsq, wrap_angle
from .models import CalibrationResult, Observation, PoseEstimate


class PoseCorrector:
    """Fit and apply a laser-based correction model.

    The model uses M/N to compute a high-precision chassis yaw, then uses M/N/P
    wall distances plus learned fixed installation offsets to correct S.x/S.y.
    """

    def __init__(self, calibration: CalibrationResult) -> None:
        self.calibration = calibration

    @classmethod
    def fit(
        cls,
        observations: Iterable[Observation],
        mn_distance: float,
        *,
        yaw_sign: int = 1,
        ridge: float = 1e-9,
        method: str = "explicit_geometry",
        position_scale: float = 0.02,
        yaw_scale: float = 0.02,
    ) -> "PoseCorrector":
        """Fit the correction function from synchronized samples.

        ``explicit_geometry`` is the recommended method for your layout. It
        explicitly fits the independent mounting positions of the AB-side
        lasers and the BC-side laser relative to S.
        """

        samples = list(observations)
        if not samples:
            raise ValueError("observations must not be empty")
        if mn_distance <= 0.0:
            raise ValueError("mn_distance must be positive")
        if yaw_sign not in (-1, 1):
            raise ValueError("yaw_sign must be either 1 or -1")
        if method not in ("linear", "robust_geometric", "explicit_geometry"):
            raise ValueError("method must be 'linear', 'robust_geometric', or 'explicit_geometry'")
        if position_scale <= 0.0:
            raise ValueError("position_scale must be positive")
        if yaw_scale <= 0.0:
            raise ValueError("yaw_scale must be positive")

        laser_yaws = np.asarray(
            [cls.laser_yaw(sample.m, sample.n, mn_distance, yaw_sign) for sample in samples],
            dtype=float,
        )
        odin_yaws = np.asarray([sample.s_yaw for sample in samples], dtype=float)
        yaw_offset = circular_mean(odin_yaws - laser_yaws)
        corrected_yaws = np.asarray([wrap_angle(yaw + yaw_offset) for yaw in laser_yaws])

        c = np.cos(laser_yaws)
        hx = np.asarray([0.5 * (sample.m + sample.n) for sample in samples], dtype=float) * c
        hy = np.asarray([sample.p for sample in samples], dtype=float) * c

        features = np.column_stack(
            [
                np.ones(len(samples), dtype=float),
                np.cos(corrected_yaws),
                np.sin(corrected_yaws),
            ]
        )

        x_target = np.asarray([sample.s_x for sample in samples], dtype=float) - hx
        y_target = np.asarray([sample.s_y for sample in samples], dtype=float) - hy
        x_coefficients = ridge_lstsq(features, x_target, ridge)
        y_coefficients = ridge_lstsq(features, y_target, ridge)

        optimizer_cost = None
        optimizer_iterations = None
        ab_sensor_x = None
        ab_sensor_mid_y = None
        p_sensor_x = None
        p_sensor_y = None
        if method == "explicit_geometry":
            if len(samples) < 6:
                raise ValueError("explicit_geometry requires at least 6 observations; 20+ is recommended")
            initial_geometry = cls._initial_explicit_geometry(
                samples=samples,
                corrected_yaws=corrected_yaws,
                yaw_offset=yaw_offset,
            )

            def geometry_residuals(params: np.ndarray) -> np.ndarray:
                offset, ab_x, ab_mid_y, p_x, p_y = params
                pred = cls._predict_explicit_geometry(
                    samples=samples,
                    mn_distance=mn_distance,
                    yaw_sign=yaw_sign,
                    yaw_offset=offset,
                    ab_sensor_x=ab_x,
                    ab_sensor_mid_y=ab_mid_y,
                    p_sensor_x=p_x,
                    p_sensor_y=p_y,
                )
                pred_x = np.asarray([pose.x for pose in pred], dtype=float)
                pred_y = np.asarray([pose.y for pose in pred], dtype=float)
                pred_yaw = np.asarray([pose.yaw for pose in pred], dtype=float)
                return np.concatenate(
                    [
                        (pred_x - np.asarray([s.s_x for s in samples])) / position_scale,
                        (pred_y - np.asarray([s.s_y for s in samples])) / position_scale,
                        np.asarray([wrap_angle(a - b) for a, b in zip(pred_yaw, odin_yaws)]) / yaw_scale,
                    ]
                )

            result = least_squares(
                geometry_residuals,
                initial_geometry,
                method="trf",
                loss="soft_l1",
                f_scale=1.0,
                max_nfev=2000,
            )
            yaw_offset = float(result.x[0])
            ab_sensor_x = float(result.x[1])
            ab_sensor_mid_y = float(result.x[2])
            p_sensor_x = float(result.x[3])
            p_sensor_y = float(result.x[4])
            optimizer_cost = float(result.cost)
            optimizer_iterations = int(result.nfev)

            poses = cls._predict_explicit_geometry(
                samples=samples,
                mn_distance=mn_distance,
                yaw_sign=yaw_sign,
                yaw_offset=yaw_offset,
                ab_sensor_x=ab_sensor_x,
                ab_sensor_mid_y=ab_sensor_mid_y,
                p_sensor_x=p_sensor_x,
                p_sensor_y=p_sensor_y,
            )
            x_pred = np.asarray([pose.x for pose in poses], dtype=float)
            y_pred = np.asarray([pose.y for pose in poses], dtype=float)
            corrected_yaws = np.asarray([pose.yaw for pose in poses], dtype=float)
            x_coefficients = np.zeros(3, dtype=float)
            y_coefficients = np.zeros(3, dtype=float)

        if method == "robust_geometric":
            if len(samples) < 6:
                raise ValueError("robust_geometric requires at least 6 observations; 20+ is recommended")
            initial = np.asarray(
                [yaw_offset, *x_coefficients.tolist(), *y_coefficients.tolist()],
                dtype=float,
            )

            def residuals(params: np.ndarray) -> np.ndarray:
                offset = params[0]
                x_coef = params[1:4]
                y_coef = params[4:7]
                pred_yaws = np.asarray([wrap_angle(v + offset) for v in laser_yaws])
                basis = np.column_stack(
                    [
                        np.ones(len(samples), dtype=float),
                        np.cos(pred_yaws),
                        np.sin(pred_yaws),
                    ]
                )
                pred_x = hx + basis @ x_coef
                pred_y = hy + basis @ y_coef
                yaw_res = np.asarray([wrap_angle(a - b) for a, b in zip(pred_yaws, odin_yaws)])
                return np.concatenate(
                    [
                        (pred_x - np.asarray([s.s_x for s in samples])) / position_scale,
                        (pred_y - np.asarray([s.s_y for s in samples])) / position_scale,
                        yaw_res / yaw_scale,
                    ]
                )

            result = least_squares(
                residuals,
                initial,
                method="trf",
                loss="soft_l1",
                f_scale=1.0,
                max_nfev=2000,
            )
            yaw_offset = float(result.x[0])
            x_coefficients = result.x[1:4]
            y_coefficients = result.x[4:7]
            optimizer_cost = float(result.cost)
            optimizer_iterations = int(result.nfev)

            corrected_yaws = np.asarray([wrap_angle(yaw + yaw_offset) for yaw in laser_yaws])
            features = np.column_stack(
                [
                    np.ones(len(samples), dtype=float),
                    np.cos(corrected_yaws),
                    np.sin(corrected_yaws),
                ]
            )

        if method != "explicit_geometry":
            x_pred = hx + features @ x_coefficients
            y_pred = hy + features @ y_coefficients
        yaw_errors = np.asarray([wrap_angle(a - b) for a, b in zip(corrected_yaws, odin_yaws)])

        calibration = CalibrationResult(
            mn_distance=float(mn_distance),
            yaw_offset=float(yaw_offset),
            x_coefficients=tuple(float(v) for v in x_coefficients),
            y_coefficients=tuple(float(v) for v in y_coefficients),
            yaw_sign=yaw_sign,
            residual_rms_x=float(np.sqrt(np.mean((x_pred - np.asarray([s.s_x for s in samples])) ** 2))),
            residual_rms_y=float(np.sqrt(np.mean((y_pred - np.asarray([s.s_y for s in samples])) ** 2))),
            residual_rms_yaw=float(np.sqrt(np.mean(yaw_errors**2))),
            sample_count=len(samples),
            fit_method=method,
            position_scale=float(position_scale),
            yaw_scale=float(yaw_scale),
            optimizer_cost=optimizer_cost,
            optimizer_iterations=optimizer_iterations,
            ab_sensor_x=ab_sensor_x,
            ab_sensor_mid_y=ab_sensor_mid_y,
            p_sensor_x=p_sensor_x,
            p_sensor_y=p_sensor_y,
        )
        return cls(calibration)

    @classmethod
    def _initial_explicit_geometry(
        cls,
        *,
        samples: list[Observation],
        corrected_yaws: np.ndarray,
        yaw_offset: float,
    ) -> np.ndarray:
        c = np.cos(corrected_yaws)
        s = np.sin(corrected_yaws)
        avg_mn = np.asarray([0.5 * (sample.m + sample.n) for sample in samples], dtype=float)
        p = np.asarray([sample.p for sample in samples], dtype=float)
        sx = np.asarray([sample.s_x for sample in samples], dtype=float)
        sy = np.asarray([sample.s_y for sample in samples], dtype=float)

        ab_features = np.column_stack([-c, s])
        ab_target = sx - avg_mn * c
        ab_x, ab_mid_y = np.linalg.lstsq(ab_features, ab_target, rcond=None)[0]

        p_features = np.column_stack([-s, -c])
        p_target = sy - p * c
        p_x, p_y = np.linalg.lstsq(p_features, p_target, rcond=None)[0]
        return np.asarray([yaw_offset, ab_x, ab_mid_y, p_x, p_y], dtype=float)

    @classmethod
    def _predict_explicit_geometry(
        cls,
        *,
        samples: list[Observation],
        mn_distance: float,
        yaw_sign: int,
        yaw_offset: float,
        ab_sensor_x: float,
        ab_sensor_mid_y: float,
        p_sensor_x: float,
        p_sensor_y: float,
    ) -> list[PoseEstimate]:
        poses = []
        for sample in samples:
            laser_yaw = cls.laser_yaw(sample.m, sample.n, mn_distance, yaw_sign)
            yaw = wrap_angle(laser_yaw + yaw_offset)
            c = math.cos(yaw)
            s = math.sin(yaw)
            x = 0.5 * (sample.m + sample.n) * c - ab_sensor_x * c + ab_sensor_mid_y * s
            y = sample.p * c - p_sensor_x * s - p_sensor_y * c
            poses.append(PoseEstimate(x=x, y=y, yaw=yaw))
        return poses

    @staticmethod
    def laser_yaw(m: float, n: float, mn_distance: float, yaw_sign: int = 1) -> float:
        """Compute chassis yaw from the M/N distance difference.

        With the default sign convention, positive yaw means M reads farther
        than N. If the physical wiring/order is opposite, fit/use yaw_sign=-1.
        """

        if mn_distance <= 0.0:
            raise ValueError("mn_distance must be positive")
        if yaw_sign not in (-1, 1):
            raise ValueError("yaw_sign must be either 1 or -1")
        return math.atan2(yaw_sign * (m - n), mn_distance)

    def correct(self, observation: Observation) -> PoseEstimate:
        cal = self.calibration
        if cal.fit_method == "explicit_geometry":
            if (
                cal.ab_sensor_x is None
                or cal.ab_sensor_mid_y is None
                or cal.p_sensor_x is None
                or cal.p_sensor_y is None
            ):
                raise ValueError("explicit_geometry calibration is missing sensor mounting parameters")
            return self._predict_explicit_geometry(
                samples=[observation],
                mn_distance=cal.mn_distance,
                yaw_sign=cal.yaw_sign,
                yaw_offset=cal.yaw_offset,
                ab_sensor_x=cal.ab_sensor_x,
                ab_sensor_mid_y=cal.ab_sensor_mid_y,
                p_sensor_x=cal.p_sensor_x,
                p_sensor_y=cal.p_sensor_y,
            )[0]

        laser_yaw = self.laser_yaw(observation.m, observation.n, cal.mn_distance, cal.yaw_sign)
        yaw = wrap_angle(laser_yaw + cal.yaw_offset)
        c_laser = math.cos(laser_yaw)

        hx = 0.5 * (observation.m + observation.n) * c_laser
        hy = observation.p * c_laser
        basis = np.asarray([1.0, math.cos(yaw), math.sin(yaw)], dtype=float)

        x = hx + float(basis @ np.asarray(cal.x_coefficients, dtype=float))
        y = hy + float(basis @ np.asarray(cal.y_coefficients, dtype=float))
        return PoseEstimate(x=x, y=y, yaw=yaw)

    def correct_many(self, observations: Iterable[Observation]) -> list[PoseEstimate]:
        return [self.correct(sample) for sample in observations]
