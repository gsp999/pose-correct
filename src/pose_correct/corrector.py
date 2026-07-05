from __future__ import annotations

import math
from collections.abc import Iterable

import numpy as np

from .models import CalibrationResult, Observation, PoseEstimate


class PoseCorrector:
    """Fit and apply the two-laser pose correction model.

    The odin yaw is treated as accurate. The model uses the M laser reading to
    correct S.x and the N laser reading to correct S.y, while fitting each
    laser's fixed body-frame position relative to S.
    """

    def __init__(self, calibration: CalibrationResult) -> None:
        self.calibration = calibration

    @classmethod
    def fit(cls, observations: Iterable[Observation]) -> "PoseCorrector":
        samples = list(observations)
        if not samples:
            raise ValueError("observations must not be empty")

        yaws = np.asarray([sample.s_yaw for sample in samples], dtype=float)
        cos_yaw = np.cos(yaws)
        sin_yaw = np.sin(yaws)

        m_distances = np.asarray([sample.m for sample in samples], dtype=float)
        n_distances = np.asarray([sample.n for sample in samples], dtype=float)
        odin_x = np.asarray([sample.s_x for sample in samples], dtype=float)
        odin_y = np.asarray([sample.s_y for sample in samples], dtype=float)

        # S.x = m*cos(yaw) - m_sensor_x*cos(yaw) + m_sensor_y*sin(yaw)
        m_features = np.column_stack([-cos_yaw, sin_yaw])
        m_target = odin_x - m_distances * cos_yaw
        m_sensor_x, m_sensor_y = np.linalg.lstsq(m_features, m_target, rcond=None)[0]

        # S.y = n*cos(yaw) - n_sensor_x*sin(yaw) - n_sensor_y*cos(yaw)
        n_features = np.column_stack([-sin_yaw, -cos_yaw])
        n_target = odin_y - n_distances * cos_yaw
        n_sensor_x, n_sensor_y = np.linalg.lstsq(n_features, n_target, rcond=None)[0]

        poses = [
            cls._correct_with_parameters(
                sample,
                m_sensor_x=float(m_sensor_x),
                m_sensor_y=float(m_sensor_y),
                n_sensor_x=float(n_sensor_x),
                n_sensor_y=float(n_sensor_y),
            )
            for sample in samples
        ]
        predicted_x = np.asarray([pose.x for pose in poses], dtype=float)
        predicted_y = np.asarray([pose.y for pose in poses], dtype=float)

        calibration = CalibrationResult(
            residual_rms_x=float(np.sqrt(np.mean((predicted_x - odin_x) ** 2))),
            residual_rms_y=float(np.sqrt(np.mean((predicted_y - odin_y) ** 2))),
            sample_count=len(samples),
            m_sensor_x=float(m_sensor_x),
            m_sensor_y=float(m_sensor_y),
            n_sensor_x=float(n_sensor_x),
            n_sensor_y=float(n_sensor_y),
        )
        return cls(calibration)

    @staticmethod
    def _correct_with_parameters(
        observation: Observation,
        *,
        m_sensor_x: float,
        m_sensor_y: float,
        n_sensor_x: float,
        n_sensor_y: float,
    ) -> PoseEstimate:
        yaw = observation.s_yaw
        c = math.cos(yaw)
        s = math.sin(yaw)
        x = observation.m * c - m_sensor_x * c + m_sensor_y * s
        y = observation.n * c - n_sensor_x * s - n_sensor_y * c
        return PoseEstimate(x=x, y=y, yaw=yaw)

    def correct(self, observation: Observation) -> PoseEstimate:
        cal = self.calibration
        return self._correct_with_parameters(
            observation,
            m_sensor_x=cal.m_sensor_x,
            m_sensor_y=cal.m_sensor_y,
            n_sensor_x=cal.n_sensor_x,
            n_sensor_y=cal.n_sensor_y,
        )

    def correct_many(self, observations: Iterable[Observation]) -> list[PoseEstimate]:
        return [self.correct(sample) for sample in observations]
