import math

import pytest

from pose_correct import Observation, PoseCorrector


def make_sample(
    *,
    x: float,
    y: float,
    yaw: float,
    m_sensor_x: float,
    m_sensor_y: float,
    n_sensor_x: float,
    n_sensor_y: float,
) -> Observation:
    c = math.cos(yaw)
    s = math.sin(yaw)
    m = (x + m_sensor_x * c - m_sensor_y * s) / c
    n = (y + n_sensor_x * s + n_sensor_y * c) / c
    return Observation(m=m, n=n, s_x=x, s_y=y, s_yaw=yaw)


def test_fit_and_correct_recovers_two_laser_geometry() -> None:
    m_sensor_x = -0.23
    m_sensor_y = 0.11
    n_sensor_x = 0.18
    n_sensor_y = -0.21
    samples = [
        make_sample(
            x=1.0 + 0.03 * i,
            y=0.7 + 0.02 * (i % 4),
            yaw=-0.24 + 0.02 * i,
            m_sensor_x=m_sensor_x,
            m_sensor_y=m_sensor_y,
            n_sensor_x=n_sensor_x,
            n_sensor_y=n_sensor_y,
        )
        for i in range(25)
    ]

    corrector = PoseCorrector.fit(samples)
    pose = corrector.correct(samples[17])

    assert pose.x == pytest.approx(samples[17].s_x, abs=1e-10)
    assert pose.y == pytest.approx(samples[17].s_y, abs=1e-10)
    assert pose.yaw == pytest.approx(samples[17].s_yaw, abs=1e-12)
    assert corrector.calibration.m_sensor_x == pytest.approx(m_sensor_x, abs=1e-10)
    assert corrector.calibration.m_sensor_y == pytest.approx(m_sensor_y, abs=1e-10)
    assert corrector.calibration.n_sensor_x == pytest.approx(n_sensor_x, abs=1e-10)
    assert corrector.calibration.n_sensor_y == pytest.approx(n_sensor_y, abs=1e-10)


def test_fit_requires_observations() -> None:
    with pytest.raises(ValueError, match="observations must not be empty"):
        PoseCorrector.fit([])
