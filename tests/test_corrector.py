import math

import pytest

from pose_correct import Observation, PoseCorrector


def make_explicit_sample(
    *,
    x: float,
    y: float,
    yaw: float,
    d: float,
    ab_x: float,
    ab_mid_y: float,
    p_x: float,
    p_y: float,
) -> Observation:
    c = math.cos(yaw)
    s = math.sin(yaw)
    m_y = ab_mid_y - d / 2.0
    n_y = ab_mid_y + d / 2.0
    m = (x + ab_x * c - m_y * s) / c
    n = (x + ab_x * c - n_y * s) / c
    p = (y + p_x * s + p_y * c) / c
    return Observation(m=m, n=n, p=p, s_x=x, s_y=y, s_yaw=yaw)


def test_laser_yaw_uses_mn_distance_difference() -> None:
    yaw = 0.2
    d = 0.4
    m = 1.0
    n = m - d * math.tan(yaw)

    assert PoseCorrector.laser_yaw(m, n, d) == pytest.approx(yaw)


def test_fit_and_correct_recovers_synthetic_pose() -> None:
    d = 0.5
    samples = []
    for yaw in (-0.3, -0.2, -0.1, 0.0, 0.08, 0.15, 0.25, 0.32):
        x = 1.2 + 0.3 * math.cos(yaw)
        y = 0.9 + 0.2 * math.sin(yaw)
        m = x / math.cos(yaw) + 0.1
        n = m - d * math.tan(yaw)
        p = y / math.cos(yaw) - 0.05
        samples.append(Observation(m=m, n=n, p=p, s_x=x, s_y=y, s_yaw=yaw))

    corrector = PoseCorrector.fit(samples, mn_distance=d, ridge=0.0, method="robust_geometric")
    pose = corrector.correct(samples[3])

    assert pose.x == pytest.approx(samples[3].s_x, abs=1e-9)
    assert pose.y == pytest.approx(samples[3].s_y, abs=1e-9)
    assert pose.yaw == pytest.approx(samples[3].s_yaw, abs=1e-9)


def test_robust_fit_limits_outlier_influence() -> None:
    d = 0.45
    samples = []
    for i in range(24):
        yaw = -0.25 + i * 0.02
        x = 1.0 + 0.02 * i + 0.12 * math.cos(yaw)
        y = 0.7 + 0.01 * i - 0.08 * math.sin(yaw)
        m = x / math.cos(yaw) + 0.06
        n = m - d * math.tan(yaw)
        p = y / math.cos(yaw) - 0.04
        odin_x = x
        odin_y = y
        if i == 10:
            odin_x += 0.25
            odin_y -= 0.20
        samples.append(Observation(m=m, n=n, p=p, s_x=odin_x, s_y=odin_y, s_yaw=yaw))

    corrector = PoseCorrector.fit(samples, mn_distance=d, method="robust_geometric")
    clean_sample = samples[12]
    pose = corrector.correct(clean_sample)

    assert abs(pose.x - clean_sample.s_x) < 0.03
    assert abs(pose.y - clean_sample.s_y) < 0.03


def test_explicit_geometry_handles_large_sensor_separation() -> None:
    d = 0.52
    ab_x = -0.24
    ab_mid_y = 0.0
    p_x = 0.26
    p_y = -0.23
    samples = [
        make_explicit_sample(
            x=1.0 + 0.03 * i,
            y=0.65 + 0.02 * (i % 5),
            yaw=-0.22 + 0.02 * i,
            d=d,
            ab_x=ab_x,
            ab_mid_y=ab_mid_y,
            p_x=p_x,
            p_y=p_y,
        )
        for i in range(24)
    ]

    corrector = PoseCorrector.fit(samples, mn_distance=d, method="explicit_geometry", ridge=0.0)
    pose = corrector.correct(samples[17])

    assert pose.x == pytest.approx(samples[17].s_x, abs=1e-8)
    assert pose.y == pytest.approx(samples[17].s_y, abs=1e-8)
    assert pose.yaw == pytest.approx(samples[17].s_yaw, abs=1e-8)
    assert corrector.calibration.ab_sensor_x == pytest.approx(ab_x, abs=1e-8)
    assert corrector.calibration.p_sensor_y == pytest.approx(p_y, abs=1e-8)
