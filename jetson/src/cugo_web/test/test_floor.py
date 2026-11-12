import math

import numpy as np

from cugo_web.floor import corrected_pose, fit_floor, rotation_matrix


def rot_y(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_x(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def simulate(true_pitch, true_roll, true_z, est_pitch, est_roll, est_z, cam=(0.24, 0.0),
             n=4000, seed=1):
    """本当の姿勢のカメラで床を見て、推定姿勢で base 座標に戻した点群を作る。"""
    rng = np.random.default_rng(seed)
    floor = np.column_stack([rng.uniform(0.4, 2.4, n), rng.uniform(-0.9, 0.9, n), np.zeros(n)])
    # 障害物（箱）も混ぜる
    box = np.column_stack([rng.uniform(1.0, 1.2, 400), rng.uniform(-0.1, 0.1, 400),
                           rng.uniform(0.0, 0.3, 400)])
    world = np.vstack([floor, box])
    t_true = np.array([cam[0], cam[1], true_z])
    t_est = np.array([cam[0], cam[1], est_z])
    # URDF の rpy は R = Rz(yaw) Ry(pitch) Rx(roll)
    R_true = rot_y(true_pitch) @ rot_x(true_roll)
    R_est = rot_y(est_pitch) @ rot_x(est_roll)
    pc = (world - t_true) @ R_true            # = R_true^T (p - t)
    return pc @ R_est.T + t_est               # 推定姿勢で戻す


def test_level_floor_needs_no_correction():
    pts = simulate(0.1, 0.0, 0.2, 0.1, 0.0, 0.2)
    fit = fit_floor(pts)
    assert fit is not None
    assert abs(fit.a) < 0.005 and abs(fit.b) < 0.005 and abs(fit.c) < 0.005


def test_recovers_pitch_roll_height():
    true = (0.15, -0.05, 0.17)
    est = [0.0, 0.0, 0.25]
    for _ in range(3):  # 実機と同じく数回繰り返して収束させる
        pts = simulate(*true, *est)
        fit = fit_floor(pts)
        assert fit is not None
        est = list(corrected_pose(fit, est[0], est[1], est[2], 0.24, 0.0))
    assert math.isclose(est[0], true[0], abs_tol=0.01)
    assert math.isclose(est[1], true[1], abs_tol=0.01)
    assert math.isclose(est[2], true[2], abs_tol=0.01)


def test_no_floor_returns_none():
    rng = np.random.default_rng(0)
    wall = np.column_stack([np.full(2000, 1.0), rng.uniform(-1, 1, 2000), rng.uniform(0, 1, 2000)])
    assert fit_floor(wall) is None
    assert fit_floor(np.zeros((0, 3))) is None


def test_rotation_matrix_identity_and_yaw():
    assert np.allclose(rotation_matrix((0, 0, 0, 1)), np.eye(3))
    s = math.sin(math.pi / 4)
    R = rotation_matrix((0, 0, s, s))  # yaw 90°
    assert np.allclose(R @ [1, 0, 0], [0, 1, 0])
