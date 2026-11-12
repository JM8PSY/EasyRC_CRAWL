"""床面からカメラ（D455）の取付姿勢を補正する（ROS 非依存）。

現在の URDF で base_footprint 座標に変換した点群から床面 z = a + b·x + c·y を求め、
床が z = 0 の水平面になるように camera_pitch / camera_roll / camera_z を直す。

導出（現在の値からのずれを δ とする。xacro の rpy は pitch 正 = 下向き、roll 正 = 左が上）:
- 実際のカメラが想定より δp だけ下を向いていると、床は前方ほど高く見える: b ≈ tan δp → pitch += atan(b)
- 実際のカメラが想定より δr だけ左上がりだと、床は左ほど低く見える: c ≈ -tan δr → roll -= atan(c)
- カメラの真下で床が Δ = a + b·xc + c·yc の高さに見える → 実際のカメラは Δ だけ低い: z -= Δ
"""
import math
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class FloorFit:
    a: float
    b: float
    c: float
    inliers: int
    candidates: int


def fit_floor(points: np.ndarray, *, x_range=(0.3, 2.5), y_abs=1.0, z_abs=0.35,
              threshold=0.015, iterations=300, min_inliers=300, max_tilt_deg=20.0,
              seed: int = 0) -> Optional[FloorFit]:
    """base_footprint 座標の点 (N, 3) から床面を RANSAC で求める。見つからなければ None。"""
    p = np.asarray(points, dtype=np.float64)
    if p.ndim != 2 or p.shape[1] != 3:
        return None
    m = ((p[:, 0] > x_range[0]) & (p[:, 0] < x_range[1]) & (np.abs(p[:, 1]) < y_abs)
         & (np.abs(p[:, 2]) < z_abs) & np.all(np.isfinite(p), axis=1))
    cand = p[m]
    if len(cand) < min_inliers:
        return None
    rng = np.random.default_rng(seed)
    min_nz = math.cos(math.radians(max_tilt_deg))
    best_mask, best_count = None, 0
    for _ in range(iterations):
        s = cand[rng.choice(len(cand), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if n[2] < 0:
            n = -n
        if n[2] < min_nz:
            continue  # ほぼ水平な面だけ
        d = n @ s[0]
        mask = np.abs(cand @ n - d) < threshold
        count = int(mask.sum())
        if count > best_count:
            best_mask, best_count = mask, count
    if best_mask is None or best_count < min_inliers:
        return None
    q = cand[best_mask]
    # 最小二乗 z = a + b x + c y
    A = np.column_stack([np.ones(len(q)), q[:, 0], q[:, 1]])
    (a, b, c), *_ = np.linalg.lstsq(A, q[:, 2], rcond=None)
    return FloorFit(float(a), float(b), float(c), best_count, int(len(cand)))


def corrected_pose(fit: FloorFit, pitch: float, roll: float, z: float,
                   cam_x: float, cam_y: float):
    """(新しい pitch, roll, z) を返す。"""
    new_pitch = pitch + math.atan(fit.b)
    new_roll = roll - math.atan(fit.c)
    delta = fit.a + fit.b * cam_x + fit.c * cam_y
    return new_pitch, new_roll, z - delta


def rotation_matrix(q) -> np.ndarray:
    """クォータニオン (x, y, z, w) → 3x3。"""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
