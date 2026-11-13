"""RICOH THETA（2 眼の魚眼）の画像から「床がどこまで続いているか」を方向ごとに求める。

ROS に依存しない部分だけを置く（theta_node.py が使う）。

座標（ロボット基準 base_footprint）: x 前、y 左、z 上。方位 φ は前 = 0、左回りが正。
仰角 e は水平 = 0、上が正（床は負）。

1. 2 眼魚眼 → 正距円筒（equirectangular）: 等距離射影（r = R·θ / (fov/2)）のレンズ 2 つ。
   列 = 方位（中央 = 前、左へ行くほど左）、行 = 仰角（上 = +90°）。
2. 床の境界: 方位ごとに、近く（ロボットの足元のすぐ外）から水平線へ向かって画素を見ていき、
   床の色（ロボットの前の床から取る）でなくなった所を境界とする。
3. 距離: カメラの高さ h、境界の俯角 δ から d = h / tan δ（平らな床を仮定）。
"""
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover  テスト環境に OpenCV が無いとき（remap だけ使えない）
    cv2 = None


class Lens:
    """魚眼 1 つ。axis_yaw: 光軸の方位 [rad]（前 = 0）、roll: 画像の回転 [rad]。"""

    def __init__(self, cx: float, cy: float, radius: float, fov_deg: float = 195.0,
                 axis_yaw: float = 0.0, roll: float = 0.0):
        self.cx, self.cy, self.radius = float(cx), float(cy), float(radius)
        self.half_fov = math.radians(fov_deg) / 2.0
        self.axis_yaw, self.roll = float(axis_yaw), float(roll)

    def project(self, d: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """方向ベクトル d (..., 3) → (u, v, 光軸からの角度 θ)。"""
        a = np.array([math.cos(self.axis_yaw), math.sin(self.axis_yaw), 0.0])
        r = np.array([math.sin(self.axis_yaw), -math.cos(self.axis_yaw), 0.0])  # 画像の右
        up = np.array([0.0, 0.0, 1.0])
        x, y, z = d @ r, d @ up, d @ a
        theta = np.arctan2(np.hypot(x, y), z)
        rho = self.radius * theta / self.half_fov
        ang = np.arctan2(y, x) + self.roll
        return self.cx + rho * np.cos(ang), self.cy - rho * np.sin(ang), theta


def bearings(width: int) -> np.ndarray:
    """正距円筒の各列の方位 [rad]。中央の列が前（0）、左端が真後ろ（+π 側）。"""
    return math.pi - (np.arange(width) + 0.5) / width * 2.0 * math.pi


def elevations(height: int) -> np.ndarray:
    """正距円筒の各行の仰角 [rad]。上端 +π/2。"""
    return math.pi / 2.0 - (np.arange(height) + 0.5) / height * math.pi


def directions(width: int, height: int) -> np.ndarray:
    phi = bearings(width)[None, :]
    e = elevations(height)[:, None]
    return np.stack([np.cos(e) * np.cos(phi), np.cos(e) * np.sin(phi),
                     np.sin(e) * np.ones_like(phi)], axis=-1)


def equirect_maps(lenses: Sequence[Lens], width: int, height: int
                  ) -> Tuple[np.ndarray, np.ndarray]:
    """cv2.remap 用の (map_x, map_y)（float32）。各画素は光軸に近い方のレンズから取る。"""
    d = directions(width, height)
    best_theta = np.full((height, width), np.inf)
    mx = np.full((height, width), -1.0, np.float32)
    my = np.full((height, width), -1.0, np.float32)
    for lens in lenses:
        u, v, th = lens.project(d)
        use = (th < best_theta) & (th <= lens.half_fov)
        mx[use], my[use], best_theta[use] = u[use], v[use], th[use]
    return mx, my


def to_equirect(img: np.ndarray, maps: Tuple[np.ndarray, np.ndarray]) -> np.ndarray:
    if cv2 is None:
        raise RuntimeError('OpenCV が必要です')
    return cv2.remap(img, maps[0], maps[1], cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)


def find_lens_circles(img: np.ndarray, thresh: int = 25) -> List[Tuple[float, float, float]]:
    """左右に並んだ 2 つの円（魚眼の写っている範囲）を明るさで探す。(cx, cy, r) を左から順に。"""
    gray = img if img.ndim == 2 else img.mean(axis=2)
    h, w = gray.shape
    out = []
    for x0, x1 in ((0, w // 2), (w // 2, w)):
        ys, xs = np.nonzero(gray[:, x0:x1] > thresh)
        if len(xs) < 100:
            out.append(((x0 + x1) / 2.0, h / 2.0, min(h, x1 - x0) / 2.0))
            continue
        cx, cy = (xs.min() + xs.max()) / 2.0 + x0, (ys.min() + ys.max()) / 2.0
        r = max(xs.max() - xs.min(), ys.max() - ys.min()) / 2.0
        out.append((float(cx), float(cy), float(r)))
    return out


def depression_for_range(rng, height: float):
    """床の距離 → 俯角 [rad]（正）。"""
    return np.arctan2(height, rng)


def range_for_depression(dep, height: float, max_range: float):
    """俯角 → 床の距離。水平線より上（俯角 0 以下）は max_range。"""
    dep = np.asarray(dep, float)
    with np.errstate(divide='ignore'):
        r = np.where(dep > 1e-6, height / np.tan(np.maximum(dep, 1e-6)), max_range)
    return np.minimum(r, max_range)


def to_lab(img_bgr: np.ndarray) -> np.ndarray:
    if cv2 is not None:
        return cv2.cvtColor(img_bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    return img_bgr.astype(np.float32)  # テスト用（色空間はそのまま）


def floor_model(lab: np.ndarray, rows: np.ndarray, cols: np.ndarray
                ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """基準の床（rows × cols の画素）の色の中央値とばらつき（MAD、最小値つき）。"""
    patch = lab[np.ix_(rows, cols)].reshape(-1, lab.shape[2])
    patch = patch[np.isfinite(patch).all(axis=1)]
    if len(patch) < 20:
        return None
    med = np.median(patch, axis=0)
    mad = np.median(np.abs(patch - med), axis=0) * 1.4826
    return med, np.maximum(mad, np.array([4.0, 2.0, 2.0])[:lab.shape[2]])


def floor_ranges(lab: np.ndarray, height: float, *, near: float = 0.6, max_range: float = 5.0,
                 ref_near: float = 0.7, ref_far: float = 1.1, ref_half_width: float = 0.35,
                 k: float = 4.0, gap: int = 3, valid: Optional[np.ndarray] = None
                 ) -> Tuple[np.ndarray, np.ndarray]:
    """方位ごとの「床が続く距離」。

    lab: 正距円筒（Lab、行 = 仰角、列 = 方位）。height: カメラの高さ [m]。
    near: ここより近くはロボット自身が写るので見ない（ここから外へ向かって床をたどる）。
    基準の床: 前方 ref_near〜ref_far [m]、方位 ±ref_half_width [rad] の画素の色。
    k: 床とみなす色の差（ばらつきの何倍まで）。gap: これ以上続けて床でなければ境界。
    valid: 使ってよい画素（魚眼の外・ロボットの写り込みを除く）。

    戻り値: (ranges [m]（方位ごと。near で床でなければ 0、境界が見つからなければ max_range）,
             base_ok: 基準の床が取れたか（取れなければ全部 0）)。
    """
    h, w = lab.shape[:2]
    e = elevations(h)
    phi = bearings(w)
    dep = -e  # 俯角（床は正）
    rows_all = np.nonzero(dep > 0)[0]
    # 外へ向かう順 = 俯角が大きい（下）→ 小さい（水平線）= 行番号の大きい方から
    dep_near = float(depression_for_range(near, height))
    scan_rows = [r for r in rows_all[::-1] if dep[r] <= dep_near]
    ref_rows = np.nonzero((dep <= depression_for_range(ref_near, height))
                          & (dep >= depression_for_range(ref_far, height)))[0]
    ref_cols = np.nonzero(np.abs(phi) <= ref_half_width)[0]
    zero = np.zeros(w)
    if not scan_rows or len(ref_rows) == 0 or len(ref_cols) == 0:
        return zero, np.array(False)
    model = floor_model(lab, ref_rows, ref_cols)
    if model is None:
        return zero, np.array(False)
    med, mad = model
    score = np.sqrt((((lab - med) / mad) ** 2).sum(axis=2)) if lab.ndim == 3 else \
        np.abs(lab - med) / mad
    is_floor = score <= k
    if valid is not None:
        is_floor &= valid
    sub = is_floor[scan_rows]  # (len(scan_rows), w) 外へ向かう順
    n = len(scan_rows)
    out = np.empty(w)
    for c in range(w):
        col = sub[:, c]
        if not col[0]:
            out[c] = 0.0
            continue
        miss = 0
        last = 0
        for i in range(1, n):
            if col[i]:
                last = i
                miss = 0
            else:
                miss += 1
                if miss >= gap:
                    break
        out[c] = float(range_for_depression(dep[scan_rows[last]], height, max_range))
    return out, np.array(True)


def clip_to_lidar(theta_ranges: np.ndarray, theta_bearings: np.ndarray,
                  lidar_ranges: np.ndarray, lidar_bearings: np.ndarray,
                  margin: float = 0.2, tol: float = math.radians(3.0)) -> np.ndarray:
    """LiDAR が手前で何かに当たっている方位は、その距離 − margin で切る。

    LiDAR が「反射なし（inf）」の方位や、近くに LiDAR の値が無い方位はカメラの値をそのまま使う。
    → カメラは LiDAR の届かない所（廊下の奥・ロボット自身の陰）を補う役にする。
    """
    out = np.array(theta_ranges, float, copy=True)
    if len(lidar_ranges) == 0:
        return out
    lb = np.asarray(lidar_bearings, float)
    lr = np.asarray(lidar_ranges, float)
    order = np.argsort(lb)
    lb, lr = lb[order], lr[order]
    for i, b in enumerate(theta_bearings):
        # 角度差の最小（±π の折り返しを考える）
        diff = np.abs((lb - b + math.pi) % (2 * math.pi) - math.pi)
        near = diff <= tol
        if not near.any():
            continue
        finite = lr[near][np.isfinite(lr[near]) & (lr[near] > 0)]
        if len(finite):
            out[i] = min(out[i], max(0.0, float(finite.min()) - margin))
    return out


def free_points(ranges: np.ndarray, bearings_rad: np.ndarray, *, near: float = 0.6,
                step: float = 0.05, sector: float = math.pi,
                offset: Tuple[float, float] = (0.0, 0.0)) -> np.ndarray:
    """方位ごとに near〜range の床の点（base_footprint、(N, 2)）。sector: 前方 ±sector の方位だけ。"""
    pts = []
    for r, b in zip(ranges, bearings_rad):
        if r <= near or abs(b) > sector:
            continue
        d = np.arange(near, r + 1e-9, step)
        pts.append(np.column_stack([offset[0] + d * math.cos(b), offset[1] + d * math.sin(b)]))
    if not pts:
        return np.zeros((0, 2))
    return np.vstack(pts)


def estimate_height(theta_dep: np.ndarray, lidar_ranges: np.ndarray,
                    max_range: float = 4.0) -> Optional[float]:
    """壁の根元の俯角と、同じ方位の LiDAR の距離からカメラの高さを推定（h = d·tan δ の中央値）。"""
    dep = np.asarray(theta_dep, float)
    lr = np.asarray(lidar_ranges, float)
    ok = np.isfinite(lr) & (lr > 0.3) & (lr < max_range) & (dep > 0.02)
    if ok.sum() < 10:
        return None
    return float(np.median(lr[ok] * np.tan(dep[ok])))
