"""進入禁止エリア（多角形）の検証・保存・ラスタ化（ROS 非依存）。

- Nav2 の KeepoutFilter に渡すマスク（OccupancyGrid: 100 = 禁止、0 = 制限なし）を作る
- 自動探索のフロンティア選択で、禁止エリア内を「占有」扱いにして選ばないようにする
"""
import math
import os
import tempfile
from typing import List, Optional, Sequence, Tuple

import numpy as np
import yaml

MAX_ZONES = 50
MAX_POINTS = 64
MAX_COORD = 1000.0  # [m] 地図座標の妥当な範囲


def validate_points(pts) -> Optional[List[Tuple[float, float]]]:
    """ブラウザから来た頂点列を検証する。不正なら None。"""
    if not isinstance(pts, (list, tuple)) or not 3 <= len(pts) <= MAX_POINTS:
        return None
    out = []
    for p in pts:
        if not isinstance(p, (list, tuple)) or len(p) != 2:
            return None
        try:
            x, y = float(p[0]), float(p[1])
        except (TypeError, ValueError):
            return None
        if not (math.isfinite(x) and math.isfinite(y)) or abs(x) > MAX_COORD or abs(y) > MAX_COORD:
            return None
        out.append((round(x, 3), round(y, 3)))
    if abs(polygon_area(out)) < 1e-4:
        return None  # 面積ゼロ（一直線）
    return out


def polygon_area(pts: Sequence[Tuple[float, float]]) -> float:
    a = 0.0
    for (x1, y1), (x2, y2) in zip(pts, list(pts[1:]) + [pts[0]]):
        a += x1 * y2 - x2 * y1
    return a / 2.0


def points_in_polygon(xs: np.ndarray, ys: np.ndarray, poly: Sequence[Tuple[float, float]]) -> np.ndarray:
    """偶奇規則で点群が多角形内かを判定する（ベクトル化）。"""
    inside = np.zeros(np.broadcast(xs, ys).shape, dtype=bool)
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if y1 == y2:
            continue
        cond = (ys >= min(y1, y2)) & (ys < max(y1, y2))
        xcross = x1 + (ys - y1) * (x2 - x1) / (y2 - y1)
        inside ^= cond & (xs < xcross)
    return inside


def rasterize(zones, width: int, height: int, resolution: float,
              origin: Tuple[float, float]) -> np.ndarray:
    """地図と同じ格子（row 0 = 下端）で、禁止エリア内なら True の配列を返す。"""
    mask = np.zeros((height, width), dtype=bool)
    if not zones or width <= 0 or height <= 0:
        return mask
    ox, oy = origin
    for z in zones:
        pts = z['pts']
        xs_all = [p[0] for p in pts]
        ys_all = [p[1] for p in pts]
        c0 = max(0, int(math.floor((min(xs_all) - ox) / resolution)))
        c1 = min(width, int(math.ceil((max(xs_all) - ox) / resolution)) + 1)
        r0 = max(0, int(math.floor((min(ys_all) - oy) / resolution)))
        r1 = min(height, int(math.ceil((max(ys_all) - oy) / resolution)) + 1)
        if c0 >= c1 or r0 >= r1:
            continue
        cx = ox + (np.arange(c0, c1) + 0.5) * resolution
        cy = oy + (np.arange(r0, r1) + 0.5) * resolution
        gx, gy = np.meshgrid(cx, cy)
        mask[r0:r1, c0:c1] |= points_in_polygon(gx, gy, pts)
    return mask


def mask_grid(zones, resolution: float = 0.05, margin: float = 1.0):
    """禁止エリアを覆う格子を作る（KeepoutFilter 用）。(data int8 1D, width, height, origin) を返す。

    エリアが無いときは 1x1 の制限なしマスク。
    """
    if not zones:
        return np.zeros(1, dtype=np.int8), 1, 1, (0.0, 0.0)
    xs = [p[0] for z in zones for p in z['pts']]
    ys = [p[1] for z in zones for p in z['pts']]
    ox, oy = min(xs) - margin, min(ys) - margin
    width = int(math.ceil((max(xs) + margin - ox) / resolution - 1e-6))
    height = int(math.ceil((max(ys) + margin - oy) / resolution - 1e-6))
    m = rasterize(zones, width, height, resolution, (ox, oy))
    data = np.where(m, 100, 0).astype(np.int8).ravel()
    return data, width, height, (ox, oy)


def load(path: str) -> list:
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        return []
    zones = []
    for i, z in enumerate(data.get('zones', []) if isinstance(data, dict) else []):
        pts = validate_points(z.get('pts') if isinstance(z, dict) else None)
        if pts:
            zones.append({'id': str(z.get('id') or 'z%d' % (i + 1)), 'pts': pts})
    return zones[:MAX_ZONES]


def save(zones, path: str) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    body = '# CuGo 進入禁止エリア（地図座標 [m]）\n' + yaml.safe_dump(
        {'zones': [{'id': z['id'], 'pts': [list(p) for p in z['pts']]} for z in zones]},
        default_flow_style=None, allow_unicode=True)
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(os.path.abspath(path)), prefix='.keepout.')
    with os.fdopen(fd, 'w') as f:
        f.write(body)
    os.replace(tmp, path)
