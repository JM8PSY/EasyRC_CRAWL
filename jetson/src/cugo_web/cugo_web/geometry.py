"""ROS に依存しない変換処理（テスト可能）。

- 占有格子 → グレースケール PNG（北が上）
- LaserScan → 地図座標の点列
- yaw ⇔ クォータニオン、速度指令のクランプ
"""
import base64
import io
import math
from typing import Iterable, List, Sequence, Tuple

import numpy as np
from PIL import Image

# map_server と同じ配色: 空き=白(254)、占有=黒(0)、不明=灰(205)
FREE = 254
UNKNOWN = 205


def occupancy_to_png(data: Sequence[int], width: int, height: int) -> bytes:
    """OccupancyGrid.data（行優先、0 行目が地図の下端）を北が上の PNG にする。

    値: -1 = 不明、0〜100 = 占有確率。
    """
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    img = np.full(grid.shape, UNKNOWN, dtype=np.uint8)
    known = grid >= 0
    # 0 → 254（白）、100 → 0（黒）
    img[known] = (FREE - np.clip(grid[known], 0, 100) * FREE // 100).astype(np.uint8)
    img = np.flipud(img)  # 画像の 0 行目 = 地図の上端
    buf = io.BytesIO()
    Image.fromarray(img, mode='L').save(buf, format='PNG', optimize=False)
    return buf.getvalue()


def png_b64(png: bytes) -> str:
    return base64.b64encode(png).decode('ascii')


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """(x, y, z, w)"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def scan_to_points(ranges: Sequence[float], angle_min: float, angle_inc: float,
                   range_min: float, range_max: float,
                   tx: float, ty: float, tyaw: float,
                   max_points: int = 400) -> List[float]:
    """スキャンを、センサー姿勢 (tx, ty, tyaw)（地図座標）で地図座標の [x0, y0, x1, y1, ...] にする。

    無効値（inf / NaN / 範囲外）は除く。点数が max_points を超えたら等間隔に間引く。
    """
    n = len(ranges)
    step = max(1, math.ceil(n / max_points)) if max_points > 0 else 1
    c, s = math.cos(tyaw), math.sin(tyaw)
    out: List[float] = []
    for i in range(0, n, step):
        r = ranges[i]
        if not math.isfinite(r) or r < range_min or r > range_max or r <= 0.0:
            continue
        a = angle_min + i * angle_inc
        lx, ly = r * math.cos(a), r * math.sin(a)
        out.append(round(tx + c * lx - s * ly, 3))
        out.append(round(ty + s * lx + c * ly, 3))
    return out


def path_to_points(xy: Iterable[Tuple[float, float]], max_points: int = 300) -> List[float]:
    pts = list(xy)
    step = max(1, math.ceil(len(pts) / max_points)) if max_points > 0 else 1
    picked = pts[::step]
    if pts and picked[-1] != pts[-1]:
        picked.append(pts[-1])  # 終点は必ず残す
    out: List[float] = []
    for x, y in picked:
        out.append(round(x, 3))
        out.append(round(y, 3))
    return out


def clamp_drive(v, w, max_v: float, max_w: float) -> Tuple[float, float]:
    """ブラウザから来た速度指令を検証してクランプする。数値でなければ 0。"""
    def num(x):
        try:
            f = float(x)
        except (TypeError, ValueError):
            return 0.0
        return f if math.isfinite(f) else 0.0
    v, w = num(v), num(w)
    return (max(-max_v, min(max_v, v)), max(-max_w, min(max_w, w)))


def valid_map_name(name) -> bool:
    return (isinstance(name, str) and 0 < len(name) <= 64
            and all(ch.isalnum() and ch.isascii() or ch in '-_' for ch in name))
