"""点群から下り階段・段差落ち（崖）を見つける（ROS 非依存）。

入力は base_footprint 座標（床が z = 0）の点 (N, 3)。xy を cell × cell の格子に分け、
- 崖セル: 床より drop_threshold 以上低い点が min_points 個以上あり、かつセルの z の中央値も
  -drop_threshold より低い（ノイズや数点の外れ値では反応しない）
- 床セル: 点が min_points 個以上あり、z の中央値の絶対値が floor_band 以下（崖セルは除く）
として、セル中心に高さ mark_z の点を 1 つずつ出す（崖 = marks、床 = clears）。
mark_z は Nav2 ObstacleLayer の min/max_obstacle_height の範囲に入る高さにする
（クリア用の点も高さの窓の外だと捨てられるため）。

camera_origin（base 座標でのカメラ位置）を渡すと、低い点は「カメラからその点への視線が
z = 0 を横切る位置」に移してから格子に入れる。段差の縁より先の下の段はカメラから見えず
（縁の陰になる）、実際に見えるのはずっと奥の低い面なので、観測位置のまま印を付けると
縁から大きく離れた所にしか印が付かない。視線が床の高さを横切る位置は「床が続いている
可能性のある最も遠い点」なので、縁の位置（の奥側の上限）になる。
"""
from typing import Optional, Sequence, Tuple

import numpy as np


def quaternion_to_matrix(q: Sequence[float]) -> np.ndarray:
    """四元数 (x, y, z, w) → 3×3 回転行列。"""
    x, y, z, w = (float(v) for v in q)
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def transform_points(xyz: np.ndarray, translation: Sequence[float],
                     quaternion: Sequence[float]) -> np.ndarray:
    """点 (N, 3) を回転（四元数 x, y, z, w）してから平行移動する。"""
    R = quaternion_to_matrix(quaternion)
    pts = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    return pts @ R.T + np.asarray(translation, dtype=np.float64).reshape(1, 3)


def project_to_floor(points: np.ndarray, origin: Sequence[float]) -> np.ndarray:
    """origin から各点への視線が z = 0 を横切る位置 (N, 3) を返す（z は元の値のまま）。

    点は z < 0、origin は z > 0 を前提とする（そうでない点は動かさない）。
    """
    o = np.asarray(origin, dtype=np.float64).reshape(3)
    out = points.copy()
    dz = o[2] - points[:, 2]
    ok = (o[2] > 0) & (points[:, 2] < 0) & (dz > 1e-9)
    t = np.zeros(len(points))
    t[ok] = o[2] / dz[ok]
    out[ok, 0] = o[0] + t[ok] * (points[ok, 0] - o[0])
    out[ok, 1] = o[1] + t[ok] * (points[ok, 1] - o[1])
    return out


def fit_floor_plane(pts: np.ndarray, *, x_range=(0.3, 1.2), y_abs_max: float = 0.6,
                    max_offset: float = 0.10, max_slope: float = 0.12, inlier: float = 0.015,
                    min_inliers: int = 200, iterations: int = 40) -> Tuple[float, float, float]:
    """近くの床に平面 z = a + b·x + c·y を当てる（RANSAC + 最小二乗）。

    床が真っ平らでない・車体がわずかに傾いている・カメラ角度の調整値が少しずれている、
    といったとき、z = 0 のままだと遠くの床が「低い」と判定されて誤検出になる
    （実機で 2〜3° の傾きで起きた）。その都度の床面からの高さで判定するために使う。
    当てはまらない（点が少ない・傾き max_slope 超・高さ max_offset 超）ときは (0, 0, 0)。
    """
    c = pts[(pts[:, 0] >= x_range[0]) & (pts[:, 0] <= x_range[1])
            & (np.abs(pts[:, 1]) <= y_abs_max) & (np.abs(pts[:, 2]) <= max_offset)]
    if len(c) < min_inliers:
        return 0.0, 0.0, 0.0
    A = np.column_stack([np.ones(len(c)), c[:, 0], c[:, 1]])
    rng = np.random.default_rng(0)
    best, best_n = None, 0
    for _ in range(iterations):
        idx = rng.choice(len(c), 3, replace=False)
        try:
            coef = np.linalg.solve(A[idx], c[idx, 2])
        except np.linalg.LinAlgError:
            continue
        if abs(coef[1]) > max_slope or abs(coef[2]) > max_slope:
            continue
        n = int((np.abs(A @ coef - c[:, 2]) < inlier).sum())
        if n > best_n:
            best, best_n = coef, n
    if best is None or best_n < min_inliers:
        return 0.0, 0.0, 0.0
    m = np.abs(A @ best - c[:, 2]) < inlier
    coef, *_ = np.linalg.lstsq(A[m], c[m, 2], rcond=None)
    a, b, cc = (float(v) for v in coef)
    if abs(a) > max_offset or abs(b) > max_slope or abs(cc) > max_slope:
        return 0.0, 0.0, 0.0
    return a, b, cc


def level_to_floor(pts: np.ndarray, origin: Optional[Sequence[float]] = None, **fit_kw):
    """床平面からの高さに直した点・カメラ位置と平面係数を返す。(pts, origin, (a, b, c))"""
    pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
    pts = pts[np.isfinite(pts).all(axis=1)]
    a, b, c = fit_floor_plane(pts, **fit_kw)
    out = pts.copy()
    out[:, 2] -= a + b * pts[:, 0] + c * pts[:, 1]
    o = None
    if origin is not None:
        ox, oy, oz = (float(v) for v in origin)
        o = (ox, oy, oz - (a + b * ox + c * oy))
    return out, o, (a, b, c)


def mirror_mask(pts: np.ndarray, low: np.ndarray, *, cell: float = 0.05, radius: float = 0.10,
                obstacle_z: float = 0.05, min_points: int = 3) -> np.ndarray:
    """光沢床の映り込み（鏡像）とみなす低い点を True で返す。

    つるつるの床は鏡になり、深度カメラは壁や物の鏡像を「床の下」にある物として測る。
    鏡像は実物と同じ xy に、高さだけ反転して現れる（実機: 1.2〜1.6 m 先の壁の真下に
    -0.1〜-0.3 m の点が大量に出た）。本物の下の段の上に壁は立っていないので、
    床より高い点（obstacle_z 超）が min_points 個以上あるセルから radius 以内にある低い点は捨てる。
    pts は観測位置（視線に沿って移す前）で渡すこと。
    """
    out = np.zeros(len(pts), dtype=bool)
    if not low.any():
        return out
    high = pts[:, 2] > obstacle_z
    if not high.any():
        return out
    ix = np.floor(pts[:, 0] / cell).astype(np.int64)
    iy = np.floor(pts[:, 1] / cell).astype(np.int64)
    span = 1 << 20
    hk, hc = np.unique((ix[high] << 21) + (iy[high] + span), return_counts=True)
    hk = hk[hc >= min_points]
    if len(hk) == 0:
        return out
    k = int(np.ceil(radius / cell - 1e-9))
    hx, hy = hk >> 21, (hk & ((1 << 21) - 1)) - span
    dil = np.unique(np.concatenate([((hx + dx) << 21) + (hy + dy + span)
                                    for dx in range(-k, k + 1) for dy in range(-k, k + 1)]))
    out[low] = np.isin((ix[low] << 21) + (iy[low] + span), dil)
    return out


def _empty() -> np.ndarray:
    return np.zeros((0, 3), dtype=np.float64)


def detect(points_xyz_base: np.ndarray, *, cell: float = 0.05, drop_threshold: float = 0.06,
           floor_band: float = 0.03, min_points: int = 3,
           x_range: Tuple[float, float] = (0.2, 2.0), y_abs_max: float = 1.5,
           mark_z: float = 0.1, camera_origin: Optional[Sequence[float]] = None,
           drop_per_meter: float = 0.0, min_neighbors: int = 0,
           mirror_radius: float = 0.0, max_low_range: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
    """base 座標の点 (N, 3) から (marks (M, 3), clears (K, 3)) を返す。

    camera_origin を渡すと低い点（z < -drop_threshold）を視線に沿って z = 0 の位置へ移す
    （モジュールの説明を参照）。None なら観測位置のまま使う。

    drop_per_meter: 「低い」とみなす閾値をカメラからの水平距離 1 m ごとにこれだけ深くする。
    D455 の深度誤差は距離とともに急に大きくなり、光沢のある床では遠くの床が数 cm〜10 cm
    低く写る点が出る（実機: 1.2 m より先で 4〜7% の点が -6 cm 未満）。階段は 15 cm 以上
    落ちるので、遠くほど閾値を深くしても検出できる。

    min_neighbors: 崖セルは周り 8 セルに崖セルがこの数以上あるものだけ残す。段差の縁は
    セルが線状に並ぶが、反射・ノイズの誤検出はぽつんと 1 セルだけ出ることが多い。

    mirror_radius > 0: 床より高い物の真下（この距離以内）にある低い点を鏡像として捨てる
    （mirror_mask 参照）。

    max_low_range > 0: カメラから水平にこれより遠い低い点は使わない。遠くの低い点は、
    視野外の物（天井の照明・壁の上の方）の鏡像であることが多く、実物が見えないので
    mirror_mask で消せない。下の段は縁までの距離の約 2 倍の所に見えるので、
    2.2 m なら縁が約 1.2 m 以内の段差まで検出できる。
    """
    pts = np.asarray(points_xyz_base, dtype=np.float64).reshape(-1, 3)
    pts = pts[np.isfinite(pts).all(axis=1)]
    if len(pts) == 0:
        return _empty(), _empty()

    # 判定用の高さ: 遠い点ほど持ち上げる（= 閾値を深くする）。床判定には元の z を使う
    if drop_per_meter > 0.0:
        o = camera_origin if camera_origin is not None else (0.0, 0.0, 0.0)
        dist = np.hypot(pts[:, 0] - o[0], pts[:, 1] - o[1])
        zj = pts[:, 2] + drop_per_meter * dist
    else:
        zj = pts[:, 2]
    low = zj < -drop_threshold
    if max_low_range > 0.0 and low.any():
        o = camera_origin if camera_origin is not None else (0.0, 0.0, 0.0)
        far = low & (np.hypot(pts[:, 0] - o[0], pts[:, 1] - o[1]) > max_low_range)
        pts, zj, low = pts[~far], zj[~far], low[~far]
    if mirror_radius > 0.0 and low.any():
        mirror = mirror_mask(pts, low, cell=cell, radius=mirror_radius)
        keep = ~mirror
        pts, zj, low = pts[keep], zj[keep], low[keep]
    if camera_origin is not None and low.any():
        pts = pts.copy()
        pts[low] = project_to_floor(pts[low], camera_origin)

    keep = ((pts[:, 0] >= x_range[0]) & (pts[:, 0] < x_range[1])
            & (np.abs(pts[:, 1]) <= y_abs_max))
    pts, low, zj = pts[keep], low[keep], zj[keep]
    if len(pts) == 0:
        return _empty(), _empty()

    # セル番号（x, y の整数格子）を 1 つの整数キーにまとめる
    ix = np.floor(pts[:, 0] / cell).astype(np.int64)
    iy = np.floor(pts[:, 1] / cell).astype(np.int64)
    ix0, iy0 = ix.min(), iy.min()
    ny = int(iy.max() - iy0) + 1
    key = (ix - ix0) * ny + (iy - iy0)

    # キー順・z 順に並べ、セルごとの中央値を取る
    order = np.lexsort((pts[:, 2], key))
    key_s, z_s = key[order], pts[order, 2]
    cells, start, counts = np.unique(key_s, return_index=True, return_counts=True)
    median = 0.5 * (z_s[start + (counts - 1) // 2] + z_s[start + counts // 2])
    order_j = np.lexsort((zj, key))
    zj_s = zj[order_j]
    median_j = 0.5 * (zj_s[start + (counts - 1) // 2] + zj_s[start + counts // 2])
    low_counts = np.bincount(np.searchsorted(cells, key[low]), minlength=len(cells))

    enough = counts >= min_points
    cliff = enough & (low_counts >= min_points) & (median_j < -drop_threshold)
    if min_neighbors > 0 and cliff.any():
        gx = (cells // ny).astype(np.int64)
        gy = (cells % ny).astype(np.int64)
        cg = np.zeros((int(gx.max()) + 3, ny + 2), dtype=np.int32)
        cg[gx[cliff] + 1, gy[cliff] + 1] = 1
        nb = sum(np.roll(np.roll(cg, dx, 0), dy, 1)
                 for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy)
        cliff = cliff & (nb[gx + 1, gy + 1] >= min_neighbors)
    floor = enough & (np.abs(median) <= floor_band) & ~cliff

    cx = (cells // ny + ix0 + 0.5) * cell
    cy = (cells % ny + iy0 + 0.5) * cell

    def out(mask):
        n = int(mask.sum())
        return np.column_stack([cx[mask], cy[mask], np.full(n, float(mark_z))])

    return out(cliff), out(floor)
