"""カメラ（RealSense）が床として見たセルを覚え、自動探索用の地図の「未知」を「空き」にする。

slam_toolbox の /map は 2D LiDAR だけで作るので、LiDAR の高さより低い床の中は未知のまま残りやすい。
cliff_detector の /cliff/floor（床が見えたセル）を map 座標で数えておき、探索の判断用の地図にだけ重ねる。
slam_toolbox や Nav2 の地図は変えない。

安全: 既知のセル（0..100）は決して変えない。特に占有はそのまま（ガラスや障害物）。
変えるのは「未知」→「空き」だけ。走れるかどうかは Nav2 の costmap が決める。
"""
import math
import threading

import numpy as np

# セル番号（符号付き）を 64bit の 1 つの番号にまとめるためのずらし。
# 1 << 31 だと上位 32bit が符号ビットにかかってあふれる
_OFF = 1 << 30


def _pack(ix: np.ndarray, iy: np.ndarray) -> np.ndarray:
    return ((ix + _OFF) << 32) | (iy + _OFF)


def _unpack(keys: np.ndarray):
    return (keys >> 32) - _OFF, (keys & 0xFFFFFFFF) - _OFF


class SeenFloor:
    def __init__(self, cell: float = 0.05, min_hits: int = 3, max_cells: int = 200000):
        self.cell = float(cell)
        self.min_hits = int(min_hits)
        self.max_cells = int(max_cells)
        self._lock = threading.Lock()
        self._keys = np.zeros(0, np.int64)    # 並べ替え済みのセル番号
        self._hits = np.zeros(0, np.int32)

    def add(self, xy: np.ndarray) -> None:
        """map 座標の点 (N, 2) を足す。1 回の呼び出しで同じセルは 1 回と数える（1 フレーム = 1 回）。"""
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        xy = xy[np.isfinite(xy).all(axis=1)]
        if len(xy) == 0:
            return
        idx = np.floor(xy / self.cell).astype(np.int64)
        lim = _OFF - 1
        idx = idx[(np.abs(idx) < lim).all(axis=1)]  # 桁あふれする遠すぎる点は捨てる
        new = np.unique(_pack(idx[:, 0], idx[:, 1]))
        with self._lock:
            keys, hits = self._keys, self._hits
            pos = np.searchsorted(keys, new)
            found = np.zeros(len(new), bool)
            if len(keys):
                found = (pos < len(keys)) & (keys[np.minimum(pos, len(keys) - 1)] == new)
            hits = hits.copy()
            hits[pos[found]] += 1
            fresh = new[~found]
            room = max(0, self.max_cells - len(keys))
            fresh = fresh[:room]  # 満杯なら新しいセルは足さない（覚えたセルは残す）
            if len(fresh):
                keys = np.concatenate([keys, fresh])
                hits = np.concatenate([hits, np.ones(len(fresh), np.int32)])
                order = np.argsort(keys, kind='stable')
                keys, hits = keys[order], hits[order]
            self._keys, self._hits = keys, hits

    def clear(self) -> None:
        with self._lock:
            self._keys = np.zeros(0, np.int64)
            self._hits = np.zeros(0, np.int32)

    def __len__(self) -> int:
        with self._lock:
            return int((self._hits >= self.min_hits).sum())

    def overlay(self, grid: np.ndarray, res: float, origin) -> np.ndarray:
        """grid（行 = y, 列 = x、-1 未知、0..100）の写しを返す。
        min_hits 回以上見た床セルに重なる未知のセルだけ 0（空き）にする。"""
        out = np.array(grid, dtype=np.int16, copy=True)
        with self._lock:
            keys = self._keys[self._hits >= self.min_hits]
        if len(keys) == 0 or out.size == 0:
            return out
        ix, iy = _unpack(keys)
        # 見たセルが地図のセルより大きいときは、中を k x k 点に分けて全部の地図セルを覆う
        k = max(1, int(math.ceil(self.cell / res - 1e-9)))
        sub = (np.arange(k) + 0.5) / k
        xs = ((ix[:, None] + sub[None, :]) * self.cell).ravel()
        ys = ((iy[:, None] + sub[None, :]) * self.cell).ravel()
        if k > 1:
            xs = np.repeat(xs, k)                                # x を k 回ずつ
            ys = np.tile(ys.reshape(-1, k), (1, k)).ravel()      # y の k 個を k 回くり返す
        h, w = out.shape
        c = np.floor((xs - origin[0]) / res).astype(np.int64)
        r = np.floor((ys - origin[1]) / res).astype(np.int64)
        ok = (c >= 0) & (c < w) & (r >= 0) & (r < h)
        r, c = r[ok], c[ok]
        unk = out[r, c] < 0
        out[r[unk], c[unk]] = 0
        return out
