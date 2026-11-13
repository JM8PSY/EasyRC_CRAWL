"""SeenFloor（カメラが床として見たセルを、探索用の地図の未知に重ねる）。"""
import time

import numpy as np

from cugo_web.seen import SeenFloor

UNK, FREE, OCC = -1, 0, 100


def unknown(h=20, w=20):
    return np.full((h, w), UNK, dtype=np.int16)


def centre(r, c, res=0.05, origin=(0.0, 0.0)):
    """地図セル (r, c) の中心の map 座標 [x, y]。"""
    return [origin[0] + (c + 0.5) * res, origin[1] + (r + 0.5) * res]


def add_times(s, pts, n):
    for _ in range(n):
        s.add(np.array(pts, dtype=float))


def test_min_hits_threshold():
    s = SeenFloor(cell=0.05, min_hits=3)
    add_times(s, [centre(5, 7)], 2)
    assert len(s) == 0
    out = s.overlay(unknown(), 0.05, (0.0, 0.0))
    assert (out == UNK).all()
    s.add(np.array([centre(5, 7)]))
    assert len(s) == 1
    out = s.overlay(unknown(), 0.05, (0.0, 0.0))
    assert out[5, 7] == FREE
    assert (out == FREE).sum() == 1


def test_duplicates_in_one_add_count_once():
    s = SeenFloor(cell=0.05, min_hits=3)
    s.add(np.array([centre(5, 7)] * 5))
    assert len(s) == 0


def test_known_cells_never_change_and_returns_copy():
    g = unknown()
    g[3, 3] = FREE
    g[4, 4] = OCC
    g[6, 6] = 50
    s = SeenFloor(cell=0.05, min_hits=1)
    s.add(np.array([centre(3, 3), centre(4, 4), centre(6, 6), centre(8, 8)]))
    out = s.overlay(g, 0.05, (0.0, 0.0))
    assert out[3, 3] == FREE
    assert out[4, 4] == OCC
    assert out[6, 6] == 50
    assert out[8, 8] == FREE
    assert g[8, 8] == UNK  # 元の地図は変えない
    assert out is not g


def test_out_of_grid_points_ignored():
    s = SeenFloor(cell=0.05, min_hits=1)
    origin = (-0.5, -0.5)
    s.add(np.array([[-10.0, 0.0], [0.0, 10.0], [-0.6, -0.6], [0.6, 0.6], [0.0, 0.0]]))
    out = s.overlay(unknown(), 0.05, origin)
    assert (out == FREE).sum() == 1
    assert out[10, 10] == FREE


def test_negative_coordinates():
    s = SeenFloor(cell=0.05, min_hits=1)
    origin = (-1.0, -1.0)
    s.add(np.array([[-0.975, -0.425], [0.475, -0.025]]))
    out = s.overlay(unknown(40, 40), 0.05, origin)
    assert out[11, 0] == FREE and out[19, 29] == FREE
    assert (out == FREE).sum() == 2


def test_max_cells_cap_keeps_existing():
    s = SeenFloor(cell=0.05, min_hits=1, max_cells=3)
    s.add(np.array([centre(1, 1), centre(2, 2)]))
    s.add(np.array([centre(3, 3), centre(4, 4), centre(5, 5)]))
    assert len(s) <= 3
    # 既にあるセルは数え続けられる
    s2 = SeenFloor(cell=0.05, min_hits=2, max_cells=2)
    s2.add(np.array([centre(1, 1), centre(2, 2)]))
    s2.add(np.array([centre(9, 9)]))  # 満杯なので入らない
    s2.add(np.array([centre(1, 1), centre(2, 2), centre(9, 9)]))
    assert len(s2) == 2
    out = s2.overlay(unknown(), 0.05, (0.0, 0.0))
    assert out[1, 1] == FREE and out[2, 2] == FREE and out[9, 9] == UNK


def test_clear():
    s = SeenFloor(cell=0.05, min_hits=1)
    s.add(np.array([centre(1, 1)]))
    assert len(s) == 1
    s.clear()
    assert len(s) == 0
    assert (s.overlay(unknown(), 0.05, (0.0, 0.0)) == UNK).all()


def test_empty_add_is_ok():
    s = SeenFloor()
    s.add(np.zeros((0, 2)))
    assert len(s) == 0


def test_coarser_map_resolution():
    # 見たセル 0.05 m、地図 0.1 m: 0.05 のセル 4 つが同じ地図セルに入る
    s = SeenFloor(cell=0.05, min_hits=1)
    s.add(np.array([[0.325, 0.425], [0.375, 0.475]]))  # どちらも地図セル (4, 3)
    out = s.overlay(unknown(), 0.1, (0.0, 0.0))
    assert out[4, 3] == FREE
    assert (out == FREE).sum() == 1


def test_finer_map_resolution_covers_whole_seen_cell():
    # 見たセル 0.1 m、地図 0.05 m: 見たセル 1 つが地図セル 2x2 を覆う
    s = SeenFloor(cell=0.1, min_hits=1)
    s.add(np.array([[0.35, 0.45]]))  # 見たセル x 0.3..0.4, y 0.4..0.5
    out = s.overlay(unknown(), 0.05, (0.0, 0.0))
    assert (out == FREE).sum() == 4
    assert (out[8:10, 6:8] == FREE).all()


def test_overlay_is_fast():
    rng = np.random.default_rng(0)
    s = SeenFloor(cell=0.05, min_hits=1)
    pts = rng.uniform(0.0, 50.0, size=(50000, 2))
    s.add(pts)
    assert len(s) > 40000
    grid = np.full((1000, 1000), UNK, dtype=np.int16)
    t0 = time.perf_counter()
    out = s.overlay(grid, 0.05, (0.0, 0.0))
    dt = time.perf_counter() - t0
    assert dt < 0.05, dt
    assert (out == FREE).sum() == len(s)
