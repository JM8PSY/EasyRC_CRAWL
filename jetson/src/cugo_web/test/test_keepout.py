import numpy as np

from cugo_web import keepout

SQUARE = [(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0)]


def test_validate_points():
    assert keepout.validate_points([[0, 0], [1, 0], [1, 1]]) == [(0, 0), (1, 0), (1, 1)]
    for bad in (None, [], [[0, 0], [1, 1]], [[0, 0], [1, 1], [2, 2]],  # 一直線
                [[0, 0], [1, 0], ['x', 1]], [[0, 0], [1, 0], [float('nan'), 1]],
                [[0, 0], [1, 0], [5000, 1]], 'abc', [[0, 0, 0], [1, 0], [1, 1]]):
        assert keepout.validate_points(bad) is None


def test_points_in_polygon():
    xs = np.array([1.5, 0.5, 2.5, 1.5])
    ys = np.array([1.5, 1.5, 1.5, 2.5])
    assert keepout.points_in_polygon(xs, ys, SQUARE).tolist() == [True, False, False, False]


def test_rasterize_matches_map_grid():
    zones = [{'id': 'z1', 'pts': SQUARE}]
    m = keepout.rasterize(zones, 60, 60, 0.05, (0.0, 0.0))
    # 1m 四方 = 20x20 セル
    assert m.sum() == 400
    r, c = int(1.5 / 0.05), int(1.5 / 0.05)
    assert m[r, c] and not m[int(0.5 / 0.05), c]


def test_rasterize_outside_map_is_ignored():
    zones = [{'id': 'z1', 'pts': [(10, 10), (11, 10), (11, 11)]}]
    assert keepout.rasterize(zones, 20, 20, 0.05, (0.0, 0.0)).sum() == 0


def test_mask_grid():
    data, w, h, (ox, oy) = keepout.mask_grid([{'id': 'z1', 'pts': SQUARE}], 0.05, margin=0.5)
    assert (ox, oy) == (0.5, 0.5) and w == 40 and h == 40
    assert data.dtype == np.int8 and set(np.unique(data)) == {0, 100}
    assert int((data == 100).sum()) == 400
    data, w, h, _ = keepout.mask_grid([])
    assert (w, h) == (1, 1) and data.tolist() == [0]


def test_save_load_roundtrip(tmp_path):
    p = str(tmp_path / 'k' / 'keepout.yaml')
    zones = [{'id': 'z1', 'pts': SQUARE}, {'id': 'z2', 'pts': [(0, 0), (1, 0), (0, 1)]}]
    keepout.save(zones, p)
    loaded = keepout.load(p)
    assert [z['id'] for z in loaded] == ['z1', 'z2']
    assert loaded[0]['pts'] == SQUARE
    assert keepout.load(str(tmp_path / 'missing.yaml')) == []
