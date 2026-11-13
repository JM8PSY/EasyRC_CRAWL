"""falsefree（カメラ床の誤って空き率）と SeenFloor の記録用の情報。"""
import math
import os
import sys

import numpy as np
import pytest

from cugo_web import falsefree as ff
from cugo_web.seen import SRC_CLIFF, SRC_THETA, SeenFloor

UNK, FREE, OCC = ff.UNKNOWN, ff.FREE, ff.OCCUPIED


# ---------------------------------------------------------------- 地図ファイル

def write_p5(path, img, maxval=255, comment=True):
    h, w = img.shape
    head = b'P5\n' + (b'# made by test\n' if comment else b'') + b'%d %d\n%d\n' % (w, h, maxval)
    body = img.astype('>u2' if maxval > 255 else np.uint8).tobytes()
    with open(path, 'wb') as f:
        f.write(head + body)


def write_p2(path, img, maxval=255):
    h, w = img.shape
    lines = ['P2', '# comment', '%d %d' % (w, h), str(maxval)]
    lines += [' '.join(str(int(v)) for v in row) + '  # row' for row in img]
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def write_yaml(path, image, res=0.05, origin=(0.0, 0.0, 0.0), negate=0, occ=0.65, free=0.196):
    with open(path, 'w') as f:
        f.write('image: %s\nmode: trinary\nresolution: %s\norigin: [%s, %s, %s]\n'
                'negate: %d\noccupied_thresh: %s\nfree_thresh: %s\n'
                % (image, res, origin[0], origin[1], origin[2], negate, occ, free))


def sample_img():
    """上の行（画像の 0 行目）が黒 = 占有、真ん中が灰 = 未知、下が白 = 空き。"""
    img = np.full((3, 4), 254, np.uint8)
    img[0] = 0
    img[1] = 205
    return img


def test_read_pgm_p5_p2_and_16bit(tmp_path):
    img = sample_img()
    write_p5(tmp_path / 'a.pgm', img)
    write_p2(tmp_path / 'b.pgm', img)
    assert (ff.read_pgm(str(tmp_path / 'a.pgm')) == img).all()
    assert (ff.read_pgm(str(tmp_path / 'b.pgm')) == img).all()
    write_p5(tmp_path / 'c.pgm', img.astype(np.uint16) * 257, maxval=65535, comment=False)
    assert (ff.read_pgm(str(tmp_path / 'c.pgm')) == img).all()


def test_load_map_yaml_trinary_flip_and_origin(tmp_path):
    write_p5(tmp_path / 'm.pgm', sample_img())
    write_yaml(tmp_path / 'm.yaml', 'm.pgm', res=0.1, origin=(-1.0, 2.0, 0.0))
    m = ff.load_map_yaml(str(tmp_path / 'm.yaml'))
    assert m.resolution == 0.1 and m.origin == (-1.0, 2.0, 0.0)
    # 行 0 = 一番下（画像の一番下の行 = 白 = 空き）
    assert (m.data[0] == FREE).all() and (m.data[1] == UNK).all() and (m.data[2] == OCC).all()
    assert m.lookup(np.array([[-0.95, 2.05], [-0.95, 2.25], [5.0, 5.0]])).tolist() == \
        [FREE, OCC, UNK]


def test_load_map_yaml_negate_and_thresholds(tmp_path):
    write_p2(tmp_path / 'm.pgm', sample_img())
    write_yaml(tmp_path / 'm.yaml', 'm.pgm', negate=1)
    m = ff.load_map_yaml(str(tmp_path / 'm.yaml'))
    # negate: 黒 = 空き、白 = 占有、205/255 = 0.80 > 0.65 → 占有
    assert (m.data[2] == FREE).all() and (m.data[0] == OCC).all() and (m.data[1] == OCC).all()
    write_yaml(tmp_path / 'n.yaml', 'm.pgm', occ=0.9, free=0.1)
    n = ff.load_map_yaml(str(tmp_path / 'n.yaml'))
    assert (n.data[0] == FREE).all()  # 254: p = 0.004 < 0.1
    assert (n.data[2] == OCC).all()   # 0: p = 1.0 > 0.9
    assert (n.data[1] == UNK).all()   # 205: p = 0.196


def test_parse_yaml_fallback():
    d = ff._parse_yaml('image: "a.pgm"\nresolution: 0.05\norigin: [-1.5, 2, 0.1]  # c\n'
                       'negate: 0\n')
    assert d == {'image': 'a.pgm', 'resolution': 0.05, 'origin': [-1.5, 2.0, 0.1], 'negate': 0}


def test_gridmap_yaw_origin():
    data = np.full((10, 10), FREE, np.int16)
    data[0, 5] = OCC  # 局所 x = 0.275, y = 0.025
    m = ff.GridMap(data, 0.05, (1.0, 1.0, math.pi / 2))
    # 地図が 90 度回っている: 局所 (x, y) → 世界 (1 - y, 1 + x)
    assert m.lookup(np.array([[1.0 - 0.025, 1.0 + 0.275]]))[0] == OCC
    c = m.cell_centers(data == OCC)
    assert np.allclose(c, [[0.975, 1.275]])


# ---------------------------------------------------------------- 分類

def room(h=60, w=80, res=0.05):
    """壁で囲まれた部屋（外は未知）。"""
    g = np.full((h, w), UNK, np.int16)
    g[5:h - 5, 5:w - 5] = OCC
    g[7:h - 7, 7:w - 7] = FREE
    return ff.GridMap(g, res, (0.0, 0.0, 0.0))


def test_classify_basic_and_tolerance():
    gt = room()
    pts = np.array([[2.0, 1.5],     # 部屋の真ん中: 空き
                    [0.3, 1.5],     # 壁の上（列 6）: 占有
                    [0.40, 1.5],    # 壁（列 6 の終わり 0.35）から 0.05 の床
                    [0.10, 0.10],   # 部屋の外の未知（壁からも遠い）
                    [10.0, 10.0]])  # 地図の外
    lab, dist, nontrav = ff.classify(pts, gt, tol=0.10, robot_radius=0.3)
    assert lab.tolist() == [ff.TRUE_FREE, ff.FALSE_FREE, ff.FALSE_FREE, ff.UNVERIFIED,
                            ff.UNVERIFIED]
    lab0, _, _ = ff.classify(pts, gt, tol=0.0)
    assert lab0.tolist()[:3] == [ff.TRUE_FREE, ff.FALSE_FREE, ff.TRUE_FREE]
    # 壁の際の床は「通れない」側
    lab2, _, nontrav2 = ff.classify(np.array([[0.40, 1.5], [2.0, 1.5]]), gt, tol=0.0,
                                    robot_radius=0.3)
    assert nontrav2.tolist() == [True, False]


def test_classify_with_transform():
    gt = room()
    # 実行の座標は正解より x に +0.5 ずれている → tf で -0.5 戻す
    pts = np.array([[0.8, 1.5]])
    assert ff.classify(pts, gt, ff.Transform(), tol=0.0)[0][0] == ff.TRUE_FREE
    assert ff.classify(pts, gt, ff.Transform(-0.5, 0.0, 0.0), tol=0.0)[0][0] == ff.FALSE_FREE


def test_distance_brute_matches_scipy():
    rng = np.random.default_rng(1)
    mask = rng.random((30, 40)) > 0.97
    a = ff.distance_to(mask, 0.05, cap=0.3)
    b = ff._distance_brute(mask, 0.05, 0.3)
    assert np.allclose(a, b)
    assert (ff.distance_to(np.zeros((3, 3), bool), 0.05, cap=0.5) == 0.5).all()


def make_run(gt_points, cells_xy, hits=None, src=None, dist=None, grid=None, res=0.05,
             origin=(0.0, 0.0, 0.0), min_hits=3):
    n = len(cells_xy)
    return {'cells_xy': np.asarray(cells_xy, float).reshape(-1, 2),
            'hits': np.asarray(hits if hits is not None else np.full(n, 5), np.int32),
            'src': np.asarray(src if src is not None else np.full(n, SRC_CLIFF), np.uint8),
            'obs_dist': np.asarray(dist if dist is not None else np.full(n, np.nan), np.float32),
            'cell': 0.05, 'min_hits': min_hits, 'grid': grid, 'grid_res': res,
            'grid_origin': np.asarray(origin, float), 'traj': np.zeros((0, 3)), 'meta': {}}


def test_report_counts_sources_distance_and_min_hits():
    gt = room()
    cells = [[2.0, 1.5], [2.5, 1.5], [0.3, 1.5], [0.1, 0.1], [3.0, 1.0]]
    run = make_run(None, cells, hits=[5, 5, 5, 5, 1],
                   src=[SRC_CLIFF, SRC_THETA, SRC_THETA, SRC_CLIFF | SRC_THETA, SRC_CLIFF],
                   dist=[0.5, 2.5, 3.5, 0.2, 0.5])
    rep = ff.report(run, gt, tol=0.0)
    t = rep['total']
    assert (t['cells'], t['false_free'], t['true_free'], t['unknown']) == (4, 1, 2, 1)
    assert t['false_free_rate'] == pytest.approx(1 / 3, abs=1e-4)
    assert rep['by_source']['theta']['false_free'] == 1
    assert rep['by_source']['theta']['cells'] == 3
    assert rep['by_source']['realsense_only']['cells'] == 1
    assert rep['by_source']['both']['unknown'] == 1
    bands = {c['lo']: c for c in rep['by_distance']}
    assert bands[0.0]['cells'] == 2 and bands[3.0]['false_free'] == 1
    assert [c['tol'] for c in rep['tol_sweep']] == [0.0, 0.05, 0.10, 0.20]
    assert ff.report(run, gt, tol=0.0, min_hits=1)['total']['cells'] == 5
    # 距離が不明なら距離別は出さない
    assert ff.report(make_run(None, cells), gt)['by_distance'] == []


# ---------------------------------------------------------------- 座標合わせ

def l_room(res=0.05):
    """左右非対称な部屋（L 字の角と柱）。合わせがひと通りに決まるように。"""
    g = np.full((100, 120), UNK, np.int16)
    g[10:90, 10:110] = FREE
    g[10:90, 10] = OCC
    g[10:90, 109] = OCC
    g[10, 10:110] = OCC
    g[89, 10:110] = OCC
    g[50:89, 70:72] = OCC    # 仕切り
    g[25:30, 30:35] = OCC    # 柱
    return ff.GridMap(g, res, (-1.0, -1.0, 0.0))


def shifted(gt: ff.GridMap, tf: ff.Transform) -> ff.GridMap:
    """gt の占有を tf の逆で動かした「実行の地図」（p_run = tf^-1 p_gt）。"""
    occ = gt.cell_centers(gt.data == OCC)
    c, s = math.cos(tf.yaw), math.sin(tf.yaw)
    d = occ - [tf.x, tf.y]
    run_pts = np.column_stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]])
    g = np.full(gt.shape, UNK, np.int16)
    run = ff.GridMap(g, gt.resolution, gt.origin)
    r, col = run.world_to_cell(run_pts)
    ok = (r >= 0) & (r < g.shape[0]) & (col >= 0) & (col < g.shape[1])
    g[r[ok], col[ok]] = OCC
    return run


def test_align_recovers_offset_and_rotation():
    gt = l_room()
    true = ff.Transform(0.20, -0.15, math.radians(4.0))
    run = shifted(gt, true)
    tf, info = ff.align(run, gt, max_xy=0.3, max_yaw_deg=6.0)
    assert abs(tf.x - true.x) <= 0.05 + 1e-9 and abs(tf.y - true.y) <= 0.05 + 1e-9
    assert abs(math.degrees(tf.yaw) - 4.0) <= 1.0 + 1e-9
    assert info['score'] > info['score_identity']
    assert info['inliers'] > 0.9


def test_align_identity_when_already_aligned():
    gt = l_room()
    tf, info = ff.align(gt, gt, max_xy=0.2, max_yaw_deg=3.0)
    assert (tf.x, tf.y, tf.yaw) == (0.0, 0.0, 0.0)
    assert info['score'] == pytest.approx(1.0)


def test_align_empty_map():
    gt = l_room()
    empty = ff.GridMap(np.full((5, 5), UNK, np.int16), 0.05)
    tf, info = ff.align(empty, gt)
    assert (tf.x, tf.y, tf.yaw) == (0.0, 0.0, 0.0) and info['points'] == 0


# ---------------------------------------------------------------- 記録（npz）と自己チェック

def test_seenfloor_snapshot_tracks_source_and_distance():
    s = SeenFloor(cell=0.05, min_hits=2)
    s.add(np.array([[1.02, 0.02], [2.02, 0.02]]), SRC_CLIFF, (0.0, 0.0))
    s.add(np.array([[1.02, 0.02]]), SRC_THETA, (0.9, 0.0))
    s.add(np.array([[3.02, 0.02]]))  # 古い呼び方（src・位置なし）
    snap = s.snapshot()
    order = np.argsort(snap['xy'][:, 0])
    xy, hits = snap['xy'][order], snap['hits'][order]
    src, dist = snap['src'][order], snap['dist'][order]
    assert np.allclose(xy, [[1.025, 0.025], [2.025, 0.025], [3.025, 0.025]])
    assert hits.tolist() == [2, 1, 1]
    assert src.tolist() == [SRC_CLIFF | SRC_THETA, SRC_CLIFF, 0]
    assert dist[0] == pytest.approx(math.hypot(0.125, 0.025), abs=1e-5)  # 近い方
    assert dist[1] == pytest.approx(math.hypot(2.025, 0.025), abs=1e-5)
    assert np.isnan(dist[2])
    assert len(s) == 1 and snap['min_hits'] == 2
    s.clear()
    assert len(s.snapshot()['xy']) == 0


def test_npz_roundtrip(tmp_path):
    s = SeenFloor(cell=0.05, min_hits=3)
    s.add(np.array([[0.5, 0.5], [0.6, 0.5]]), SRC_THETA, (0.0, 0.0))
    grid = np.full((4, 6), -1, np.int8)
    grid[1, 2] = 100
    grid[2, 3] = 0
    traj = ff.traj_from_records([{'ev': 'meta'}, {'ev': 'sample', 't': 1.0, 'x': 0.1, 'y': 0.2},
                                 {'ev': 'sample', 't': 2.0, 'x': 0.3, 'y': 0.4}])
    path = ff.npz_path(str(tmp_path / 'explore_20261114_120000.jsonl'))
    assert path.endswith('explore_20261114_120000.seen.npz')
    ff.save_run(path, s.snapshot(), grid, 0.05, (-1.0, -2.0, 0.1), traj, {'log': 'x.jsonl'})
    assert not os.path.exists(path + '.tmp.npz')
    run = ff.load_run(path)
    assert run['cells_xy'].shape == (2, 2) and run['src'].tolist() == [SRC_THETA, SRC_THETA]
    assert run['hits'].tolist() == [1, 1] and run['min_hits'] == 3 and run['cell'] == 0.05
    assert (run['grid'] == grid).all() and run['grid_res'] == 0.05
    assert run['grid_origin'].tolist() == [-1.0, -2.0, 0.1]
    assert run['traj'].tolist() == [[1.0, 0.1, 0.2], [2.0, 0.3, 0.4]]
    assert run['meta'] == {'log': 'x.jsonl'}
    # カメラの床が無い（use_camera_floor:=false）・地図が無いときも書ける
    ff.save_run(str(tmp_path / 'e.seen.npz'), None, None, None, None)
    e = ff.load_run(str(tmp_path / 'e.seen.npz'))
    assert len(e['cells_xy']) == 0 and e['grid'].shape == (0, 0)


def test_self_check_uses_run_map():
    grid = np.full((40, 40), -1, np.int8)
    grid[5:35, 5:35] = 0
    grid[20, 5:35] = 100      # 後で LiDAR が見つけた壁（y = 1.0..1.05）
    grid[10, 10] = 50         # 中間の値は未知
    cells = [[0.7, 0.5],      # 空き
             [1.0, 1.02],     # 壁の上 → 誤り
             [0.525, 0.525],  # 中間の値のセル → 未知
             [1.9, 1.9]]      # 未知
    run = make_run(None, cells, grid=grid)
    rep = ff.self_check(run, tol=0.0)
    t = rep['total']
    assert (t['true_free'], t['false_free'], t['unknown']) == (1, 1, 2)


def test_eval_script_runs(tmp_path, capsys):
    gt = room()
    img = np.where(gt.data == OCC, 0, np.where(gt.data == FREE, 254, 205)).astype(np.uint8)
    write_p5(tmp_path / 'gt.pgm', img[::-1])
    write_yaml(tmp_path / 'gt.yaml', 'gt.pgm')
    grid = gt.data.astype(np.int8)
    snap = {'xy': np.array([[2.0, 1.5], [0.3, 1.5]]), 'hits': np.array([4, 4]),
            'src': np.array([1, 2]), 'dist': np.array([1.0, 2.5]), 'cell': 0.05, 'min_hits': 3}
    npz = str(tmp_path / 'explore_x.seen.npz')
    ff.save_run(npz, snap, grid, 0.05, (0.0, 0.0, 0.0))
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(here, '..', '..', '..', 'scripts'))
    try:
        import eval_false_free
    finally:
        sys.path.pop(0)
    out_json = str(tmp_path / 'o.json')
    out_csv = str(tmp_path / 'o.csv')
    assert eval_false_free.main(['--gt', str(tmp_path / 'gt.yaml'), '--tol', '0',
                                 '--json', out_json, '--csv', out_csv, npz]) == 0
    text = capsys.readouterr().out
    assert 'rate=50.00%' in text and 'transform: x=+0.000' in text
    assert eval_false_free.main(['--self-check', npz]) == 0
    assert eval_false_free.main(['--gt', str(tmp_path / 'gt.yaml'), '--tf', '0', '0', '0',
                                 npz]) == 0
    with open(out_csv) as f:
        assert len(f.readlines()) == 3
