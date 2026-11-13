"""「反射なし」の境界（noreturn.py）: LiDAR の反射が無い方向に面した境界の割合（実験 2-1 / 2-2）。"""
import math
import time
import types

import numpy as np
import pytest

from cugo_web import explog, noreturn as nr
from cugo_web.noreturn import HIT, NORETURN, PASSED, UNOBSERVED

N = 720
INC = 2 * math.pi / N
AMIN = -math.pi


def angles(n=N, amin=AMIN, inc=INC):
    return amin + np.arange(n) * inc


def full_scan(value=5.0):
    return np.full(N, value, float)


def set_sector(r, center_deg, half_deg, value, amin=AMIN, inc=INC):
    """laser_frame の角度 center±half [deg] の値を value に（±180 をまたいでもよい）。"""
    a = angles(len(r), amin, inc)
    d = np.abs((a - math.radians(center_deg) + math.pi) % (2 * math.pi) - math.pi)
    r[d <= math.radians(half_deg)] = value
    return r


def classify(cells, r, laser=(0.0, 0.0), yaw=0.0, amin=AMIN, inc=INC, rmin=0.12, rmax=10.0,
             **kw):
    return list(nr.classify_frontier_cells(cells, laser, yaw, r, amin, inc, rmin, rmax, **kw))


# ---- セル 1 つずつの分類 ----

def test_basic_labels():
    r = set_sector(full_scan(5.0), 90, 10, np.inf)   # 左は反射なし、他は 5 m で当たる
    cells = [(8.0, 0.0),    # 5 m で当たる（境界の手前）→ hit
             (0.0, 8.0),    # 反射なしの方向 → noreturn
             (0.0, 12.0),   # 届く距離（10 + 0.3 m）の外 → unobserved
             (3.0, 0.0),    # 境界を越えて 5 m で当たる → passed
             (4.8, 0.0),    # 5 m は境界 + 0.3 m 以内 → hit
             (0.05, 0.0)]   # range_min 未満 → unobserved
    assert classify(cells, r) == [HIT, NORETURN, UNOBSERVED, PASSED, HIT, UNOBSERVED]


def test_range_slack_keeps_range_limit_frontier_observable():
    r = full_scan(np.inf)
    assert classify([(10.2, 0.0)], r) == [NORETURN]
    assert classify([(10.4, 0.0)], r) == [UNOBSERVED]
    assert classify([(10.4, 0.0)], r, range_slack=0.5) == [NORETURN]


def test_special_values():
    cell = [(6.0, 0.0)]
    assert classify(cell, set_sector(full_scan(), 0, 5, 0.0)) == [NORETURN]       # 生ドライバの 0
    assert classify(cell, set_sector(full_scan(), 0, 5, 11.0)) == [NORETURN]      # > range_max
    assert classify(cell, set_sector(full_scan(), 0, 5, np.nan)) == [UNOBSERVED]  # 車体で遮る
    assert classify(cell, set_sector(full_scan(), 0, 5, -np.inf)) == [HIT]        # 近すぎる
    assert classify(cell, set_sector(full_scan(), 0, 5, 0.05)) == [HIT]           # < range_min


def test_noreturn_needs_all_beams_in_window():
    r = set_sector(full_scan(), 0, 5, np.inf)
    r[N // 2 + 2] = 3.0   # +1° の 1 本だけ 3 m で当たる（±1.5° の窓の中）
    assert classify([(6.0, 0.0)], r) == [HIT]
    r[N // 2 + 2] = np.nan   # データなしは数えない（残りは全部反射なし）
    assert classify([(6.0, 0.0)], r) == [NORETURN]
    r[N // 2 + 2] = 3.0
    assert classify([(6.0, 0.0)], r, tol=math.radians(0.6)) == [NORETURN]  # 窓を狭めると外れる


def test_wraparound_near_pi():
    r = set_sector(full_scan(5.0), 180, 4, np.inf)   # 真後ろ（配列の両端）が反射なし
    cells = [(-8.0, 0.01), (-8.0, -0.01), (-8.0, 0.0)]
    assert classify(cells, r) == [NORETURN] * 3
    r[0] = 3.0   # -180° の 1 本だけ当たる → 両隣の窓に入る
    assert classify(cells, r) == [HIT] * 3


def test_rotated_laser_mount():
    """実機の LiDAR は base に対して -167° 回して付いている。laser_frame の +90° が map の -77°。"""
    yaw = math.radians(-167)
    r = set_sector(full_scan(5.0), 90, 5, np.inf)
    a = math.radians(-77)
    b = math.radians(90)
    cells = [(1.0 + 8 * math.cos(a), 2.0 + 8 * math.sin(a)),
             (1.0 + 8 * math.cos(b), 2.0 + 8 * math.sin(b))]
    assert classify(cells, r, laser=(1.0, 2.0), yaw=yaw) == [NORETURN, HIT]
    assert classify(cells, r, laser=(1.0, 2.0), yaw=0.0) == [HIT, NORETURN]  # 向きを無視すると逆


def test_partial_coverage_and_negative_increment():
    n, amin, inc = 361, -math.pi / 2, math.radians(0.5)   # 前方 180° だけ
    r = np.full(n, np.inf)
    r[0] = 2.0   # -90° は当たる
    cells = [(-6.0, 0.0),                                          # 真後ろ: 範囲外
             (6 * math.cos(math.radians(-91)), 6 * math.sin(math.radians(-91))),  # -91°: 窓に beam 0
             (6.0, 0.0)]
    assert classify(cells, r, amin=amin, inc=inc) == [UNOBSERVED, HIT, NORETURN]
    # 時計回りのスキャン（angle_increment < 0）も同じ結果
    rev = r[::-1]
    assert classify(cells, rev, amin=math.pi / 2, inc=-inc) == [UNOBSERVED, HIT, NORETURN]


def test_empty_inputs():
    assert classify([], full_scan()) == []
    assert classify([(1.0, 0.0)], []) == [UNOBSERVED]


# ---- 地図の境界のかたまり ----

RES = 0.05
LASER = (0.5, 2.025)


def corridor_map():
    """幅 1 m・長さ 10.5 m の廊下（x 方向、先は未知）と、x=2〜3 m で上に出る脇道（先は未知）。"""
    g = -np.ones((80, 260), np.int16)
    g[30:51, 1:210] = 0
    g[29, 0:210] = 100
    g[51, 0:210] = 100
    g[29:52, 0] = 100
    g[51:71, 40:61] = 0     # 脇道（row 51 の壁に開口）
    g[51:71, 39] = 100
    g[51:71, 61] = 100
    return g


def corridor_scan(end=np.inf, rmax=10.0):
    """LASER から見た廊下: 上下の壁（0.525 m）・後ろの壁（0.45 m）。前方は end（既定: 反射なし）。"""
    out = []
    for a in angles():
        dx, dy = math.cos(a), math.sin(a)
        t = 0.525 / abs(dy) if abs(dy) > 1e-9 else np.inf
        t = min(t, end / dx if dx > 1e-9 else (0.45 / -dx if dx < -1e-9 else np.inf))
        out.append(t if t <= rmax else np.inf)
    return np.array(out)


def stats(scan, laser=LASER, **kw):
    a = nr.analyze(corridor_map(), RES, (0.0, 0.0), laser, 0.0, scan, AMIN, INC, 0.12,
                   kw.pop('rmax', 10.0), **kw)
    return a, nr.stats_of(a)


def test_corridor_end_beyond_range_is_noreturn():
    a, s = stats(corridor_scan())
    assert s['clusters']['n'] == 2
    # 廊下の先（9.98 m、前方は反射なし）→ noreturn、脇道（壁の陰）→ hit
    end = nr.label_near(a, 10.475, 2.025)
    side = nr.label_near(a, 2.5, 3.525)
    assert end['cluster'] == 'noreturn' and end['cell'] == 'noreturn'
    assert side['cluster'] == 'hit'
    assert s['clusters']['noreturn'] == 1 and s['clusters']['hit'] == 1
    assert s['clusters']['frac'] == 0.5 and s['clusters']['frac_obs'] == 0.5
    c = s['cells']
    assert c['n'] == c['noreturn'] + c['hit'] + c['passed'] + c['unobserved']
    assert c['noreturn'] == 9 and c['hit'] == 9   # 安全マスク（0.3 m）を引いた幅 9 セルずつ
    assert c['frac_obs'] == 0.5
    assert s['cells_all'] >= c['n']
    assert nr.label_near(a, 6.0, 2.0) is None   # 境界から 0.5 m 以上離れている


def test_corridor_end_blocked_by_obstacle_is_hit():
    _, s = stats(corridor_scan(end=4.0))   # 4 m 先に人（地図には無い）
    assert s['clusters']['hit'] == 2 and s['clusters']['frac_obs'] == 0.0


def test_corridor_end_passed_and_unobserved():
    _, s = stats(corridor_scan(end=11.5, rmax=12.0), rmax=12.0)   # 境界を越えて 11.5 m で当たる
    assert s['clusters']['passed'] == 1 and s['clusters']['noreturn'] == 0
    _, s = stats(corridor_scan(), rmax=8.0)            # 届かない → 見えていない
    assert s['clusters']['unobserved'] == 1
    assert s['clusters']['frac'] == 0.0 and s['clusters']['frac_obs'] == 0.0


def test_cluster_majority_rule():
    scan = corridor_scan()
    a = nr.analyze(corridor_map(), RES, (0.0, 0.0), LASER, 0.0, scan, AMIN, INC, 0.12, 10.0)
    k = nr.label_near(a, 10.475, 2.025)
    assert k['cluster'] == 'noreturn'
    # 前方 +1〜+2.5° に当たるもの → 9 セル中 6 セルが hit、3 セルが noreturn
    scan2 = scan.copy()
    scan2[N // 2 + 2:N // 2 + 6] = 4.0
    args = (corridor_map(), RES, (0.0, 0.0), LASER, 0.0, scan2, AMIN, INC, 0.12, 10.0)
    a2 = nr.analyze(*args)
    end = a2.xy[:, 0] > 10
    assert sorted(a2.cell_labels[end].tolist()) == [NORETURN] * 3 + [HIT] * 6
    assert nr.label_near(a2, 10.475, 2.025)['cluster'] == 'hit'        # 3/9 < 0.5
    a3 = nr.analyze(*args, cluster_ratio=0.3)
    assert nr.label_near(a3, 10.475, 2.025)['cluster'] == 'noreturn'   # 3/9 >= 0.3


def test_frontier_stats_empty_map():
    g = np.zeros((20, 20), np.int16)
    s = nr.frontier_stats(g, RES, (0.0, 0.0), (0.5, 0.5), 0.0, full_scan(), AMIN, INC, 0.12, 10.0)
    assert s['cells']['n'] == 0 and s['cells']['frac'] is None and s['clusters']['frac_obs'] is None


# ---- まとめ（explog.summarize）と比較表 ----

def test_summary_integration(tmp_path):
    from test_explog import load_report, make_records, write_run
    recs = make_records()
    samples = [r for r in recs if r['ev'] == 'sample']
    samples[0]['noreturn'] = {'cells': {'frac': 0.2, 'frac_obs': 0.4},
                              'clusters': {'frac': 0.5, 'frac_obs': 1.0}}
    samples[1]['noreturn'] = {'cells': {'frac': 0.0, 'frac_obs': None},
                              'clusters': {'frac': 0.0, 'frac_obs': None}}
    samples[2]['noreturn'] = {'cells': {'frac': 0.4, 'frac_obs': 0.6},
                              'clusters': {'frac': 0.25, 'frac_obs': 0.5}}
    goals = [r for r in recs if r['ev'] == 'goal']
    goals[0]['goal_noreturn'] = 'noreturn'
    goals[1]['goal_noreturn'] = 'hit'   # goals[2] はラベルなし（計算できなかった）
    s = explog.summarize(recs)
    assert s['nr_samples'] == 3
    assert s['nr_cells'] == pytest.approx(0.5) and s['nr_cells_all'] == pytest.approx(0.2)
    assert s['nr_clusters'] == pytest.approx(0.75)
    assert s['nr_clusters_all'] == pytest.approx(0.25)
    assert s['goals_noreturn'] == 1 and s['goals_labeled'] == 2
    assert s['goals_noreturn_frac'] == 0.5
    old = explog.summarize(make_records())   # noreturn の無い古い記録
    assert old['nr_samples'] == 0 and old['nr_cells'] is None and old['goals_noreturn_frac'] is None

    rep = load_report()
    write_run(tmp_path / 'a.jsonl', recs)
    assert rep.main([str(tmp_path / 'a.jsonl'), '--csv', str(tmp_path / 'o.csv')]) == 0
    head = (tmp_path / 'o.csv').read_text().splitlines()[0]
    assert 'nr_cells' in head and 'goals_noreturn_frac' in head and 'nr_cells_all' in head
    assert rep.table([('x', s)], 'run').splitlines()[0].count('nr') == 3


def test_sample_noreturn_round_trip(tmp_path):
    log = explog.ExploreLog(str(tmp_path), {})
    a, s = stats(corridor_scan())
    log.sample(0.5, 2.0, 0.0, 1.0, 1.0, noreturn=dict(s, scan_age=0.05))
    log.event('goal', x=10.4, y=2.0, goal_noreturn=nr.label_near(a, 10.4, 2.0)['cluster'])
    summ = log.close('completed')
    rec = [r for r in explog.read_jsonl(log.path) if r['ev'] == 'sample'][0]
    assert rec['noreturn']['clusters']['noreturn'] == 1
    assert summ['nr_clusters'] == 0.5 and summ['goals_noreturn_frac'] == 1.0


# ---- Explorer の目的地に付ける（test_explorer の偽ロボット・ガラス入りの廊下）----

def virtual_scan(x, y, rng=6.0):
    """test_explorer の世界で (x, y) から 0.5° ごとのスキャン。ガラス・届かない → inf。"""
    from test_explorer import GLASS, OCC, H, W, RES as R
    out = np.full(N, np.inf)
    for i, th in enumerate(angles()):
        for s_ in np.arange(0.05, rng, R / 2):
            c, r = int((x + s_ * math.cos(th)) / R), int((y + s_ * math.sin(th)) / R)
            if not (0 <= r < H and 0 <= c < W) or GLASS[r, c]:
                break
            if OCC[r, c]:
                out[i] = s_
                break
    return out


def run_explorer(tmp_path, monkeypatch, goal_fn):
    import cugo_web.explorer as ex
    from test_explorer import FakeNode
    monkeypatch.setattr(ex, 'CHECK_PERIOD', 0.0)
    node = FakeNode()
    node.get_logger = lambda: types.SimpleNamespace(info=lambda m: None, error=lambda m: None,
                                                    warn=lambda m: None)
    node.noreturn_goal = lambda x, y: goal_fn(node, x, y)
    e = ex.Explorer(node, lambda level, text: None)
    e.log_factory = lambda cfg: explog.ExploreLog(str(tmp_path), cfg)
    e.start()
    log = e.log
    end = time.monotonic() + 60
    while e.state == 'running' and time.monotonic() < end:
        time.sleep(0.05)
    assert e.state == 'done'
    return node, explog.read_jsonl(log.path)


def test_explorer_labels_goals(tmp_path, monkeypatch):
    def goal_fn(node, x, y):
        a = nr.analyze(node.known, 0.1, (0.0, 0.0), node.pose, 0.0, virtual_scan(*node.pose),
                       AMIN, INC, 0.12, 6.0)
        return nr.label_near(a, x, y)
    node, recs = run_explorer(tmp_path, monkeypatch, goal_fn)
    goals = [r for r in recs if r['ev'] == 'goal']
    assert goals and len(goals) == len(node.goals)
    assert all(g['goal_noreturn'] in nr.LABELS + (None,) for g in goals)
    assert any(g['goal_noreturn'] for g in goals)
    assert recs[-1]['goals_labeled'] >= 1


def test_explorer_survives_noreturn_failure(tmp_path, monkeypatch):
    def boom(node, x, y):
        raise RuntimeError('no scan')
    node, recs = run_explorer(tmp_path, monkeypatch, boom)
    goals = [r for r in recs if r['ev'] == 'goal']
    assert goals and all(g['goal_noreturn'] is None for g in goals)
    assert recs[-2]['ev'] == 'stop' and recs[-2]['reason'] in ('completed', 'no_frontiers')
