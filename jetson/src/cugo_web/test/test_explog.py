"""自動探索の記録（explog.py）と比較スクリプト（jetson/scripts/explog_report.py）。"""
import importlib.util
import json
import math
import os
import time
import types

import numpy as np
import pytest

from cugo_web import explog

REPORT = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'scripts', 'explog_report.py')


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


# ---- 純粋な計算 ----

def test_map_areas_counts_known_and_free():
    g = -np.ones((10, 10), np.int16)
    g[0:2, :] = 0      # 20 空き
    g[2, :] = 100      # 10 占有
    g[3, 0] = 50       # 1 既知（空きでも占有でもない）
    known, free = explog.map_areas(g, 0.05)
    assert known == pytest.approx(31 * 0.0025)
    assert free == pytest.approx(20 * 0.0025)
    assert explog.map_areas(-np.ones((3, 3)), 0.1) == (0.0, 0.0)


def test_cell_value():
    g = -np.ones((10, 20), np.int16)
    g[2, 5] = 0
    assert explog.cell_value(g, 0.1, (0.0, 0.0), 0.55, 0.25) == 0
    assert explog.cell_value(g, 0.1, (0.0, 0.0), 0.15, 0.15) == -1
    assert explog.cell_value(g, 0.1, (0.0, 0.0), -0.1, 0.1) is None
    assert explog.cell_value(g, 0.1, (0.0, 0.0), 0.1, 5.0) is None


def test_path_length_rejects_localization_jumps():
    poses = [(0, 0.0, 0.0), (1, 0.3, 0.0), (2, 0.6, 0.0),
             (3, 2.6, 0.0),            # 1 秒で 2 m: 飛び → 捨てる
             (4, 2.6, 0.4)]
    d, jumps = explog.path_length(poses)
    assert d == pytest.approx(1.0)
    assert jumps == 1


def test_path_increment_allows_long_gaps():
    # 姿勢が 10 秒取れなかった後の 2 m は、0.5 m/s * 10 s 以内なので走った距離として数える
    d, jumped = explog.path_increment((0.0, 0.0), (2.0, 0.0), 10.0)
    assert d == pytest.approx(2.0) and not jumped
    d, jumped = explog.path_increment((0.0, 0.0), (6.0, 0.0), 10.0)
    assert d == 0.0 and jumped
    assert explog.path_increment(None, (5.0, 5.0), 0.0) == (0.0, False)


def test_count_backtracks():
    # 東 → 東北東（変化小）→ 西（引き返し）→ 北西（90°以内）→ 南東（引き返し）
    goals = [(0, 0, 3, 0), (3, 0, 6, 1), (6, 1, 2, 1), (2, 1, 1, 2), (1, 2, 3, 0)]
    assert explog.count_backtracks(goals) == 2
    assert explog.count_backtracks(goals, deg=170.0) == 1
    assert explog.count_backtracks([]) == 0
    # 向きが決まらないもの（ロボットの真上）は飛ばす
    assert explog.count_backtracks([(0, 0, 1, 0), (1, 0, 1, 0), (1, 0, 2, 0)]) == 0


def test_angle_diff_wraps():
    assert explog.angle_diff(math.radians(170), math.radians(-170)) == pytest.approx(
        math.radians(20))


def test_time_to_fraction():
    s = [{'t': 0, 'path': 0, 'known': 2.0}, {'t': 10, 'path': 2, 'known': 5.0},
         {'t': 20, 'path': 5, 'known': 9.0}, {'t': 30, 'path': 7, 'known': 10.0}]
    assert explog.time_to_fraction(s, 0.5) == 10
    assert explog.time_to_fraction(s, 0.9) == 20
    assert explog.time_to_fraction(s, 0.9, key='path') == 5
    assert explog.time_to_fraction([], 0.5) is None


def make_records():
    recs = [{'ev': 'meta', 'config': {'use_camera_floor': True}}]
    for i in range(11):
        recs.append({'ev': 'sample', 't': float(i), 'x': 0.25 * i, 'y': 0.0, 'yaw': 0.0,
                     'path': 0.25 * i, 'known': 4.0 + i, 'free': 3.0 + i, 'seen': 0})
    recs += [
        {'ev': 'goal', 't': 0.5, 'x': 2.0, 'y': 0.0, 'robot': [0.0, 0.0], 'camera_only': True},
        {'ev': 'result', 't': 4.0, 'result': 'explored', 'blacklisted': False},
        {'ev': 'goal', 't': 4.1, 'x': -1.0, 'y': 0.0, 'robot': [1.0, 0.0]},
        {'ev': 'result', 't': 7.0, 'result': 'aborted', 'blacklisted': True},
        {'ev': 'blacklist', 't': 7.5, 'x': 0.0, 'y': 0.0, 'cause': 'repeat'},
        {'ev': 'goal', 't': 8.0, 'x': 3.0, 'y': 1.0, 'robot': [2.0, 0.0]},
        {'ev': 'result', 't': 9.5, 'result': 'succeeded', 'blacklisted': False},
        {'ev': 'stop', 't': 10.0, 'reason': 'completed'},
    ]
    return recs


def test_summarize():
    s = explog.summarize(make_records())
    assert s['time'] == 10.0
    assert s['path'] == pytest.approx(2.5)
    assert s['known_start'] == 4.0 and s['known_final'] == 14.0
    assert s['area_per_m'] == pytest.approx(10.0 / 2.5)
    assert s['goals'] == 3 and s['failures'] == 1
    assert s['results'] == {'explored': 1, 'aborted': 1, 'succeeded': 1}
    assert s['backtracks'] == 2   # 東 → 西 → 北東
    assert s['camera_only_goals'] == 1
    assert s['blacklisted'] == 2
    assert s['stop_reason'] == 'completed' and s['complete'] and s['end_cause'] is None
    assert s['t50'] == 3.0        # 14 * 0.5 = 7 に届くのは t=3
    assert s['t90'] == 9.0        # 12.6 → t=9
    assert s['d50'] == pytest.approx(0.75)


def test_summarize_without_stop_or_samples():
    s = explog.summarize([{'ev': 'meta'}])
    assert s['goals'] == 0 and s['path'] == 0.0 and s['stop_reason'] is None
    assert not s['complete'] and s['t50'] is None and s['area_per_m'] is None


# ---- JSONL ----

def test_jsonl_round_trip(tmp_path):
    clock = FakeClock()
    log = explog.ExploreLog(str(tmp_path / 'logs'), {'variant': 'cam', 'w': np.float32(1.5)},
                            clock=clock, wall=lambda: 1_700_000_000.0)
    assert os.path.basename(log.path).startswith('explore_') and log.path.endswith('.jsonl')
    for i in range(5):
        clock.now += 1.0
        log.sample(0.2 * i, 0.0, 0.0, known=1.0 + i, free=0.5 + i, seen=np.int64(i))
    clock.now += 0.5
    log.event('goal', x=1.0, y=2.0, robot=(0.0, 0.0), score=float('nan'))
    # 書いた行はすぐにファイルにある（落ちても残る）
    with open(log.path) as f:
        assert len(f.readlines()) == 7
    s = log.close('user_stop', 'test')
    assert log.close('error') is s  # 2 回目は何もしない
    log.sample(9, 9, 0, 1, 1)       # 閉じたあとの書き込みは捨てる
    recs = explog.read_jsonl(log.path)
    assert [r['ev'] for r in recs] == ['meta'] + ['sample'] * 5 + ['goal', 'stop', 'summary']
    assert recs[0]['config'] == {'variant': 'cam', 'w': 1.5}
    assert recs[0]['start_unix'] == 1_700_000_000.0
    assert recs[1]['t'] == 1.0 and recs[5]['path'] == pytest.approx(0.8)
    assert recs[6]['score'] is None and recs[6]['robot'] == [0.0, 0.0]
    assert recs[7]['reason'] == 'user_stop' and recs[7]['detail'] == 'test'
    assert recs[8]['path'] == pytest.approx(0.8) and recs[8]['goals'] == 1
    assert explog.summarize(recs[:-1])['known_final'] == 5.0


def test_sample_marks_jumps(tmp_path):
    clock = FakeClock()
    log = explog.ExploreLog(str(tmp_path), {}, clock=clock)
    log.sample(0, 0, 0, 1, 1)
    clock.now += 1
    log.sample(3, 0, 0, 1, 1)
    clock.now += 1
    log.sample(3.2, 0, 0, 1, 1)
    recs = [r for r in log.records if r['ev'] == 'sample']
    assert recs[1].get('jump') and recs[2]['path'] == pytest.approx(0.2)


def test_same_second_gets_new_file(tmp_path):
    a = explog.ExploreLog(str(tmp_path), {}, wall=lambda: 1_700_000_000.0)
    b = explog.ExploreLog(str(tmp_path), {}, wall=lambda: 1_700_000_000.0)
    assert a.path != b.path


def test_read_jsonl_skips_truncated_last_line(tmp_path):
    p = tmp_path / 'x.jsonl'
    p.write_text('{"ev":"meta"}\n{"ev":"sample","t":1,"x":0,"y":0,"known":1,"free":1}\n{"ev":"sa')
    assert [r['ev'] for r in explog.read_jsonl(str(p))] == ['meta', 'sample']


# ---- 目的地が無い理由 ----

def corridor(open_end=True):
    g = -np.ones((30, 80), np.int16)
    g[10, 2:60] = 100
    g[20, 2:60] = 100
    g[10:21, 2] = 100
    g[11:20, 3:40] = 0
    if not open_end:
        g[10:21, 40] = 100
    return g


def test_diagnose_no_goal():
    d = explog.diagnose_no_goal(corridor(), 0.1, (0.0, 0.0), (1.0, 1.5))
    assert d['cause'] == 'ok' and d['candidates'] >= 1
    # 境界の目的地をブラックリストに入れると「行ける目的地が無い」（完了ではない）
    goal = explog.frontier.find_frontier_goal(corridor(), 0.1, (0.0, 0.0), (1.0, 1.5))
    d = explog.diagnose_no_goal(corridor(), 0.1, (0.0, 0.0), (1.0, 1.5),
                                blacklist=[(goal.x, goal.y)])
    assert d['cause'] == 'blacklisted' and explog.stop_reason_for(d) == 'no_frontiers'
    # 行った場所の近く（見えなかった未知）
    d = explog.diagnose_no_goal(corridor(), 0.1, (0.0, 0.0), (1.0, 1.5),
                                visited=[(goal.x, goal.y)])
    assert d['cause'] == 'visited'
    d = explog.diagnose_no_goal(corridor(), 0.1, (0.0, 0.0), (1.0, 1.5),
                                blacklist=[(goal.x, goal.y)], visited=[(goal.x, goal.y)])
    assert d['cause'] == 'filtered'
    # 廊下の端を壁でふさぐ → 境界なし → 完了
    d = explog.diagnose_no_goal(corridor(open_end=False), 0.1, (0.0, 0.0), (1.0, 1.5))
    assert d['cause'] in ('none', 'small') and explog.stop_reason_for(d) == 'completed'
    # ロボットが地図の外（自己位置がずれた）→ 出発点なし
    d = explog.diagnose_no_goal(corridor(), 0.1, (0.0, 0.0), (7.5, 0.3))
    assert d['cause'] == 'no_start' and explog.stop_reason_for(d) == 'no_frontiers'
    assert explog.stop_reason_for(None) == 'no_frontiers'


# ---- Explorer に記録をつなぐ（test_explorer の偽ロボット）----

def test_explorer_writes_log(tmp_path, monkeypatch):
    import cugo_web.explorer as ex
    from test_explorer import FakeNode
    monkeypatch.setattr(ex, 'CHECK_PERIOD', 0.0)
    node = FakeNode()
    node.get_logger = lambda: types.SimpleNamespace(info=lambda m: None, error=lambda m: None,
                                                    warn=lambda m: None)
    e = ex.Explorer(node, lambda level, text: None)
    e.log_factory = lambda cfg: explog.ExploreLog(str(tmp_path), dict(cfg, variant='fake'))
    e.start()
    log = e.log
    end = time.monotonic() + 30
    while e.state == 'running' and time.monotonic() < end:
        if e.log is not None:
            p = node.robot_pose()
            known, free = explog.map_areas(node.known, 0.1)
            e.log.sample(p['x'], p['y'], p['yaw'], known, free)
        time.sleep(0.05)
    assert e.state == 'done' and e.log is None
    recs = explog.read_jsonl(log.path)
    evs = [r['ev'] for r in recs]
    assert evs[0] == 'meta' and evs[-2:] == ['stop', 'summary']
    assert recs[0]['config']['variant'] == 'fake'
    assert recs[0]['config']['frontier']['size_weight'] == 1.0
    assert evs.count('goal') == len(node.goals)
    assert evs.count('result') == len(node.goals)
    assert evs.count('no_goal') == ex.NO_FRONTIER_CONFIRM
    stop = recs[-2]
    assert stop['reason'] in ('completed', 'no_frontiers') and 'diag' in stop
    g = next(r for r in recs if r['ev'] == 'goal')
    assert {'x', 'y', 'dist', 'score', 'n_cand', 'camera_only', 'robot'} <= set(g)
    assert recs[-1]['goals'] == len(node.goals)


def test_explorer_stop_cause(tmp_path):
    import cugo_web.explorer as ex
    node = types.SimpleNamespace(
        get_logger=lambda: types.SimpleNamespace(info=lambda m: None, warn=lambda m: None),
        cancel_goal=lambda: None)
    e = ex.Explorer(node, lambda level, text: None)
    e.log_factory = lambda cfg: explog.ExploreLog(str(tmp_path), cfg)
    e._open_log()
    e.state = 'running'   # スレッドは動かさない
    log = e.log
    e.stop('手動で操縦した', cause='manual_drive')
    stop = [r for r in explog.read_jsonl(log.path) if r['ev'] == 'stop'][0]
    assert stop['reason'] == 'manual_drive' and stop['detail'] == '手動で操縦した'


# ---- 比較スクリプト ----

def load_report():
    spec = importlib.util.spec_from_file_location('explog_report', REPORT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def write_run(path, recs, summary=True):
    with open(path, 'w') as f:
        for r in recs:
            f.write(json.dumps(r) + '\n')
        if summary:
            f.write(json.dumps(dict({'ev': 'summary'}, **explog.summarize(recs))) + '\n')


def test_report_parses_and_prints(tmp_path, capsys):
    rep = load_report()
    a = make_records()
    a[0]['config']['variant'] = 'cam'
    b = make_records()
    b[0]['config'] = {'use_camera_floor': False}
    b = b[:-1]  # 途中で落ちた（stop も summary も無い）
    write_run(tmp_path / 'a.jsonl', a)
    write_run(tmp_path / 'b.jsonl', b, summary=False)
    run = rep.load_run(str(tmp_path / 'b.jsonl'))
    assert run['variant'] == 'lidar' and run['summary']['stop_reason'] is None
    assert run['summary']['goals'] == 3
    assert rep.variant_name({'use_camera_floor': True, 'use_theta': True,
                             'use_cliff_floor': True}) == 'cam+theta'
    csv_path = tmp_path / 'o.csv'
    assert rep.main([str(tmp_path / 'a.jsonl'), str(tmp_path / 'b.jsonl'), '--group',
                     '--csv', str(csv_path)]) == 0
    out = capsys.readouterr().out
    assert 'cam a.jsonl' in out and 'lidar b.jsonl' in out
    assert 'cam (n=1)' in out and '1/1 completed' in out
    assert rep.stop_label({'stop_reason': 'no_frontiers', 'end_cause': 'visited'}) == \
        'no_frontiers:visited'
    assert rep.stop_label({'stop_reason': None}) == '-'
    lines = csv_path.read_text().splitlines()
    assert lines[0].startswith('file,variant,start,time,path')
    assert len(lines) == 3


def test_report_plot_optional(tmp_path):
    rep = load_report()
    write_run(tmp_path / 'a.jsonl', make_records())
    out = tmp_path / 'p.png'
    ok = rep.main([str(tmp_path / 'a.jsonl'), '--plot', str(out)])
    assert ok == 0
    try:
        import matplotlib  # noqa: F401
    except ImportError:
        assert not out.exists()
    else:
        assert out.exists()
