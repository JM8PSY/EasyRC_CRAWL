"""自動探索の 1 回分を JSONL に記録する（論文用に探索の方法どうしを数字で比べるため）。ROS には依存しない。

1 行 = 1 つの出来事（JSON）。1 行目は実行の情報（meta）。1 行ごとに flush するので、途中で落ちても
そこまでは残る（summary が無いファイルは explog_report.py が残りの行から集計し直す）。

出来事（"ev"）:
- meta:      開始時刻、設定（use_camera_floor・境界の選び方の重みなど。node / explorer が渡す dict）
- sample:    定期（既定 1Hz）。t, 姿勢 x/y/yaw, 走った距離 path, 既知面積 known・空き面積 free [m²]
             （slam_toolbox の /map だけで数える。カメラの床は含めない）、カメラが床と見たセル数 seen
- goal:      選んだ目的地（座標・経路距離・点数・候補数・「SLAM では未知だがカメラでは空き」か）
- result:    目的地の結果（succeeded / explored / aborted / rejected / timeout / skipped / canceled）と所要時間
- blacklist: 同じ所が何度も選ばれてブラックリストに入れた
- no_goal:   目的地が見つからなかった（なぜ無いかの内訳 diag 付き）
- wait / wait_end: 地図や姿勢（TF）が無くて待っている / 待ち終わった
- stop:      終わった理由（reason）。下の STOP_REASONS
- summary:   まとめ（summarize() の結果）

t は開始からの秒（単調時計）。
"""
import inspect
import json
import math
import os
import threading
import time
from typing import Iterable, List, Optional, Sequence

import numpy as np
from scipy import ndimage

from cugo_web import frontier

# 終わった理由
STOP_REASONS = {
    'completed':    '境界が無くなった（地図が閉じた／残りは小さすぎる境界だけ）',
    'no_frontiers': '境界は残っているが、行ける目的地が無い（ブラックリスト・行った場所・届かない・出発点が無い）',
    'user_stop':    '停止ボタン・探索オフ',
    'manual_drive': '手動操縦',
    'manual_goal':  '手動の目的地指定',
    'error':        '探索スレッドの例外',
    'shutdown':     'ノードの終了',
}
# 「失敗」に数える結果（stopped は止めただけなので数えない）
FAIL_RESULTS = ('aborted', 'rejected', 'timeout', 'skipped', 'canceled')
MAX_JUMP = 1.0      # 1 回でこれより大きく動いたら位置の飛び（自己位置の修正）とみなす [m]
MAX_SPEED = 0.5     # ただし間隔が長いときは MAX_SPEED * 間隔 まで認める [m/s]
BACKTRACK_DEG = 120.0


# ---------------------------------------------------------------- 純粋な計算

def map_areas(grid, resolution: float, free_thresh: int = 25):
    """占有格子（-1 未知、0..100）から (既知面積, 空き面積) [m²]。既知 = 空き + 占有 = 0 以上のセル。"""
    g = np.asarray(grid)
    cell = float(resolution) ** 2
    known = int(np.count_nonzero(g >= 0))
    free = int(np.count_nonzero((g >= 0) & (g <= free_thresh)))
    return known * cell, free * cell


def cell_value(grid, resolution, origin, x, y) -> Optional[int]:
    """(x, y) を含むセルの値。地図の外なら None。"""
    g = np.asarray(grid)
    r, c = frontier.world_to_cell(x, y, resolution, origin)
    if 0 <= r < g.shape[0] and 0 <= c < g.shape[1]:
        return int(g[r, c])
    return None


def path_increment(prev, cur, dt: float, max_jump: float = MAX_JUMP,
                   max_speed: float = MAX_SPEED):
    """前の姿勢 prev から cur までの走行距離 [m] と、飛びとして捨てたか。prev が None なら 0。

    1 回の移動が max(max_jump, max_speed * dt) より大きいときは自己位置の飛び（ループ閉じ込みなど）
    とみなして 0 にする。"""
    if prev is None:
        return 0.0, False
    d = math.hypot(cur[0] - prev[0], cur[1] - prev[1])
    if d > max(max_jump, max_speed * max(dt, 0.0)):
        return 0.0, True
    return d, False


def path_length(poses: Iterable, max_jump: float = MAX_JUMP, max_speed: float = MAX_SPEED):
    """(t, x, y) の列から走行距離 [m] と飛びの回数。"""
    total, jumps, prev = 0.0, 0, None
    for t, x, y in poses:
        d, jumped = path_increment(None if prev is None else prev[1:], (x, y),
                                   0.0 if prev is None else t - prev[0], max_jump, max_speed)
        total += d
        jumps += jumped
        prev = (t, x, y)
    return total, jumps


def angle_diff(a: float, b: float) -> float:
    """2 つの向きの差の大きさ [rad]（0..pi）。"""
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def count_backtracks(goals: Sequence, deg: float = BACKTRACK_DEG) -> int:
    """引き返しの回数。goals は (ロボット x, y, 目的地 x, y) の列（選んだ順）。
    続く 2 つの目的地で「ロボットから目的地への向き」が deg 度より大きく変わったら 1 回。
    ロボットと目的地が同じ位置（向きが決まらない）のものは飛ばす。"""
    lim = math.radians(deg)
    n, prev = 0, None
    for rx, ry, gx, gy in goals:
        if math.hypot(gx - rx, gy - ry) < 1e-6:
            continue
        a = math.atan2(gy - ry, gx - rx)
        if prev is not None and angle_diff(a, prev) > lim:
            n += 1
        prev = a
    return n


def time_to_fraction(samples: Sequence[dict], frac: float, key: str = 't',
                     final: Optional[float] = None) -> Optional[float]:
    """既知面積が最後の値の frac 倍に初めて届いたときの key（'t' なら時間、'path' なら距離）。"""
    if not samples:
        return None
    final = samples[-1]['known'] if final is None else final
    if final <= 0:
        return None
    for s in samples:
        if s['known'] >= frac * final - 1e-9:
            return s[key]
    return None


def summarize(records: Sequence[dict]) -> dict:
    """出来事の列（meta から）からまとめを作る。summary 行が無い（途中で落ちた）ファイルにも使う。"""
    samples = [r for r in records if r.get('ev') == 'sample']
    goals = [r for r in records if r.get('ev') == 'goal']
    results = [r for r in records if r.get('ev') == 'result']
    stops = [r for r in records if r.get('ev') == 'stop']
    times = [r['t'] for r in records if isinstance(r.get('t'), (int, float))]
    path, jumps = path_length((s['t'], s['x'], s['y']) for s in samples)
    known0 = samples[0]['known'] if samples else 0.0
    known1 = samples[-1]['known'] if samples else 0.0
    nres = {}
    for r in results:
        nres[r['result']] = nres.get(r['result'], 0) + 1
    out = {
        'time': round(max(times), 2) if times else 0.0,
        'path': round(path, 3),
        'jumps': jumps,
        'known_start': round(known0, 3),
        'known_final': round(known1, 3),
        'free_final': round(samples[-1]['free'], 3) if samples else 0.0,
        'area_per_m': round((known1 - known0) / path, 3) if path > 0 else None,
        'goals': len(goals),
        'failures': sum(nres.get(k, 0) for k in FAIL_RESULTS),
        'results': nres,
        'camera_only_goals': sum(1 for g in goals if g.get('camera_only')),
        'backtracks': count_backtracks([tuple(g['robot']) + (g['x'], g['y'])
                                        for g in goals if g.get('robot')]),
        'blacklisted': sum(1 for r in results if r.get('blacklisted'))
                       + sum(1 for r in records if r.get('ev') == 'blacklist'),
        'stop_reason': stops[-1].get('reason') if stops else None,
        # 目的地が無くて終わったときの内訳（diagnose_no_goal の cause）
        'end_cause': (stops[-1].get('diag') or {}).get('cause') if stops else None,
        'complete': bool(stops) and stops[-1].get('reason') == 'completed',
    }
    for pct in (50, 90):
        t = time_to_fraction(samples, pct / 100.0)
        d = time_to_fraction(samples, pct / 100.0, key='path')
        out['t%d' % pct] = None if t is None else round(t, 2)
        out['d%d' % pct] = None if d is None else round(d, 3)
    return out


def frontier_defaults() -> dict:
    """find_frontier_goal の既定の引数（explorer は既定値で呼ぶので、これが実際に使った値）。"""
    sig = inspect.signature(frontier.find_frontier_goal)
    return {k: v.default for k, v in sig.parameters.items()
            if v.default is not inspect.Parameter.empty and k not in ('blacklist', 'visited',
                                                                       'prefer')}


def diagnose_no_goal(grid, resolution, origin, robot_xy, blacklist=(), visited=(),
                     candidates: Optional[int] = None) -> dict:
    """find_frontier_goal が None を返した理由の内訳（探索が途中で「完了」になる原因を調べるため）。

    cause:
      none        境界のセルが 1 つも無い（地図が閉じた）
      small       境界はあるが、どれも小さすぎる／小さな囲まれた未知にしか接していない
      no_start    ロボットの周り 1 m に安全な空きセルが無い（障害物に近すぎる・地図とずれた）
      unreachable 境界はあるが、安全な空きを通って届かない
      too_close   届く境界はあるが、どれも近すぎる（min_goal_distance 未満）
      visited     届く境界はあるが、どれも行った場所の近く（行っても見えなかった未知: ガラス・LiDAR の死角）
      blacklisted 届く境界はあるが、どれもブラックリストの近く（Nav2 の失敗・時間切れ・同じ所の選び直し）
      filtered    届く境界はあるが、ブラックリストと行った場所の両方で全部消えた
      ok          目的地が見つかる（地図が更新された）
    'none' と 'small' だけが本当の完了。
    candidates: 呼び出し側で数えた count_frontiers（同じ blacklist・visited）があれば渡す（計算を省く）。
    """
    p = frontier_defaults()
    g = np.asarray(grid)
    res = float(resolution)
    safe = frontier.safe_free_mask(g, res, p['robot_radius'], p['free_thresh'], p['occ_thresh'])
    unknown = g < 0
    front = safe & ndimage.binary_dilation(unknown, structure=frontier._EIGHT)
    labels, n = ndimage.label(front, structure=frontier._EIGHT)
    sizes = np.bincount(labels.ravel(), minlength=n + 1)
    big = sizes * res >= p['min_frontier_size']
    big[0] = False
    valid = big.copy()
    if valid.any() and p['min_unknown_area'] > 0:
        valid &= frontier._adjacent_unknown_area(labels, n, unknown, res) >= p['min_unknown_area']
    start = frontier._start_cell(safe, res, (float(origin[0]), float(origin[1])),
                                 (float(robot_xy[0]), float(robot_xy[1])))
    reachable = 0
    if start is not None and valid.any():
        dist = frontier._path_distances(safe, start[0], start[1], res)
        ok = valid[labels] & np.isfinite(dist)
        reachable = int(len(np.unique(labels[ok])))
    kw = {k: p[k] for k in ('robot_radius', 'min_frontier_size', 'blacklist_radius',
                            'min_goal_distance', 'free_thresh', 'occ_thresh', 'visited_radius',
                            'min_unknown_area')}
    blacklist, visited = list(blacklist), list(visited)
    unfiltered = cands = only_bl = only_vis = 0
    if reachable:  # 届く境界が無ければ候補も無い（重い経路計算を省く）
        unfiltered = frontier.count_frontiers(g, res, origin, robot_xy, (), visited=(), **kw)
        cands = (int(candidates) if candidates is not None else
                 frontier.count_frontiers(g, res, origin, robot_xy, blacklist, visited=visited,
                                          **kw))
        if unfiltered and not cands:  # どちらで消えたか
            only_bl = frontier.count_frontiers(g, res, origin, robot_xy, blacklist, visited=(),
                                               **kw)
            only_vis = frontier.count_frontiers(g, res, origin, robot_xy, (), visited=visited,
                                                **kw)
    if n == 0:
        cause = 'none'
    elif not valid.any():
        cause = 'small'
    elif start is None:
        cause = 'no_start'
    elif reachable == 0:
        cause = 'unreachable'
    elif unfiltered == 0:
        cause = 'too_close'
    elif cands == 0:
        cause = ('visited' if only_vis == 0 and only_bl > 0 else
                 'blacklisted' if only_bl == 0 and only_vis > 0 else 'filtered')
    else:
        cause = 'ok'
    return {'cause': cause, 'frontier_cells': int(front.sum()), 'clusters': int(n),
            'clusters_big': int(big.sum()), 'clusters_valid': int(valid.sum()),
            'start_ok': start is not None, 'clusters_reachable': reachable,
            'candidates_unfiltered': int(unfiltered), 'candidates': int(cands),
            'blacklist': len(blacklist), 'visited': len(visited)}


def stop_reason_for(diag: Optional[dict]) -> str:
    """目的地が無くて終わったときの reason。"""
    if diag is not None and diag.get('cause') in ('none', 'small'):
        return 'completed'
    return 'no_frontiers'


# ---------------------------------------------------------------- 読み書き

def read_jsonl(path: str) -> List[dict]:
    """JSONL を読む。最後の行が書きかけ（落ちたとき）でも、それまでの行を返す。"""
    out = []
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except ValueError:
                continue
    return out


def _clean(v):
    """JSON にできる形にする（numpy の数・nan/inf・タプル）。"""
    if isinstance(v, dict):
        return {str(k): _clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_clean(x) for x in v]
    if isinstance(v, np.generic):
        v = v.item()
    if isinstance(v, float):
        return round(v, 4) if math.isfinite(v) else None
    if v is None or isinstance(v, (bool, int, str)):
        return v
    return str(v)


class ExploreLog:
    """1 回の自動探索を 1 つの JSONL ファイルに書く。複数のスレッド（探索・ROS タイマー）から呼べる。"""

    def __init__(self, log_dir: str = '~/cugo_logs', config: Optional[dict] = None,
                 clock=time.monotonic, wall=time.time):
        self._clock = clock
        self._t0 = clock()
        self._lock = threading.Lock()
        self.records: List[dict] = []
        self.closed = False
        self.summary: Optional[dict] = None
        self._prev = None      # (t, x, y)
        self._path = 0.0       # 飛びを除いた走行距離
        start = wall()
        d = os.path.expanduser(log_dir)
        os.makedirs(d, exist_ok=True)
        base = os.path.join(d, time.strftime('explore_%Y%m%d_%H%M%S', time.localtime(start)))
        path, k = base + '.jsonl', 1
        while os.path.exists(path):  # 同じ秒に 2 回始めたとき
            k += 1
            path = '%s_%d.jsonl' % (base, k)
        self.path = path
        self._f = open(path, 'w', encoding='utf-8')
        self._write({'ev': 'meta', 'start': time.strftime('%Y-%m-%dT%H:%M:%S%z',
                                                          time.localtime(start)),
                     'start_unix': round(start, 3), 'config': dict(config or {})})

    def t(self) -> float:
        return round(self._clock() - self._t0, 3)

    def _write(self, rec: dict) -> None:
        with self._lock:
            if self.closed:
                return
            rec = _clean(rec)
            self.records.append(rec)
            self._f.write(json.dumps(rec, ensure_ascii=False, separators=(',', ':')) + '\n')
            self._f.flush()

    def event(self, ev: str, **fields) -> None:
        rec = {'ev': ev, 't': self.t()}
        rec.update(fields)
        self._write(rec)

    def sample(self, x: float, y: float, yaw: float, known: float, free: float,
               seen: int = 0, **extra) -> None:
        """定期の記録。走行距離はここで前の姿勢から足す（飛びは捨てる）。"""
        t = self.t()
        d, jumped = path_increment(None if self._prev is None else self._prev[1:], (x, y),
                                   0.0 if self._prev is None else t - self._prev[0])
        self._path += d
        self._prev = (t, x, y)
        rec = {'ev': 'sample', 't': t, 'x': x, 'y': y, 'yaw': yaw, 'path': round(self._path, 3),
               'known': known, 'free': free, 'seen': int(seen)}
        if jumped:
            rec['jump'] = True
        rec.update(extra)
        self._write(rec)

    def close(self, reason: str, detail: str = '', **fields) -> Optional[dict]:
        """stop と summary を書いて閉じる。2 回目以降は何もしない。"""
        if self.closed:
            return self.summary
        self.event('stop', reason=reason, detail=detail, **fields)
        with self._lock:
            records = list(self.records)
        summary = summarize(records)
        self._write(dict({'ev': 'summary', 't': self.t()}, **summary))
        with self._lock:
            self.closed = True
            self.summary = summary
            try:
                self._f.close()
            except OSError:
                pass
        return summary
