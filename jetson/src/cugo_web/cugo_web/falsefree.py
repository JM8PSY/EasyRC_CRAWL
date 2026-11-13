"""カメラの床（SeenFloor）の「誤って空き」率を調べる（論文の実験 4-1）。ROS には依存しない。

カメラが床と見たセル（/cliff/floor・/theta/floor）は、探索の判断用の地図で「未知 → 空き」にだけ使う。
そのセルのうち、正解の地図（ground truth, GT）で実は壁・障害物だったものの割合（false-free rate）を出す。

1. 実行の終わりに save_run() が JSONL の隣に <同じ名前>.seen.npz を書く（node.dump_explore_run）
   - cells_xy (N, 2) セル中心の map 座標、hits (N,) 見た回数、src (N,) 見たカメラ（seen.SRC_* の和）、
     obs_dist (N,) 見たときのセンサーからの最短距離 [m]（不明は nan）、cell・min_hits（SeenFloor の設定）
   - grid (H, W) int8 最後の slam_toolbox の /map（行 = y、-1 未知、0..100）、
     grid_res、grid_origin [x, y, yaw]
   - traj (M, 3) 記録の sample の t, x, y、meta（JSON 文字列）
   min_hits 未満のセルも入れる（評価のときに min_hits を変えて試せるように）。
2. eval_false_free.py（scripts）が load_run() と正解の地図（load_map_yaml）で classify() する。

座標: 実行の map 座標 p_run を正解の地図の座標へ p_gt = R(yaw) p_run + (x, y) で移す（Transform）。
出発点と向きを同じにすれば Transform() = 恒等。少しのずれは align() で探す。
"""
import json
import math
import os
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

try:
    from scipy import ndimage as _ndimage
except ImportError:  # pragma: no cover - scipy の無い PC 用
    _ndimage = None

UNKNOWN, FREE, OCCUPIED = -1, 0, 100
SRC_CLIFF, SRC_THETA = 1, 2  # seen.py と同じ
SRC_NAMES = {SRC_CLIFF: 'realsense', SRC_THETA: 'theta'}
FALSE_FREE, TRUE_FREE, UNVERIFIED = 'false_free', 'true_free', 'unknown'

# 実行の /map（slam_toolbox）を三値にするしきい値（frontier.find_frontier_goal の既定と同じ）
SLAM_FREE_THRESH = 25
SLAM_OCC_THRESH = 65
DEFAULT_TOL = 0.10           # 壁からこの距離以内のカメラ床は「誤り」に数える [m]
DEFAULT_ROBOT_RADIUS = 0.30  # frontier.find_frontier_goal の robot_radius
DEFAULT_BANDS = (0.0, 1.0, 2.0, 3.0, 5.0)


# ---------------------------------------------------------------- 地図

@dataclass
class GridMap:
    """三値の地図。data は (H, W)、行 0 = 一番下（ROS の OccupancyGrid と同じ）、-1/0/100。
    origin = (x, y, yaw): セル (0, 0) の角の座標と地図の回転（map_server の origin）。"""
    data: np.ndarray
    resolution: float
    origin: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    @property
    def shape(self):
        return self.data.shape

    def world_to_cell(self, xy: np.ndarray):
        """(N, 2) の座標 → (行, 列)（整数、地図の外もそのまま）。"""
        xy = np.asarray(xy, float).reshape(-1, 2)
        ox, oy, yaw = self.origin
        dx, dy = xy[:, 0] - ox, xy[:, 1] - oy
        c, s = math.cos(yaw), math.sin(yaw)
        lx, ly = c * dx + s * dy, -s * dx + c * dy  # 回転を戻す
        return (np.floor(ly / self.resolution).astype(np.int64),
                np.floor(lx / self.resolution).astype(np.int64))

    def cell_centers(self, mask: np.ndarray) -> np.ndarray:
        """mask が真のセルの中心の座標 (N, 2)。"""
        r, col = np.nonzero(mask)
        lx, ly = (col + 0.5) * self.resolution, (r + 0.5) * self.resolution
        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        return np.column_stack([ox + c * lx - s * ly, oy + s * lx + c * ly])

    def lookup(self, xy: np.ndarray, layer: Optional[np.ndarray] = None, outside=UNKNOWN):
        """各点のセルの値（layer を渡せばその配列の値）。地図の外は outside。"""
        layer = self.data if layer is None else layer
        r, c = self.world_to_cell(xy)
        h, w = layer.shape
        ok = (r >= 0) & (r < h) & (c >= 0) & (c < w)
        out = np.full(len(r), outside, dtype=np.result_type(layer.dtype, np.asarray(outside)))
        out[ok] = layer[r[ok], c[ok]]
        return out


def trinary(grid, free_thresh=SLAM_FREE_THRESH, occ_thresh=SLAM_OCC_THRESH) -> np.ndarray:
    """OccupancyGrid の値（-1, 0..100）を -1/0/100 にする。中間の値は未知。"""
    g = np.asarray(grid, dtype=np.int16)
    out = np.full(g.shape, UNKNOWN, np.int16)
    out[(g >= 0) & (g <= free_thresh)] = FREE
    out[g >= occ_thresh] = OCCUPIED
    return out


def read_pgm(path: str) -> np.ndarray:
    """PGM（P5 バイナリ・P2 テキスト）を (H, W) の配列で読む。画像の上の行が 0 行目。
    maxval が 255 でなければ 0..255 に直す。"""
    with open(path, 'rb') as f:
        raw = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:  # 魔法の数・幅・高さ・maxval（# から行末は注釈）
        while pos < len(raw) and raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b'#':
            while pos < len(raw) and raw[pos:pos + 1] not in (b'\n', b'\r'):
                pos += 1
            continue
        start = pos
        while pos < len(raw) and not raw[pos:pos + 1].isspace() and raw[pos:pos + 1] != b'#':
            pos += 1
        if start == pos:
            raise ValueError('%s: PGM のヘッダが短い' % path)
        tokens.append(raw[start:pos].decode('ascii'))
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic == 'P5':
        pos += 1  # ヘッダの後の空白 1 文字
        dt = np.dtype('>u2') if maxval > 255 else np.uint8
        img = np.frombuffer(raw, dtype=dt, count=w * h, offset=pos).reshape(h, w)
    elif magic == 'P2':
        body = b' '.join(line.split(b'#')[0] for line in raw[pos:].splitlines())
        img = np.array(body.split()[:w * h], dtype=np.int64)
        if img.size != w * h:
            raise ValueError('%s: 画素が足りない' % path)
        img = img.reshape(h, w)
    else:
        raise ValueError('%s: P5/P2 の PGM ではない (%s)' % (path, magic))
    img = img.astype(np.float64)
    if maxval != 255:
        img = img * 255.0 / maxval
    return np.round(img).astype(np.uint8)


def _parse_yaml(text: str) -> dict:
    """map_server の YAML（key: value の 1 段だけ）。PyYAML が無いとき用。"""
    out = {}
    for line in text.splitlines():
        line = line.split('#')[0].strip()
        if ':' not in line:
            continue
        k, v = line.split(':', 1)
        v = v.strip().strip('"\'')
        if v.startswith('['):
            out[k.strip()] = [float(x) for x in v.strip('[]').split(',') if x.strip()]
            continue
        try:
            out[k.strip()] = int(v)
        except ValueError:
            try:
                out[k.strip()] = float(v)
            except ValueError:
                out[k.strip()] = v
    return out


def pgm_to_trinary(img: np.ndarray, negate=0, occupied_thresh=0.65, free_thresh=0.196):
    """map_server の trinary の読み方。p = (255 - 値) / 255（negate なら 値 / 255）、
    p > occupied_thresh → 占有、p < free_thresh → 空き、それ以外 → 未知。画像の上下を返して行 0 = 下。"""
    img = np.asarray(img, dtype=np.float64)
    p = img / 255.0 if negate else (255.0 - img) / 255.0
    out = np.full(img.shape, UNKNOWN, np.int16)
    out[p > occupied_thresh] = OCCUPIED
    out[p < free_thresh] = FREE
    return out[::-1].copy()


def load_map_yaml(path: str) -> GridMap:
    """map_server 形式（YAML + PGM）を読む。negate・occupied_thresh・free_thresh・origin（yaw 含む）を使う。"""
    with open(path, encoding='utf-8') as f:
        text = f.read()
    try:
        import yaml
        meta = yaml.safe_load(text)
    except ImportError:  # pragma: no cover
        meta = _parse_yaml(text)
    image = str(meta['image'])
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(path)), image)
    mode = str(meta.get('mode', 'trinary'))
    if mode != 'trinary':
        raise ValueError('mode=%s は未対応（trinary だけ）' % mode)
    data = pgm_to_trinary(read_pgm(image), int(meta.get('negate', 0)),
                          float(meta.get('occupied_thresh', 0.65)),
                          float(meta.get('free_thresh', 0.196)))
    o = list(meta.get('origin', [0.0, 0.0, 0.0])) + [0.0, 0.0, 0.0]
    return GridMap(data, float(meta['resolution']), (float(o[0]), float(o[1]), float(o[2])))


# ---------------------------------------------------------------- 距離

def distance_to(mask: np.ndarray, res: float, cap: float = 1.0) -> np.ndarray:
    """各セルから mask が真の一番近いセルまでの距離 [m]（セル中心どうし）。cap より遠い・無いときは cap。"""
    mask = np.asarray(mask, bool)
    if not mask.any():
        return np.full(mask.shape, float(cap))
    if _ndimage is not None:
        return np.minimum(_ndimage.distance_transform_edt(~mask) * res, cap)
    return _distance_brute(mask, res, cap)


def _distance_brute(mask: np.ndarray, res: float, cap: float) -> np.ndarray:
    """scipy が無いとき: cap 以内のずらしを全部試す（cap が数十 cm なら十分速い）。"""
    h, w = mask.shape
    n = int(math.ceil(cap / res))
    out = np.full(mask.shape, float(cap))
    for dy in range(-n, n + 1):
        for dx in range(-n, n + 1):
            d = math.hypot(dx, dy) * res
            if d >= cap:
                continue
            sh = np.zeros_like(mask)
            sh[max(0, dy):h + min(0, dy), max(0, dx):w + min(0, dx)] = \
                mask[max(0, -dy):h + min(0, -dy), max(0, -dx):w + min(0, -dx)]
            out[sh] = np.minimum(out[sh], d)
    return out


# ---------------------------------------------------------------- 座標の合わせ方

@dataclass
class Transform:
    """実行の map 座標 → 正解の地図の座標: p_gt = R(yaw) p + (x, y)。"""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def apply(self, xy: np.ndarray) -> np.ndarray:
        xy = np.asarray(xy, float).reshape(-1, 2)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.column_stack([c * xy[:, 0] - s * xy[:, 1] + self.x,
                                s * xy[:, 0] + c * xy[:, 1] + self.y])


def _score(dist: np.ndarray, dmax: float) -> float:
    return float(np.mean(np.clip(1.0 - dist / dmax, 0.0, 1.0))) if len(dist) else 0.0


def align(run: GridMap, gt: GridMap, max_xy: float = 0.5, max_yaw_deg: float = 10.0,
          step_xy: Optional[float] = None, step_yaw_deg: float = 1.0, dmax: float = 0.2,
          max_points: int = 4000, seed: int = 0) -> Tuple[Transform, dict]:
    """実行の /map の占有セルが正解の地図の占有に一番重なる (dx, dy, dyaw) を総当たりで探す。

    点数 = 平均 max(0, 1 - d / dmax)（d = 正解の占有までの距離）。回転は実行の map 座標の原点
    （= 出発点）まわり。返り値: (Transform, {'score', 'score_identity', 'inliers', 'points'})。
    inliers = d <= dmax / 2 の割合。"""
    step_xy = float(step_xy or gt.resolution)
    pts = run.cell_centers(run.data == OCCUPIED)
    info = {'points': int(len(pts))}
    if len(pts) == 0 or not (gt.data == OCCUPIED).any():
        info.update(score=0.0, score_identity=0.0, inliers=0.0)
        return Transform(), info
    if len(pts) > max_points:
        pts = pts[np.random.default_rng(seed).choice(len(pts), max_points, replace=False)]
    dist = distance_to(gt.data == OCCUPIED, gt.resolution, cap=dmax)
    nx = int(round(max_xy / step_xy))
    offs = np.arange(-nx, nx + 1) * step_xy
    ox, oy = np.meshgrid(offs, offs, indexing='ij')
    ox, oy = ox.ravel(), oy.ravel()
    ny = int(round(max_yaw_deg / step_yaw_deg))
    yaws = np.radians(np.arange(-ny, ny + 1) * step_yaw_deg)
    best = (-1.0, 0.0, 0.0, 0.0)
    for yaw in yaws:
        p = Transform(0.0, 0.0, float(yaw)).apply(pts)
        allp = (p[None, :, :] + np.stack([ox, oy], axis=1)[:, None, :]).reshape(-1, 2)
        d = gt.lookup(allp, dist, outside=dmax).reshape(len(ox), len(p))
        sc = np.clip(1.0 - d / dmax, 0.0, 1.0).mean(axis=1)
        # 同点なら動きの小さい方
        k = int(np.lexsort((np.hypot(ox, oy), -sc))[0])
        cand = (float(sc[k]), float(ox[k]), float(oy[k]), float(yaw))
        if cand[0] > best[0] + 1e-12 or (abs(cand[0] - best[0]) <= 1e-12 and
                                         math.hypot(cand[1], cand[2]) + abs(cand[3]) <
                                         math.hypot(best[1], best[2]) + abs(best[3])):
            best = cand
    tf = Transform(best[1], best[2], best[3])
    d_best = gt.lookup(tf.apply(pts), dist, outside=dmax)
    d_id = gt.lookup(pts, dist, outside=dmax)
    info.update(score=round(best[0], 4), score_identity=round(_score(d_id, dmax), 4),
                inliers=round(float(np.mean(d_best <= dmax / 2)), 4))
    return tf, info


# ---------------------------------------------------------------- 分類

def classify(cells_xy: np.ndarray, gt: GridMap, tf: Optional[Transform] = None,
             tol: float = DEFAULT_TOL, robot_radius: float = DEFAULT_ROBOT_RADIUS):
    """カメラの床セル（実行の map 座標の中心）を正解の地図で分ける。

    - false_free: 正解で占有のセルから tol 以内（tol=0 なら中心のセル自体が占有）
    - true_free:  それ以外で、中心のセルが正解で空き
    - unknown:    それ以外（正解で未知・地図の外）→ 確かめられない
    tol は SLAM の壁の厚み（1〜2 セル）と位置合わせの誤差のぶん。壁の際の床も誤りに数える側（保守的）。
    返り値: (labels (N,) の文字列, occ_dist (N,) 正解の占有までの距離 [m], nontrav (N,) bool)
    nontrav = true_free のうち、正解の占有から robot_radius 以内（床ではあるがロボットの中心は入れない）。
    """
    xy = (tf or Transform()).apply(cells_xy)
    occ = gt.data == OCCUPIED
    cap = max(tol, robot_radius) + gt.resolution
    occ_dist = gt.lookup(xy, distance_to(occ, gt.resolution, cap=cap), outside=np.inf)
    val = gt.lookup(xy)
    labels = np.full(len(xy), UNVERIFIED, dtype=object)
    is_false = occ_dist <= tol + 1e-9
    labels[(val == FREE) & ~is_false] = TRUE_FREE
    labels[is_false] = FALSE_FREE
    nontrav = (labels == TRUE_FREE) & (occ_dist <= robot_radius + 1e-9)
    return labels, occ_dist, nontrav


def _counts(labels, nontrav) -> dict:
    nf = int(np.sum(labels == FALSE_FREE))
    nt = int(np.sum(labels == TRUE_FREE))
    out = {'cells': int(len(labels)), FALSE_FREE: nf, TRUE_FREE: nt,
           UNVERIFIED: int(np.sum(labels == UNVERIFIED)),
           'non_traversable': int(np.sum(nontrav)),
           'false_free_rate': round(nf / (nf + nt), 4) if nf + nt else None}
    return out


def source_groups(src: np.ndarray) -> Dict[str, np.ndarray]:
    """出どころ別の選び方。realsense / theta は「そのカメラが見た」（両方も含む）、*_only・both も。"""
    src = np.asarray(src, np.uint8)
    rs, th = (src & SRC_CLIFF) > 0, (src & SRC_THETA) > 0
    return {'realsense': rs, 'theta': th, 'realsense_only': rs & ~th,
            'theta_only': th & ~rs, 'both': rs & th, 'unknown_source': ~rs & ~th}


def report(run: dict, gt: GridMap, tf: Optional[Transform] = None, tol: float = DEFAULT_TOL,
           robot_radius: float = DEFAULT_ROBOT_RADIUS, min_hits: Optional[int] = None,
           bands: Sequence[float] = DEFAULT_BANDS, tol_sweep=(0.0, 0.05, 0.10, 0.20)) -> dict:
    """1 回の実行（load_run の dict）を正解の地図で数える。"""
    mh = int(run['min_hits'] if min_hits is None else min_hits)
    keep = run['hits'] >= mh
    xy, src, dist = run['cells_xy'][keep], run['src'][keep], run['obs_dist'][keep]
    labels, occ_dist, nontrav = classify(xy, gt, tf, tol, robot_radius)
    out = {'min_hits': mh, 'tol': tol, 'robot_radius': robot_radius,
           'total': _counts(labels, nontrav), 'by_source': {}, 'by_distance': [],
           'tol_sweep': []}
    for name, m in source_groups(src).items():
        if m.any():
            out['by_source'][name] = _counts(labels[m], nontrav[m])
    if np.isfinite(dist).any():
        edges = list(bands) + [np.inf]
        for lo, hi in zip(edges[:-1], edges[1:]):
            m = (dist >= lo) & (dist < hi)
            c = _counts(labels[m], nontrav[m])
            c.update(lo=lo, hi=None if math.isinf(hi) else hi)
            out['by_distance'].append(c)
    for t in tol_sweep:
        lab, _, nt = classify(xy, gt, tf, t, robot_radius)
        c = _counts(lab, nt)
        c['tol'] = t
        out['tol_sweep'].append(c)
    out['_labels'] = labels  # 図・CSV 用（JSON にはしない）
    out['_xy'] = xy
    out['_occ_dist'] = occ_dist
    out['_src'] = src
    out['_dist'] = dist
    return out


def self_check(run: dict, **kw) -> dict:
    """正解の地図が無いとき: 実行自身の最後の /map（LiDAR）を正解の代わりにする。
    false_free = カメラが床と見たが、後で LiDAR が占有と見たセル（の tol 以内）。座標合わせは要らない。"""
    return report(run, run_grid(run), Transform(), **kw)


def run_grid(run: dict) -> GridMap:
    o = [float(v) for v in run['grid_origin']]
    return GridMap(trinary(run['grid']), float(run['grid_res']), (o[0], o[1], o[2]))


# ---------------------------------------------------------------- 実行の記録（npz）

def npz_path(jsonl_path: str) -> str:
    base = jsonl_path[:-6] if jsonl_path.endswith('.jsonl') else jsonl_path
    return base + '.seen.npz'


def save_run(path: str, snapshot: Optional[dict], grid, grid_res: float, grid_origin,
             traj=None, meta: Optional[dict] = None) -> str:
    """実行の終わりの記録を npz に書く（形は上の説明）。snapshot = SeenFloor.snapshot()（None なら 0 個）。
    いったん一時ファイルに書いてから名前を変える（途中で落ちても壊れたファイルを残さない）。"""
    snap = snapshot or {}
    xy = np.asarray(snap.get('xy', np.zeros((0, 2))), np.float64).reshape(-1, 2)
    n = len(xy)
    g = np.asarray(grid if grid is not None else np.zeros((0, 0)), dtype=np.int8)
    o = list(grid_origin if grid_origin is not None else (0.0, 0.0)) + [0.0, 0.0, 0.0]
    tmp = path + '.tmp.npz'
    np.savez_compressed(
        tmp, cells_xy=xy,
        hits=np.asarray(snap.get('hits', np.zeros(n)), np.int32),
        src=np.asarray(snap.get('src', np.zeros(n)), np.uint8),
        obs_dist=np.asarray(snap.get('dist', np.full(n, np.nan)), np.float32),
        cell=np.float64(snap.get('cell', 0.05)), min_hits=np.int32(snap.get('min_hits', 3)),
        grid=g, grid_res=np.float64(grid_res or 0.0),
        grid_origin=np.asarray(o[:3], np.float64),
        traj=np.asarray(traj if traj is not None and len(traj) else np.zeros((0, 3)),
                        np.float64).reshape(-1, 3),
        meta=np.array(json.dumps(meta or {}, ensure_ascii=False)))
    os.replace(tmp, path)
    return path


def load_run(path: str) -> dict:
    with np.load(path, allow_pickle=False) as z:
        out = {k: z[k] for k in z.files}
    for k in ('cell', 'grid_res'):
        out[k] = float(out[k])
    out['min_hits'] = int(out['min_hits'])
    out['meta'] = json.loads(str(out['meta']))
    return out


def traj_from_records(records) -> np.ndarray:
    """ExploreLog.records の sample から (t, x, y)。"""
    rows = [(r['t'], r['x'], r['y']) for r in records
            if r.get('ev') == 'sample' and r.get('x') is not None]
    return np.asarray(rows, np.float64).reshape(-1, 3)
