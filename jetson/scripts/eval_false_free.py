#!/usr/bin/env python3
"""カメラの床の「誤って空き」率（false-free rate）を出す（論文の実験 4-1）。

実行ごとの記録 ~/cugo_logs/explore_*.seen.npz（探索の終わりに node が書く）を、正解の地図
（map_server 形式の YAML + PGM）と比べる。手順は docs/research/ground-truth-map.md。

使い方:
  python3 jetson/scripts/eval_false_free.py --gt ~/maps/gt.yaml ~/cugo_logs/explore_*.seen.npz
  python3 jetson/scripts/eval_false_free.py --gt gt.yaml --no-align run.seen.npz
  python3 jetson/scripts/eval_false_free.py --gt gt.yaml --tf 0.1 -0.05 2.0 run.seen.npz  # yaw は度
  python3 jetson/scripts/eval_false_free.py --self-check run.seen.npz   # 正解の地図が無いとき
  python3 jetson/scripts/eval_false_free.py --gt gt.yaml --csv out.csv --plot out.png run.seen.npz

分け方（各カメラ床セルの中心を正解の地図に移して）:
  false_free  正解の占有から --tol [m] 以内（既定 0.10: SLAM の壁の厚み 1〜2 セルと位置合わせの誤差）
  true_free   それ以外で、正解で空き（うち --robot-radius 以内に占有があるものを non_traversable と数える）
  unknown     正解で未知・地図の外（確かめられない）
  false-free rate = false_free / (false_free + true_free)
--self-check は正解の代わりに実行自身の最後の /map（LiDAR）を使う（後で LiDAR が壁と見たカメラ床）。
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

try:
    from cugo_web import falsefree as ff
except ImportError:  # colcon でビルドしていない PC: リポジトリの中から読む
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    '..', 'src', 'cugo_web'))
    from cugo_web import falsefree as ff


def fmt_counts(c: dict) -> str:
    rate = c['false_free_rate']
    return ('cells=%6d false=%5d true=%6d unknown=%5d nontrav=%5d rate=%s'
            % (c['cells'], c['false_free'], c['true_free'], c['unknown'], c['non_traversable'],
               '-' if rate is None else '%.2f%%' % (100 * rate)))


def print_report(name: str, rep: dict, tf, align_info) -> None:
    print('== %s' % name)
    if tf is not None:
        print('  transform: x=%+.3f m y=%+.3f m yaw=%+.2f deg%s'
              % (tf.x, tf.y, math.degrees(tf.yaw),
                 '' if not align_info else
                 '  (align score=%.3f identity=%.3f inliers=%.1f%% points=%d)'
                 % (align_info['score'], align_info['score_identity'],
                    100 * align_info['inliers'], align_info['points'])))
    print('  min_hits=%d tol=%.2f m robot_radius=%.2f m'
          % (rep['min_hits'], rep['tol'], rep['robot_radius']))
    print('  total          %s' % fmt_counts(rep['total']))
    for k, c in rep['by_source'].items():
        print('  %-14s %s' % (k, fmt_counts(c)))
    for c in rep['by_distance']:
        band = '%.0f-%s m' % (c['lo'], '' if c['hi'] is None else '%.0f' % c['hi'])
        print('  dist %-9s %s' % (band, fmt_counts(c)))
    print('  tol sweep: ' + '  '.join(
        '%.2f:%s' % (c['tol'], '-' if c['false_free_rate'] is None
                     else '%.2f%%' % (100 * c['false_free_rate'])) for c in rep['tol_sweep']))


def plot(path: str, gt, rep: dict, tf) -> None:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('--plot: matplotlib が無いので図は作らない', file=sys.stderr)
        return
    fig, ax = plt.subplots(figsize=(8, 8))
    occ = gt.cell_centers(gt.data == ff.OCCUPIED)
    free = gt.cell_centers(gt.data == ff.FREE)
    ax.scatter(free[:, 0], free[:, 1], s=1, c='#e0e0e0', marker='s', linewidths=0)
    ax.scatter(occ[:, 0], occ[:, 1], s=1, c='black', marker='s', linewidths=0)
    xy = (tf or ff.Transform()).apply(rep['_xy'])
    colors = {ff.TRUE_FREE: '#2a9d8f', ff.FALSE_FREE: '#e63946', ff.UNVERIFIED: '#f4a261'}
    for lab, col in colors.items():
        m = rep['_labels'] == lab
        ax.scatter(xy[m, 0], xy[m, 1], s=3, c=col, marker='s', linewidths=0,
                   label='%s (%d)' % (lab, int(m.sum())))
    ax.set_aspect('equal')
    ax.legend(loc='upper right', markerscale=4)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print('plot: %s' % path)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('runs', nargs='+', help='explore_*.seen.npz')
    ap.add_argument('--gt', help='正解の地図（map_server の YAML）')
    ap.add_argument('--self-check', action='store_true',
                    help='正解の代わりに実行自身の最後の /map を使う')
    ap.add_argument('--no-align', action='store_true', help='座標合わせをしない（恒等）')
    ap.add_argument('--tf', nargs=3, type=float, metavar=('X', 'Y', 'YAW_DEG'),
                    help='実行 → 正解の座標変換を手で与える（yaw は度）')
    ap.add_argument('--align-xy', type=float, default=0.5, help='探す平行移動の範囲 ±[m]')
    ap.add_argument('--align-yaw', type=float, default=10.0, help='探す回転の範囲 ±[度]')
    ap.add_argument('--align-step-xy', type=float, default=None, help='既定: 正解の解像度')
    ap.add_argument('--align-step-yaw', type=float, default=1.0)
    ap.add_argument('--tol', type=float, default=ff.DEFAULT_TOL)
    ap.add_argument('--robot-radius', type=float, default=ff.DEFAULT_ROBOT_RADIUS)
    ap.add_argument('--min-hits', type=int, default=None, help='既定: 実行のときの min_hits')
    ap.add_argument('--csv', help='セルごとの分類を CSV に（複数の実行は run 列で区別）')
    ap.add_argument('--json', help='まとめを JSON に')
    ap.add_argument('--plot', help='図（PNG。実行が複数なら名前に番号を足す）')
    a = ap.parse_args(argv)
    if not a.gt and not a.self_check:
        ap.error('--gt か --self-check のどちらかが要る')
    gt = ff.load_map_yaml(a.gt) if a.gt and not a.self_check else None
    summaries, rows = {}, []
    for i, path in enumerate(a.runs):
        run = ff.load_run(path)
        name = os.path.basename(path)
        tf, info = None, None
        kw = dict(tol=a.tol, robot_radius=a.robot_radius, min_hits=a.min_hits)
        if a.self_check:
            ref = ff.run_grid(run)
            rep = ff.self_check(run, **kw)
        else:
            ref = gt
            if a.tf:
                tf = ff.Transform(a.tf[0], a.tf[1], math.radians(a.tf[2]))
            elif a.no_align:
                tf = ff.Transform()
            else:
                tf, info = ff.align(ff.run_grid(run), gt, a.align_xy, a.align_yaw,
                                    a.align_step_xy, a.align_step_yaw)
            rep = ff.report(run, gt, tf, **kw)
        print_report(name + (' [self-check]' if a.self_check else ''), rep, tf, info)
        summaries[name] = {k: v for k, v in rep.items() if not k.startswith('_')}
        if tf is not None:
            summaries[name]['transform'] = {'x': tf.x, 'y': tf.y, 'yaw_deg': math.degrees(tf.yaw)}
        if info:
            summaries[name]['align'] = info
        if a.csv:
            for xy, lab, od, s, d in zip(rep['_xy'], rep['_labels'], rep['_occ_dist'],
                                         rep['_src'], rep['_dist']):
                rows.append([name, '%.3f' % xy[0], '%.3f' % xy[1], lab,
                             '' if not np.isfinite(od) else '%.3f' % od, int(s),
                             '' if not np.isfinite(d) else '%.2f' % d])
        if a.plot:
            out = a.plot
            if len(a.runs) > 1:
                root, ext = os.path.splitext(a.plot)
                out = '%s_%d%s' % (root, i, ext or '.png')
            plot(out, ref, rep, tf)
    if a.csv:
        with open(a.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['run', 'x', 'y', 'label', 'occ_dist', 'src', 'obs_dist'])
            w.writerows(rows)
        print('csv: %s' % a.csv)
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(summaries, f, ensure_ascii=False, indent=1)
        print('json: %s' % a.json)
    return 0


if __name__ == '__main__':
    sys.exit(main())
