#!/usr/bin/env python3
"""自動探索の記録（~/cugo_logs/explore_*.jsonl）を読んで、方法どうしを比べる表を出す。

使い方:
  python3 jetson/scripts/explog_report.py ~/cugo_logs/explore_*.jsonl
  python3 jetson/scripts/explog_report.py --group logs/*.jsonl            # 方法ごとの平均も出す
  python3 jetson/scripts/explog_report.py --csv out.csv --plot out.png logs/*.jsonl

方法の名前（variant）は、起動時の explore_variant パラメータ。空なら設定から作る（例: cam+theta）。
summary 行が無い（途中で落ちた）ファイルは、残りの行から集計し直す（終了理由は "-"）。
--plot は matplotlib があるときだけ（既知面積と時間・走行距離のグラフ）。
"""
import argparse
import csv
import os
import sys

try:
    from cugo_web import explog
except ImportError:  # colcon でビルドしていない PC: リポジトリの中から読む
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    '..', 'src', 'cugo_web'))
    from cugo_web import explog

COLUMNS = [  # (見出し, summary のキー, 書式)
    ('time[s]', 'time', '%.0f'),
    ('path[m]', 'path', '%.1f'),
    ('area[m2]', 'known_final', '%.1f'),
    ('m2/m', 'area_per_m', '%.2f'),
    ('goals', 'goals', '%d'),
    ('fail', 'failures', '%d'),
    ('back', 'backtracks', '%d'),
    ('cam', 'camera_only_goals', '%d'),
    ('t50[s]', 't50', '%.0f'),
    ('t90[s]', 't90', '%.0f'),
    ('stop', 'stop', '%s'),
]


def stop_label(summary: dict) -> str:
    """終了理由。目的地が無くて終わったときは内訳も（例: no_frontiers:visited）。"""
    r = summary.get('stop_reason') or '-'
    c = summary.get('end_cause')
    return '%s:%s' % (r, c) if c and r != '-' else r


def variant_name(config: dict) -> str:
    if config.get('variant'):
        return str(config['variant'])
    if not config.get('use_camera_floor', False):
        return 'lidar'
    parts = ['cam' if config.get('use_cliff_floor', True) else 'nocam']
    if config.get('use_theta'):
        parts.append('theta')
    return '+'.join(parts)


def load_run(path: str) -> dict:
    """1 つのファイルを読み、{'file', 'variant', 'config', 'summary', 'samples'} を返す。"""
    records = explog.read_jsonl(path)
    meta = next((r for r in records if r.get('ev') == 'meta'), {})
    config = meta.get('config', {})
    summary = next((r for r in reversed(records) if r.get('ev') == 'summary'), None)
    if summary is None:
        summary = explog.summarize(records)
    summary = dict(summary, stop=stop_label(summary))
    return {'file': path, 'start': meta.get('start', ''), 'variant': variant_name(config),
            'config': config, 'summary': summary,
            'samples': [r for r in records if r.get('ev') == 'sample']}


def fmt(v, f):
    if v is None:
        return '-'
    try:
        return f % v
    except TypeError:
        return str(v)


def table(rows, first) -> str:
    """rows: [(先頭の列の値, summary)]。"""
    head = [first] + [c[0] for c in COLUMNS]
    body = [[name] + [fmt(s.get(k), f) for _, k, f in COLUMNS] for name, s in rows]
    w = [max(len(r[i]) for r in [head] + body) for i in range(len(head))]
    lines = ['  '.join(c.ljust(w[i]) if i == 0 else c.rjust(w[i]) for i, c in enumerate(r))
             for r in [head] + body]
    lines.insert(1, '  '.join('-' * x for x in w))
    return '\n'.join(lines)


def group_means(runs):
    """方法ごとの平均（数値の列だけ）。キー n は回数、complete は完了した回数。"""
    groups = {}
    for r in runs:
        groups.setdefault(r['variant'], []).append(r['summary'])
    out = []
    for name, ss in groups.items():
        m = {'stop': '%d/%d completed' % (sum(1 for s in ss if s.get('complete')), len(ss))}
        for _, k, _f in COLUMNS:
            vals = [s[k] for s in ss if isinstance(s.get(k), (int, float))
                    and not isinstance(s.get(k), bool)]
            if vals and k != 'stop':
                m[k] = sum(vals) / len(vals)
        out.append(('%s (n=%d)' % (name, len(ss)), m))
    return out


def plot(runs, out_png) -> bool:
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib が無いのでグラフは作りません', file=sys.stderr)
        return False
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.2))
    for r in runs:
        s = r['samples']
        if not s:
            continue
        label = '%s %s' % (r['variant'], os.path.basename(r['file']))
        a1.plot([x['t'] for x in s], [x['known'] for x in s], label=label)
        a2.plot([x['path'] for x in s], [x['known'] for x in s], label=label)
    a1.set_xlabel('time [s]')
    a2.set_xlabel('path length [m]')
    for a in (a1, a2):
        a.set_ylabel('known area [m$^2$]')
        a.grid(True, alpha=0.3)
    a1.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    return True


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('files', nargs='+', help='explore_*.jsonl')
    ap.add_argument('--group', action='store_true', help='方法（variant）ごとの平均も出す')
    ap.add_argument('--csv', help='1 回 1 行の CSV に書く')
    ap.add_argument('--plot', help='既知面積-時間・既知面積-距離のグラフ（PNG）')
    a = ap.parse_args(argv)
    runs = []
    for f in a.files:
        try:
            runs.append(load_run(f))
        except (OSError, ValueError) as exc:
            print('%s: 読めません (%s)' % (f, exc), file=sys.stderr)
    if not runs:
        return 1
    runs.sort(key=lambda r: (r['variant'], r['start']))
    print(table([('%s %s' % (r['variant'], os.path.basename(r['file'])), r['summary'])
                 for r in runs], 'run'))
    if a.group:
        print()
        print(table(group_means(runs), 'variant'))
    if a.csv:
        keys = (['file', 'variant', 'start'] + [k for _, k, _f in COLUMNS]
                + ['stop_reason', 'end_cause', 'd50', 'd90', 'jumps'])
        with open(a.csv, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(keys)
            for r in runs:
                w.writerow([r.get(k, r['summary'].get(k)) if k in ('file', 'variant', 'start')
                            else r['summary'].get(k) for k in keys])
    if a.plot and plot(runs, a.plot):
        print('グラフ: %s' % a.plot)
    return 0


if __name__ == '__main__':
    sys.exit(main())
