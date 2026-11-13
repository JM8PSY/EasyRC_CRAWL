"""No-return frontier analysis (experiment 2-1 / 2-2). No ROS dependencies.

Question: with a cheap 2D LiDAR (YDLIDAR X4 PRO, range_max 10 m) how many of the frontiers on
the slam_toolbox map border bearings where the LiDAR gets *no return* (corridor beyond range,
glass, dark/absorbing walls)? Such frontiers cannot be "explored away" by looking from where
the robot already is, so they mislead frontier exploration.

Frontier cells / clusters
    Exactly the frontier.py definition: free cells (0..free_thresh) not within robot_radius
    of an occupied cell, 8-adjacent to an unknown (-1) cell of the *raw* SLAM map, grouped into
    8-connected clusters. Only clusters that frontier.py would consider (size >=
    min_frontier_size and adjacent unknown area >= min_unknown_area) are analysed, i.e. the
    clusters that can become goal candidates (reachability, blacklist and visited filters are
    not applied). Defaults are those of find_frontier_goal.

Per-cell label, from ONE scan taken at the laser pose (lx, ly, laser_yaw) in the map frame.
Let d = distance laser -> cell centre and b = bearing of the cell in the laser frame.
    unobserved  d > range_max + range_slack, d < range_min, b outside the scan's angular
                coverage, or every beam in the window is NaN (NaN = "no data", e.g. the
                sectors masked by cugo_perception/scan_inf because the robot body blocks them).
    noreturn    every usable beam within +-tol of b returned nothing: +inf, 0 (raw YDLIDAR
                driver), > range_max, or a negative finite value.
    hit         some usable beam in the window ends at or before the frontier:
                min range <= d + pass_margin. -inf and 0 < r < range_min (REP 117 "too close")
                count as a hit at range 0. The unknown behind the frontier is an occlusion
                shadow / the frontier lies next to a surface -- ordinary, explorable.
    passed      every returning beam in the window ends beyond d + pass_margin. The beam crossed
                the frontier and hit something farther away, so slam_toolbox should have
                cleared the unknown cells behind it; seeing this means the map lags the scan
                (map_update_interval), the frontier sits slightly off the beam, or the unknown
                was observed from another pose. Kept as a separate category, not no-return.
    "no-return" is claimed only if *all* beams in the window are no-return (conservative).

Thresholds (parameters, defaults explained):
    tol = 1.5 deg      X4 PRO gives ~0.5 deg/beam -> ~7 beams; +-1.5 deg is +-26 cm at 10 m,
                       about the width of one goal cell cluster. Always at least the nearest beam.
    pass_margin = 0.3 m  6 cells at 0.05 m/cell + X4 PRO range noise (~2 % at 10 m = 0.2 m).
    range_slack = 0.3 m  slam_toolbox max_laser_range = 10 m = scan range_max, so a frontier
                       created by the range limit lies ~range_max from where it was seen; the
                       slack keeps those observable from (nearly) the same place.

Per-cluster label (over its observable cells): noreturn if noreturn >= cluster_ratio (0.5) of
the observable cells, otherwise hit or passed (whichever has more cells, ties -> hit);
unobserved if no cell is observable.

Fractions: frac = noreturn / all analysed; frac_obs = noreturn / observable (= not unobserved).
Limitation: one scan judges a map built from many past scans; a frontier seen from elsewhere
is judged from the current pose only.
"""
from __future__ import annotations

import inspect
import math
from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy import ndimage

from cugo_web import frontier

UNOBSERVED, NORETURN, HIT, PASSED = 0, 1, 2, 3
LABELS = ('unobserved', 'noreturn', 'hit', 'passed')
TWO_PI = 2.0 * math.pi

DEFAULT_TOL = math.radians(1.5)
DEFAULT_PASS_MARGIN = 0.3
DEFAULT_RANGE_SLACK = 0.3
DEFAULT_CLUSTER_RATIO = 0.5


def _frontier_params() -> dict:
    sig = inspect.signature(frontier.find_frontier_goal)
    keys = ('robot_radius', 'free_thresh', 'occ_thresh', 'min_frontier_size', 'min_unknown_area')
    return {k: sig.parameters[k].default for k in keys}


def classify_frontier_cells(cells_xy, laser_xy, laser_yaw, ranges, angle_min, angle_inc,
                            range_min, range_max, tol=DEFAULT_TOL,
                            pass_margin=DEFAULT_PASS_MARGIN,
                            range_slack=DEFAULT_RANGE_SLACK) -> np.ndarray:
    """Label (UNOBSERVED / NORETURN / HIT / PASSED, int8) for each map point in cells_xy (N, 2).

    laser_xy / laser_yaw: laser frame origin and heading in the map frame. ranges, angle_min,
    angle_inc, range_min, range_max: the LaserScan fields. See the module docstring."""
    xy = np.asarray(cells_xy, dtype=float).reshape(-1, 2)
    r = np.asarray(ranges, dtype=float).ravel()
    n, npts = r.size, xy.shape[0]
    out = np.full(npts, UNOBSERVED, dtype=np.int8)
    inc = float(angle_inc)
    if npts == 0 or n == 0 or inc == 0.0 or not math.isfinite(inc):
        return out
    a0 = float(angle_min)
    if inc < 0:  # clockwise scans: reverse so that angles increase
        r = r[::-1]
        a0 = a0 + (n - 1) * inc
        inc = -inc
    tol = max(float(tol), 0.5 * inc + 1e-9)  # always at least the nearest beam

    dx = xy[:, 0] - float(laser_xy[0])
    dy = xy[:, 1] - float(laser_xy[1])
    d = np.hypot(dx, dy)
    f = np.mod(np.arctan2(dy, dx) - float(laser_yaw) - a0, TWO_PI)  # angle past angle_min
    full = n * inc >= TWO_PI - 0.5 * inc
    span = (n - 1) * inc
    covered = np.ones(npts, bool) if full else (f <= span + tol) | (f >= TWO_PI - tol)
    if not full:  # just before angle_min: use a negative angle so beam 0 is in the window
        f = np.where(f >= TWO_PI - tol, f - TWO_PI, f)
    gate = covered & (d <= float(range_max) + range_slack) & (d >= float(range_min))

    m = int(math.ceil(tol / inc)) + 1
    ks = np.arange(-m, m + 2)
    j = np.floor(f / inc).astype(np.int64)[:, None] + ks[None, :]  # (N, K) beam indices
    if full:
        jj = np.mod(j, n)
        ok = np.ones(j.shape, bool)
    else:
        ok = (j >= 0) & (j < n)
        jj = np.clip(j, 0, n - 1)
    diff = np.mod(f[:, None] - jj * inc + math.pi, TWO_PI) - math.pi
    win = ok & (np.abs(diff) <= tol)

    vals = r[jj]
    nan = np.isnan(vals)
    near = (vals == -np.inf) | ((vals > 0) & (vals < float(range_min)))
    noret = ~nan & ~near & ((vals == np.inf) | (vals <= 0) | (vals > float(range_max)))
    usable = win & ~nan
    eff = np.where(near, 0.0, np.where(noret | nan, np.inf, vals))
    eff = np.where(usable & ~noret, eff, np.inf)
    rmin = eff.min(axis=1)

    any_usable = usable.any(axis=1)
    all_noret = any_usable & ~(usable & ~noret).any(axis=1)
    lab = np.where(rmin <= d + pass_margin, HIT, PASSED)
    lab = np.where(all_noret, NORETURN, lab)
    lab = np.where(gate & any_usable, lab, UNOBSERVED)
    return lab.astype(np.int8)


@dataclass
class Analysis:
    xy: np.ndarray              # (N, 2) analysed frontier cell centres (map frame)
    cell_labels: np.ndarray     # (N,) int8
    cell_cluster: np.ndarray    # (N,) index into cluster_labels
    cluster_labels: np.ndarray  # (K,) int8
    cluster_sizes: np.ndarray   # (K,) cells
    cells_all: int              # all frontier cells (before the cluster filters)


def analyze(grid, resolution, origin, laser_xy, laser_yaw, ranges, angle_min, angle_inc,
            range_min, range_max, *, tol=DEFAULT_TOL, pass_margin=DEFAULT_PASS_MARGIN,
            range_slack=DEFAULT_RANGE_SLACK, cluster_ratio=DEFAULT_CLUSTER_RATIO,
            **frontier_kw) -> Analysis:
    """Frontier cells/clusters of the raw map (frontier.py definition) with their labels.
    frontier_kw overrides robot_radius, free_thresh, occ_thresh, min_frontier_size,
    min_unknown_area (defaults: find_frontier_goal)."""
    p = _frontier_params()
    p.update({k: v for k, v in frontier_kw.items() if k in p})
    g = np.asarray(grid)
    res = float(resolution)
    empty = Analysis(np.zeros((0, 2)), np.zeros(0, np.int8), np.zeros(0, np.int64),
                     np.zeros(0, np.int8), np.zeros(0, np.int64), 0)
    if g.ndim != 2 or g.size == 0:
        return empty
    safe = frontier.safe_free_mask(g, res, p['robot_radius'], p['free_thresh'], p['occ_thresh'])
    unknown = g < 0
    front = safe & ndimage.binary_dilation(unknown, structure=frontier._EIGHT)
    labels, n = ndimage.label(front, structure=frontier._EIGHT)
    empty.cells_all = int(front.sum())
    if n == 0:
        return empty
    sizes = np.bincount(labels.ravel(), minlength=n + 1)
    valid = sizes * res >= p['min_frontier_size']
    valid[0] = False
    if valid.any() and p['min_unknown_area'] > 0:
        valid &= frontier._adjacent_unknown_area(labels, n, unknown, res) >= p['min_unknown_area']
    if not valid.any():
        return empty
    rs, cs = np.nonzero(valid[labels])
    xy = np.column_stack([float(origin[0]) + (cs + 0.5) * res,
                          float(origin[1]) + (rs + 0.5) * res])
    cell_labels = classify_frontier_cells(xy, laser_xy, laser_yaw, ranges, angle_min, angle_inc,
                                          range_min, range_max, tol, pass_margin, range_slack)
    ids = np.nonzero(valid)[0]
    remap = np.zeros(n + 1, np.int64)
    remap[ids] = np.arange(ids.size)
    cell_cluster = remap[labels[rs, cs]]
    k = ids.size
    counts = np.stack([np.bincount(cell_cluster[cell_labels == c], minlength=k)
                       for c in range(len(LABELS))])  # (4, K)
    obs = counts[NORETURN] + counts[HIT] + counts[PASSED]
    cl = np.where(counts[PASSED] > counts[HIT], PASSED, HIT)
    cl = np.where(counts[NORETURN] >= cluster_ratio * obs, NORETURN, cl)
    cl = np.where(obs > 0, cl, UNOBSERVED).astype(np.int8)
    return Analysis(xy, cell_labels, cell_cluster, cl, sizes[ids].astype(np.int64),
                    int(front.sum()))


def _counts(labels: np.ndarray) -> dict:
    c = np.bincount(np.asarray(labels, np.int64), minlength=len(LABELS))
    total = int(c.sum())
    obs = total - int(c[UNOBSERVED])
    out = {'n': total}
    out.update({name: int(c[i]) for i, name in enumerate(LABELS)})
    out['frac'] = round(c[NORETURN] / total, 4) if total else None
    out['frac_obs'] = round(c[NORETURN] / obs, 4) if obs else None
    return out


def stats_of(a: Analysis) -> dict:
    """{'cells': {...}, 'clusters': {...}, 'cells_all': int}; each {...} has n, one count per
    label, frac (noreturn / n) and frac_obs (noreturn / observable), None when undefined."""
    return {'cells': _counts(a.cell_labels), 'clusters': _counts(a.cluster_labels),
            'cells_all': a.cells_all}


def frontier_stats(grid, resolution, origin, laser_xy, laser_yaw, ranges, angle_min, angle_inc,
                   range_min, range_max, **kw) -> dict:
    """Counts and no-return fractions of the frontier cells and clusters (see stats_of)."""
    return stats_of(analyze(grid, resolution, origin, laser_xy, laser_yaw, ranges, angle_min,
                            angle_inc, range_min, range_max, **kw))


def label_near(a: Analysis, x: float, y: float, radius: float = 0.5) -> Optional[dict]:
    """Labels for the analysed frontier cell nearest (x, y) (e.g. a chosen goal) within radius:
    {'cluster': label of its cluster, 'cell': its own label, 'd': distance}. None if none."""
    if a.xy.shape[0] == 0:
        return None
    dd = np.hypot(a.xy[:, 0] - x, a.xy[:, 1] - y)
    i = int(np.argmin(dd))
    if dd[i] > radius:
        return None
    return {'cluster': LABELS[int(a.cluster_labels[a.cell_cluster[i]])],
            'cell': LABELS[int(a.cell_labels[i])], 'd': round(float(dd[i]), 3)}
