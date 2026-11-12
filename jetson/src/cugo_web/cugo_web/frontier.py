"""Frontier-exploration goal selection on a 2D occupancy grid (no ROS dependencies).

Grid convention (same as nav_msgs/OccupancyGrid reshaped to (H, W)):
row 0 = bottom (min y), values -1 unknown, 0..100 occupancy probability.
Cell (r, c) center: x = ox + (c + 0.5) * res, y = oy + (r + 0.5) * res.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
from scipy import ndimage

SQRT2 = math.sqrt(2.0)
START_SEARCH_RADIUS = 1.0  # m: how far to look for a safe start cell if the robot cell is not safe


@dataclass
class FrontierGoal:
    x: float
    y: float
    yaw: float       # faces the cluster centroid from the goal (path arrival heading if close)
    size: int        # cells in the frontier cluster
    distance: float  # path distance (m) from robot to the goal cell through safe free space
    cx: float        # cluster centroid (world)
    cy: float
    score: float = 0.0  # size_weight * size_m - distance_weight * nearest-cell distance


# ---------------------------------------------------------------- coordinates

def cell_to_world(r, c, resolution, origin) -> Tuple[float, float]:
    """Center of cell (r, c) in world coordinates."""
    return (origin[0] + (c + 0.5) * resolution, origin[1] + (r + 0.5) * resolution)


def world_to_cell(x, y, resolution, origin) -> Tuple[int, int]:
    """(row, col) of the cell containing world point (x, y). May be out of bounds."""
    return (int(math.floor((y - origin[1]) / resolution)),
            int(math.floor((x - origin[0]) / resolution)))


# ---------------------------------------------------------------- grid analysis

def _disk(radius_cells: int) -> np.ndarray:
    r = radius_cells
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    return (xx * xx + yy * yy) <= r * r


def safe_free_mask(grid, resolution, robot_radius=0.30, free_thresh=25, occ_thresh=65):
    """Free cells not within robot_radius of any occupied cell."""
    grid = np.asarray(grid)
    free = (grid >= 0) & (grid <= free_thresh)
    occ = grid >= occ_thresh
    rad = int(math.ceil(robot_radius / resolution)) if robot_radius > 0 else 0
    if rad > 0 and occ.any():
        inflated = ndimage.binary_dilation(occ, structure=_disk(rad))
    else:
        inflated = occ
    return free & ~inflated


def _path_distances(safe: np.ndarray, start: Tuple[int, int], start_cost: float,
                    resolution: float) -> np.ndarray:
    """8-connected Dijkstra (diagonal cost sqrt2) over safe cells.

    Returns metres, inf = unreached.
    """
    h, w = safe.shape
    # Pad with a non-traversable border so neighbour offsets need no bounds checks.
    pw = w + 2
    passable = np.zeros((h + 2, pw), dtype=bool)
    passable[1:-1, 1:-1] = safe
    passable_flat = passable.ravel().tolist()
    dist = [math.inf] * len(passable_flat)

    s = (start[0] + 1) * pw + (start[1] + 1)
    dist[s] = start_cost
    nbrs = ((1, 1.0), (-1, 1.0), (pw, 1.0), (-pw, 1.0),
            (pw + 1, SQRT2), (pw - 1, SQRT2), (-pw + 1, SQRT2), (-pw - 1, SQRT2))
    heap = [(start_cost, s)]
    pop, push = heapq.heappop, heapq.heappush
    while heap:
        d, i = pop(heap)
        if d > dist[i]:
            continue
        for off, cost in nbrs:
            j = i + off
            if passable_flat[j]:
                nd = d + cost
                if nd < dist[j]:
                    dist[j] = nd
                    push(heap, (nd, j))

    out = np.asarray(dist, dtype=float).reshape(h + 2, pw)[1:-1, 1:-1]
    return out * resolution  # start_cost is given in cells, scaled here too


def _start_cell(safe, resolution, origin, robot_xy):
    """Robot cell if safe, else nearest safe cell within START_SEARCH_RADIUS. Returns
    ((r, c), start_cost_in_cells) or None."""
    h, w = safe.shape
    r0, c0 = world_to_cell(robot_xy[0], robot_xy[1], resolution, origin)
    if 0 <= r0 < h and 0 <= c0 < w and safe[r0, c0]:
        return (r0, c0), 0.0
    k = int(math.ceil(START_SEARCH_RADIUS / resolution)) + 1
    rlo, rhi = max(r0 - k, 0), min(r0 + k + 1, h)
    clo, chi = max(c0 - k, 0), min(c0 + k + 1, w)
    if rlo >= rhi or clo >= chi:
        return None
    rs, cs = np.nonzero(safe[rlo:rhi, clo:chi])
    if rs.size == 0:
        return None
    rs = rs + rlo
    cs = cs + clo
    xs = origin[0] + (cs + 0.5) * resolution
    ys = origin[1] + (rs + 0.5) * resolution
    d = np.hypot(xs - robot_xy[0], ys - robot_xy[1])
    i = int(np.argmin(d))
    if d[i] > START_SEARCH_RADIUS:
        return None
    return (int(rs[i]), int(cs[i])), float(d[i]) / resolution


def _near_blacklist(x, y, blacklist: Sequence[Tuple[float, float]], radius: float) -> bool:
    r2 = radius * radius
    for bx, by in blacklist:
        if (x - bx) ** 2 + (y - by) ** 2 <= r2:
            return True
    return False


_EIGHT = np.ones((3, 3), dtype=bool)
_OFFSETS8 = ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1))
YAW_CENTROID_MIN_DIST = 0.2  # m: closer centroid than this -> yaw = path arrival heading
ARRIVAL_LOOKBACK = 0.5       # m: look-back radius for the path arrival heading


def _arrival_yaw(dist, r, c, k, disk) -> Optional[float]:
    """Heading of the path's final approach into cell (r, c): from the mean position of the
    cells within k cells (disk) that are closer to the robot (lower path distance) towards
    (r, c). Unlike the straight robot->goal line this follows corners (e.g. an L-shaped
    corridor); averaging avoids the diagonal bias of single 8-connected Dijkstra cells.
    None if no such cell exists."""
    h, w = dist.shape
    r0, r1, c0, c1 = max(r - k, 0), min(r + k + 1, h), max(c - k, 0), min(c + k + 1, w)
    sub = dist[r0:r1, c0:c1]
    behind = disk[r0 - r + k:r1 - r + k, c0 - c + k:c1 - c + k] & (sub < dist[r, c])
    br, bc = np.nonzero(behind)
    if br.size == 0:
        return None
    dr = r - (br.mean() + r0)
    dc = c - (bc.mean() + c0)
    if dr * dr + dc * dc < 1e-12:
        return None
    return math.atan2(dr, dc)


def _adjacent_unknown_area(labels, n, unknown, res) -> np.ndarray:
    """Per frontier cluster label (0..n): total area (m^2) of the distinct 8-connected unknown
    components touching the cluster dilated by one cell. Components touching the grid border
    count as infinite (they continue beyond the map)."""
    ulab, nu = ndimage.label(unknown, structure=_EIGHT)
    adj = np.zeros(n + 1, dtype=float)
    if nu == 0:
        return adj
    uarea = np.bincount(ulab.ravel(), minlength=nu + 1).astype(float) * (res * res)
    border = np.concatenate((ulab[0, :], ulab[-1, :], ulab[:, 0], ulab[:, -1]))
    uarea[np.unique(border)] = math.inf
    uarea[0] = 0.0

    fr, fc = np.nonzero(labels)
    flab = labels[fr, fc].astype(np.int64)
    up = np.pad(ulab, 1)  # zero (= not unknown) outside the grid
    keys = []
    for dr, dc in _OFFSETS8:
        u = up[fr + 1 + dr, fc + 1 + dc].astype(np.int64)
        m = u > 0
        keys.append(flab[m] * (nu + 1) + u[m])
    keys = np.unique(np.concatenate(keys))  # distinct (cluster, unknown component) pairs
    adj += np.bincount(keys // (nu + 1), weights=uarea[keys % (nu + 1)], minlength=n + 1)
    return adj


def _candidates(grid, resolution, origin, robot_xy, blacklist, robot_radius,
                min_frontier_size, blacklist_radius, min_goal_distance,
                free_thresh, occ_thresh, *, visited=(), visited_radius=0.75,
                min_unknown_area=0.25, goal_advance=0.5, size_weight=1.0,
                distance_weight=1.0) -> List[FrontierGoal]:
    grid = np.asarray(grid)
    if grid.ndim != 2 or grid.size == 0:
        return []
    res = float(resolution)
    origin = (float(origin[0]), float(origin[1]))
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    blacklist = [(float(b[0]), float(b[1])) for b in blacklist]
    visited = [(float(v[0]), float(v[1])) for v in visited]

    safe = safe_free_mask(grid, res, robot_radius, free_thresh, occ_thresh)
    unknown = grid < 0
    near_unknown = ndimage.binary_dilation(unknown, structure=_EIGHT)
    frontier = safe & near_unknown
    if not frontier.any():
        return []

    labels, n = ndimage.label(frontier, structure=_EIGHT)
    if n == 0:
        return []
    sizes = np.bincount(labels.ravel(), minlength=n + 1)
    valid = sizes * res >= min_frontier_size
    valid[0] = False
    if valid.any() and min_unknown_area > 0:
        # Enclosed-unknown filter: clusters only bordering small closed unknown pockets
        # (LiDAR shadows, unseen floor patches) can never be "explored away".
        valid &= _adjacent_unknown_area(labels, n, unknown, res) >= min_unknown_area
    if not valid.any():
        return []

    start = _start_cell(safe, res, origin, (rx, ry))
    if start is None:
        return []
    dist = _path_distances(safe, start[0], start[1], res)

    # Centroids of all clusters (row, col), vectorised.
    rows_idx, cols_idx = np.indices(grid.shape)
    lab_flat = labels.ravel()
    sum_r = np.bincount(lab_flat, weights=rows_idx.ravel(), minlength=n + 1)
    sum_c = np.bincount(lab_flat, weights=cols_idx.ravel(), minlength=n + 1)
    safe_sizes = np.maximum(sizes, 1)
    cen_r = sum_r / safe_sizes
    cen_c = sum_c / safe_sizes

    # Reachable frontier cells of valid clusters.
    cand_mask = valid[labels] & np.isfinite(dist)
    rs, cs = np.nonzero(cand_mask)  # row-major order
    if rs.size == 0:
        return []
    labs = labels[rs, cs]
    ds = dist[rs, cs]

    # Nearest reachable cell distance per cluster.
    d_near = np.full(n + 1, math.inf)
    np.minimum.at(d_near, labs, ds)

    # Goal placement rule: among the cluster's reachable frontier cells whose path distance is
    # at most d_nearest + goal_advance, take the one closest (Euclidean) to the cluster
    # centroid; ties -> smaller path distance -> row-major order. This advances the goal up to
    # goal_advance into the cluster without overshooting the centroid, and centres it across
    # a corridor-end frontier.
    cdist = np.round(np.hypot(rs - cen_r[labs], cs - cen_c[labs]), 9)
    inelig = ds > d_near[labs] + max(float(goal_advance), 0.0) + 1e-9
    idx = np.arange(rs.size)
    order = np.lexsort((idx, ds, cdist, inelig, labs))
    labs_sorted = labs[order]
    first = np.ones(order.size, dtype=bool)
    first[1:] = labs_sorted[1:] != labs_sorted[:-1]

    arrive_k = max(1, int(round(ARRIVAL_LOOKBACK / res)))
    disk_k = _disk(arrive_k)
    out: List[FrontierGoal] = []
    for k in order[first]:
        lab = int(labs[k])
        dn = float(d_near[lab])
        if dn < min_goal_distance:
            continue
        gx, gy = cell_to_world(int(rs[k]), int(cs[k]), res, origin)
        size = int(sizes[lab])
        cx, cy = cell_to_world(cen_r[lab], cen_c[lab], res, origin)
        cx, cy = float(cx), float(cy)
        if blacklist and (_near_blacklist(gx, gy, blacklist, blacklist_radius)
                          or _near_blacklist(cx, cy, blacklist, blacklist_radius)):
            continue
        if visited and (_near_blacklist(gx, gy, visited, visited_radius)
                        or _near_blacklist(cx, cy, visited, visited_radius)):
            continue
        if math.hypot(cx - gx, cy - gy) >= YAW_CENTROID_MIN_DIST:
            yaw = math.atan2(cy - gy, cx - gx)
        else:
            yaw = _arrival_yaw(dist, int(rs[k]), int(cs[k]), arrive_k, disk_k)
            if yaw is None:
                yaw = math.atan2(gy - ry, gx - rx)
        score = size_weight * size * res - distance_weight * dn
        out.append(FrontierGoal(x=gx, y=gy, yaw=yaw, size=size, distance=float(ds[k]),
                                cx=cx, cy=cy, score=float(score)))
    return out


# ---------------------------------------------------------------- public API

def find_frontier_goal(grid, resolution, origin, robot_xy, blacklist: Iterable = (), *,
                       robot_radius=0.30, min_frontier_size=0.30, blacklist_radius=0.5,
                       min_goal_distance=0.4, free_thresh=25, occ_thresh=65,
                       size_weight=1.0, distance_weight=1.0,
                       visited: Iterable = (), visited_radius=0.75,
                       min_unknown_area=0.25, goal_advance=0.5,
                       prefer: Optional[Tuple[float, float]] = None, prefer_radius=1.5,
                       switch_margin=1.0) -> Optional[FrontierGoal]:
    """Best reachable frontier goal, or None when exploration is complete / impossible.

    Score = size_weight * (cluster size in m) - distance_weight * path distance (m) to the
    cluster's nearest reachable cell; ties go to the nearer goal.

    Filters: clusters whose adjacent unknown components total < min_unknown_area m^2
    (border-touching unknown = infinite) are ignored; clusters whose goal or centroid lies
    within blacklist_radius of a blacklist point or visited_radius of a visited point are
    skipped.

    Hysteresis: when `prefer` (centroid of the cluster being pursued) is given and a
    candidate's centroid lies within prefer_radius of it (closest one wins), that candidate
    is kept unless the best candidate's score is higher by more than switch_margin.

    Goal: see _candidates (advanced up to goal_advance m into the cluster towards its
    centroid). Yaw faces the cluster centroid from the goal; when the centroid is within
    0.2 m of the goal (e.g. a frontier line across a corridor end) it is the heading of the
    path's final ~0.5 m approach (robot->goal straight-line heading if unavailable), so the
    robot keeps looking down an L-shaped corridor instead of at its wall.
    """
    cands = _candidates(grid, resolution, origin, robot_xy, blacklist, robot_radius,
                        min_frontier_size, blacklist_radius, min_goal_distance,
                        free_thresh, occ_thresh, visited=visited,
                        visited_radius=visited_radius, min_unknown_area=min_unknown_area,
                        goal_advance=goal_advance, size_weight=size_weight,
                        distance_weight=distance_weight)
    if not cands:
        return None
    best = max(cands, key=lambda g: (g.score, -g.distance))
    if prefer is not None:
        px, py = float(prefer[0]), float(prefer[1])
        near = [(math.hypot(g.cx - px, g.cy - py), i) for i, g in enumerate(cands)]
        near = [t for t in near if t[0] <= prefer_radius]
        if near:
            kept = cands[min(near)[1]]
            if best.score - kept.score <= switch_margin:
                return kept
    return best


def count_frontiers(grid, resolution, origin, robot_xy, blacklist: Iterable = (), *,
                    robot_radius=0.30, min_frontier_size=0.30, blacklist_radius=0.5,
                    min_goal_distance=0.4, free_thresh=25, occ_thresh=65,
                    visited: Iterable = (), visited_radius=0.75, min_unknown_area=0.25,
                    **_ignored) -> int:
    """Number of frontier clusters that would be valid goal candidates (same filters as
    find_frontier_goal; scoring/hysteresis/goal-placement options are ignored)."""
    return len(_candidates(grid, resolution, origin, robot_xy, blacklist, robot_radius,
                           min_frontier_size, blacklist_radius, min_goal_distance,
                           free_thresh, occ_thresh, visited=visited,
                           visited_radius=visited_radius, min_unknown_area=min_unknown_area))
