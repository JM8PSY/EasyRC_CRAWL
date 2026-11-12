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
    yaw: float       # heading from robot toward goal
    size: int        # cells in the frontier cluster
    distance: float  # path distance (m) from robot through safe free space
    cx: float        # cluster centroid (world)
    cy: float


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
    """8-connected Dijkstra (diagonal cost sqrt2) over safe cells. Returns metres, inf = unreached."""
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


def _candidates(grid, resolution, origin, robot_xy, blacklist, robot_radius,
                min_frontier_size, blacklist_radius, min_goal_distance,
                free_thresh, occ_thresh) -> List[FrontierGoal]:
    grid = np.asarray(grid)
    if grid.ndim != 2 or grid.size == 0:
        return []
    res = float(resolution)
    origin = (float(origin[0]), float(origin[1]))
    rx, ry = float(robot_xy[0]), float(robot_xy[1])
    blacklist = [(float(b[0]), float(b[1])) for b in blacklist]

    safe = safe_free_mask(grid, res, robot_radius, free_thresh, occ_thresh)
    unknown = grid < 0
    eight = np.ones((3, 3), dtype=bool)
    near_unknown = ndimage.binary_dilation(unknown, structure=eight)
    frontier = safe & near_unknown
    if not frontier.any():
        return []

    labels, n = ndimage.label(frontier, structure=eight)
    if n == 0:
        return []
    sizes = np.bincount(labels.ravel(), minlength=n + 1)
    valid = sizes * res >= min_frontier_size
    valid[0] = False
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

    # Reachable frontier cells of valid clusters; nearest cell per cluster.
    cand_mask = valid[labels] & np.isfinite(dist)
    rs, cs = np.nonzero(cand_mask)
    if rs.size == 0:
        return []
    labs = labels[rs, cs]
    ds = dist[rs, cs]
    order = np.lexsort((ds, labs))  # by label, then distance
    labs_sorted = labs[order]
    first = np.ones(order.size, dtype=bool)
    first[1:] = labs_sorted[1:] != labs_sorted[:-1]

    out: List[FrontierGoal] = []
    for k in order[first]:
        lab = int(labs[k])
        d = float(ds[k])
        if d < min_goal_distance:
            continue
        gx, gy = cell_to_world(int(rs[k]), int(cs[k]), res, origin)
        size = int(sizes[lab])
        cx, cy = cell_to_world(sum_r[lab] / size, sum_c[lab] / size, res, origin)
        if blacklist and (_near_blacklist(gx, gy, blacklist, blacklist_radius)
                          or _near_blacklist(cx, cy, blacklist, blacklist_radius)):
            continue
        out.append(FrontierGoal(x=gx, y=gy, yaw=math.atan2(gy - ry, gx - rx), size=size,
                                distance=d, cx=float(cx), cy=float(cy)))
    return out


# ---------------------------------------------------------------- public API

def find_frontier_goal(grid, resolution, origin, robot_xy, blacklist: Iterable = (), *,
                       robot_radius=0.30, min_frontier_size=0.30, blacklist_radius=0.5,
                       min_goal_distance=0.4, free_thresh=25, occ_thresh=65,
                       size_weight=1.0, distance_weight=1.0) -> Optional[FrontierGoal]:
    """Best reachable frontier goal, or None when exploration is complete / impossible.

    Score = size_weight * (cluster size in m) - distance_weight * path distance (m);
    ties go to the nearer frontier.
    """
    cands = _candidates(grid, resolution, origin, robot_xy, blacklist, robot_radius,
                        min_frontier_size, blacklist_radius, min_goal_distance,
                        free_thresh, occ_thresh)
    if not cands:
        return None
    res = float(resolution)
    return max(cands, key=lambda g: (size_weight * g.size * res - distance_weight * g.distance,
                                     -g.distance))


def count_frontiers(grid, resolution, origin, robot_xy, blacklist: Iterable = (), *,
                    robot_radius=0.30, min_frontier_size=0.30, blacklist_radius=0.5,
                    min_goal_distance=0.4, free_thresh=25, occ_thresh=65, **_ignored) -> int:
    """Number of frontier clusters that would be valid goal candidates."""
    return len(_candidates(grid, resolution, origin, robot_xy, blacklist, robot_radius,
                           min_frontier_size, blacklist_radius, min_goal_distance,
                           free_thresh, occ_thresh))
