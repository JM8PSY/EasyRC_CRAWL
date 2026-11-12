import math

import numpy as np
from scipy import ndimage

import time

from cugo_web.frontier import (cell_to_world, count_frontiers, find_frontier_goal,
                               safe_free_mask, world_to_cell)

RES = 0.1
UNK, FREE, OCC = -1, 0, 100


def unknown_grid(h, w):
    return np.full((h, w), UNK, dtype=np.int16)


def corridor(h=30, w=100, open_right=True):
    """Free corridor rows 6..23 (walls rows 5 and 24), cols 5..60; wall at col 5 (closed
    left end); the right end (col 61+) is unknown when open_right."""
    g = unknown_grid(h, w)
    g[5:25, 5:61] = FREE
    g[5, 5:61] = OCC
    g[24, 5:61] = OCC
    g[5:25, 5] = OCC
    if not open_right:
        g[5:25, 60] = OCC
    return g


def xy(r, c, res=RES, origin=(0.0, 0.0)):
    return cell_to_world(r, c, res, origin)


def test_closed_room_has_no_frontier():
    g = unknown_grid(50, 50)
    g[5:45, 5:45] = FREE
    g[5, 5:45] = OCC
    g[44, 5:45] = OCC
    g[5:45, 5] = OCC
    g[5:45, 44] = OCC
    robot = xy(25, 25)
    assert find_frontier_goal(g, RES, (0.0, 0.0), robot) is None
    assert count_frontiers(g, RES, (0.0, 0.0), robot) == 0


def test_corridor_goal_at_open_end():
    g = corridor()
    robot = xy(14, 10)
    goal = find_frontier_goal(g, RES, (0.0, 0.0), robot)
    assert goal is not None
    r, c = world_to_cell(goal.x, goal.y, RES, (0.0, 0.0))
    assert c == 60                       # last free column, next to unknown
    assert 9 <= r <= 20                  # inside the safe band (3 cells from walls)
    assert abs(goal.yaw) < math.radians(20)
    assert goal.distance > 4.5           # ~5 m along the corridor
    assert goal.size == 12
    assert abs(goal.cx - xy(0, 60)[0]) < 1e-9
    assert count_frontiers(g, RES, (0.0, 0.0), robot) == 1


def two_frontier_grid():
    """Long corridor: small opening on the left (14 cells, near robot), a 51-cell gap in
    the top wall far to the right (bigger frontier)."""
    g = unknown_grid(100, 200)
    g[39:61, 10:191] = FREE
    g[39, 10:191] = OCC
    g[60, 10:191] = OCC
    g[39:61, 190] = OCC          # right end closed; left end (col 9) unknown
    g[60, 130:181] = UNK         # gap in the top wall
    return g


def test_nearer_frontier_wins_by_default_bigger_wins_with_size_weight():
    g = two_frontier_grid()
    robot = xy(50, 20)
    assert count_frontiers(g, RES, (0.0, 0.0), robot) == 2

    near = find_frontier_goal(g, RES, (0.0, 0.0), robot)
    assert near is not None
    assert world_to_cell(near.x, near.y, RES, (0.0, 0.0))[1] == 10
    assert near.distance < 1.5
    assert abs(abs(near.yaw) - math.pi) < math.radians(20)   # pointing -x

    far = find_frontier_goal(g, RES, (0.0, 0.0), robot, size_weight=20.0)
    assert far is not None
    r, c = world_to_cell(far.x, far.y, RES, (0.0, 0.0))
    assert 130 <= c <= 180 and r == 59
    assert far.size > 2 * near.size
    assert far.distance > 10.0


def test_frontier_behind_wall_is_ignored():
    g = unknown_grid(50, 100)
    # room A (robot), closed
    g[5:45, 5:45] = FREE
    g[5, 5:45] = OCC
    g[44, 5:45] = OCC
    g[5:45, 5] = OCC
    g[5:45, 44] = OCC
    # room B, open to unknown on its right but not connected to A
    g[5:45, 45:80] = FREE
    g[5, 45:80] = OCC
    g[44, 45:80] = OCC
    robot = xy(25, 25)
    assert find_frontier_goal(g, RES, (0.0, 0.0), robot) is None
    assert count_frontiers(g, RES, (0.0, 0.0), robot) == 0
    # sanity: from inside room B the frontier is found
    assert find_frontier_goal(g, RES, (0.0, 0.0), xy(25, 60)) is not None


def test_blacklist_near_only_frontier():
    g = corridor()
    robot = xy(14, 10)
    goal = find_frontier_goal(g, RES, (0.0, 0.0), robot)
    assert goal is not None
    assert find_frontier_goal(g, RES, (0.0, 0.0), robot, [(goal.x, goal.y)]) is None
    assert count_frontiers(g, RES, (0.0, 0.0), robot, [(goal.x, goal.y)]) == 0
    # a blacklist point far away does not matter
    assert find_frontier_goal(g, RES, (0.0, 0.0), robot, [(1.0, 1.0)]) is not None


def test_robot_inside_inflated_zone_uses_nearest_safe_cell():
    g = corridor()
    robot = xy(7, 10)          # 2 cells from the wall at row 5 -> inflated
    goal = find_frontier_goal(g, RES, (0.0, 0.0), robot)
    assert goal is not None
    assert world_to_cell(goal.x, goal.y, RES, (0.0, 0.0))[1] == 60
    # robot far outside any free space (> 1 m) -> None
    assert find_frontier_goal(g, RES, (0.0, 0.0), (9.5, 0.2)) is None


def test_tiny_frontier_below_min_size_is_ignored():
    g = unknown_grid(50, 50)
    g[5:45, 5:45] = FREE
    g[5, 5:45] = OCC
    g[44, 5:45] = OCC
    g[5:45, 5] = OCC
    g[5:45, 44] = OCC
    g[25, 35] = UNK            # single unknown cell -> 8-cell ring frontier (0.8 m)
    robot = xy(25, 15)
    # The single unknown cell is an enclosed 0.01 m^2 pocket, which the enclosed-unknown
    # filter (min_unknown_area, default 0.25 m^2) rejects; disable it to test the size limit.
    assert find_frontier_goal(g, RES, (0.0, 0.0), robot, min_frontier_size=1.0,
                              min_unknown_area=0.0) is None
    goal = find_frontier_goal(g, RES, (0.0, 0.0), robot, min_frontier_size=0.5,
                              min_unknown_area=0.0)
    assert goal is not None
    assert goal.size == 8
    assert world_to_cell(goal.x, goal.y, RES, (0.0, 0.0)) == (25, 34)


def test_min_goal_distance_skips_frontier_under_robot():
    g = corridor()
    robot = xy(14, 60)
    assert find_frontier_goal(g, RES, (0.0, 0.0), robot) is None
    assert find_frontier_goal(g, RES, (0.0, 0.0), robot, min_goal_distance=0.0) is not None


def test_coordinates_with_offset_origin():
    res = 0.05
    origin = (-10.3, 4.2)
    assert cell_to_world(0, 0, res, origin) == (-10.3 + 0.025, 4.2 + 0.025)
    x, y = cell_to_world(12, 34, res, origin)
    assert abs(x - (-10.3 + 34.5 * 0.05)) < 1e-9
    assert abs(y - (4.2 + 12.5 * 0.05)) < 1e-9
    assert world_to_cell(x, y, res, origin) == (12, 34)
    assert world_to_cell(-10.3 + 0.001, 4.2 + 0.001, res, origin) == (0, 0)
    assert world_to_cell(-10.3 - 0.001, 4.2, res, origin) == (0, -1)

    # Same corridor at res 0.05 with the offset origin: goal lands on the open end.
    g = corridor()
    robot = cell_to_world(14, 10, res, origin)
    goal = find_frontier_goal(g, res, origin, robot, robot_radius=0.15)
    assert goal is not None
    r, c = world_to_cell(goal.x, goal.y, res, origin)
    assert c == 60 and 9 <= r <= 20
    assert g[r, c] == FREE and g[r, c + 1] == UNK
    expect_x, expect_y = cell_to_world(r, c, res, origin)
    assert abs(goal.x - expect_x) < 1e-9 and abs(goal.y - expect_y) < 1e-9
    # Centroid is on the goal (frontier across the corridor end) -> yaw = path arrival
    # heading, which in a straight corridor is ~ the robot->goal heading.
    assert abs(goal.yaw - math.atan2(goal.y - robot[1], goal.x - robot[0])) < math.radians(6)


# ---------------------------------------------------------------- enclosed unknown filter

def room_with_pocket_and_exit():
    """Closed room (rows 5..44, cols 5..60) with a 3x3 enclosed unknown pocket near the robot
    and a corridor (rows 20..29) leaving through the right wall to col 90; beyond col 90 the
    unknown reaches the grid border."""
    g = unknown_grid(50, 120)
    g[5:45, 5:61] = FREE
    g[5, 5:61] = OCC
    g[44, 5:61] = OCC
    g[5:45, 5] = OCC
    g[5:45, 60] = OCC
    g[20:30, 60:91] = FREE
    g[19, 60:91] = OCC
    g[30, 60:91] = OCC
    g[24:27, 19:22] = UNK          # 0.09 m^2 enclosed pocket
    return g


def test_enclosed_unknown_pocket_is_ignored_open_corridor_chosen():
    g = room_with_pocket_and_exit()
    robot = xy(25, 12)
    goal = find_frontier_goal(g, RES, (0.0, 0.0), robot)
    assert goal is not None
    assert world_to_cell(goal.x, goal.y, RES, (0.0, 0.0))[1] == 90
    assert count_frontiers(g, RES, (0.0, 0.0), robot) == 1
    # Without the filter the (nearer, bigger) pocket ring wins.
    pocket = find_frontier_goal(g, RES, (0.0, 0.0), robot, min_unknown_area=0.0)
    assert pocket is not None
    assert world_to_cell(pocket.x, pocket.y, RES, (0.0, 0.0))[1] < 25
    assert count_frontiers(g, RES, (0.0, 0.0), robot, min_unknown_area=0.0) == 2


def test_large_enclosed_unknown_area_still_counts():
    g = room_with_pocket_and_exit()
    g[15:35, 25:45] = UNK          # 4 m^2 enclosed unknown block (unseen room part)
    robot = xy(25, 12)
    goal = find_frontier_goal(g, RES, (0.0, 0.0), robot)
    assert goal is not None
    assert world_to_cell(goal.x, goal.y, RES, (0.0, 0.0))[1] < 60
    assert find_frontier_goal(g, RES, (0.0, 0.0), robot, min_unknown_area=5.0).x > 8.0


# ---------------------------------------------------------------- visited memory

def test_visited_point_suppresses_cluster():
    g = corridor()
    robot = xy(14, 10)
    goal = find_frontier_goal(g, RES, (0.0, 0.0), robot)
    assert goal is not None
    assert find_frontier_goal(g, RES, (0.0, 0.0), robot, visited=[(goal.cx, goal.cy)]) is None
    assert count_frontiers(g, RES, (0.0, 0.0), robot, visited=[(goal.x, goal.y)]) == 0
    assert find_frontier_goal(g, RES, (0.0, 0.0), robot, visited=[(1.0, 1.0)]) is not None

    g = two_frontier_grid()
    robot = xy(50, 20)
    near = find_frontier_goal(g, RES, (0.0, 0.0), robot)
    other = find_frontier_goal(g, RES, (0.0, 0.0), robot, visited=[(near.x, near.y)])
    assert other is not None
    assert world_to_cell(other.x, other.y, RES, (0.0, 0.0))[1] >= 130
    assert count_frontiers(g, RES, (0.0, 0.0), robot, visited=[(near.x, near.y)]) == 1


# ---------------------------------------------------------------- hysteresis

def test_prefer_keeps_cluster_unless_alternative_much_better():
    g = two_frontier_grid()
    robot = xy(50, 20)
    near = find_frontier_goal(g, RES, (0.0, 0.0), robot)
    far = find_frontier_goal(g, RES, (0.0, 0.0), robot, size_weight=20.0)
    sn, sf = near.size * RES, far.size * RES
    dn = sn - near.score                       # score = size_m - dist with unit weights
    df = (20.0 * sf - far.score)

    def weight_for(diff):                      # size_weight making score_far - score_near = diff
        return (diff + df - dn) / (sf - sn)

    def is_far(goal):
        return world_to_cell(goal.x, goal.y, RES, (0.0, 0.0))[1] >= 130

    w = weight_for(0.5)                        # far only slightly better
    assert is_far(find_frontier_goal(g, RES, (0.0, 0.0), robot, size_weight=w))
    kept = find_frontier_goal(g, RES, (0.0, 0.0), robot, size_weight=w,
                              prefer=(near.cx, near.cy))
    assert not is_far(kept)
    best = find_frontier_goal(g, RES, (0.0, 0.0), robot, size_weight=w)
    assert 0.0 < best.score - kept.score <= 1.0

    w = weight_for(3.0)                        # far much better -> switch
    assert is_far(find_frontier_goal(g, RES, (0.0, 0.0), robot, size_weight=w,
                                     prefer=(near.cx, near.cy)))

    w = weight_for(-0.5)                       # near slightly better, pursuing far -> keep far
    assert not is_far(find_frontier_goal(g, RES, (0.0, 0.0), robot, size_weight=w))
    assert is_far(find_frontier_goal(g, RES, (0.0, 0.0), robot, size_weight=w,
                                     prefer=(far.cx + 0.5, far.cy)))
    # prefer point far from every cluster -> plain best
    assert not is_far(find_frontier_goal(g, RES, (0.0, 0.0), robot, size_weight=w,
                                         prefer=(0.0, 0.0)))


# ---------------------------------------------------------------- goal placement / yaw

def wall_gap_grid():
    """Wide corridor rows 11..29 (walls rows 10 and 30, ends closed at cols 5 and 100) with a
    gap in the top wall at cols 35..90 opening to unknown above."""
    g = unknown_grid(60, 110)
    g[10:31, 5:101] = FREE
    g[10, 5:101] = OCC
    g[30, 5:101] = OCC
    g[10:31, 5] = OCC
    g[10:31, 100] = OCC
    g[30, 35:91] = UNK
    return g


def test_yaw_faces_cluster_centroid_not_robot_heading():
    g = wall_gap_grid()
    robot = xy(15, 40)
    goal = find_frontier_goal(g, RES, (0.0, 0.0), robot)
    assert goal is not None
    r, c = world_to_cell(goal.x, goal.y, RES, (0.0, 0.0))
    assert r == 29
    assert goal.cx - goal.x > 0.2
    assert abs(goal.yaw - math.atan2(goal.cy - goal.y, goal.cx - goal.x)) < 1e-9
    assert abs(goal.yaw) < math.radians(10)      # along the gap, towards its unseen part
    heading = math.atan2(goal.y - robot[1], goal.x - robot[0])
    assert heading > math.radians(30)            # the old yaw would have pointed up-right


def test_goal_advance_moves_goal_into_cluster():
    g = wall_gap_grid()
    robot = xy(15, 40)
    base = find_frontier_goal(g, RES, (0.0, 0.0), robot, goal_advance=0.0)
    adv = find_frontier_goal(g, RES, (0.0, 0.0), robot)          # default 0.5 m
    assert base is not None and adv is not None
    assert world_to_cell(base.x, base.y, RES, (0.0, 0.0)) == (29, 40)  # straight up
    assert base.distance + 0.3 < adv.distance <= base.distance + 0.5 + 1e-9
    assert adv.score == base.score                               # scoring unaffected
    assert math.hypot(adv.x - adv.cx, adv.y - adv.cy) < math.hypot(base.x - base.cx,
                                                                   base.y - base.cy)
    r, c = world_to_cell(adv.x, adv.y, RES, (0.0, 0.0))
    safe = safe_free_mask(g, RES)
    assert safe[r, c] and (g[r - 1:r + 2, c - 1:c + 2] < 0).any()   # safe frontier cell
    assert c > 40


def l_corridor_grid():
    """L-shaped corridor: horizontal leg rows 10..29 cols 5..60, vertical leg cols 41..60
    going up from row 30 to row 90; its top end (row 91+) is unknown."""
    g = unknown_grid(100, 70)
    g[10:30, 5:61] = FREE
    g[30:91, 41:61] = FREE
    g[10, 5:61] = OCC
    g[10:30, 5] = OCC
    g[29, 5:41] = OCC
    g[10:91, 60] = OCC
    g[29:91, 41] = OCC
    return g


def test_corridor_end_yaw_follows_path_around_corner():
    g = l_corridor_grid()
    robot = xy(20, 10)
    goal = find_frontier_goal(g, RES, (0.0, 0.0), robot)
    assert goal is not None
    r, c = world_to_cell(goal.x, goal.y, RES, (0.0, 0.0))
    assert r == 90 and 44 <= c <= 57
    assert math.hypot(goal.cx - goal.x, goal.cy - goal.y) < 0.2
    # Straight-line robot->goal heading would be ~55 deg; the robot arrives going +y.
    assert abs(goal.yaw - math.pi / 2) < math.radians(15)


# ---------------------------------------------------------------- performance

def random_corridor_grid(h=600, w=500, seed=0):
    rng = np.random.default_rng(seed)
    g = unknown_grid(h, w)
    for _ in range(14):
        width = int(rng.integers(20, 40))
        if rng.random() < 0.5:
            r = int(rng.integers(10, h - width - 10))
            c0, c1 = sorted(int(v) for v in rng.integers(10, w - 10, 2))
            g[r:r + width, c0:c1] = FREE
            g[r, c0:c1] = OCC
            g[r + width - 1, c0:c1] = OCC
        else:
            c = int(rng.integers(10, w - width - 10))
            r0, r1 = sorted(int(v) for v in rng.integers(10, h - 10, 2))
            g[r0:r1, c:c + width] = FREE
            g[r0:r1, c] = OCC
            g[r0:r1, c + width - 1] = OCC
    # unknown speckle (LiDAR shadows) in free space and gaps in the walls
    walls = g == OCC
    g[(rng.random((h, w)) < 0.02) & (g == FREE)] = UNK
    g[walls & (rng.random((h, w)) < 0.1)] = UNK
    return g


def test_find_frontier_runs_fast_on_large_grid():
    g = random_corridor_grid()
    res = 0.05
    labels, _ = ndimage.label(safe_free_mask(g, res), structure=np.ones((3, 3)))
    biggest = np.argmax(np.bincount(labels.ravel())[1:]) + 1   # robot in the largest area
    rs, cs = np.nonzero(labels == biggest)
    i = len(rs) // 2
    robot = cell_to_world(int(rs[i]), int(cs[i]), res, (0.0, 0.0))
    find_frontier_goal(g, res, (0.0, 0.0), robot)               # warm-up
    t0 = time.perf_counter()
    goal = find_frontier_goal(g, res, (0.0, 0.0), robot, visited=[(1.0, 1.0)],
                              prefer=(5.0, 5.0))
    n = count_frontiers(g, res, (0.0, 0.0), robot)
    elapsed = time.perf_counter() - t0
    assert goal is not None and n >= 1
    assert elapsed < 1.5
