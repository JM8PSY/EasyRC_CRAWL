import math

import numpy as np

from cugo_web.frontier import (cell_to_world, count_frontiers, find_frontier_goal,
                               world_to_cell)

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
    assert find_frontier_goal(g, RES, (0.0, 0.0), robot, min_frontier_size=1.0) is None
    goal = find_frontier_goal(g, RES, (0.0, 0.0), robot, min_frontier_size=0.5)
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
    assert abs(goal.yaw - math.atan2(goal.y - robot[1], goal.x - robot[0])) < 1e-9
