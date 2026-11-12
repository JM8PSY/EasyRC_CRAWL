import math

import numpy as np

from cugo_perception.cliff import (detect, fit_floor_plane, level_to_floor, project_to_floor,
                                  transform_points)

CAM = (0.0, 0.0, 0.165)


def grid(x0, x1, y0, y1, z, step=0.01):
    xs, ys = np.meshgrid(np.arange(x0, x1, step), np.arange(y0, y1, step))
    return np.column_stack([xs.ravel(), ys.ravel(), np.full(xs.size, z)])


def noisy(pts, sigma=0.005, seed=0):
    rng = np.random.default_rng(seed)
    out = pts.copy()
    out[:, 2] += rng.normal(0, sigma, len(pts))
    return out


def test_flat_floor_no_marks_many_clears():
    marks, clears = detect(noisy(grid(0.0, 2.5, -1.0, 1.0, 0.0)))
    assert len(marks) == 0
    assert len(clears) > 1000
    assert np.allclose(clears[:, 2], 0.1)
    assert clears[:, 0].min() >= 0.2 and clears[:, 0].max() <= 2.0 + 0.025


def test_lower_step_beyond_edge():
    floor = grid(0.3, 1.0, -0.5, 0.5, 0.0)
    step = grid(1.0, 1.8, -0.5, 0.5, -0.17)
    marks, clears = detect(noisy(np.vstack([floor, step])))
    assert len(marks) > 100
    assert marks[:, 0].min() > 1.0
    assert clears[:, 0].max() < 1.0
    assert np.allclose(marks[:, 2], 0.1)
    # 崖セルと床セルは重ならない
    cells = lambda a: set(map(tuple, np.round(a[:, :2], 3)))  # noqa: E731
    assert not cells(marks) & cells(clears)


def test_projection_moves_marks_to_floor_crossing():
    # 縁 x = 0.8。下の段（z = -0.17）は縁の陰で x ≈ 0.8 * 0.335 / 0.165 ≈ 1.62 より奥しか見えない
    floor = grid(0.3, 0.8, -0.3, 0.3, 0.0)
    step = grid(1.65, 2.6, -0.3, 0.3, -0.17)
    pts = np.vstack([floor, step])
    marks, _ = detect(pts)                      # 観測位置のまま: 縁から離れた所に印
    assert marks[:, 0].min() > 1.6
    marks_p, _ = detect(pts, camera_origin=CAM)  # 視線が床を横切る位置: 縁のすぐ奥
    assert 0.8 <= marks_p[:, 0].min() < 0.9


def test_project_to_floor_geometry():
    p = np.array([[2.0, 1.0, -0.165]])
    q = project_to_floor(p, CAM)
    assert np.allclose(q[0, :2], [1.0, 0.5])
    assert q[0, 2] == -0.165


def test_stray_low_points_ignored():
    floor = noisy(grid(0.3, 2.0, -0.5, 0.5, 0.0))
    stray = np.array([[1.02, 0.02, -0.2], [1.03, 0.03, -0.25], [1.5, 0.2, -0.3]])
    marks, _ = detect(np.vstack([floor, stray]))
    assert len(marks) == 0
    # セル内の点の過半数が床なら、低い点が min_points 以上でも中央値で崖にしない
    few = np.array([[1.01, 0.01, -0.2], [1.02, 0.01, -0.2], [1.03, 0.01, -0.2]])
    marks, _ = detect(np.vstack([floor, few]))
    assert len(marks) == 0


def test_out_of_range_ignored():
    low_far = grid(2.1, 3.0, -0.5, 0.5, -0.2)
    low_side = grid(0.5, 1.0, 1.6, 2.0, -0.2)
    low_near = grid(0.0, 0.19, -0.5, 0.5, -0.2)
    marks, clears = detect(np.vstack([low_far, low_side, low_near]))
    assert len(marks) == 0 and len(clears) == 0


def test_nans_ignored():
    floor = grid(0.3, 1.0, -0.3, 0.3, 0.0)
    bad = np.array([[np.nan, 0.0, 0.0], [1.0, np.inf, -0.2], [1.0, 0.0, np.nan]] * 50)
    marks, clears = detect(np.vstack([floor, bad]))
    assert len(marks) == 0 and len(clears) > 0
    m, c = detect(bad)
    assert m.shape == (0, 3) and c.shape == (0, 3)


def test_empty_input():
    m, c = detect(np.zeros((0, 3)))
    assert m.shape == (0, 3) and c.shape == (0, 3)


def test_obstacle_neither_mark_nor_clear():
    box = grid(1.0, 1.2, -0.1, 0.1, 0.15)
    marks, clears = detect(box)
    assert len(marks) == 0 and len(clears) == 0


def quat(yaw=0.0, pitch=0.0):
    """Rz(yaw) Ry(pitch) の四元数 (x, y, z, w)。"""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return (-sy * sp, cy * sp, sy * cp, cy * cp)


def test_transform_points_yaw():
    out = transform_points(np.array([[1.0, 0.0, 0.0]]), (0.5, 0.0, 0.2), quat(yaw=math.pi / 2))
    assert np.allclose(out, [[0.5, 1.0, 0.2]])


def test_transform_points_pitch_down():
    # pitch 正（y 軸まわり）で前方の点は下へ
    a = math.radians(4)
    out = transform_points(np.array([[1.0, 0.0, 0.0]]), (0.0, 0.0, 0.165), quat(pitch=a))
    assert np.allclose(out, [[math.cos(a), 0.0, 0.165 - math.sin(a)]])


def test_transform_points_optical_frame():
    # 光学座標（z 前、x 右、y 下）→ base（x 前、y 左、z 上）の標準回転 (-0.5, 0.5, -0.5, 0.5)
    q = (-0.5, 0.5, -0.5, 0.5)
    out = transform_points(np.array([[0.0, 0.0, 2.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
                           (0.0, 0.0, 0.0), q)
    assert np.allclose(out, [[2.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


def test_speed_60k_points():
    import time
    rng = np.random.default_rng(0)
    pts = np.column_stack([rng.uniform(0, 3, 60000), rng.uniform(-2, 2, 60000),
                           rng.normal(0, 0.01, 60000)])
    pts[pts[:, 0] > 1.5, 2] -= 0.17
    t = time.perf_counter()
    detect(pts, camera_origin=CAM)
    assert time.perf_counter() - t < 0.5


def tilt(pts, b, c):
    out = pts.copy()
    out[:, 2] += b * pts[:, 0] + c * pts[:, 1]
    return out


def test_tilted_floor_is_levelled():
    """実機で起きた例: 床が前へ 3.9 cm/m、左へ 5.4 cm/m 下がって見える → 遠くの床を崖と誤検出していた。"""
    pts = tilt(noisy(grid(0.2, 2.0, -1.0, 1.0, 0.0)), -0.039, -0.054)
    assert len(detect(pts)[0]) > 0  # 平面を当てないと誤検出する
    lv, _, (a, b, c) = level_to_floor(pts)
    assert abs(b + 0.039) < 0.005 and abs(c + 0.054) < 0.005
    marks, clears = detect(lv)
    assert len(marks) == 0 and len(clears) > 500


def test_tilted_floor_still_finds_step():
    # 縁 0.8 m。下の段は縁の陰になるので、見えるのは 0.8·(0.165+0.17)/0.165 ≈ 1.62 m より奥
    pts = np.vstack([grid(0.2, 0.8, -0.5, 0.5, 0.0), grid(1.63, 1.95, -0.5, 0.5, -0.17)])
    pts = tilt(noisy(pts), -0.03, 0.04)
    lv, origin, _ = level_to_floor(pts, CAM)
    marks, _ = detect(lv, camera_origin=origin)
    assert len(marks) > 20 and 0.75 < marks[:, 0].min() < 0.9


def test_fit_floor_plane_falls_back():
    assert fit_floor_plane(np.zeros((10, 3))) == (0.0, 0.0, 0.0)  # 点が少ない
    steep = tilt(grid(0.3, 1.2, -0.6, 0.6, 0.0), 0.3, 0.0)  # 17° は床とみなさない
    assert fit_floor_plane(steep) == (0.0, 0.0, 0.0)


def test_far_noise_needs_deeper_drop():
    """遠く（1.5 m 先）で 8 cm 低く写る床片: 固定閾値では崖、距離で深くすると崖ではない。"""
    pts = np.vstack([grid(0.3, 1.4, -0.5, 0.5, 0.0), grid(1.5, 1.7, -0.1, 0.1, -0.08)])
    assert len(detect(pts)[0]) > 0
    assert len(detect(pts, drop_per_meter=0.05)[0]) == 0
    # 本物の段（-0.17）は深くしても残る
    pts2 = np.vstack([grid(0.3, 1.4, -0.5, 0.5, 0.0), grid(1.5, 1.7, -0.3, 0.3, -0.17)])
    assert len(detect(pts2, drop_per_meter=0.05)[0]) > 0


def test_isolated_cell_removed_by_neighbor_filter():
    floor = grid(0.3, 1.5, -0.5, 0.5, 0.0)
    blob = grid(1.0, 1.04, 0.0, 0.04, -0.12, step=0.005)  # 1 セルだけ低い
    line = grid(1.2, 1.25, -0.3, 0.3, -0.15, step=0.005)  # 縁のように並んだセル
    pts = np.vstack([floor[~((np.abs(floor[:, 0] - 1.02) < 0.03) & (np.abs(floor[:, 1] - 0.02) < 0.03))
                           & ~((floor[:, 0] >= 1.2) & (floor[:, 0] < 1.25))], blob, line])
    m0 = detect(pts)[0]
    m2 = detect(pts, min_neighbors=2)[0]
    assert any(abs(x - 1.025) < 0.01 for x in m0[:, 0])
    assert not any(abs(x - 1.025) < 0.01 for x in m2[:, 0])
    assert len(m2) >= 10
