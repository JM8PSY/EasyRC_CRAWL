import math

import numpy as np

from cugo_perception.theta import (Lens, bearings, clip_to_lidar, depression_for_range,
                                   elevations, equirect_maps, estimate_height, find_lens_circles,
                                   floor_ranges, free_points, range_for_depression)

W, H = 720, 360
CAM_H = 0.5
FLOOR = np.array([150.0, 128.0, 128.0])
WALL = np.array([60.0, 140.0, 110.0])


def scene(range_fn, noise=0.0, seed=0):
    """正距円筒の合成画像: 方位 φ の床は range_fn(φ) まで、その先（上）は壁の色。"""
    e = elevations(H)[:, None]
    phi = bearings(W)[None, :]
    dep_boundary = depression_for_range(np.vectorize(range_fn)(phi), CAM_H)
    floor = (-e) >= dep_boundary  # 境界より下（俯角が大きい）が床
    img = np.where(floor[..., None], FLOOR, WALL).astype(np.float32)
    if noise:
        img += np.random.default_rng(seed).normal(0, noise, img.shape).astype(np.float32)
    return img


def col_of(phi):
    return int(np.argmin(np.abs(bearings(W) - phi)))


def test_conventions():
    b = bearings(W)
    assert abs(b[W // 2]) < 0.01                       # 中央 = 前
    assert b[W // 2 - 90] > 0                          # 中央より左の列 = 左（正）
    e = elevations(H)
    assert e[0] > 1.5 and e[-1] < -1.5


def test_corridor_ahead_is_long_sides_short():
    img = scene(lambda p: 3.0 if abs(p) < 0.3 else 1.5)
    r, ok = floor_ranges(img, CAM_H, max_range=5.0)
    assert ok
    assert abs(r[col_of(0.0)] - 3.0) < 0.3
    assert abs(r[col_of(math.pi / 2)] - 1.5) < 0.15
    assert abs(r[col_of(-math.pi / 2)] - 1.5) < 0.15


def test_open_beyond_max_range_is_capped():
    img = scene(lambda p: 50.0)
    r, ok = floor_ranges(img, CAM_H, max_range=5.0)
    assert ok and r.max() <= 5.0 and r[col_of(0.0)] > 4.0


def test_obstacle_right_next_to_robot_gives_zero():
    # 前の基準は床だが、左真横 0.65 m で床が終わる（near=0.6 のすぐ先）
    img = scene(lambda p: 0.62 if abs(p - math.pi / 2) < 0.2 else 2.0)
    r, _ = floor_ranges(img, CAM_H, near=0.6)
    assert r[col_of(math.pi / 2)] < 0.8


def test_noise_and_thin_lines_tolerated():
    img = scene(lambda p: 3.0, noise=3.0)
    rows = np.nonzero(np.abs(-elevations(H) - depression_for_range(2.0, CAM_H)) < 0.005)[0]
    img[rows, :] = WALL  # 床の目地（細い線）
    r, ok = floor_ranges(img, CAM_H, gap=3)
    assert ok and abs(r[col_of(0.0)] - 3.0) < 0.35


def test_no_reference_floor():
    img = np.tile(WALL, (H, W, 1)).astype(np.float32)  # 前も壁 → 基準の床色は「壁」になる
    img[:, :] = WALL
    valid = np.zeros((H, W), bool)                      # どの画素も使えない
    r, ok = floor_ranges(img, CAM_H, valid=valid)
    assert np.all(r == 0)


def test_range_depression_roundtrip():
    for d in (0.7, 1.0, 2.5, 4.0):
        assert abs(range_for_depression(depression_for_range(d, CAM_H), CAM_H, 10.0) - d) < 1e-9
    assert range_for_depression(-0.1, CAM_H, 5.0) == 5.0


def test_clip_to_lidar():
    tb = np.array([0.0, 0.5, 1.0])
    tr = np.array([4.0, 4.0, 4.0])
    lb = np.array([0.0, 0.5, 2.0])
    lr = np.array([np.inf, 2.0, 1.0])
    out = clip_to_lidar(tr, tb, lr, lb, margin=0.2)
    assert out[0] == 4.0                 # LiDAR は反射なし → カメラの値
    assert abs(out[1] - 1.8) < 1e-9      # LiDAR の手前で切る
    assert out[2] == 4.0                 # 近くに LiDAR の値が無い → そのまま


def test_clip_wraps_around_pi():
    out = clip_to_lidar(np.array([4.0]), np.array([math.pi - 0.01]),
                        np.array([1.0]), np.array([-math.pi + 0.01]), margin=0.0)
    assert out[0] == 1.0


def test_free_points_sector_and_spacing():
    b = np.array([0.0, math.pi / 2, math.pi])
    r = np.array([1.0, 1.0, 1.0])
    pts = free_points(r, b, near=0.6, step=0.1, sector=math.radians(100))
    assert len(pts) == 2 * 5             # 0.6,0.7,...,1.0 が 2 方位（真後ろは対象外）
    assert np.allclose(np.hypot(pts[:, 0], pts[:, 1]).max(), 1.0)
    assert free_points(np.array([0.5]), np.array([0.0])).shape == (0, 2)


def test_estimate_height():
    lr = np.linspace(0.8, 3.5, 50)
    dep = np.arctan2(0.42, lr)
    assert abs(estimate_height(dep, lr) - 0.42) < 1e-6
    assert estimate_height(dep[:5], lr[:5]) is None


def test_equirect_maps_front_and_back_lenses():
    front = Lens(320, 360, 300, fov_deg=190, axis_yaw=0.0)
    back = Lens(960, 360, 300, fov_deg=190, axis_yaw=math.pi)
    mx, my = equirect_maps([front, back], W, H)
    hr = H // 2
    # 前の水平線 → 前のレンズの中心付近、後ろの水平線 → 後ろのレンズの中心付近
    assert abs(mx[hr, col_of(0.0)] - 320) < 3 and abs(my[hr, col_of(0.0)] - 360) < 3
    assert abs(mx[hr, col_of(math.pi - 0.01)] - 960) < 3
    # 真下（天底）付近はどちらかのレンズの円の縁近く
    rr = np.hypot(mx[-1, col_of(0.0)] - 320, my[-1, col_of(0.0)] - 360)
    assert 250 < rr <= 300 + 1
    # 前方の床（下）は前のレンズの中心より下（画像の y が大きい）
    assert my[hr + 40, col_of(0.0)] > 360


def test_find_lens_circles():
    img = np.zeros((720, 1280), np.uint8)
    yy, xx = np.mgrid[:720, :1280]
    img[(xx - 320) ** 2 + (yy - 360) ** 2 < 300 ** 2] = 200
    img[(xx - 960) ** 2 + (yy - 360) ** 2 < 300 ** 2] = 200
    (c1, c2) = find_lens_circles(img)
    assert abs(c1[0] - 320) < 2 and abs(c2[0] - 960) < 2 and abs(c1[2] - 300) < 2
