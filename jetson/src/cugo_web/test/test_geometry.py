import io
import math

import numpy as np
from PIL import Image

from cugo_web.geometry import (clamp_drive, occupancy_to_png, path_to_points, quat_from_yaw,
                               scan_to_points, valid_map_name, yaw_from_quat)


def decode(png):
    return np.asarray(Image.open(io.BytesIO(png)))


def test_occupancy_colors_and_flip():
    # 2x3（幅 3、高さ 2）。data の 0 行目 = 地図の下端
    data = [0, 100, -1,
            50, 0, 0]
    img = decode(occupancy_to_png(data, 3, 2))
    assert img.shape == (2, 3)
    # 画像の下の行 = data の 0 行目
    assert list(img[1]) == [254, 0, 205]
    assert img[0][0] == 254 - 50 * 254 // 100
    assert list(img[0][1:]) == [254, 254]


def test_yaw_quat_roundtrip():
    for yaw in (-3.0, -1.0, 0.0, 0.5, 2.9):
        assert math.isclose(yaw_from_quat(*quat_from_yaw(yaw)), yaw, abs_tol=1e-9)


def test_scan_to_points_transforms_and_filters():
    # 0° に 1m、90° に inf、180° に 範囲外
    ranges = [1.0, float('inf'), 20.0, float('nan')]
    pts = scan_to_points(ranges, 0.0, math.pi / 2, 0.1, 10.0, 2.0, 3.0, math.pi / 2)
    # センサーが (2,3) で左（+90°）を向いている → 前方 1m は (2, 4)
    assert pts == [2.0, 4.0]


def test_scan_to_points_downsamples():
    ranges = [1.0] * 1000
    pts = scan_to_points(ranges, 0.0, 0.001, 0.1, 10.0, 0, 0, 0, max_points=100)
    assert len(pts) // 2 <= 100


def test_path_keeps_last_point():
    pts = path_to_points([(i, 0) for i in range(10)], max_points=3)
    assert pts[-2:] == [9, 0]
    assert len(pts) // 2 <= 4


def test_clamp_drive():
    assert clamp_drive(1.0, -5, 0.4, 1.5) == (0.4, -1.5)
    assert clamp_drive('x', None, 0.4, 1.5) == (0.0, 0.0)
    assert clamp_drive(float('nan'), float('inf'), 0.4, 1.5) == (0.0, 0.0)
    assert clamp_drive(-0.1, 0.2, 0.4, 1.5) == (-0.1, 0.2)


def test_valid_map_name():
    assert valid_map_name('room_1-a')
    for bad in ('', '../x', 'a/b', 'あ', 'a b', None, 'x' * 65):
        assert not valid_map_name(bad)
