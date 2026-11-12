import math

from cugo_perception.scan_utils import blocked_sectors, fix_ranges, sector_mask


def test_zero_and_nan_become_inf():
    out = fix_ranges([0.0, float('nan'), 1.5, 0.05, 9.9], range_min=0.12)
    assert out[0] == math.inf and out[1] == math.inf
    assert out[2] == 1.5 and out[4] == 9.9
    assert out[3] == -math.inf  # 近すぎる


def test_existing_inf_kept():
    assert fix_ranges([math.inf], 0.12) == [math.inf]


def test_ignored_sector_becomes_nan():
    ranges = [0.0, 0.0, 2.0, 0.0]
    ignore = sector_mask(4, math.radians(-180), math.radians(90), [-185.0, -175.0, 85.0, 95.0])
    assert ignore == [True, False, False, True]
    out = fix_ranges(ranges, 0.12, ignore)
    assert math.isnan(out[0]) and out[1] == math.inf and out[2] == 2.0 and math.isnan(out[3])


def test_sector_wrapping_180():
    ignore = sector_mask(8, math.radians(-180), math.radians(45), [170.0, -170.0])
    assert ignore[0] is True and sum(ignore) == 1


def test_blocked_sectors():
    angles = [float(a) for a in range(-180, 180)]
    rates = [0.0] * 360
    for a in range(100, 110):
        rates[a + 180] = 1.0
    assert blocked_sectors(rates, angles) == [(99.0, 110.0)]
