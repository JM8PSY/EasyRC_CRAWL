import math

from cugo_perception.scan_inf import fix_ranges


def test_zero_and_nan_become_inf():
    out = fix_ranges([0.0, float('nan'), 1.5, 0.05, 9.9], range_min=0.12)
    assert out[0] == math.inf and out[1] == math.inf
    assert out[2] == 1.5 and out[4] == 9.9
    assert out[3] == -math.inf  # 近すぎる


def test_existing_inf_kept():
    assert fix_ranges([math.inf], 0.12) == [math.inf]
