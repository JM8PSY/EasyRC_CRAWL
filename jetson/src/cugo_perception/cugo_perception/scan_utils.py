"""LaserScan の値の後処理（ROS 非依存。scan_inf / scan_sectors とテストが使う）。"""
import math

INF = float('inf')
NAN = float('nan')


def sector_mask(n: int, angle_min: float, angle_inc: float, sectors_deg) -> list:
    """各ビームが ignore_sectors のどれかに入るか（True = 無視）。角度は -180〜180 度に正規化。"""
    pairs = [(float(sectors_deg[i]), float(sectors_deg[i + 1]))
             for i in range(0, len(sectors_deg) - 1, 2)]
    out = []
    for k in range(n):
        a = math.degrees(angle_min + k * angle_inc)
        a = (a + 180.0) % 360.0 - 180.0
        hit = False
        for lo, hi in pairs:
            lo = (lo + 180.0) % 360.0 - 180.0
            hi = (hi + 180.0) % 360.0 - 180.0
            hit = (lo <= a <= hi) if lo <= hi else (a >= lo or a <= hi)  # 180 度をまたぐ
            if hit:
                break
        out.append(hit)
    return out


def fix_ranges(ranges, range_min: float = 0.0, ignore=None):
    """0 以下・NaN を +inf にする。0 < r < range_min（近すぎる）は -inf（REP 117）。

    ignore[i] が True のビームは NaN（車体に遮られる方向。消しも書きもしない）。
    """
    out = []
    for i, r in enumerate(ranges):
        if ignore is not None and i < len(ignore) and ignore[i]:
            out.append(NAN)
        elif r != r or r <= 0.0:          # NaN または 0（反射なし）
            out.append(INF)
        elif r < range_min:
            out.append(-INF)
        else:
            out.append(r)
    return out


def blocked_sectors(rates, angles_deg, threshold=0.9, min_width_deg=1.0):
    """rates[i] = ビーム i が「無効 or 近すぎ」だった割合。threshold 以上が続く角度範囲を返す。"""
    out = []
    start = None
    for i, r in enumerate(list(rates) + [0.0]):
        if r >= threshold and start is None:
            start = i
        elif r < threshold and start is not None:
            a0, a1 = angles_deg[start], angles_deg[i - 1]
            if a1 - a0 >= min_width_deg - 1e-9 or i - start >= 2:
                out.append((round(a0 - 1.0, 1), round(a1 + 1.0, 1)))  # 1 度の余裕
            start = None
    # -180 と 180 をまたぐものを 1 つにまとめる
    if len(out) >= 2 and out[0][0] <= -179.0 and out[-1][1] >= 179.0:
        out = [(out[-1][0], out[0][1])] + out[1:-1]
    return out
