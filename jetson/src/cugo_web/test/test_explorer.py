"""Explorer の往復防止（偽のロボット + ガラス入りの L 字廊下 + 仮想 LiDAR）。

実機で起きた例: ガラス（通れないが LiDAR は反射なし）の向こうは未知のまま残るので、
古い選び方は 2 つのガラスの前を延々と往復し、探索が終わらなかった。
"""
import math
import time
import types

import numpy as np

import cugo_web.explorer as ex

RES = 0.1
W = H = 120  # 12 m 四方


def make_world():
    occ = np.ones((H, W), bool)
    occ[10:30, 10:100] = False     # 横の廊下
    occ[10:110, 80:100] = False    # 縦の廊下
    glass = np.zeros((H, W), bool)
    glass[9, 30:60] = True         # 横の廊下の下の壁がガラス
    glass[40:80, 100] = True       # 縦の廊下の右の壁がガラス
    return occ | glass, glass


OCC, GLASS = make_world()


def scan(known, x, y, rng=6.0):
    for k in range(0, 360, 2):
        th = math.radians(k)
        for s in np.arange(0, rng, RES / 2):
            c, r = int((x + s * math.cos(th)) / RES), int((y + s * math.sin(th)) / RES)
            if not (0 <= r < H and 0 <= c < W) or GLASS[r, c]:
                break
            if OCC[r, c]:
                known[r, c] = 100
                break
            known[r, c] = 0


class FakeNode:
    zones = []

    def __init__(self):
        self.known = -np.ones((H, W), np.int16)
        self.pose = [2.0, 2.0]
        scan(self.known, *self.pose)
        self.nav_client = types.SimpleNamespace(server_is_ready=lambda: True)
        self.goals = []

    @property
    def map_raw(self):
        pos = types.SimpleNamespace(x=0.0, y=0.0)
        info = types.SimpleNamespace(width=W, height=H, resolution=RES,
                                     origin=types.SimpleNamespace(position=pos))
        return types.SimpleNamespace(info=info, data=self.known.ravel())

    def robot_pose(self):
        return {'x': self.pose[0], 'y': self.pose[1], 'yaw': 0.0}

    def navigate(self, x, y, yaw, on_result=None):
        self.goals.append((x, y))
        x0, y0 = self.pose
        n = max(1, int(math.hypot(x - x0, y - y0)))
        for k in range(1, n + 1):
            scan(self.known, x0 + (x - x0) * k / n, y0 + (y - y0) * k / n)
        self.pose = [x, y]
        return {'status': 'succeeded'}

    goal_status = staticmethod(lambda t: t['status'])

    def cancel_goal(self):
        pass

    def get_logger(self):
        return types.SimpleNamespace(info=lambda m: None, error=lambda m: None)


def test_explores_l_corridor_with_glass_and_finishes(monkeypatch):
    monkeypatch.setattr(ex, 'CHECK_PERIOD', 0.0)
    node = FakeNode()
    e = ex.Explorer(node, lambda level, text: None)
    e.start()
    end = time.monotonic() + 30
    while e.state == 'running' and time.monotonic() < end:
        time.sleep(0.05)
    e.stop()
    assert e.state == 'done'
    assert len(node.goals) <= 12
    # 同じ所（1 m 以内）に 3 回以上行っていない
    for g in node.goals:
        assert sum(1 for h in node.goals if math.dist(g, h) < 1.0) <= 2
    free = ~OCC
    assert ((node.known == 0) & free).sum() / free.sum() > 0.95
