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


# ---- カメラが床として見たセル（seen_floor）を探索用の地図に重ねる ----

def straight_corridor():
    """x 方向にまっすぐな廊下（幅 0.9 m）。LiDAR の地図では手前 3 m だけ既知、その先は未知。
    両側の壁は 6 m 先まで既知（LiDAR は壁は見えるが、低い床の中は見えていない想定）。"""
    g = -np.ones((30, 120), np.int16)
    g[10, 5:60] = 100
    g[20, 5:60] = 100
    g[10:21, 5] = 100
    g[11:20, 6:31] = 0
    return g


class GridNode:
    zones = []

    def __init__(self, grid, seen=None):
        self.grid = grid
        if seen is not None:
            self.seen_floor = seen

    @property
    def map_raw(self):
        pos = types.SimpleNamespace(x=0.0, y=0.0)
        h, w = self.grid.shape
        info = types.SimpleNamespace(width=w, height=h, resolution=RES,
                                     origin=types.SimpleNamespace(position=pos))
        return types.SimpleNamespace(info=info, data=self.grid.ravel())


def corridor_seen(x0=3.1, x1=5.0):
    from cugo_web.seen import SeenFloor
    s = SeenFloor(cell=0.05, min_hits=3)
    xs, ys = np.meshgrid(np.arange(x0, x1, 0.05) + 0.025, np.arange(1.1, 2.0, 0.05) + 0.025)
    pts = np.column_stack([xs.ravel(), ys.ravel()])
    for _ in range(3):
        s.add(pts)
    return s


def pick(node):
    e = ex.Explorer(node, lambda level, text: None)
    grid, res, origin = e._grid()
    return ex.frontier.find_frontier_goal(grid, res, origin, (1.0, 1.5)), e


def test_seen_floor_pushes_frontier_down_the_corridor():
    without, _ = pick(GridNode(straight_corridor()))
    with_seen, e = pick(GridNode(straight_corridor(), corridor_seen()))
    assert without is not None and with_seen is not None
    assert without.x < 3.8
    assert with_seen.x > without.x + 1.5
    # 未知だけ空きにする。壁（占有）はそのまま
    grid, _, _ = e._grid()
    assert (grid[10, 5:60] == 100).all() and (grid[20, 5:60] == 100).all()
    assert (grid[11:20, 31:50] == 0).all()


def test_seen_floor_goal_explored_uses_overlay():
    node = GridNode(straight_corridor(), corridor_seen())
    e = ex.Explorer(node, lambda level, text: None)
    goal = types.SimpleNamespace(x=4.0, y=1.5)
    assert e._goal_explored(goal, RES)
    assert not ex.Explorer(GridNode(straight_corridor()), None)._goal_explored(goal, RES)


def test_keepout_overrides_seen_floor():
    node = GridNode(straight_corridor(), corridor_seen())
    node.zones = [{'pts': [(3.5, 0.5), (4.5, 0.5), (4.5, 2.5), (3.5, 2.5)]}]
    e = ex.Explorer(node, lambda level, text: None)
    grid, _, _ = e._grid()
    assert (grid[11:20, 36:45] == 100).all()
    assert (grid[11:20, 31:34] == 0).all()
