"""自動探索（フロンティア探索）で地図を作る。

地図の「既知の空き」と「未知」の境界（フロンティア）を選んで Nav2 の目的地にし、着いたら次を選ぶ。
フロンティアが無くなったら終了。失敗した目的地はブラックリストに入れて選ばない。

安全:
- 停止ボタン・手動操縦・手動の目的地指定で探索をやめる（node 側から stop() を呼ぶ）
- 進入禁止エリア（node.zones）の中は「占有」として扱い、行き先に選ばない。
  移動中の行き先が新しく描いた禁止エリアに入ったら、取り消して次を選ぶ
- skip() で今の行き先をやめて次へ（ブラックリストに入れる）

往復を防ぐ工夫（実機で 2 か所を行ったり来たりした対策）:
- visited: 着いた（または途中で周りが見えた）場所を覚え、その近くの境界は選ばない。
  行っても埋まらない未知（LiDAR の死角の影など）を何度も選び直さないため
- prefer: 追いかけていた境界のかたまりを、少し良い程度の別候補では乗り換えない（ヒステリシス）
- 同じ所が 3 回選ばれたらブラックリスト
- 途中で周りが見えたときは取り消さずに次の目的地で上書きする（止まらずに走り続ける）
- 速度は Nav2 の上限（0.3 m/s）と cugo_base / ESP32 の上限に従う
"""
import math
import threading
import time
from typing import Callable, List, Optional, Tuple

import numpy as np

from cugo_web import frontier, keepout

GOAL_TIMEOUT = 90.0       # 1 つの目的地にかける最大時間 [s]
NO_FRONTIER_CONFIRM = 3   # フロンティア無しがこの回数続いたら完了
CHECK_PERIOD = 2.0        # 移動中に「目的地付近がもう既知になったか」を確かめる周期 [s]
VISITED_RADIUS = 0.75     # 着いた場所からこの距離以内の境界は選ばない [m]
REPEAT_RADIUS = 0.5       # この距離以内の目的地は「同じ所」とみなす [m]
REPEAT_LIMIT = 3          # 同じ所がこの回数選ばれたらブラックリスト


class Explorer:
    def __init__(self, node, notify: Callable[[str, str], None]):
        self.node = node
        self.notify = notify
        self.state = 'off'          # off | running | done | failed
        self.frontiers: Optional[int] = None
        self.blacklist: List[Tuple[float, float]] = []
        self.visited: List[Tuple[float, float]] = []
        self.prefer: Optional[Tuple[float, float]] = None
        self._picked: List[Tuple[float, float]] = []
        self._stop = threading.Event()
        self._skip = threading.Event()
        self.target: Optional[Tuple[float, float]] = None
        self._thread: Optional[threading.Thread] = None

    # ---- 操作 ----
    def start(self) -> None:
        if self.state == 'running':
            return
        if not self.node.nav_client.server_is_ready():
            self.notify('error', 'Nav2 が起動していません')
            return
        if self.node.map_raw is None:
            self.notify('error', '地図がまだありません（mode:=slam で起動してください）')
            return
        self._stop.clear()
        self.blacklist = []
        self.visited = []
        self.prefer = None
        self._picked = []
        self.state = 'running'
        self.notify('info', '自動探索を開始しました')
        self.node.get_logger().info('explore: start')
        self._thread = threading.Thread(target=self._run, name='explorer', daemon=True)
        self._thread.start()

    def forget(self) -> None:
        """地図リセット時: 覚えた場所（座標が変わるので無効）を消す。"""
        self.blacklist = []
        self.visited = []
        self.prefer = None
        self._picked = []

    def skip(self) -> None:
        if self.state == 'running':
            self._skip.set()

    def stop(self, reason: Optional[str] = None) -> None:
        if self.state != 'running':
            return
        self._stop.set()
        self.state = 'off'
        self.node.get_logger().info('explore: stop (%s)' % (reason or 'no reason'))
        self.node.cancel_goal()
        if reason:
            self.notify('warn', reason)

    # ---- 本体 ----
    def _grid(self):
        msg = self.node.map_raw
        if msg is None:
            return None
        info = msg.info
        grid = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        origin = (info.origin.position.x, info.origin.position.y)
        zones = list(self.node.zones)
        if zones:
            # 禁止エリアは「占有」にする → その中と、そこに接する境界は選ばれない
            grid = grid.copy()
            grid[keepout.rasterize(zones, info.width, info.height, info.resolution, origin)] = 100
        return grid, info.resolution, origin

    def _run(self) -> None:
        empty = 0
        moving = False  # 前の目的地を取り消さずに走っている（'explored' のあと）
        try:
            while not self._stop.is_set():
                g = self._grid()
                pose = self.node.robot_pose()
                if g is None or pose is None:
                    self._stop.wait(1.0)
                    continue
                grid, res, origin = g
                robot = (pose['x'], pose['y'])
                goal = frontier.find_frontier_goal(grid, res, origin, robot, self.blacklist,
                                                   visited=self.visited, prefer=self.prefer)
                self.frontiers = frontier.count_frontiers(grid, res, origin, robot, self.blacklist,
                                                          visited=self.visited)
                if goal is not None and self._repeated(goal):
                    self.blacklist.append((goal.x, goal.y))
                    self.node.get_logger().info('explore: (%.2f, %.2f) picked %d times -> blacklist'
                                                % (goal.x, goal.y, REPEAT_LIMIT))
                    continue
                if goal is None:
                    if moving:
                        self.node.cancel_goal()
                        moving = False
                    empty += 1
                    self.node.get_logger().info('explore: no frontier goal (%d/%d, frontiers=%s)'
                                                % (empty, NO_FRONTIER_CONFIRM, self.frontiers))
                    if empty >= NO_FRONTIER_CONFIRM:
                        self.state = 'done'
                        self.node.get_logger().info('explore: done')
                        self.frontiers = 0
                        self.notify('info', '自動探索が完了しました（未探索の境界がありません）。'
                                            '「地図保存」で保存できます')
                        return
                    self._stop.wait(2.0)  # 地図の更新を待ってもう一度
                    continue
                empty = 0
                self._picked.append((goal.x, goal.y))
                self.node.get_logger().info(
                    'explore: goal (%.2f, %.2f) size=%d dist=%.2f score=%.2f'
                    % (goal.x, goal.y, goal.size, goal.distance, getattr(goal, 'score', 0.0)))
                t0 = time.monotonic()
                result = self._go(goal, res)
                self.node.get_logger().info('explore: %s after %.1fs'
                                            % (result, time.monotonic() - t0))
                if self._stop.is_set():
                    return
                moving = result == 'explored'
                if result in ('succeeded', 'explored'):
                    self.visited.append((goal.x, goal.y))
                    self.prefer = (goal.cx, goal.cy)
                else:
                    self.prefer = None
                if result in ('aborted', 'rejected', 'timeout', 'skipped'):
                    self.blacklist.append((goal.x, goal.y))
                    self.node.get_logger().info('explore: %s -> blacklist (%.2f, %.2f)'
                                                % (result, goal.x, goal.y))
                if not moving:
                    self._stop.wait(0.5)
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().error('explorer: %s' % exc)
            self.state = 'failed'
            self.notify('error', '自動探索でエラー: %s' % exc)

    def _repeated(self, goal) -> bool:
        r2 = REPEAT_RADIUS * REPEAT_RADIUS
        n = sum(1 for x, y in self._picked if (x - goal.x) ** 2 + (y - goal.y) ** 2 <= r2)
        return n >= REPEAT_LIMIT - 1  # これで REPEAT_LIMIT 回目

    def _go(self, goal, res) -> str:
        """目的地へ送り、終わるまで待つ。succeeded / explored / aborted / rejected / timeout / stopped"""
        self._skip.clear()
        self.target = (goal.x, goal.y)
        handle = self.node.navigate(goal.x, goal.y, goal.yaw)
        if handle is None:
            return 'rejected'
        start = time.monotonic()
        last_check = start
        while not self._stop.is_set():
            status = self.node.goal_status(handle)
            if status is not None:
                return status
            if self._skip.is_set() or self._in_keepout(goal):
                self.node.cancel_goal()
                self.notify('info', '行き先をスキップして次を探します')
                return 'skipped'
            now = time.monotonic()
            if now - start > GOAL_TIMEOUT:
                self.node.cancel_goal()
                return 'timeout'
            if now - last_check > CHECK_PERIOD:
                last_check = now
                if self._goal_explored(goal, res):
                    # もう周りが見えたので次へ（失敗ではない）。取り消さずに次の目的地で上書きする
                    # （取り消すと一度止まる）。次が無ければ _run が取り消す
                    return 'explored'
            self._stop.wait(0.2)
        return 'stopped'

    def _in_keepout(self, goal) -> bool:
        x, y = np.array([goal.x]), np.array([goal.y])
        return any(bool(keepout.points_in_polygon(x, y, z['pts'])[0]) for z in self.node.zones)

    def _goal_explored(self, goal, res, radius=0.3) -> bool:
        g = self._grid()
        if g is None:
            return False
        grid, res, (ox, oy) = g
        h, w = grid.shape
        c = int((goal.x - ox) / res)
        r = int((goal.y - oy) / res)
        k = max(1, int(math.ceil(radius / res)))
        r0, r1, c0, c1 = max(0, r - k), min(h, r + k + 1), max(0, c - k), min(w, c + k + 1)
        if r0 >= r1 or c0 >= c1:
            return False
        return not bool((grid[r0:r1, c0:c1] < 0).any())
