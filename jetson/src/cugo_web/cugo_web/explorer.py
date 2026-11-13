"""自動探索（フロンティア探索）で地図を作る。

地図の「既知の空き」と「未知」の境界（フロンティア）を選んで Nav2 の目的地にし、着いたら次を選ぶ。
フロンティアが無くなったら終了。失敗した目的地はブラックリストに入れて選ばない。

安全:
- 停止ボタン・手動操縦・手動の目的地指定で探索をやめる（node 側から stop() を呼ぶ）
- カメラが床として見たセル（node.seen_floor）は、地図で未知なら空きとして扱う（判断用の地図だけ。
  slam_toolbox / Nav2 の地図は変えない。既知のセル、特に占有は変えない）
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

評価用の記録（explog.ExploreLog）:
- log_factory（node が入れる）があれば、start() で 1 回分の記録を始め、目的地の選択・結果・終了理由を書く。
  定期の記録（姿勢・面積）は node のタイマーが self.log に書く。記録の失敗で探索は止めない
- 終了理由は explog.STOP_REASONS。目的地が無くて終わるときは、なぜ無いかの内訳（diagnose_no_goal）も残す
"""
import math
import threading
import time
from typing import Callable, List, Optional, Tuple

import numpy as np

from cugo_web import explog, frontier, keepout

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
        self.log_factory: Optional[Callable[[dict], object]] = None  # config -> ExploreLog | None
        self.log = None
        self._waiting: Optional[Tuple[str, float]] = None

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
        self._open_log()
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

    def stop(self, reason: Optional[str] = None, cause: str = 'user_stop') -> None:
        """cause は記録用の終了理由（explog.STOP_REASONS のどれか）。"""
        if self.state != 'running':
            return
        self._stop.set()
        self.state = 'off'
        self.node.get_logger().info('explore: stop (%s, %s)' % (cause, reason or 'no reason'))
        self._close_log(cause, reason or '',
                        target=list(self.target) if self.target is not None else None)
        self.node.cancel_goal()
        if reason:
            self.notify('warn', reason)

    # ---- 記録（失敗しても探索は続ける）----
    def config(self) -> dict:
        """記録用の設定（探索の定数と、境界の選び方の既定値）。"""
        return {'explorer': {'goal_timeout': GOAL_TIMEOUT,
                             'no_frontier_confirm': NO_FRONTIER_CONFIRM,
                             'check_period': CHECK_PERIOD, 'visited_radius': VISITED_RADIUS,
                             'repeat_radius': REPEAT_RADIUS, 'repeat_limit': REPEAT_LIMIT},
                'frontier': explog.frontier_defaults()}

    def _warn(self, text: str) -> None:
        log = self.node.get_logger()
        getattr(log, 'warn', log.info)(text)

    def _open_log(self) -> None:
        self.log = None
        self._waiting = None
        if self.log_factory is None:
            return
        try:
            self.log = self.log_factory(self.config())
        except Exception as exc:  # noqa: BLE001
            self._warn('explore log: %s' % exc)
        if self.log is not None:
            self.node.get_logger().info('explore log: %s' % self.log.path)

    def _ev(self, ev: str, **fields) -> None:
        log = self.log
        if log is None:
            return
        try:
            log.event(ev, **fields)
        except Exception as exc:  # noqa: BLE001
            self._warn('explore log: %s' % exc)

    def _close_log(self, reason: str, detail: str = '', **fields) -> None:
        log, self.log = self.log, None
        if log is None:
            return
        try:
            s = log.close(reason, detail, **fields)
            self.node.get_logger().info(
                'explore log: %s time=%.0fs path=%.1fm known=%.1fm2 goals=%d failures=%d'
                % (reason, s['time'], s['path'], s['known_final'], s['goals'], s['failures']))
        except Exception as exc:  # noqa: BLE001
            self._warn('explore log: %s' % exc)

    def _wait(self, what: Optional[str]) -> None:
        """地図・姿勢が無くて待つ間を記録する（変わったときだけ）。what=None で待ち終わり。"""
        now = time.monotonic()
        if what is None:
            if self._waiting is not None:
                self._ev('wait_end', what=self._waiting[0], dur=round(now - self._waiting[1], 2))
                self._waiting = None
            return
        if self._waiting is None or self._waiting[0] != what:
            self._waiting = (what, now)
            self.node.get_logger().info('explore: waiting for %s' % what)
            self._ev('wait', what=what)

    def _goal_fields(self, goal, grid, res, origin, robot) -> dict:
        """goal の記録内容。camera_only = SLAM の地図では未知だが、判断用の地図（カメラの床）では空き。"""
        used = explog.cell_value(grid, res, origin, goal.x, goal.y)
        raw = None
        msg = self.node.map_raw
        if msg is not None:
            info = msg.info
            raw_grid = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
            raw = explog.cell_value(raw_grid, info.resolution,
                                    (info.origin.position.x, info.origin.position.y),
                                    goal.x, goal.y)
        return {'x': goal.x, 'y': goal.y, 'yaw': goal.yaw, 'dist': goal.distance,
                'size': goal.size, 'size_m': goal.size * res,
                'score': getattr(goal, 'score', None),
                'cx': goal.cx, 'cy': goal.cy, 'n_cand': self.frontiers,
                'robot': [robot[0], robot[1]],
                'prefer': list(self.prefer) if self.prefer else None,
                'cell_slam': raw, 'cell_used': used,
                'camera_only': raw is not None and raw < 0 and used is not None and used >= 0,
                'blacklist_n': len(self.blacklist), 'visited_n': len(self.visited)}

    # ---- 本体 ----
    def _grid(self):
        msg = self.node.map_raw
        if msg is None:
            return None
        info = msg.info
        grid = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        origin = (info.origin.position.x, info.origin.position.y)
        seen = getattr(self.node, 'seen_floor', None)
        if seen is not None:
            # カメラが床として見た未知のセルを空きにする（既知のセルは変えない。写しを返す）。
            # 禁止エリアより先に重ねる → 禁止エリアは必ず「占有」になる
            grid = seen.overlay(grid, info.resolution, origin)
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
                    self._wait('map' if g is None else 'pose')
                    self._stop.wait(1.0)
                    continue
                self._wait(None)
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
                    self._ev('blacklist', x=goal.x, y=goal.y, cause='repeat')
                    continue
                if goal is None:
                    if moving:
                        self.node.cancel_goal()
                        moving = False
                    empty += 1
                    diag = None
                    if self.log is not None or empty >= NO_FRONTIER_CONFIRM:
                        try:  # なぜ目的地が無いか（本当に完了か、行けないだけか）
                            diag = explog.diagnose_no_goal(grid, res, origin, robot,
                                                           self.blacklist, self.visited,
                                                           candidates=self.frontiers)
                        except Exception as exc:  # noqa: BLE001
                            self._warn('explore: diagnose failed: %s' % exc)
                    self.node.get_logger().info(
                        'explore: no frontier goal (%d/%d, frontiers=%s, cause=%s)'
                        % (empty, NO_FRONTIER_CONFIRM, self.frontiers,
                           diag['cause'] if diag else '?'))
                    self._ev('no_goal', n=empty, robot=[robot[0], robot[1]], diag=diag)
                    if empty >= NO_FRONTIER_CONFIRM:
                        self.state = 'done'
                        reason = explog.stop_reason_for(diag)
                        self.node.get_logger().info('explore: done (%s, %s)' % (reason, diag))
                        self._close_log(reason, 'no frontier goal %d times' % empty, diag=diag)
                        self.frontiers = 0
                        if reason == 'completed':
                            self.notify('info', '自動探索が完了しました（未探索の境界がありません）。'
                                                '「地図保存」で保存できます')
                        else:  # 境界は残っているが行けない（障害物・Nav2 の失敗・自己位置など）
                            self.notify('warn', '自動探索を終了しました: 境界は残っていますが'
                                                '行ける目的地がありません（原因: %s）'
                                        % (diag['cause'] if diag else '不明'))
                        return
                    self._stop.wait(2.0)  # 地図の更新を待ってもう一度
                    continue
                empty = 0
                self._picked.append((goal.x, goal.y))
                self.node.get_logger().info(
                    'explore: goal (%.2f, %.2f) size=%d dist=%.2f score=%.2f'
                    % (goal.x, goal.y, goal.size, goal.distance, getattr(goal, 'score', 0.0)))
                if self.log is not None:
                    self._ev('goal', **self._goal_fields(goal, grid, res, origin, robot))
                t0 = time.monotonic()
                result = self._go(goal, res)
                self.node.get_logger().info('explore: %s after %.1fs'
                                            % (result, time.monotonic() - t0))
                if self._stop.is_set():
                    return
                if self.log is not None:
                    end = self.node.robot_pose()
                    self._ev('result', result=result, dur=round(time.monotonic() - t0, 2),
                             goal=[goal.x, goal.y],
                             blacklisted=result in ('aborted', 'rejected', 'timeout', 'skipped'),
                             robot=[end['x'], end['y']] if end else None)
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
            self._close_log('error', repr(exc))
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
