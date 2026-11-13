"""カメラ映像を MJPEG（multipart/x-mixed-replace）でブラウザに送る。

ROS に依存しない部分だけを置く（node.py が CompressedImage を受けて FrameHub.put する）。

- FrameHub: 最新の JPEG 1 枚と版番号を持つ。ROS のスレッドから put、asyncio から wait_newer
- make_mjpeg_handler: /video.mjpg のハンドラ。fps 以下に間引いて送り続ける
  - 視聴者数（viewers）は node.py が「見ている人がいる間だけ購読する」のに使う
"""
import asyncio
import contextlib
import threading
from typing import Awaitable, Callable, Iterator, Optional, Tuple

from aiohttp import web

BOUNDARY = 'frame'
_HEAD = b'--' + BOUNDARY.encode() + b'\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n'


def mjpeg_part(jpeg: bytes) -> bytes:
    """multipart の 1 パート（区切り行・ヘッダ・JPEG 本体）。"""
    return (_HEAD % len(jpeg)) + jpeg + b'\r\n'


class FrameHub:
    """最新フレームの受け渡し（スレッドセーフ）。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._frame: Optional[bytes] = None
        self._version = 0
        self._viewers = 0
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._cond: Optional[asyncio.Condition] = None
        self._notify_task: Optional[asyncio.Task] = None

    def bind(self, loop: asyncio.AbstractEventLoop) -> None:
        """待つ側が動く asyncio のループを登録する（そのループの中から呼ぶ）。"""
        self._loop = loop
        self._cond = asyncio.Condition()

    @property
    def viewers(self) -> int:
        return self._viewers

    @contextlib.contextmanager
    def viewer(self) -> Iterator[None]:
        """視聴者数を数える。途中で例外が出ても必ず減らす。"""
        with self._lock:
            self._viewers += 1
        try:
            yield
        finally:
            with self._lock:
                self._viewers -= 1

    def put(self, jpeg: bytes) -> None:
        """新しいフレーム（どのスレッドからでも呼べる）。"""
        with self._lock:
            self._frame = jpeg
            self._version += 1
        loop = self._loop
        if loop is None:
            return
        try:
            loop.call_soon_threadsafe(self._wake)
        except RuntimeError:  # ループが閉じた後（終了処理中）
            pass

    def _wake(self) -> None:
        # Condition の通知にはロックが要るので、ループ上のタスクで行う。
        # 待つ側は最新の版を見るので、通知が溜まっていれば 1 つにまとめてよい
        if self._cond is None or self._loop is None:
            return
        if self._notify_task is None or self._notify_task.done():
            self._notify_task = self._loop.create_task(self._notify_all())

    async def _notify_all(self) -> None:
        async with self._cond:
            self._cond.notify_all()

    def _newer(self, version: int) -> Optional[Tuple[int, bytes]]:
        with self._lock:
            if self._version > version and self._frame is not None:
                return self._version, self._frame
        return None

    async def wait_newer(self, version: int, timeout: float) -> Optional[Tuple[int, bytes]]:
        """版番号が version より新しいフレームを待つ。時間切れなら None。"""
        got = self._newer(version)
        if got is not None:
            return got
        if self._cond is None:
            self.bind(asyncio.get_running_loop())
        try:
            async with self._cond:
                await asyncio.wait_for(
                    self._cond.wait_for(lambda: self._newer(version) is not None), timeout)
        except asyncio.TimeoutError:
            return None
        return self._newer(version)


def _disconnected(request: web.Request) -> bool:
    transport = request.transport
    return transport is None or transport.is_closing()


def make_mjpeg_handler(hub: FrameHub, fps: float, idle_timeout: float = 5.0
                       ) -> Callable[[web.Request], Awaitable[web.StreamResponse]]:
    """/video.mjpg のハンドラを作る。送る間隔は 1/fps 以上空ける。"""
    period = 1.0 / fps if fps > 0 else 0.0

    async def handler(request: web.Request) -> web.StreamResponse:
        resp = web.StreamResponse(headers={
            'Content-Type': 'multipart/x-mixed-replace; boundary=%s' % BOUNDARY,
            'Cache-Control': 'no-cache',
        })
        loop = asyncio.get_running_loop()
        with hub.viewer():
            try:
                await resp.prepare(request)
                version = 0
                while True:
                    got = await hub.wait_newer(version, idle_timeout)
                    if got is None:
                        # フレームが来ない（カメラが止まっている）。何も送らずに待ち続けるが、
                        # aiohttp の版や設定（handler_cancellation）によっては切断でハンドラが
                        # 止まらないので、念のためここでも切断を確かめる
                        if _disconnected(request):
                            break
                        continue
                    version, jpeg = got
                    sent = loop.time()
                    await resp.write(mjpeg_part(jpeg))
                    wait = period - (loop.time() - sent)
                    if wait > 0:
                        await asyncio.sleep(wait)
            except ConnectionResetError:
                pass  # ブラウザがタブを閉じた（CancelledError は止めずに上へ流す）
        return resp

    return handler
