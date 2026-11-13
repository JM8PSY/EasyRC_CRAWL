import asyncio
import threading
import time

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from cugo_web.video import FrameHub, make_mjpeg_handler, mjpeg_part

PART_HEAD = b'--frame\r\nContent-Type: image/jpeg'


def test_mjpeg_part_format():
    p = mjpeg_part(b'\xff\xd8abc')
    assert p.startswith(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 5\r\n\r\n')
    assert p.endswith(b'\xff\xd8abc\r\n')


def test_wait_newer_returns_frame_put_from_other_thread():
    async def main():
        hub = FrameHub()
        hub.bind(asyncio.get_running_loop())
        threading.Timer(0.05, hub.put, args=(b'J1',)).start()
        return await hub.wait_newer(0, timeout=1.0)
    v, data = asyncio.run(main())
    assert v == 1 and data == b'J1'


def test_wait_newer_times_out_without_frames():          # Review Focus 2
    async def main():
        hub = FrameHub()
        hub.bind(asyncio.get_running_loop())
        return await hub.wait_newer(0, timeout=0.1)
    assert asyncio.run(main()) is None


# ---------------- ここから HTTP 経由のテスト ----------------
def _app(hub, fps):
    app = web.Application()
    app.router.add_get('/video.mjpg', make_mjpeg_handler(hub, fps, idle_timeout=0.2))
    return app


def _producer(hub, period, stop, duration=None):
    """別スレッドから put する（ROS のコールバックと同じ状況）。"""
    t0 = time.monotonic()
    i = 0

    def run():
        nonlocal i
        while not stop.is_set():
            if duration is not None and time.monotonic() - t0 >= duration:
                break
            i += 1
            hub.put(b'JPEG%d' % i)
            time.sleep(period)
    th = threading.Thread(target=run, daemon=True)
    th.start()
    return th


async def _read_parts(resp, n, timeout=2.0):
    """パートを n 個受け取るまで読む。"""
    buf = b''
    while buf.count(PART_HEAD) < n:
        chunk = await asyncio.wait_for(resp.content.readany(), timeout)
        if not chunk:
            break
        buf += chunk
    return buf


async def _wait_viewers(hub, n, timeout=0.5):
    deadline = time.monotonic() + timeout
    while hub.viewers != n and time.monotonic() < deadline:
        await asyncio.sleep(0.01)
    return hub.viewers


def test_stream_sends_parts_and_viewer_count_returns_to_zero():   # Review Focus 1
    async def main():
        hub = FrameHub()
        hub.bind(asyncio.get_running_loop())
        stop = threading.Event()
        _producer(hub, 0.02, stop)
        try:
            async with TestClient(TestServer(_app(hub, 20.0))) as client:
                resp = await client.get('/video.mjpg')
                assert resp.status == 200
                assert resp.headers['Content-Type'] == 'multipart/x-mixed-replace; boundary=frame'
                assert resp.headers['Cache-Control'] == 'no-cache'
                assert hub.viewers == 1
                buf = await _read_parts(resp, 2)
                assert buf.count(PART_HEAD) >= 2
                resp.close()  # 切断
                return await _wait_viewers(hub, 0, timeout=0.5)
        finally:
            stop.set()
    assert asyncio.run(main()) == 0


def test_viewer_count_returns_to_zero_even_without_frames():      # Review Focus 1 + 2
    # カメラが止まっていてフレームが来ないまま切断しても、配信処理は終わる
    async def main():
        hub = FrameHub()
        hub.bind(asyncio.get_running_loop())
        async with TestClient(TestServer(_app(hub, 10.0))) as client:
            resp = await client.get('/video.mjpg')
            assert resp.status == 200
            assert await _wait_viewers(hub, 1) == 1
            resp.close()
            # 切断でハンドラが止まらない場合も idle_timeout=0.2 秒ごとに切断を確かめるので、1 秒で十分
            return await _wait_viewers(hub, 0, timeout=1.0)
    assert asyncio.run(main()) == 0


def test_two_viewers_both_receive_and_one_leaving_keeps_other():  # Review Focus 3
    async def main():
        hub = FrameHub()
        hub.bind(asyncio.get_running_loop())
        stop = threading.Event()
        _producer(hub, 0.02, stop)
        try:
            async with TestClient(TestServer(_app(hub, 20.0))) as client:
                a = await client.get('/video.mjpg')
                b = await client.get('/video.mjpg')
                assert await _wait_viewers(hub, 2) == 2
                assert (await _read_parts(a, 2)).count(PART_HEAD) >= 2
                assert (await _read_parts(b, 2)).count(PART_HEAD) >= 2
                a.close()  # 1 台が切れる
                assert await _wait_viewers(hub, 1) == 1
                # 残った方には引き続き届く
                assert (await _read_parts(b, 3)).count(PART_HEAD) >= 3
                b.close()
                return await _wait_viewers(hub, 0)
        finally:
            stop.set()
    assert asyncio.run(main()) == 0


def test_fps_limit():
    # put を 100 Hz で 1 秒間行い、fps=5 のストリームで受け取ったパート数が 4〜6 に収まる
    async def main():
        hub = FrameHub()
        hub.bind(asyncio.get_running_loop())
        stop = threading.Event()
        async with TestClient(TestServer(_app(hub, 5.0))) as client:
            resp = await client.get('/video.mjpg')
            assert await _wait_viewers(hub, 1) == 1
            th = _producer(hub, 0.01, stop, duration=1.0)
            buf = b''
            deadline = time.monotonic() + 1.0
            try:
                while time.monotonic() < deadline:
                    try:
                        chunk = await asyncio.wait_for(resp.content.readany(),
                                                       deadline - time.monotonic())
                    except asyncio.TimeoutError:
                        break
                    if not chunk:
                        break
                    buf += chunk
            finally:
                stop.set()
                th.join()
                resp.close()
            return buf.count(PART_HEAD)
    n = asyncio.run(main())
    assert 4 <= n <= 6, n


def test_close_ends_open_streams_quickly():
    """サーバー終了時: hub.close() で開いたままの配信が終わる（aiohttp の 60 秒待ちにしない）。"""
    async def main():
        hub = FrameHub()
        hub.bind(asyncio.get_running_loop())
        app = web.Application()
        app.router.add_get('/video.mjpg', make_mjpeg_handler(hub, 10.0, idle_timeout=5.0))
        async with TestClient(TestServer(app)) as client:
            resp = await client.get('/video.mjpg')
            assert await _wait_viewers(hub, 1) == 1
            t = time.monotonic()
            hub.close()
            n = await _wait_viewers(hub, 0, timeout=1.0)
            resp.close()
            return n, time.monotonic() - t
    n, dt = asyncio.run(main())
    assert n == 0 and dt < 1.0


def test_wait_newer_returns_none_after_close():
    async def main():
        hub = FrameHub()
        hub.bind(asyncio.get_running_loop())
        asyncio.get_running_loop().call_later(0.05, hub.close)
        t = time.monotonic()
        got = await hub.wait_newer(0, timeout=5.0)
        return got, time.monotonic() - t, hub.closed
    got, dt, closed = asyncio.run(main())
    assert got is None and dt < 1.0 and closed
