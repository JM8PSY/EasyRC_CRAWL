import asyncio

from conftest import DeadCapture, wait_until

from cugo_rc.camera import Camera
from cugo_rc.config import CameraConfig


def test_mock_produces_jpeg():
    c = Camera(CameraConfig(width=160, height=120), mock=True)
    c.start()
    assert wait_until(lambda: c.latest_jpeg() is not None)
    assert c.latest_jpeg()[:2] == b"\xff\xd8" and c.ok
    c.shutdown()


def test_unavailable_camera_sets_not_ok_and_retries():
    opens = []

    def opener():
        opens.append(1)
        return DeadCapture()

    c = Camera(CameraConfig(), opener=opener, retry_s=0.05)
    c.start()
    assert wait_until(lambda: len(opens) >= 2)
    assert c.ok is False and c.latest_jpeg() is None
    c.shutdown()


def test_mjpeg_chunk_format():
    c = Camera(CameraConfig(width=160, height=120), mock=True)
    c.start()
    chunk = asyncio.run(anext(c.mjpeg()))
    assert chunk.startswith(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n\xff\xd8")
    c.shutdown()
