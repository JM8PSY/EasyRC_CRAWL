import time

import pytest
from conftest import dead_opener, wait_until
from fastapi.testclient import TestClient

from cugo_rc.camera import Camera
from cugo_rc.config import CameraConfig, MotorConfig, default_config
from cugo_rc.motor import MockBackend, MotorController
from cugo_rc.server import TAKEOVER_S, create_app, make_uvicorn_config


@pytest.fixture
def backend():
    return MockBackend()


@pytest.fixture
def motor(backend):
    return MotorController(MotorConfig(), backend)


@pytest.fixture
def app(motor):
    cam = Camera(CameraConfig(width=160, height=120), mock=True)
    return create_app(default_config(), motor, cam)


@pytest.fixture
def client(app):
    with TestClient(app) as c:
        yield c


def test_index(client):
    assert "CuGo RC" in client.get("/").text


def test_driver_command_sets_target(client, motor):
    with client.websocket_connect("/ws") as ws:
        assert ws.receive_json() == {"role": "driver"}
        ws.send_json({"l": 0.5, "r": -0.5})
        assert wait_until(lambda: motor.target == (0.5, -0.5))


def test_second_connection_is_viewer_and_ignored(client, motor):
    with client.websocket_connect("/ws") as d:
        d.receive_json()
        with client.websocket_connect("/ws") as v:
            assert v.receive_json() == {"role": "viewer"}
            v.send_json({"l": 1, "r": 1})
            d.send_json({"l": 0.1, "r": 0.1})
            assert wait_until(lambda: motor.target == (0.1, 0.1))
            time.sleep(0.1)
            assert motor.target == (0.1, 0.1)


def test_stop_message_is_immediate(client, motor, backend):
    with client.websocket_connect("/ws") as ws:
        ws.receive_json()
        ws.send_json({"l": 1, "r": 1})
        assert wait_until(lambda: motor.current[0] > 0.1)
        ws.send_json({"stop": True})
        assert wait_until(lambda: backend.duty == {12: 0.0, 13: 0.0} and motor.target == (0, 0))


def test_disconnect_stops_and_releases(client, motor):
    with client.websocket_connect("/ws") as ws:
        ws.receive_json()
        ws.send_json({"l": 1, "r": 1})
        assert wait_until(lambda: motor.target == (1, 1))
    assert wait_until(lambda: motor.target == (0, 0))
    assert wait_until(lambda: client.get("/status").json()["driver_connected"] is False)
    with client.websocket_connect("/ws") as ws2:
        assert ws2.receive_json() == {"role": "driver"}


@pytest.mark.parametrize(
    "bad", ['{"l": NaN, "r": 0}', '{"l": true, "r": 0}', '{"l": "1", "r": 0}', "xx", '{"l": 1}', "[1, 2]"]
)
def test_invalid_messages_ignored(client, motor, bad):
    with client.websocket_connect("/ws") as ws:
        ws.receive_json()
        ws.send_text(bad)
        ws.send_json({"l": 0.2, "r": 0.2})  # 接続が生きていることの確認
        assert wait_until(lambda: motor.target == (0.2, 0.2))


def test_watchdog_stops_when_commands_cease(client, motor):
    with client.websocket_connect("/ws") as ws:
        ws.receive_json()
        ws.send_json({"l": 1, "r": 1})
        assert wait_until(lambda: motor.target == (1, 1))
        assert wait_until(lambda: motor.target == (0, 0), timeout=1.0)  # 300ms 後


def test_status_reports_camera_down(motor):
    cam = Camera(CameraConfig(), opener=dead_opener, retry_s=0.05)
    with TestClient(create_app(default_config(), motor, cam)) as c:
        assert c.get("/status").json()["camera"] is False
        with c.websocket_connect("/ws") as ws:  # カメラ無しでも操縦可
            assert ws.receive_json() == {"role": "driver"}


def test_shutdown_closes_motor(app, backend):
    with TestClient(app):
        pass
    assert backend.closed


class FlakyBackend(MockBackend):
    """set_pwm を1回だけ失敗させられるバックエンド（pigpio の一時的なエラー相当）。"""

    def __init__(self):
        super().__init__()
        self.fail_next = False

    def set_pwm(self, pin, freq, duty):
        if self.fail_next:
            self.fail_next = False
            raise RuntimeError("pigpio glitch")
        super().set_pwm(pin, freq, duty)


def test_control_loop_survives_tick_error_and_stops():
    backend = FlakyBackend()
    motor = MotorController(MotorConfig(), backend)
    cam = Camera(CameraConfig(width=160, height=120), mock=True)
    with TestClient(create_app(default_config(), motor, cam)) as c:
        with c.websocket_connect("/ws") as ws:
            ws.receive_json()
            ws.send_json({"l": 1, "r": 1})
            assert wait_until(lambda: motor.current[0] > 0.1)
            backend.fail_next = True
            assert wait_until(lambda: motor.current == (0.0, 0.0) and backend.duty == {12: 0.0, 13: 0.0})
            for _ in range(5):  # 制御ループが生きていれば再び加速する
                ws.send_json({"l": 0.3, "r": 0.3})
                time.sleep(0.05)
            assert wait_until(lambda: motor.current[0] > 0.1)


def test_stale_driver_is_taken_over(client, motor):
    with client.websocket_connect("/ws") as old:
        old.receive_json()
        old.send_json({"l": 1, "r": 1})
        assert wait_until(lambda: motor.target == (1, 1))
        time.sleep(TAKEOVER_S + 0.2)  # 旧操縦者は無言（半開きの接続を想定）
        with client.websocket_connect("/ws") as new:
            assert new.receive_json() == {"role": "driver"}
            new.send_json({"l": 0.4, "r": 0.4})
            assert wait_until(lambda: motor.target == (0.4, 0.4))


def test_shutdown_completes_with_open_video_stream(app, backend):
    import socket
    import threading

    import uvicorn

    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    ucfg = make_uvicorn_config(app, default_config())
    ucfg.host, ucfg.port = "127.0.0.1", port
    server = uvicorn.Server(ucfg)
    t = threading.Thread(target=server.run, daemon=True)
    t.start()
    assert wait_until(lambda: server.started, timeout=10)
    viewer = socket.create_connection(("127.0.0.1", port))
    viewer.sendall(b"GET /video HTTP/1.1\r\nHost: x\r\n\r\n")
    assert b"--frame" in viewer.recv(4096) + viewer.recv(4096)
    server.should_exit = True  # Ctrl+C / SIGTERM 相当
    t.join(timeout=10)
    viewer.close()
    assert not t.is_alive()
    assert backend.closed
