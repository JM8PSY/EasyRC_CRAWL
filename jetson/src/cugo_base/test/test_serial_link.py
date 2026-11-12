"""SerialLink の無応答リセット（pty を使う。実デバイスは開かない）。"""
import os
import time
import tty

import pytest

pytest.importorskip('serial')

from cugo_base.serial_link import SerialLink  # noqa: E402


def make_pty():
    master, slave = os.openpty()
    tty.setraw(slave)
    return master, slave, os.ttyname(slave)


def wait_for(cond, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


def test_silent_device_is_reset_then_reopened():
    master, slave, path = make_pty()
    events = []
    link = SerialLink(path, 115200, lambda _l: None, reconnect_interval=0.1,
                      on_event=events.append, silence_timeout=0.3)
    try:
        link.start()
        # 何も送らない → 0.3s ごとにリセット 2 回 → 閉じて開き直す
        assert wait_for(lambda: link.reset_count >= 2, 3.0)
        assert wait_for(lambda: link.connect_count >= 2, 3.0)
        assert any('リセット' in e for e in events)
        assert any(e.startswith('disconnected') for e in events)
    finally:
        link.stop()
        os.close(master)
        os.close(slave)


def test_garbage_only_device_is_reset():
    """ブート ROM の表示やゴミだけ流れていてもプロトコル行が無ければリセットする（実機で発生）。"""
    master, slave, path = make_pty()
    link = SerialLink(path, 115200, lambda _l: None, reconnect_interval=0.1,
                      silence_timeout=0.3)
    try:
        link.start()
        end = time.monotonic() + 1.2
        while time.monotonic() < end:
            os.write(master, b'ets Jul 29 2019 12:21:46\r\n\x00p\x8cpp|p\x00\n')
            time.sleep(0.05)
        assert link.reset_count >= 1
    finally:
        link.stop()
        os.close(master)
        os.close(slave)


def test_talking_device_is_not_reset():
    master, slave, path = make_pty()
    lines = []
    link = SerialLink(path, 115200, lines.append, reconnect_interval=0.1,
                      silence_timeout=0.3)
    try:
        link.start()
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            os.write(master, b'$T,A,0.000,0.000,0.50,5,0*00\n')
            time.sleep(0.05)
        assert link.reset_count == 0
        assert link.connect_count == 1
        assert lines
    finally:
        link.stop()
        os.close(master)
        os.close(slave)


def test_disabled_when_zero():
    master, slave, path = make_pty()
    link = SerialLink(path, 115200, lambda _l: None, reconnect_interval=0.1,
                      silence_timeout=0.0)
    try:
        link.start()
        time.sleep(0.6)
        assert link.reset_count == 0
        assert link.connect_count == 1
    finally:
        link.stop()
        os.close(master)
        os.close(slave)
