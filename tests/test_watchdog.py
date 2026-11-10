import threading

from cugo_rc.watchdog import Watchdog


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def test_no_fire_before_first_feed():
    clk, calls = FakeClock(), []
    w = Watchdog(0.3, lambda: calls.append(1), clock=clk)
    clk.t = 10
    assert w.check() is False and calls == []


def test_fires_after_timeout_once():
    clk, calls = FakeClock(), []
    w = Watchdog(0.3, lambda: calls.append(1), clock=clk)
    w.feed()
    clk.t = 0.29
    assert w.check() is False
    clk.t = 0.31
    assert w.check() is True
    clk.t = 1.0
    assert w.check() is False
    assert calls == [1]


def test_feed_resets():
    clk, calls = FakeClock(), []
    w = Watchdog(0.3, lambda: calls.append(1), clock=clk)
    w.feed()
    clk.t = 0.2
    w.feed()
    clk.t = 0.45
    assert w.check() is False


def test_thread_fires_with_real_clock():
    ev = threading.Event()
    w = Watchdog(0.05, ev.set)
    w.start(0.01)
    w.feed()
    assert ev.wait(1.0)
    w.shutdown()


def test_failed_stop_is_retried():
    clk, calls = FakeClock(), []

    def on_timeout():
        calls.append(1)
        if len(calls) == 1:
            raise RuntimeError("pigpio glitch")

    w = Watchdog(0.3, on_timeout, clock=clk)
    w.feed()
    clk.t = 0.31
    assert w.check() is False
    assert w.check() is True
    assert calls == [1, 1]
