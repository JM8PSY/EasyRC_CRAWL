"""指令途絶を監視し、タイムアウトで停止処理を呼ぶ。"""
from __future__ import annotations

import logging
import threading
import time
from typing import Callable

log = logging.getLogger(__name__)


class Watchdog:
    def __init__(
        self,
        timeout_s: float,
        on_timeout: Callable[[], None],
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._timeout_s = timeout_s
        self._on_timeout = on_timeout
        self._clock = clock
        self._lock = threading.Lock()
        self._last = 0.0
        self._armed = False
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def feed(self) -> None:
        with self._lock:
            self._last = self._clock()
            self._armed = True

    def check(self) -> bool:
        with self._lock:
            if not self._armed or self._clock() - self._last <= self._timeout_s:
                return False
            self._armed = False
        log.warning("指令が %.0fms 途絶えたため停止します", self._timeout_s * 1000)
        self._on_timeout()
        return True

    def start(self, interval_s: float = 0.02) -> None:
        def run() -> None:
            while not self._stop_event.wait(interval_s):
                try:
                    self.check()
                except Exception:
                    log.exception("watchdog の停止処理で例外が発生しました")

        self._thread = threading.Thread(target=run, name="watchdog", daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join()
