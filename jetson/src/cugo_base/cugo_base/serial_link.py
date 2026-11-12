"""USB シリアルの接続管理（ROS 非依存）。

バックグラウンドスレッドで開く・読む・切断時に再接続する（既定 1 秒間隔）。
受信した行は ``on_line`` コールバックに渡す。例外で落ちることはない。

無応答の検出（silence_timeout 秒、0 で無効）: 開いているのに 1 バイトも届かないときは、
USB 抜き差し・再起動の直後に ESP32 が正常に起動できていないことがある（実機で確認）。
RTS で EN を Low に 0.1 秒（esptool の hard reset と同じ）してリセットする。
リセットしても 2 回続けて無応答なら、閉じて開き直す。リセット中は MD20A への出力も 0 になる。
"""
from __future__ import annotations

import threading
import time
from typing import Callable, Optional

import serial

from cugo_base.protocol import LineAssembler


class SerialLink:
    def __init__(self, port: str, baud: int, on_line: Callable[[str], None],
                 reconnect_interval: float = 1.0,
                 on_event: Optional[Callable[[str], None]] = None,
                 silence_timeout: float = 3.0) -> None:
        self.port = port
        self.baud = baud
        self._on_line = on_line
        self._on_event = on_event or (lambda _msg: None)
        self._interval = reconnect_interval
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._asm = LineAssembler()
        self._thread = threading.Thread(target=self._run, name='cugo_serial', daemon=True)
        self.last_error = ''
        self.connect_count = 0
        self.write_errors = 0
        self.silence_timeout = silence_timeout
        self.reset_count = 0
        self._last_rx = 0.0
        self._silent_resets = 0

    # ---- public ----

    @property
    def connected(self) -> bool:
        return self._ser is not None

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._close()

    def write(self, data: bytes) -> bool:
        """書き込む。未接続なら False。エラー時は切断して再接続に任せる。"""
        with self._lock:
            ser = self._ser
            if ser is None:
                return False
            try:
                ser.write(data)
                return True
            except (serial.SerialException, OSError) as exc:
                self.write_errors += 1
                self.last_error = 'write: %s' % exc
        self._close()
        return False

    # ---- thread ----

    def _run(self) -> None:
        while not self._stop.is_set():
            if self._ser is None and not self._open():
                self._stop.wait(self._interval)
                continue
            self._read_once()
            self._check_silence()

    def _check_silence(self) -> None:
        if self.silence_timeout <= 0 or self._ser is None:
            return
        if time.monotonic() - self._last_rx < self.silence_timeout:
            return
        if self._silent_resets >= 2:
            self.last_error = 'ESP32 から %d 回リセットしても応答なし' % self._silent_resets
            self._close()
            return
        self._silent_resets += 1
        self.reset_count += 1
        self._on_event('ESP32 から %.0f 秒応答がないのでリセットします（%d 回目）'
                       % (self.silence_timeout, self._silent_resets))
        self.reset_device()
        self._last_rx = time.monotonic()  # 起動を待つ

    def reset_device(self) -> None:
        """EN を Low に 0.1 秒（ESP32 DevKit の自動リセット回路: RTS → EN）。"""
        with self._lock:
            ser = self._ser
            if ser is None:
                return
            try:
                ser.dtr = False
                ser.rts = True
                time.sleep(0.1)
                ser.rts = False
            except (serial.SerialException, OSError, ValueError) as exc:
                self.last_error = 'reset: %s' % exc  # pty など制御線の無いポート

    def _open(self) -> bool:
        try:
            ser = serial.Serial()
            ser.port = self.port
            ser.baudrate = self.baud
            ser.timeout = 0.1
            ser.write_timeout = 0.1
            # DTR/RTS を落としたまま開く（ESP32 DevKit の自動リセット回路を動かさない）
            ser.dtr = False
            ser.rts = False
            ser.exclusive = True
            ser.open()
        except (serial.SerialException, OSError, ValueError) as exc:
            self.last_error = 'open %s: %s' % (self.port, exc)
            return False
        self._asm.reset()
        self._last_rx = time.monotonic()
        self._silent_resets = 0
        with self._lock:
            self._ser = ser
        self.connect_count += 1
        self.last_error = ''
        self._on_event('connected: %s' % self.port)
        return True

    def _read_once(self) -> None:
        ser = self._ser
        if ser is None:
            return
        try:
            data = ser.read(max(1, ser.in_waiting))
        except (serial.SerialException, OSError, TypeError) as exc:
            # TypeError: 別スレッドで close された直後の pyserial の挙動
            self.last_error = 'read: %s' % exc
            self._close()
            self._stop.wait(self._interval)
            return
        if not data:
            return
        self._last_rx = time.monotonic()
        self._silent_resets = 0
        for line in self._asm.feed(data):
            try:
                self._on_line(line)
            except Exception as exc:  # コールバックの不具合でスレッドを殺さない
                self.last_error = 'on_line: %s' % exc

    def _close(self) -> None:
        with self._lock:
            ser, self._ser = self._ser, None
        if ser is None:
            return
        try:
            ser.close()
        except Exception:
            pass
        self._on_event('disconnected: %s (%s)' % (self.port, self.last_error))
