"""USB シリアルの接続管理（ROS 非依存）。

バックグラウンドスレッドで開く・読む・切断時に再接続する（既定 1 秒間隔）。
受信した行は ``on_line`` コールバックに渡す。例外で落ちることはない。
"""
from __future__ import annotations

import threading
from typing import Callable, Optional

import serial

from cugo_base.protocol import LineAssembler


class SerialLink:
    def __init__(self, port: str, baud: int, on_line: Callable[[str], None],
                 reconnect_interval: float = 1.0,
                 on_event: Optional[Callable[[str], None]] = None) -> None:
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
