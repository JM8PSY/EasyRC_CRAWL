"""ESP32 ⇔ Jetson シリアルプロトコル v1（ROS 非依存）。

行形式: ``$<BODY>*<HH>\\n``。HH は BODY の全バイトの XOR（2 桁大文字 16 進）。
``$`` で始まらない行（人間向けログ）とチェックサム不一致・形式不正の行は無視する。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Union

MODE_NAMES = {'I': 'idle', 'A': 'auto', 'M': 'manual', 'X': 'test'}

FLAG_AUTO_ALLOWED = 1 << 0
FLAG_WEB_CONNECTED = 1 << 1
FLAG_JETSON_ACTIVE = 1 << 2

_HEX = set('0123456789abcdefABCDEF')

BytesOrStr = Union[bytes, bytearray, str]


def _to_bytes(data: BytesOrStr) -> bytes:
    if isinstance(data, str):
        return data.encode('utf-8', errors='replace')
    return bytes(data)


def checksum(body: BytesOrStr) -> int:
    """BODY の全バイトの XOR。"""
    value = 0
    for b in _to_bytes(body):
        value ^= b
    return value


def format_line(body: str) -> bytes:
    """BODY を ``$BODY*HH\\n`` に包む。"""
    return ('$%s*%02X\n' % (body, checksum(body))).encode('ascii')


def _duty_str(value: float) -> str:
    """-1.0〜1.0 にクランプして小数 3 桁。NaN/inf は 0、-0.000 は出さない。"""
    if not math.isfinite(value):
        value = 0.0
    value = max(-1.0, min(1.0, value))
    text = '%.3f' % value
    return '0.000' if text == '-0.000' else text


def format_drive(left: float, right: float) -> bytes:
    """``$D,<l>,<r>*HH\\n``（左右の絶対 duty）。"""
    return format_line('D,%s,%s' % (_duty_str(left), _duty_str(right)))


def format_stop() -> bytes:
    """``$S*HH\\n``（即時停止）。"""
    return format_line('S')


def parse_line(line: BytesOrStr) -> Optional[str]:
    """枠とチェックサムを検証し、正しければ BODY を返す。不正なら None。"""
    if isinstance(line, (bytes, bytearray)):
        line = bytes(line).decode('ascii', errors='replace')
    line = line.strip()
    if not line.startswith('$'):
        return None
    star = line.rfind('*')
    if star < 1:
        return None
    body, hh = line[1:star], line[star + 1:]
    if len(hh) != 2 or not set(hh) <= _HEX:
        return None
    if not body.isascii() or checksum(body) != int(hh, 16):
        return None
    return body


@dataclass(frozen=True)
class Status:
    """ESP32 の ``$T`` ステータス。"""

    mode: str     # 'I' | 'A' | 'M' | 'X'
    l: float      # noqa: E741  現在の出力 duty（論理値）
    r: float
    cap: float    # AUTO_MAX_DUTY
    flags: int
    bad: int      # ESP32 が受けた不正行の数

    @property
    def mode_name(self) -> str:
        return MODE_NAMES[self.mode]

    @property
    def auto_allowed(self) -> bool:
        return bool(self.flags & FLAG_AUTO_ALLOWED)

    @property
    def web_connected(self) -> bool:
        return bool(self.flags & FLAG_WEB_CONNECTED)

    @property
    def jetson_active(self) -> bool:
        return bool(self.flags & FLAG_JETSON_ACTIVE)


def _finite(text: str) -> float:
    value = float(text)
    if not math.isfinite(value):
        raise ValueError(text)
    return value


def _nonneg_int(text: str) -> int:
    if not text.isdigit():
        raise ValueError(text)
    return int(text)


def parse_status(line: BytesOrStr) -> Optional[Status]:
    """``$T,<mode>,<l>,<r>,<cap>,<flags>,<bad>*HH`` を解釈する。不正なら None。"""
    body = parse_line(line)
    if body is None:
        return None
    fields = body.split(',')
    if len(fields) != 7 or fields[0] != 'T' or fields[1] not in MODE_NAMES:
        return None
    try:
        return Status(mode=fields[1], l=_finite(fields[2]), r=_finite(fields[3]),
                      cap=_finite(fields[4]), flags=_nonneg_int(fields[5]),
                      bad=_nonneg_int(fields[6]))
    except ValueError:
        return None


class LineAssembler:
    """任意のバイト列の断片から完成した行を取り出す。

    - 部分読み込みに対応（行末 ``\\n`` まで溜める）
    - ``\\r\\n`` / 末尾 ``\\r`` を除去、空行は捨てる
    - 非 UTF-8 のゴミ（ESP32 ブート ROM の 74880bps 出力など）は置換文字にして通す
      （チェックサムで弾かれる）
    - ``max_len`` を超える行は次の改行まで捨てて再同期する
    """

    def __init__(self, max_len: int = 256) -> None:
        self._max_len = max_len
        self._buf = bytearray()
        self._discarding = False

    def reset(self) -> None:
        self._buf.clear()
        self._discarding = False

    def feed(self, chunk: bytes) -> List[str]:
        lines: List[str] = []
        start = 0
        while True:
            nl = chunk.find(b'\n', start)
            if nl < 0:
                self._append(chunk[start:])
                return lines
            self._append(chunk[start:nl])
            line = self._take_line()
            if line:
                lines.append(line)
            start = nl + 1

    def _append(self, data: bytes) -> None:
        if self._discarding or not data:
            return
        self._buf.extend(data)
        if len(self._buf.rstrip(b'\r')) > self._max_len:
            self._buf.clear()
            self._discarding = True

    def _take_line(self) -> str:
        if self._discarding:
            self._discarding = False
            return ''
        raw = bytes(self._buf).rstrip(b'\r')
        self._buf.clear()
        return raw.decode('utf-8', errors='replace')
