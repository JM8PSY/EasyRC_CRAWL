"""プロトコル（ROS 非依存）の単体テスト。"""
from functools import reduce

import pytest

from cugo_base.protocol import (
    LineAssembler,
    Status,
    checksum,
    format_drive,
    format_line,
    format_stop,
    parse_line,
    parse_status,
)


def _xor(body: str) -> str:
    return '%02X' % reduce(lambda a, c: a ^ ord(c), body, 0)


def _frame(body: str) -> str:
    return '$%s*%s' % (body, _xor(body))


# ---- checksum / format ----

def test_checksum_matches_xor_of_body():
    assert checksum('S') == 0x53
    assert checksum(b'D,0.250,-0.250') == int(_xor('D,0.250,-0.250'), 16)
    assert checksum('') == 0


def test_format_line():
    assert format_line('S') == b'$S*53\n'


def test_format_stop():
    assert format_stop() == b'$S*53\n'


def test_format_drive_three_decimals():
    body = 'D,0.250,-0.250'
    assert format_drive(0.25, -0.25) == ('$%s*%s\n' % (body, _xor(body))).encode()


def test_format_drive_rounding_and_no_negative_zero():
    assert format_drive(0.12345, -0.0) == ('%s\n' % _frame('D,0.123,0.000')).encode()
    assert format_drive(-0.0001, 0.0004) == ('%s\n' % _frame('D,0.000,0.000')).encode()


def test_format_drive_clamps_to_unit_range():
    assert format_drive(1.7, -3.0) == ('%s\n' % _frame('D,1.000,-1.000')).encode()


def test_format_drive_rejects_nan_as_zero():
    assert format_drive(float('nan'), float('inf')) == (
        '%s\n' % _frame('D,0.000,0.000')).encode()


def test_format_drive_is_ascii_with_uppercase_hex():
    line = format_drive(0.5, 0.5)
    assert line.startswith(b'$') and line.endswith(b'\n')
    hh = line[-3:-1].decode()
    assert hh == hh.upper()


# ---- parse_line ----

def test_parse_line_valid():
    assert parse_line(_frame('T,A,0.1,0.1,0.50,5,0')) == 'T,A,0.1,0.1,0.50,5,0'


def test_parse_line_accepts_bytes_and_trailing_whitespace():
    assert parse_line((_frame('S') + '\r\n').encode()) == 'S'


def test_parse_line_accepts_lowercase_hex():
    body = 'T,I,0.000,0.000,0.50,0,0'
    assert parse_line('$%s*%s' % (body, _xor(body).lower())) == body


@pytest.mark.parametrize('line', [
    '',
    'ESP32 起動しました',
    'T,A,0,0,0.5,0,0*00',
    '$T,A,0.000,0.000,0.50,0,0',          # チェックサムなし
    '$T,A,0.000,0.000,0.50,0,0*ZZ',       # 16 進でない
    '$T,A,0.000,0.000,0.50,0,0*0',        # 1 桁
    '$T,A,0.000,0.000,0.50,0,0*123',      # 3 桁
])
def test_parse_line_rejects_malformed(line):
    assert parse_line(line) is None


def test_parse_line_rejects_bad_checksum():
    body = 'T,A,0.000,0.000,0.50,0,0'
    bad = (int(_xor(body), 16) ^ 0x01)
    assert parse_line('$%s*%02X' % (body, bad)) is None


# ---- parse_status ----

def test_parse_status_valid():
    st = parse_status(_frame('T,A,0.250,-0.125,0.50,5,3'))
    assert st == Status(mode='A', l=0.25, r=-0.125, cap=0.5, flags=5, bad=3)
    assert st.mode_name == 'auto'
    assert st.auto_allowed is True
    assert st.web_connected is False
    assert st.jetson_active is True


@pytest.mark.parametrize('ch,name', [
    ('I', 'idle'), ('A', 'auto'), ('M', 'manual'), ('X', 'test')])
def test_parse_status_mode_names(ch, name):
    st = parse_status(_frame('T,%s,0.000,0.000,0.50,2,0' % ch))
    assert st is not None and st.mode_name == name


@pytest.mark.parametrize('body', [
    'T,Q,0.000,0.000,0.50,0,0',       # 不明なモード
    'T,A,0.000,0.000,0.50,0',         # フィールド不足
    'T,A,0.000,0.000,0.50,0,0,9',     # フィールド過多
    'T,A,abc,0.000,0.50,0,0',         # 数値でない
    'T,A,0.000,0.000,0.50,1.5,0',     # flags が整数でない
    'T,A,0.000,0.000,0.50,0,-1',      # bad が負
    'T,A,nan,0.000,0.50,0,0',         # 非有限
    'D,0.000,0.000',                  # 別の種類の行
    'S',
])
def test_parse_status_rejects_invalid_bodies(body):
    assert parse_status(_frame(body)) is None


def test_parse_status_rejects_bad_checksum_and_logs():
    assert parse_status('$T,A,0.000,0.000,0.50,0,0*00') is None
    assert parse_status('[WiFi] 接続しました') is None


# ---- LineAssembler ----

def test_assembler_splits_complete_lines():
    asm = LineAssembler()
    assert asm.feed(b'abc\ndef\n') == ['abc', 'def']


def test_assembler_handles_partial_reads():
    asm = LineAssembler()
    assert asm.feed(b'$T,A,0.0') == []
    assert asm.feed(b'00,0.0') == []
    assert asm.feed(b'00\nxy') == ['$T,A,0.000,0.000']
    assert asm.feed(b'z\n') == ['xyz']


def test_assembler_strips_crlf():
    asm = LineAssembler()
    assert asm.feed(b'one\r\ntwo\r') == ['one']
    assert asm.feed(b'\n') == ['two']


def test_assembler_skips_empty_lines():
    asm = LineAssembler()
    assert asm.feed(b'\n\r\n\nx\n') == ['x']


def test_assembler_survives_binary_garbage():
    asm = LineAssembler()
    garbage = bytes(range(256)).replace(b'\n', b'')
    good = _frame('T,I,0.000,0.000,0.50,0,0')
    lines = asm.feed(garbage + b'\n' + good.encode() + b'\n')
    assert len(lines) == 2
    assert parse_status(lines[0]) is None
    assert parse_status(lines[1]) is not None


def test_assembler_utf8_japanese_log_lines_pass_through():
    asm = LineAssembler()
    assert asm.feed('起動\n'.encode('utf-8')) == ['起動']


def test_assembler_drops_overlong_line_and_resyncs():
    asm = LineAssembler(max_len=16)
    assert asm.feed(b'x' * 10) == []
    assert asm.feed(b'x' * 10) == []         # 上限超過 → 破棄モード
    assert asm.feed(b'yyy\nok\n') == ['ok']  # 次の改行まで捨てて復帰
    assert asm.feed(b'z' * 16 + b'\n') == ['z' * 16]


def test_assembler_overlong_in_single_chunk():
    asm = LineAssembler(max_len=8)
    assert asm.feed(b'a' * 100 + b'\nhello\n') == ['hello']


def test_assembler_reset():
    asm = LineAssembler()
    asm.feed(b'partial')
    asm.reset()
    assert asm.feed(b'x\n') == ['x']
