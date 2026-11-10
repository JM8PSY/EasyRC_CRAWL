"""設定の定義と config.yaml の読み込み。"""
from __future__ import annotations

from dataclasses import dataclass, field, fields, is_dataclass, replace
from typing import Any

import yaml


@dataclass(frozen=True)
class MotorSide:
    pwm_pin: int
    dir_pin: int
    invert: bool = False


@dataclass(frozen=True)
class MotorConfig:
    left: MotorSide = field(default_factory=lambda: MotorSide(12, 5, False))
    right: MotorSide = field(default_factory=lambda: MotorSide(13, 6, True))
    pwm_freq: int = 20000
    max_speed: float = 1.0
    max_accel: float = 2.0


@dataclass(frozen=True)
class WatchdogConfig:
    timeout_ms: int = 300


@dataclass(frozen=True)
class CameraConfig:
    device: int = 0
    width: int = 640
    height: int = 480
    fps: int = 15
    jpeg_quality: int = 70


@dataclass(frozen=True)
class ServerConfig:
    host: str = "0.0.0.0"
    port: int = 8000


@dataclass(frozen=True)
class Config:
    motor: MotorConfig = field(default_factory=MotorConfig)
    watchdog: WatchdogConfig = field(default_factory=WatchdogConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    server: ServerConfig = field(default_factory=ServerConfig)


def default_config() -> Config:
    return Config()


def _merge(base: Any, data: Any, path: str) -> Any:
    """dataclass `base` に dict `data` を再帰的に上書きした新しいインスタンスを返す。"""
    if data is None:
        return base
    if not isinstance(data, dict):
        raise ValueError(f"{path or 'config'}: セクションは辞書で指定してください")
    names = {f.name for f in fields(base)}
    updates = {}
    for key, value in data.items():
        if key not in names:
            raise ValueError(f"未知の設定キー: {path}{key}")
        current = getattr(base, key)
        updates[key] = _merge(current, value, f"{path}{key}.") if is_dataclass(current) else value
    return replace(base, **updates)


# Pi 4 のハードウェア PWM ピンとチャンネル（12/18 = PWM0, 13/19 = PWM1）
HW_PWM_CHANNEL = {12: 0, 18: 0, 13: 1, 19: 1}


def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise ValueError(msg)


def _is_int(v: Any) -> bool:
    return isinstance(v, int) and not isinstance(v, bool)


def _is_num(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def validate(cfg: Config) -> None:
    m = cfg.motor
    for name, side in (("left", m.left), ("right", m.right)):
        _require(_is_int(side.pwm_pin) and side.pwm_pin in HW_PWM_CHANNEL,
                 f"motor.{name}.pwm_pin はハードウェアPWMピン {sorted(HW_PWM_CHANNEL)} のいずれかにしてください")
        _require(_is_int(side.dir_pin) and 0 <= side.dir_pin <= 27, f"motor.{name}.dir_pin が不正です")
        _require(isinstance(side.invert, bool), f"motor.{name}.invert は true/false で指定してください")
    _require(HW_PWM_CHANNEL[m.left.pwm_pin] != HW_PWM_CHANNEL[m.right.pwm_pin],
             "左右の pwm_pin が同じPWMチャンネルです（12/18 と 13/19 から1本ずつ選んでください）")
    _require(_is_int(m.pwm_freq) and 0 < m.pwm_freq <= 30000, "motor.pwm_freq は 1〜30000 にしてください")
    _require(_is_num(m.max_speed) and 0 <= m.max_speed <= 1, "motor.max_speed は 0〜1 にしてください")
    _require(_is_num(m.max_accel) and m.max_accel > 0, "motor.max_accel は正の数にしてください")
    _require(_is_int(cfg.watchdog.timeout_ms) and cfg.watchdog.timeout_ms > 0,
             "watchdog.timeout_ms は正の整数にしてください")
    c = cfg.camera
    _require(all(_is_int(v) and v > 0 for v in (c.width, c.height, c.fps)),
             "camera.width/height/fps は正の整数にしてください")
    _require(_is_int(c.jpeg_quality) and 1 <= c.jpeg_quality <= 100, "camera.jpeg_quality は 1〜100 にしてください")
    _require(_is_int(cfg.server.port) and 0 < cfg.server.port < 65536, "server.port が不正です")


def load_config(path: str | None) -> Config:
    if path is None:
        return default_config()
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    cfg = _merge(default_config(), data, "")
    validate(cfg)
    return cfg
