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


def load_config(path: str | None) -> Config:
    if path is None:
        return default_config()
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return _merge(default_config(), data, "")
