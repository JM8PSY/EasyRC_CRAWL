"""実機調整値（~/.config/cugo/calibration.yaml）の読み書きと URDF の再生成。

cugo_description の description.launch.py が起動時に同じファイルを読む。
"""
import math
import os
import re
import tempfile
from typing import Dict, Optional

import yaml

CALIBRATION_FILE = os.path.expanduser('~/.config/cugo/calibration.yaml')


def wrap_angle(a: float) -> float:
    """[-π, π) に収める。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def load(path: str = CALIBRATION_FILE) -> Dict[str, float]:
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        return {}
    if not isinstance(data, dict):
        return {}
    return {k: float(v) for k, v in data.items()
            if isinstance(v, (int, float)) and not isinstance(v, bool)}


def save(values: Dict[str, float], path: str = CALIBRATION_FILE) -> None:
    """一時ファイルに書いてから置き換える（書き込み途中で電源が落ちても壊れにくい）。"""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    body = '# CuGo 実機調整値（Web UI が書き込む。起動時に cugo_description が読む）\n'
    body += yaml.safe_dump({k: round(float(v), 4) for k, v in values.items()},
                           default_flow_style=False)
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), prefix='.calibration.')
    try:
        with os.fdopen(fd, 'w') as f:
            f.write(body)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def build_urdf(xacro_path: str, values: Dict[str, float]) -> str:
    import xacro  # ROS 環境でのみ必要
    mappings = {k: repr(float(v)) for k, v in values.items()}
    return xacro.process_file(xacro_path, mappings=mappings).toprettyxml(indent='  ')


_JOINT_RE = re.compile(
    r'<joint[^>]*name="base_link_to_laser_frame"[^>]*>.*?<origin[^>]*rpy="([^"]+)"',
    re.S)


def lidar_yaw_from_urdf(urdf: str) -> Optional[float]:
    m = _JOINT_RE.search(urdf)
    if not m:
        return None
    try:
        return float(m.group(1).split()[2])
    except (IndexError, ValueError):
        return None
