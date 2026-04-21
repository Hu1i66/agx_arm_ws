import math
import os
from typing import Dict, Any

import yaml


def load_yaml_config(path: str) -> Dict[str, Any]:
    if not os.path.exists(path):
        raise FileNotFoundError(f'Config file not found: {path}')
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError('YAML config root must be a dict.')
    return data


def ensure_pose_keys(pose: Dict[str, Any]) -> None:
    required = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
    missing = [k for k in required if k not in pose]
    if missing:
        raise ValueError(f'Pose missing keys: {missing}')


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw
