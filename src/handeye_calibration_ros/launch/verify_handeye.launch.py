"""手眼标定结果验证 launch（RViz 可视化 TF）。

修正要点（计划 5.8）:
  1. 旧版本把 **optical frame 的外参**直接发布给 `camera_link`（并声称"两者平移为0"），
     坐标系错误 —— 现已改为发布到正确的 optical child frame
     （D455: camera_color_optical_frame；Dabai: dabai_color_optical_frame）。
  2. 外参统一来源: `config/calibration/d455_eye_to_hand.json`（由 runtime_config 校验），
     不再在本文件重复硬编码数值；可用 calibration_file:= 显式覆盖（离线/候选检查）。
  3. 需要反向（把 ^cam T_base 发布为 base←camera）时用 invert:=true，实现**完整逆**
     [Rᵀ, -Rᵀt]。旧教程"平移原样、仅四元数取共轭"是错的，已删除。
  4. Dabai 本次只做候选数学/端点验证，不发布固定 base TF（保持默认不发布）。

用法:
  ros2 launch handeye_calibration_ros verify_handeye.launch.py
  ros2 launch handeye_calibration_ros verify_handeye.launch.py \
      calibration_file:=/abs/path/candidate.json invert:=true camera:=d455 rviz:=false
"""
import math
import pathlib
import sys

import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# 定位主空间根以导入 runtime_config（launch 在任意 cwd 下运行）
_WS_ROOT = pathlib.Path(__file__).resolve().parents[3]
if str(_WS_ROOT) not in sys.path:
    sys.path.insert(0, str(_WS_ROOT))
import runtime_config as _rc  # noqa: E402


def _quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise ValueError("四元数为零")
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def _matrix_to_quat(m):
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        w, x, y, z = (m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        w, x, y, z = (m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        w, x, y, z = (m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s
    return x, y, z, w


def _build(context):
    cfg = context.launch_configurations
    camera = cfg.get("camera", "d455")
    cal_file = cfg.get("calibration_file") or None
    invert = str(cfg.get("invert", "false")).lower() in ("1", "true", "yes")
    use_rviz = str(cfg.get("rviz", "true")).lower() in ("1", "true", "yes")

    he = _rc.load_handeye(camera, cal_file)   # 缺失/非法明确失败
    x, y, z = he["position"]
    qx, qy, qz, qw = he["quaternion_normalized"]
    parent, child = he["parent"], he["child"]

    if invert:
        # 完整逆 [Rᵀ, -Rᵀt]：把 ^parent T_child 反转为 ^child T_parent
        R = _quat_to_matrix(qx, qy, qz, qw)
        Rt = [[R[j][i] for j in range(3)] for i in range(3)]
        t = [x, y, z]
        t_inv = [-sum(Rt[i][k] * t[k] for k in range(3)) for i in range(3)]
        x, y, z = t_inv
        qx, qy, qz, qw = _matrix_to_quat(Rt)
        parent, child = child, parent

    actions = [
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="handeye_verify_tf_publisher",
            arguments=["--x", str(x), "--y", str(y), "--z", str(z),
                       "--qx", str(qx), "--qy", str(qy), "--qz", str(qz), "--qw", str(qw),
                       "--frame-id", parent, "--child-frame-id", child],
            output="screen"),
    ]
    if use_rviz:
        actions.append(launch_ros.actions.Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen"))
    return actions


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument("camera", default_value="d455",
                              description="d455 | dabai"),
        DeclareLaunchArgument("calibration_file", default_value="",
                              description="显式覆盖外参文件（离线/候选检查）"),
        DeclareLaunchArgument("invert", default_value="false",
                              description="true=发布完整逆 [Rᵀ,-Rᵀt]（把 ^parent T_child 反转）"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="是否一并启动 rviz2"),
        OpaqueFunction(function=_build),
    ])
