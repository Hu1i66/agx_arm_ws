"""眼在手外 (eye-to-hand) 相机 TF 发布器

发布 base_link → camera_color_optical_frame 静态变换，
使 RViz/MoveIt 等工具能在机器人坐标系中可视化相机数据。

外参统一来源: config/calibration/d455_eye_to_hand.json
（由 runtime_config.load_handeye 校验：3 位位置、4 位 xyzw、有限数值、非零四元数）

变换方向:
  - 标定文件 position/orientation 表示 camera_color_optical_frame 在 base_link 系中的位姿
  - p_base = T_base_camera @ p_camera
  - ROS TF 中 parent → child 的 T_tf 满足 p_parent = T_tf @ p_child，
    故发布 base_link → camera_color_optical_frame 时 T_tf = T_base_camera，直接使用标定值

启动:
  ros2 launch /home/lxf/agx_arm_ws/launch/eye_to_hand_camera_tf.launch.py
  # 离线/候选检查可显式覆盖:
  ros2 launch ... calibration_file:=/abs/path/to/candidate.json

⚠️ 使用此发布器时 realsense2_camera 需 publish_tf:=false 避免冲突
  （统一入口 camera 组件已固化该设置）。
"""
import pathlib
import sys

import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction

# 定位主空间根以导入 runtime_config（launch 在任意 cwd 下运行）
_WS_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(_WS_ROOT) not in sys.path:
    sys.path.insert(0, str(_WS_ROOT))
import runtime_config as _rc  # noqa: E402


def _build_tf(context):
    cal_file = context.launch_configurations.get("calibration_file") or None
    he = _rc.load_handeye(context.launch_configurations.get("camera", "d455"), cal_file)  # 缺失/非法明确失败
    x, y, z = he["position"]
    qx, qy, qz, qw = he["quaternion_normalized"]
    return [
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=he["camera"]+"_calibrated_optical_frame_broadcaster",
            arguments=[
                "--x", str(x),
                "--y", str(y),
                "--z", str(z),
                "--qx", str(qx),
                "--qy", str(qy),
                "--qz", str(qz),
                "--qw", str(qw),
                "--frame-id", he["parent"],
                "--child-frame-id", he["child"],
            ],
        ),
    ]


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument("camera", default_value="d455", choices=["d455", "dabai"]),
        DeclareLaunchArgument(
            "calibration_file", default_value="",
            description="显式覆盖外参文件（离线/候选检查）；留空使用活动配置"),
        OpaqueFunction(function=_build_tf),
    ])
