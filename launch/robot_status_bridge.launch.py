"""机械臂状态 MQTT 桥接节点启动文件.

启动 robot_status_bridge 节点, 加载 MQTT 配置参数.

启动:
  ros2 launch /home/lxf/agx_arm_ws/launch/robot_status_bridge.launch.py

或指定自定义配置文件:
  ros2 launch /home/lxf/agx_arm_ws/launch/robot_status_bridge.launch.py \
    params_file:=/path/to/custom_config.yaml

可通过命令行覆盖参数:
  ros2 launch /home/lxf/agx_arm_ws/launch/robot_status_bridge.launch.py \
    broker_host:=192.168.1.100 topic:=robot/my_robot/action
"""
import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess


def generate_launch_description():
    # 配置文件路径
    config_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config'
    )
    default_params_file = os.path.join(config_dir, 'robot_status_bridge.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='robot_status_bridge YAML 配置文件路径',
    )

    bridge_node = ExecuteProcess(
        cmd=['python3', '/home/lxf/agx_arm_ws/robot_status_bridge.py',
             '--ros-args', '--params-file', LaunchConfiguration('params_file')],
        name='robot_status_bridge',
        output='screen',
    )

    return launch.LaunchDescription([
        params_file_arg,
        bridge_node,
    ])
