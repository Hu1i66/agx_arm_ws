"""Workbench model only; never starts CAN, arm, camera or grasp actions."""
import json
import os
import sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch_ros.actions import Node


def launch_workbench(context):
    root = Path(os.environ["AGX_ARM_WS"]).resolve()
    sys.path.insert(0, str(root))
    sys.path.insert(0, str(root / "src/agx_arm_ros/src/agx_arm_moveit/launch"))
    from _moveit_config_builder import build_moveit_config
    from runtime_geometry import load_tcp
    context.launch_configurations.update(
        arm_type="piper", effector_type="agx_gripper", revo2_type="left",
        tcp_offset=json.dumps(load_tcp()), follow="true", use_sim_time="false")
    cfg = build_moveit_config(context)
    screenshot = context.launch_configurations.get("screenshot", "")
    return [Node(package="piper_workbench", executable="piper_workbench",
                 parameters=[cfg.robot_description, cfg.robot_description_semantic,
                             cfg.robot_description_kinematics],
                 remappings=[("/joint_states", "/feedback/joint_states")],
                 prefix="gdb -batch -ex run -ex bt --args" if os.environ.get("WORKBENCH_GDB") else None,
                 arguments=(["--screenshot", screenshot] if screenshot else []) + ["--page", context.launch_configurations.get("page", "0"), "--size", context.launch_configurations.get("size", "1600x900")], output="screen")]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("size", default_value="1600x900"), DeclareLaunchArgument("page", default_value="0"), DeclareLaunchArgument("screenshot", default_value=""),
                              OpaqueFunction(function=launch_workbench)])
