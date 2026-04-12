from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    arm_type_arg = DeclareLaunchArgument(
        "arm_type",
        default_value="piper",
        choices=["nero", "piper", "piper_h", "piper_l", "piper_x"],
        description="Arm type used by MoveIt config.",
    )

    effector_type_arg = DeclareLaunchArgument(
        "effector_type",
        default_value="agx_gripper",
        choices=["none", "agx_gripper", "revo2"],
        description="End effector type used by simulation and MoveIt.",
    )

    revo2_type_arg = DeclareLaunchArgument(
        "revo2_type",
        default_value="left",
        choices=["left", "right"],
        description="Revo2 side when effector_type is revo2.",
    )

    tcp_offset_arg = DeclareLaunchArgument(
        "tcp_offset",
        default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
        description="TCP offset [x, y, z, rx, ry, rz] in meters/radians.",
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("piper_gazebo"),
                "launch",
                "piper_with_gripper",
                "piper_gazebo.launch.py",
            )
        )
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("agx_arm_moveit"),
                "launch",
                "demo.launch.py",
            )
        ),
        launch_arguments={
            "arm_type": LaunchConfiguration("arm_type"),
            "effector_type": LaunchConfiguration("effector_type"),
            "revo2_type": LaunchConfiguration("revo2_type"),
            "tcp_offset": LaunchConfiguration("tcp_offset"),
            "follow": "true",
            "use_sim_time": "true",
        }.items(),
    )

    delayed_moveit_launch = TimerAction(
        period=5.0,
        actions=[moveit_launch],
    )

    return LaunchDescription([
        arm_type_arg,
        effector_type_arg,
        revo2_type_arg,
        tcp_offset_arg,
        gazebo_launch,
        delayed_moveit_launch,
    ])
