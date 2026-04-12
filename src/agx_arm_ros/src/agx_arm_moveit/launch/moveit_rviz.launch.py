import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter

from moveit_configs_utils.launches import generate_moveit_rviz_launch

from _moveit_config_builder import build_moveit_config, declare_common_args


def _launch(context):
    follow = LaunchConfiguration("follow").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    joint_states_topic = (
        "/feedback/joint_states" if follow == "true" else "/control/joint_states"
    )

    moveit_config = build_moveit_config(context)
    rviz_components = list(generate_moveit_rviz_launch(moveit_config).entities)
    
    from launch_ros.actions import Node
    from launch.substitutions import TextSubstitution
    for entity in rviz_components:
        if isinstance(entity, Node):
            # Hack to get the node executable name since launch_ros might obscure it
            if "rviz2" in str(entity.cmd):
                # Add remapping for joint_states using internal formatted tuples
                remappings = getattr(entity, "_Node__remappings", None)
                if remappings is None:
                    remappings = []
                elif isinstance(remappings, tuple):
                    remappings = list(remappings)
                remappings.append(
                    ((TextSubstitution(text="/joint_states"),), (TextSubstitution(text=joint_states_topic),))
                )
                entity._Node__remappings = remappings


    return [SetParameter(name="use_sim_time", value=use_sim_time)] + rviz_components


def generate_launch_description():
    return LaunchDescription(declare_common_args() + [OpaqueFunction(function=_launch)])
