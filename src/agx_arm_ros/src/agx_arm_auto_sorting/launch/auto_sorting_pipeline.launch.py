from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('agx_arm_auto_sorting'),
        'config',
        'sorting_dataset_config.yaml',
    ])

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Absolute path of sorting_dataset_config.yaml'
    )

    sorter = Node(
        package='agx_arm_auto_sorting',
        executable='sorting_cycle_server',
        output='screen',
        parameters=[{'config_file': LaunchConfiguration('config_file')}],
    )

    collector = Node(
        package='agx_arm_auto_sorting',
        executable='dataset_collection_runner',
        output='screen',
        parameters=[{'config_file': LaunchConfiguration('config_file')}],
    )

    return LaunchDescription([
        config_file_arg,
        sorter,
        collector,
    ])
