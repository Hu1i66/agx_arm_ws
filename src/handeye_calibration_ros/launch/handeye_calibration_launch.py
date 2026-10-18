from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    params = {
        'mode': LaunchConfiguration('mode'),
        'min_num': LaunchConfiguration('min_num'),
        'piper_topic': LaunchConfiguration('piper_topic'),
        'image_topic': LaunchConfiguration('image_topic'),
        'camera_info_topic': LaunchConfiguration('camera_info_topic'),
        'intrinsics_yaml': LaunchConfiguration('intrinsics_yaml'),
        'result_save_path': LaunchConfiguration('result_save_path'),
        'square_length': LaunchConfiguration('square_length'),
        'marker_length': LaunchConfiguration('marker_length'),
        'aruco_dict': LaunchConfiguration('aruco_dict'),
    }

    handeye_calibration = Node(
        package='handeye_calibration_ros',
        executable='handeye_calibration',
        name="handeye_calibration",
        output='screen',
        parameters=[params],
    )

    return [handeye_calibration]


def generate_launch_description():

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='eye_in_hand',  #eye_to_hand
        choices=['eye_in_hand', 'eye_to_hand'],
        description='handeye calibration mode'
    )

    min_num_arg = DeclareLaunchArgument(
        'min_num',
        default_value='15',
        description='最少采集位姿数 (文档建议 15-25)'
    )

    piper_topic_arg = DeclareLaunchArgument(
        'piper_topic',
        default_value='/feedback/tcp_pose',
        description='机械臂 TCP 位姿话题 (geometry_msgs/PoseStamped)'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/camera/color/image_raw',
        description='彩色图像话题'
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera/color/camera_info',
        description='相机内参话题'
    )

    intrinsics_yaml_arg = DeclareLaunchArgument(
        'intrinsics_yaml',
        default_value='./result/camera_intrinsics.yaml',
        description='自标内参 YAML (存在则覆盖 camera_info)'
    )

    result_save_path_arg = DeclareLaunchArgument(
        'result_save_path',
        default_value='./result',
        description='标定结果与会话目录保存路径'
    )

    square_length_arg = DeclareLaunchArgument(
        'square_length',
        default_value='0.032',
        description='方格边长 (m), 可按卡尺实测值回填'
    )

    marker_length_arg = DeclareLaunchArgument(
        'marker_length',
        default_value='0.024',
        description='标记边长 (m)'
    )

    aruco_dict_arg = DeclareLaunchArgument(
        'aruco_dict',
        default_value='DICT_5X5_1000',
        description='ArUco 字典名'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(mode_arg)
    ld.add_action(min_num_arg)
    ld.add_action(piper_topic_arg)
    ld.add_action(image_topic_arg)
    ld.add_action(camera_info_topic_arg)
    ld.add_action(intrinsics_yaml_arg)
    ld.add_action(result_save_path_arg)
    ld.add_action(square_length_arg)
    ld.add_action(marker_length_arg)
    ld.add_action(aruco_dict_arg)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld