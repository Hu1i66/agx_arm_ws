"""眼在手外 (eye-to-hand) 相机 TF 发布器

发布 base_link → camera_color_optical_frame 静态变换,
使 RViz/MoveIt 等工具能在机器人坐标系中可视化相机数据。

标定来源: /home/lxf/handeye/result/2026-07-26_16-17-46_calibration.json

变换方向分析:
  - 标定文件的 position/orientation 表示 camera_color_optical_frame 在 base_link 系中的位姿
  - _load_handeye_matrix 返回 T_cam_to_base (将点从相机系变换到基座系: p_base = T @ p_cam)
  - ROS TF 中 parent → child 的变换矩阵 T_tf 满足 p_parent = T_tf @ p_child
  - 因此发布 base_link → camera_color_optical_frame 时, T_tf = T_cam_to_base,
    直接使用标定值即可, 无需取逆

启动:
  ros2 launch /home/lxf/agx_arm_ws/launch/eye_to_hand_camera_tf.launch.py

⚠️ 使用此发布器时, realsense2_camera 需设置 publish_tf:=false 避免冲突:
  ros2 launch realsense2_camera rs_launch.py ... publish_tf:=false
"""
import launch
import launch_ros.actions


def generate_launch_description():
    # 标定数据 (camera_color_optical_frame → base_link)
    # 来源: /home/lxf/handeye/result/2026-07-26_16-17-46_calibration.json
    # position:  [0.6647111267257422, 0.0081521692571569, 0.567034342682776]
    # orientation: [-0.6897711805628746, -0.7087083420930345, 0.11849101960225547, 0.08892740064939907]
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_color_optical_frame_broadcaster',
            arguments=[
                '--x', '0.6647111267257422',
                '--y', '0.0081521692571569',
                '--z', '0.567034342682776',
                '--qx', '-0.6897711805628746',
                '--qy', '-0.7087083420930345',
                '--qz', '0.11849101960225547',
                '--qw', '0.08892740064939907',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_color_optical_frame',
            ],
        ),
    ])
