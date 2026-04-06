import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.event_handlers import OnProcessExit

from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

import re
def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

def generate_launch_description():
    robot_name_in_model = 'agx_arm'
    package_name = 'agx_arm_moveit'
    urdf_name = "agx_arm.urdf.xacro"

    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'config/{urdf_name}')

    import subprocess
    xacro_cmd = ['xacro', urdf_model_path, 'use_gazebo:=true', 'arm_type:=piper', 'effector_type:=agx_gripper']
    result = subprocess.run(xacro_cmd, capture_output=True, text=True, check=True)
    clean_urdf = remove_comments(result.stdout)
    clean_urdf = clean_urdf.replace(': ', ' - ')

    # Start Gazebo server
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    params = {'robot_description': ParameterValue(clean_urdf, value_type=str)}

    # 启动了robot_state_publisher节点后，该节点会发布 robot_description 话题，话题内容是模型文件urdf的内容？
    node_robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher', # 必须显示指定名字，让 gazebo_ros2_control 能精准找到
    parameters=[{'use_sim_time': False}, params, {"publish_frequency": 15.0}], # 暂时用 False 打破同步死锁
    output='screen'
)

    # Launch the robot, 通过robot_description话题进行模型内容获取从而在gazebo中生成模型
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model,  '-topic', 'robot_description'], output='screen')

    # 关节状态发布器
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
             'arm_controller'],
        output='screen'
        )

    load_gripper_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
             'gripper_controller'],
        output='screen'
        )
    


    close_evt1 =  RegisterEventHandler( 
            event_handler=OnProcessExit(
                target_action=spawn_entity_cmd,
                on_exit=[load_joint_state_controller],
            )
    )

    close_evt2 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller, 
                         load_gripper_trajectory_controller],
            )
    )

    

    ld = LaunchDescription()

    ld.add_action(close_evt1)
    ld.add_action(close_evt2)
    
    ld.add_action(start_gazebo_cmd)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_entity_cmd)

    return ld
