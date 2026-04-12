#!/usr/bin/env python3
import time
from local_sdk_bootstrap import ensure_workspace_sdk_on_path

ensure_workspace_sdk_on_path()

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from piper_sdk import C_PiperInterface

def enable_gravity_compensation():
    print("========== 机械臂失能与重力补偿 ==========")
    print("1. 正在通过 ROS 确保机械臂处于可接收指令的使能状态...")
    # ROS 2 初始化
    rclpy.init()
    node = Node('gravity_comp_node')
    client = node.create_client(SetBool, '/enable_agx_arm')
    if client.wait_for_service(timeout_sec=2.0):
        req = SetBool.Request()
        req.data = False  # ⚠️修改点：让 ROS 节点端【失能】，阻止它不断下发坐标轴控制指令干扰机械臂状态
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            print("   -> ROS 节点已脱离控制权 (停止干扰下发)。")
    else:
        print("   -> (未检测到 ROS 驱动，尝试直接通过 CAN 发送)")
    
    node.destroy_node()
    rclpy.shutdown()

    # CAN 发送拖动指令
    print("\n2. 正在通过 CAN 总线发送【物理使能】与【重力补偿】指令...")
    from pyAgxArm import create_agx_arm_config, AgxArmFactory
    config = create_agx_arm_config(robot='piper', comm='can', channel='can0')
    arm = AgxArmFactory.create_arm(config)
    arm.connect()
    
    # 必须从物理硬件端直接使能电机（使得电机带电才能输出悬停力矩），此时不经过 ROS
    arm.enable()
    time.sleep(0.5)

    # 进入零力矩拖动模式 (Zero Force Drag Mode) 即重力补偿模式
    arm.set_leader_mode()
    print("   -> 已通过 pyAgxArm 下发 set_leader_mode() 指令。")
    time.sleep(0.5)
    
    print("\n✅ 指令发送成功！")
    print("现在机械臂已进入重力补偿（拖动示教）模式，您可以自由拖动机械臂。")
    print("注意：机械臂处于该模式时会输出力矩平衡自身重力，不会像完全失能那样掉落。")
    print("如需退出重力补偿恢复程序控制，执行命令:\n  ros2 service call /exit_teach_mode std_srvs/srv/Empty\n")

if __name__ == '__main__':
    enable_gravity_compensation()
