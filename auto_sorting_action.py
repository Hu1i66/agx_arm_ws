
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import math
import queue
import json
from std_msgs.msg import String

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class MoveItActionClient(Node):
    def __init__(self):
        super().__init__('piper_moveit_action_client')
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self._gripper_action_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        
        # 服务端通信设置
        self.cmd_queue = queue.Queue()
        self.cmd_sub = self.create_subscription(String, '/sorting_cmds', self.cmd_callback, 10)
        self.status_pub = self.create_publisher(String, '/sorting_status', 10)
        self.is_busy = False

    def cmd_callback(self, msg):
        # 简单将 JSON 命令推入队列给主线程处理
        self.cmd_queue.put(msg.data)
        
    def wait_for_server(self):
        self.get_logger().info('等待 MoveGroup Action 服务器...')
        self._action_client.wait_for_server()
        self.get_logger().info('✅ 已连接到 MoveIt2 规划器！')

    def send_goal(self, group_name, constraints, plan_only=False, continuous=False):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        goal_msg.request.workspace_parameters.header.frame_id = 'base_link'
        
        goal_msg.request.group_name = group_name
        goal_msg.request.num_planning_attempts = 3
        goal_msg.request.allowed_planning_time = 5.0
        # 稍微将速度降低到 50%，以保障轨迹规划安全和稳定不越界
        goal_msg.request.max_velocity_scaling_factor = 0.50
        goal_msg.request.max_acceleration_scaling_factor = 0.50
        
        goal_msg.request.goal_constraints.append(constraints)
        
        goal_msg.planning_options.plan_only = plan_only
        
        self.get_logger().info(f'发送规划目标...')
        # print("正在请求...")
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('❌ 规划目标被拒绝！(不可达或干涉)')
            return False

        self.get_logger().info('⭕ 目标被接受，正在执行...')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result
        
        # 错误码 1 代表 SUCCESS (返回1表示成功到达)
        if result.error_code.val == 1:
            self.get_logger().info('✅ 动作规划并执行成功完成！')
            # 留一点时间让机械臂稳定，避免启动状态在碰撞或未到位状态
            if continuous:
                time.sleep(0.05) # 连贯动作减少停顿
            else:
                time.sleep(0.5) 
            return True
        else:
            self.get_logger().error(f'⚠️ 执行失败，错误码: {result.error_code.val}')
            return False

    def move_arm_joint(self, joints, desc):
        print(f"\n🚀 正在规划(关节空间) -> {desc}")
        c = Constraints()
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        for name, pos in zip(joint_names, joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.01 # < 1度允许误差
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return self.send_goal('arm', c)

    def move_arm_cartesian(self, pose_dict, desc, continuous=False):
        print(f"\n🚀 正在规划(笛卡尔位置) -> {desc}")
        c = Constraints()
        
        # 1. 位置约束
        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'
        pc.link_name = 'link6'
        
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.0075]  # 允许 7.5 毫米的位置误差（帮助避障解算）
        
        p = Pose()
        p.position.x = float(pose_dict['x'])
        p.position.y = float(pose_dict['y'])
        p.position.z = float(pose_dict['z'])
        
        v = BoundingVolume()
        v.primitives.append(s)
        v.primitive_poses.append(p)
        pc.constraint_region = v
        pc.weight = 1.0
        
        # 2. 姿态约束
        oc = OrientationConstraint()
        oc.header.frame_id = 'base_link'
        oc.link_name = 'link6'
        oc.orientation.x = float(pose_dict['qx'])
        oc.orientation.y = float(pose_dict['qy'])
        oc.orientation.z = float(pose_dict['qz'])
        oc.orientation.w = float(pose_dict['qw'])
        
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0
        
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        
        return self.send_goal('arm', c, continuous=continuous)

    def operate_gripper(self, target_pos, desc):
        print(f"✊ 正在执行夹爪动作 -> {desc} (target: {target_pos})")
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['gripper_joint1', 'gripper_joint2']
        
        point = JointTrajectoryPoint()
        # 根据 agx_arm_ctrl_single_node，gripper_joint1 是宽度的 0.5，gripper_joint2 是 -0.5
        point.positions = [float(target_pos * 0.5), float(-target_pos * 0.5)]
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0
        
        goal_msg.trajectory.points.append(point)
        
        self._gripper_action_client.wait_for_server()
        send_goal_future = self._gripper_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('❌ 夹爪目标被拒绝！')
            return False

        self.get_logger().info('⭕ 夹爪动作执行中...')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        # 将夹爪延时从1.0缩短到0.2，避免开合后干等
        time.sleep(0.2)
        return True


def main():
    rclpy.init()
    node = MoveItActionClient()
    node.wait_for_server()
    
    # ---------- 预设四个坐标点 ----------
    POSES = {
        '1': {'x': 0.501042, 'y': 0.246609, 'z': 0.049905, 'qx': 0.12185194680680378, 'qy': -0.7256210488103221, 'qz': -0.12505568981132548, 'qw': -0.6655728893431684},
        '2': {'x': 0.32700399999999996, 'y': -0.256285, 'z': 0.03959, 'qx': 0.16870791136290536, 'qy': 0.6916031793914055, 'qz': -0.1343292474190838, 'qw': 0.6893318041314269},
        '3': {'x': 0.534307, 'y': 0.022779, 'z': 0.07013, 'qx': 0.005501635405972222, 'qy': 0.7615834706002862, 'qz': 0.0038409704231054664, 'qw': 0.6480320950867261},
        '4': {'x': 0.25826499999999997, 'y': -0.375905, 'z': 0.119211, 'qx': 0.36553141687104584, 'qy': 0.7117821039701792, 'qz': -0.26971119739879656, 'qw': 0.5357321063234295}
    }

    # 您测量出来的待机位(关节0)
    JOINT_STANDBY = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    GRIPPER_OPEN = 0.090
    GRIPPER_CLOSE = 0.050
    GRIPPER_GRAB = 0.000
    
    loop_count = 0
    try:
        print("\n\n======== 🔵 自动分拣服务端已启动 (等待 GUI 客户端指令) ========\n")
        
        # 初始运行时回到待机位
        node.move_arm_joint(JOINT_STANDBY, "初始回到零点/待机位 (Standby)")
        node.operate_gripper(GRIPPER_GRAB, "初始化闭合夹爪(待机)")
        
        while rclpy.ok():
            # 持续向客户端广播自身状态（当前为空闲）
            node.is_busy = False
            status_msg = String()
            status_msg.data = 'idle'
            node.status_pub.publish(status_msg)
            
            # 主循环，非阻塞获取 ROS2 队列回调和动作
            rclpy.spin_once(node, timeout_sec=0.1)
            
            try:
                cmd_data_str = node.cmd_queue.get_nowait()
            except queue.Empty:
                continue
                
            try:
                data = json.loads(cmd_data_str)
            except Exception as e:
                print(f"❌ 收到无法解析的指令: {cmd_data_str}, 错误: {e}")
                continue
                
            if data.get("cmd") == "quit":
                print("\n🛑 收到来自客户端的退出指令，准备复位...")
                node.move_arm_joint(JOINT_STANDBY, "退出前回到待机位")
                node.operate_gripper(GRIPPER_GRAB, "退出前闭合夹爪")
                break
                
            elif data.get("cmd") == "reset":
                node.is_busy = True
                status_msg.data = 'busy'
                node.status_pub.publish(status_msg)
                
                print("\n🔄 收到客户端指令：一键回到待机点并关闭夹爪")
                node.move_arm_joint(JOINT_STANDBY, "回到待机位")
                node.operate_gripper(GRIPPER_GRAB, "闭合夹爪(待机)")
                
            elif data.get("cmd") == "sort":
                node.is_busy = True
                status_msg.data = 'busy'
                node.status_pub.publish(status_msg)
                
                pick_pose = data["pick"]
                place_pose = data["place"]
                pick_id = data.get("pick_name", "Pick")
                place_id = data.get("place_name", "Place")
                
                POSE_PICK = pick_pose.copy()
                POSE_PICK_UP = POSE_PICK.copy()
                POSE_PICK_UP['z'] += 0.13  # 抬起脱离高 13cm
                
                POSE_PLACE = place_pose.copy()
                POSE_PLACE_PRE = POSE_PLACE.copy()
                POSE_PLACE_PRE['z'] += 0.05  # 放置位上方 5cm (连贯预放点)
                POSE_PLACE_UP = POSE_PLACE.copy()
                POSE_PLACE_UP['z'] += 0.13 # 抬起脱离高 13cm
                
                loop_count += 1
                print(f"\n\n====================== 第 {loop_count} 次分拣 (从 {pick_id} 到 {place_id}) ======================")

                # 以下为具体的机械臂序列
                # 放一个空的队列清理，以免堆积
                while not node.cmd_queue.empty():
                    node.cmd_queue.get_nowait() 
            
                success = True
                for _ in range(1):
                    # 【第零步】 在前往途中或起点提前张开夹爪
                    node.operate_gripper(GRIPPER_OPEN, "张开夹爪(准备抓取)")

                    # 【第一步】 抓取过渡(防止碰桌面)
                    if not node.move_arm_cartesian(POSE_PICK_UP, "抓取位上方过渡点"): success = False; break
                    
                    # 【第二步】 下降抓取
                    if not node.move_arm_cartesian(POSE_PICK, "进入抓取位 (Pick)"): success = False; break
                    
                    # 【第三步】 闭合夹爪
                    node.operate_gripper(GRIPPER_CLOSE, "闭合夹爪(拿取)")
                    
                    # 【第四步】 抬起脱离
                    if not node.move_arm_cartesian(POSE_PICK_UP, "提起物品 (Lift)"): success = False; break
                    
                    # 【第五步】 移动到放置位上方5cm
                    if not node.move_arm_cartesian(POSE_PLACE_PRE, "进入放置预备位 (上方5cm)", continuous=True): success = False; break

                    # 【第五点五步】 连贯微降到放置位
                    if not node.move_arm_cartesian(POSE_PLACE, "进入放置位 (Place)"): success = False; break
                    
                    # 【第六步】 松开夹爪释放物品
                    node.operate_gripper(GRIPPER_OPEN, "松开夹爪(释放)")
                    
                    # 【第七步】 脱离放置位回到上方过渡点
                    if not node.move_arm_cartesian(POSE_PLACE_UP, "撤出物品上方"): success = False; break
                
                if not success:
                    print("\n⚠️ 执行失败 (错误码可能为 99999 或其他)！启动自动退回防护...")
                    # 发送 error 状态通知客户端弹窗
                    status_msg.data = 'error'
                    node.status_pub.publish(status_msg)
                    
                    node.operate_gripper(GRIPPER_OPEN, "防碰撞提前张开夹爪")
                    node.move_arm_joint(JOINT_STANDBY, "自动回到待机点")
                    node.operate_gripper(GRIPPER_GRAB, "闭合夹爪(待机)")
                    
                    time.sleep(0.5)
                    continue

                print(f"🎉 第 {loop_count} 次回合顺利完成！休整一下...")
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n⏹️ 收到 Ctrl+C，正在复位机械臂...")
        try:
            # 唤醒或等待服务器准备完毕
            node.wait_for_server()
            node.move_arm_joint(JOINT_STANDBY, "退出前回到待机位")
            node.operate_gripper(GRIPPER_GRAB, "退出前闭合夹爪")
        except:
            pass
        print("⏹️ 动作停止，安全退出。")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()