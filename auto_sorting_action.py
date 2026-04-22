
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import queue
import json
import math
from std_msgs.msg import String
from rclpy.logging import LoggingSeverity, set_logger_level

from moveit_msgs.srv import GetCartesianPath, GetMotionPlan
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume, MotionPlanRequest, WorkspaceParameters
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# IK Solver imports
from agx_arm_msgs.msg import PoseCmd, IKSolution

class MoveItActionClient(Node):
    def __init__(self):
        super().__init__('piper_moveit_action_client')
        self._suppress_tf_old_data_logs()
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self._cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self._plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self._execute_action_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        
        self._gripper_action_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self._joint_states_pub = self.create_publisher(JointState, '/control/joint_states', 10)
        self.current_joints = None
        # 监听默认的仿真关节状态
        self._joint_states_sub = self.create_subscription(JointState, '/joint_states', self._joint_states_cb, 10)
        # 兼容真机控制节点：真机下的假底层控制板（Mock）会发布到 /control/joint_states
        # 为了让底层轨迹起步强同步、不报错 -4（CONTROL_FAILED），必须读取这个被控制板视为“当下圣旨”的假状态。
        self._control_joint_states_sub = self.create_subscription(JointState, '/control/joint_states', self._joint_states_cb, 10)
        
        
        # 服务端通信设置
        self.cmd_queue = queue.Queue()
        self.cmd_sub = self.create_subscription(String, '/sorting_cmds', self.cmd_callback, 10)
        self.status_pub = self.create_publisher(String, '/sorting_status', 10)
        self.cycle_result_pub = self.create_publisher(String, '/sorting/cycle_result', 10)
        self.is_busy = False
        self.last_planning_profile_name = ''
        self.last_planning_strategy = ''
        
        # ========== IK Solver 集成 ==========
        self.enable_ik = False  # 可开关的 IK 分支（默认关闭）
        self.ik_solution = None  # 缓存最新的 IK 解
        self.ik_ready = False
        
        # IK Solver 消息通信
        self.pose_cmd_pub = self.create_publisher(PoseCmd, '/pose_cmd', 10)
        self.ik_solution_sub = self.create_subscription(
            IKSolution, '/ik_solution', self._ik_solution_callback, 10
        )

    def _joint_states_cb(self, msg):
        if self.current_joints is None:
            self.current_joints = {}
        self.current_joints.update(dict(zip(msg.name, msg.position)))

    def _ik_solution_callback(self, msg: IKSolution):
        """缓存 IK 求解器的最新输出"""
        self.ik_solution = msg
        self.ik_ready = msg.success

    def _suppress_tf_old_data_logs(self):
        # Suppress noisy TF_OLD_DATA warnings in this process only.
        for logger_name in ('tf2', 'tf2_ros', 'tf2_buffer', 'tf2_ros_buffer'):
            try:
                set_logger_level(logger_name, LoggingSeverity.ERROR)
            except Exception:
                pass

    def cmd_callback(self, msg):
        # 简单将 JSON 命令推入队列给主线程处理
        self.cmd_queue.put(msg.data)
        
    def wait_for_server(self, timeout_sec=None):
        self.get_logger().info('等待 MoveGroup Action 服务器...')
        ok = self._action_client.wait_for_server(timeout_sec=timeout_sec)
        if ok:
            self.get_logger().info('✅ 已连接到 MoveIt2 规划器！')
        else:
            self.get_logger().error('❌ 未连接到 MoveIt2 规划器！')
        return ok

    def send_goal(
        self,
        group_name,
        constraints,
        plan_only=False,
        continuous=False,
        planning_attempts=3,
        allowed_planning_time=3.0,
    ):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        goal_msg.request.workspace_parameters.header.frame_id = 'base_link'
        
        goal_msg.request.group_name = group_name
        goal_msg.request.num_planning_attempts = int(planning_attempts)
        goal_msg.request.allowed_planning_time = float(allowed_planning_time)
        
        # 强制将规划起点绑定到我们追踪的 MOCK/底层的最新状态，杜绝真实机械臂状态漂移导致的 -4 错误
        if self.current_joints:
            # 【仿真兼容性修复】：只提取 URDF 中标准存在的关节（joint1~6 和标准夹爪关节）。
            # 过滤掉在仿真里或者老版本夹爪传过来的 'joint7', 'joint8', 'gripper' 等废弃伪名字，
            # 否则 MoveIt 计算时校验 URDF 发现找不到这几个名字，会直接全部拒签 (Joint not found in model)。
            valid_keys = [f"joint{i}" for i in range(1, 7)] + ['gripper_joint1', 'gripper_joint2']
            names = []
            positions = []
            for k in valid_keys:
                if k in self.current_joints:
                    names.append(k)
                    positions.append(self.current_joints[k])
                    
            goal_msg.request.start_state.joint_state.name = names
            goal_msg.request.start_state.joint_state.position = positions
            
        # 适度提升速度，减少整体节拍时间。
        goal_msg.request.max_velocity_scaling_factor = 0.65
        goal_msg.request.max_acceleration_scaling_factor = 0.65
        
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
            if plan_only:
                return True, result.planned_trajectory
            # 留一点时间让机械臂稳定，避免启动状态在碰撞或未到位状态
            if continuous:
                time.sleep(0.05) # 连贯动作减少停顿
            else:
                time.sleep(0.5) 
            return True
        else:
            self.get_logger().error(f'⚠️ 执行失败，错误码: {result.error_code.val}')
            if plan_only:
                return False, None
            return False

    def move_arm_joint(self, joints, desc, continuous=False):
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
        return self.send_goal('arm', c, continuous=continuous)

    @staticmethod
    def _normalize(v):
        n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        if n < 1e-9:
            return [0.0, 0.0, -1.0]
        return [v[0] / n, v[1] / n, v[2] / n]

    @staticmethod
    def _cross(a, b):
        return [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]

    @staticmethod
    def _matrix_to_quaternion(x_axis, y_axis, z_axis):
        m00, m01, m02 = x_axis[0], y_axis[0], z_axis[0]
        m10, m11, m12 = x_axis[1], y_axis[1], z_axis[1]
        m20, m21, m22 = x_axis[2], y_axis[2], z_axis[2]
        trace = m00 + m11 + m22
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (m21 - m12) / s
            qy = (m02 - m20) / s
            qz = (m10 - m01) / s
        elif (m00 > m11) and (m00 > m22):
            s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
            qw = (m21 - m12) / s
            qx = 0.25 * s
            qy = (m01 + m10) / s
            qz = (m02 + m20) / s
        elif m11 > m22:
            s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
            qw = (m02 - m20) / s
            qx = (m01 + m10) / s
            qy = 0.25 * s
            qz = (m12 + m21) / s
        else:
            s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
            qw = (m10 - m01) / s
            qx = (m02 + m20) / s
            qy = (m12 + m21) / s
            qz = 0.25 * s
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def _build_pick_orientation(self, target):
        z_axis = [0.0, 0.0, -1.0]
        radial_xy = [float(target['x']), float(target['y']), 0.0]
        x_axis_1 = self._normalize(radial_xy)
        if abs(x_axis_1[0]) < 1e-6 and abs(x_axis_1[1]) < 1e-6:
            x_axis_1 = [1.0, 0.0, 0.0]
        y_axis_1 = self._cross(z_axis, x_axis_1)
        y_axis_1 = self._normalize(y_axis_1)
        quat1 = self._matrix_to_quaternion(x_axis_1, y_axis_1, z_axis)

        # 反向180度抓取姿态(规避腕关节限位)
        x_axis_2 = [-x for x in x_axis_1]
        y_axis_2 = [-y for y in y_axis_1]
        quat2 = self._matrix_to_quaternion(x_axis_2, y_axis_2, z_axis)

        return [quat1, quat2]

    def _build_goal_constraints(
        self,
        pose_dict,
        orientation=None,
        pos_tolerance_radius=0.012,
        ori_tolerances=(0.2, 0.2, 3.14159),
    ):
        c = Constraints()
        
        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'
        pc.link_name = 'link6'
        
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [float(pos_tolerance_radius)]
        
        p = Pose()
        p.position.x = float(pose_dict['x'])
        p.position.y = float(pose_dict['y'])
        p.position.z = float(pose_dict['z'])
        
        v = BoundingVolume()
        v.primitives.append(s)
        v.primitive_poses.append(p)
        pc.constraint_region = v
        pc.weight = 1.0
        
        c.position_constraints.append(pc)

        if orientation is not None:
            oc = OrientationConstraint()
            oc.header.frame_id = 'base_link'
            oc.link_name = 'link6'
            oc.orientation = orientation
            oc.absolute_x_axis_tolerance = float(ori_tolerances[0])
            oc.absolute_y_axis_tolerance = float(ori_tolerances[1])
            oc.absolute_z_axis_tolerance = float(ori_tolerances[2])
            oc.weight = 1.0
            c.orientation_constraints.append(oc)

        return c

    def _create_pose(self, pose_dict, orientation):
        p = Pose()
        p.position.x = float(pose_dict['x'])
        p.position.y = float(pose_dict['y'])
        p.position.z = float(pose_dict['z'])
        if type(orientation) in (list, tuple) and len(orientation) == 4:
            p.orientation = Quaternion(x=float(orientation[0]), y=float(orientation[1]), z=float(orientation[2]), w=float(orientation[3]))
        elif isinstance(orientation, Quaternion):
            p.orientation = orientation
        return p
        
    def execute_cartesian_path(self, waypoints, desc, fraction_threshold=0.85):
        print(f"\n🚀 正在快速计算(笛卡尔直线连线) -> {desc}")
        if not self._cartesian_client.wait_for_service(timeout_sec=1.5):
            self.get_logger().error("⚠️ GetCartesianPath 服务不可用")
            return False
            
        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = 'arm'
        # 强制将规划起点绑定到我们追踪的 MOCK/底层的最新状态，杜绝真实机械臂状态漂移导致的 -4 错误
        if self.current_joints:
            valid_keys = [f"joint{i}" for i in range(1, 7)] + ['gripper_joint1', 'gripper_joint2']
            names = []
            positions = []
            for k in valid_keys:
                if k in self.current_joints:
                    names.append(k)
                    positions.append(self.current_joints[k])
            req.start_state.joint_state.name = names
            req.start_state.joint_state.position = positions
        req.waypoints = waypoints
        req.max_step = 0.005      # 极高细分度: 5mm 一步，提升控制轨迹顺滑度
        req.jump_threshold = 1.6 # 忽视轻微的冗余突变
        req.avoid_collisions = False # 端点在物品附近时关闭防碰撞，防止被误挡
        
        future = self._cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        
        if res is None:
            self.get_logger().error("计算失败: 服务无响应")
            return False
            
        if res.fraction < fraction_threshold:
            self.get_logger().warning(f"⚠️ 笛卡尔规划度不足 (\x1b[33m{res.fraction*100:.1f}%\x1b[0m)，尝试其他手段...")
            return False
            
        self.get_logger().info(f"✅ 计算完成，完整度 {res.fraction*100:.1f}%，准备极速执行...")
        if not self._execute_action_client.wait_for_server(timeout_sec=1.5):
            self.get_logger().error("⚠️ ExecuteTrajectory 动作服务器不可用")
            return False
            
        exec_req = ExecuteTrajectory.Goal()
        # Print for debugging
        
        res.solution.joint_trajectory.header.stamp.sec = 0
        res.solution.joint_trajectory.header.stamp.nanosec = 0
        
        # 【终极防封杀补偿】: 起步防抖与物理对齐
        # 笛卡尔插补同理，防止执行间隙带来的机械臂实际物理位姿与当初计算位姿存在毫厘之差被严苛的控制器(-4)一刀切。
        if self.current_joints and len(res.solution.joint_trajectory.points) > 0:
            pt0 = res.solution.joint_trajectory.points[0]
            if pt0.time_from_start.sec == 0 and pt0.time_from_start.nanosec == 0:
                pt0.velocities = [0.0] * len(pt0.velocities)
                pt0.accelerations = [0.0] * len(pt0.accelerations)
                for i, jname in enumerate(res.solution.joint_trajectory.joint_names):
                    if jname in self.current_joints:
                        pt0.positions[i] = self.current_joints[jname]
                        
        exec_req.trajectory = res.solution
        
        future_exec = self._execute_action_client.send_goal_async(exec_req)
        rclpy.spin_until_future_complete(self, future_exec)
        goal_handle = future_exec.result()
        if not goal_handle.accepted:
            self.get_logger().error("轨迹被拒绝执行")
            return False
            
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        res_exec = res_future.result()
        if res_exec and res_exec.result.error_code.val == 1:
            self.get_logger().info(f"\x1b[32m✅ 直线插补执行完毕: {desc}\x1b[0m")
            return True
            
        self.get_logger().error(f"❌ 轨迹执行失败，错误码 {res_exec.result.error_code.val}")
        return False
    
    def move_arm_via_ik(self, pose_dict, orientation_quat, desc):
        """
        使用 Pinocchio IK 求解关键点，作为 MoveIt2 的初值或备选方案
        
        Args:
            pose_dict: {'x', 'y', 'z'} 目标位置
            orientation_quat: [qx, qy, qz, qw] 目标四元数
            desc: 动作描述
        
        Returns:
            (success, joint_angles) 或 (False, None)
        """
        if not self.enable_ik or not self.pose_cmd_pub.get_subscription_count() > 0:
            print(f"⚠️ IK 求解器未启用或未准备好，跳过 IK 规划")
            return False, None
        
        print(f"\n🤖 正在调用 Pinocchio IK 求解机制 -> {desc}")
        
        # 构造 IK 请求
        pose_cmd = PoseCmd()
        pose_cmd.x = float(pose_dict['x'])
        pose_cmd.y = float(pose_dict['y'])
        pose_cmd.z = float(pose_dict['z'])
        pose_cmd.qx = float(orientation_quat[0])
        pose_cmd.qy = float(orientation_quat[1])
        pose_cmd.qz = float(orientation_quat[2])
        pose_cmd.qw = float(orientation_quat[3])
        pose_cmd.gripper_target = 0.0  # IK 仅用于关节求解，夹爪独立控制
        
        # 发送 IK 请求
        self.pose_cmd_pub.publish(pose_cmd)
        
        # 等待 IK 解（最多等待 2 秒）
        wait_start = time.time()
        while time.time() - wait_start < 2.0:
            if self.ik_ready and self.ik_solution:
                sol = self.ik_solution
                print(f"✅ IK 求解完成: error={sol.error:.4f}, time={sol.computation_time*1000:.1f}ms")
                
                if sol.success and sol.error < 0.1:  # 求解成功且误差小
                    joint_angles = [
                        sol.joint1, sol.joint2, sol.joint3,
                        sol.joint4, sol.joint5, sol.joint6
                    ]
                    self.ik_ready = False  # 重置标志
                    return True, joint_angles
                else:
                    print(f"⚠️ IK 求解成功但精度不足 (error={sol.error:.4f})")
                    self.ik_ready = False
                    return False, None
            
            time.sleep(0.05)
        
        print(f"⚠️ IK 求解超时 (无响应超过 2.0 秒)")
        return False, None

    def move_arm_cartesian(
        self,
        pose_dict,
        desc,
        continuous=False,
        preferred_orientation=None,
        allow_position_only_fallback=True,
        planning_mode='normal',
    ):
        print(f"\n🚀 正在规划(笛卡尔位置) -> {desc}")

        orientations_to_try = []
        if type(preferred_orientation) in (list, tuple) and len(preferred_orientation) == 4 and all(isinstance(x, (int, float)) for x in preferred_orientation):
            orientations_to_try = [Quaternion(x=float(preferred_orientation[0]), y=float(preferred_orientation[1]), z=float(preferred_orientation[2]), w=float(preferred_orientation[3]))]
        elif preferred_orientation is not None:
            if isinstance(preferred_orientation, list):
                orientations_to_try = preferred_orientation
            else:
                orientations_to_try = [preferred_orientation]
        else:
            # 自动计算，返回包含2个姿态的列表(正向和反向)
            orientations_to_try = self._build_pick_orientation(pose_dict)

        planning_profiles = []

        def add_profile(name, pos, ori_tol, attempts, t):
            for idx, ori in enumerate(orientations_to_try):
                suffix = "(反向)" if idx == 1 else ""
                planning_profiles.append({
                    'name': f"{name}{suffix}",
                    'orientation': ori,
                    'pos_tol': pos,
                    'ori_tol': ori_tol,
                    'attempts': attempts,
                    'time': t,
                })

        if planning_mode == 'descend':
            add_profile('下探位姿(稳健)', 0.008, (0.10, 0.10, 0.15), 4, 4.0)
            add_profile('下探位姿(放宽)', 0.015, (0.25, 0.25, 0.50), 5, 5.0)
            add_profile('下探位姿(极大放宽)', 0.025, (0.40, 0.40, 3.14159), 5, 4.0)
        elif planning_mode == 'retreat':
            add_profile('抬升位姿(稳健)', 0.010, (0.14, 0.14, 0.20), 4, 4.0)
            add_profile('抬升位姿(放宽)', 0.020, (0.30, 0.30, 0.60), 5, 5.0)
            add_profile('抬升位姿(极大放宽)', 0.030, (0.50, 0.50, 3.14159), 5, 4.0)
        else:
            add_profile('直解位姿(快速)', 0.008, (0.08, 0.08, 0.10), 2, 2.5)
            add_profile('直解位姿(放宽)', 0.015, (0.25, 0.25, 0.40), 4, 3.5)
            # 自由偏航兜底，不需要区分正反向，直接加一个就好
            planning_profiles.append({
                'name': '直解位姿(自由偏航)',
                'orientation': orientations_to_try[0],
                'pos_tol': 0.020,
                'ori_tol': (0.35, 0.35, 3.14159),
                'attempts': 4,
                'time': 3.0,
            })

        if allow_position_only_fallback:
            planning_profiles.append({
                'name': '仅位置约束(兜底)',
                'orientation': None,
                'pos_tol': 0.014,
                'ori_tol': (3.14159, 3.14159, 3.14159),
                'attempts': 2,
                'time': 2.0,
            })

        self.get_logger().info(f"⚡ 开始并发计算 {len(planning_profiles)} 种位姿解...")
        import asyncio
        import concurrent.futures

        futures = []
        for i, profile in enumerate(planning_profiles):
            req = GetMotionPlan.Request()
            req.motion_plan_request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
            req.motion_plan_request.workspace_parameters.header.frame_id = 'base_link'
            req.motion_plan_request.group_name = 'arm'
            # 强制将规划起点绑定到我们追踪的 MOCK/底层的最新状态，杜绝真实机 械臂状态漂移导致的 -4 错误
            if self.current_joints:
                valid_keys = [f"joint{i}" for i in range(1, 7)] + ['gripper_joint1', 'gripper_joint2']
                names = []
                positions = []
                for k in valid_keys:
                    if k in self.current_joints:
                        names.append(k)
                        positions.append(self.current_joints[k])
                req.motion_plan_request.start_state.joint_state.name = names
                req.motion_plan_request.start_state.joint_state.position = positions
            req.motion_plan_request.num_planning_attempts = int(profile['attempts'])
            req.motion_plan_request.allowed_planning_time = float(profile['time'])
            req.motion_plan_request.max_velocity_scaling_factor = 0.65
            req.motion_plan_request.max_acceleration_scaling_factor = 0.65
            
            c = self._build_goal_constraints(
                pose_dict,
                orientation=profile['orientation'],
                pos_tolerance_radius=profile['pos_tol'],
                ori_tolerances=profile['ori_tol'],
            )
            req.motion_plan_request.goal_constraints.append(c)
            # send fast async requests
            f = self._plan_client.call_async(req)
            futures.append((i, profile, f))

        while rclpy.ok() and any(not f.done() for _, _, f in futures):
            rclpy.spin_once(self, timeout_sec=0.01)

        best_profile = None
        best_trajectory = None
        
        # Sort by priority index (0 is best, most constrained)
        for i, p, f in sorted(futures, key=lambda x: x[0]):
            res = f.result()
            if res and res.motion_plan_response.error_code.val == 1:
                traj = res.motion_plan_response.trajectory
                num_pts = len(traj.joint_trajectory.points)
                if num_pts > 0:
                    best_profile = p
                    best_trajectory = traj
                    break

        if best_profile is not None:
            self.get_logger().info(f"\x1b[32m🎉 并发计算成功！选用最优解策略: {best_profile['name']}\x1b[0m")
            self.last_successful_orientation = best_profile['orientation']
            self.last_planning_profile_name = str(best_profile.get('name', ''))
            if '自由偏航' in self.last_planning_profile_name:
                self.last_planning_strategy = 'free_yaw'
            elif '放宽' in self.last_planning_profile_name or '极大放宽' in self.last_planning_profile_name:
                self.last_planning_strategy = 'relaxed'
            elif '快速' in self.last_planning_profile_name or '稳健' in self.last_planning_profile_name:
                self.last_planning_strategy = 'direct'
            else:
                self.last_planning_strategy = 'position_only'
            
            # Execute optimal trajectory
            exec_req = ExecuteTrajectory.Goal()
            
            
            # 【重要修复】时间戳必须置空 (0)，代表立刻执行，避免因通信与排队耗时被底层控制器认定为"过期的过去轨迹"并瞬间拒绝（错误码 -4）。
            best_trajectory.joint_trajectory.header.stamp.sec = 0
            best_trajectory.joint_trajectory.header.stamp.nanosec = 0
            
            # 【终极防封杀补偿】: 起步防抖与物理对齐
            # 机器人从接受规划起，经过1秒以上计算后，手臂受重力或上次命令的余震可能会抖动或轻微下垂，偏移超出 0.01 弧度。
            # 如果它强行从规划时刻(过去)的起点继续走的话，由于 `path_tolerance` 限制非常灵敏，控制板瞬间判定位置异常(`-4`)！
            # 解决办法：直接将首个点的起点【强行同步覆盖为当前传感器的真实关节姿态】，并且速度清零。这样控制板检查起点差异绝对是 0 误差！完美的启动。
            if self.current_joints and len(best_trajectory.joint_trajectory.points) > 0:
                pt0 = best_trajectory.joint_trajectory.points[0]
                if pt0.time_from_start.sec == 0 and pt0.time_from_start.nanosec == 0:
                    pt0.velocities = [0.0] * len(pt0.velocities)
                    pt0.accelerations = [0.0] * len(pt0.accelerations)
                    for i, jname in enumerate(best_trajectory.joint_trajectory.joint_names):
                        if jname in self.current_joints:
                            pt0.positions[i] = self.current_joints[jname]
                            
            exec_req.trajectory = best_trajectory
            future_exec = self._execute_action_client.send_goal_async(exec_req)
            rclpy.spin_until_future_complete(self, future_exec)
            goal_handle = future_exec.result()
            if not goal_handle.accepted:
                self.get_logger().error("被拒绝执行")
                return False
                
            res_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)
            res_exec = res_future.result()
            if res_exec and res_exec.result.error_code.val == 1:
                self.get_logger().info(f"\x1b[32m✅ 执行完毕: {desc}\x1b[0m")
                if continuous:
                    time.sleep(0.05)
                else:
                    time.sleep(0.5)
                return True
            else:
                ec = res_exec.result.error_code.val if res_exec else 'Unknown'
                self.get_logger().error(f"❌ 轨迹执行失败，错误码 {ec}")
                return False
                
        self.get_logger().error("❌ 所有规划策略全部失败！")
        return False

    def _try_gripper_action(self, joint_names, positions, timeout_sec=1.0):
        if not self._gripper_action_client.wait_for_server(timeout_sec=timeout_sec):
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0
        goal_msg.trajectory.points.append(point)

        send_goal_future = self._gripper_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()
        if result is None:
            return False
        return True

    def _publish_gripper_joint_state(self, target_pos):
        # Unified fallback for both real arm and simulation bridge.
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['gripper']
        msg.position = [float(target_pos)]
        msg.velocity = []
        msg.effort = [1.5]
        self._joint_states_pub.publish(msg)

    def operate_gripper(self, target_pos, desc):
        print(f"✊ 正在执行夹爪动作 -> {desc} (target: {target_pos})")

        # 1) Try common MoveIt/gripper controller naming first.
        if self._try_gripper_action(
            ['gripper_joint1', 'gripper_joint2'],
            [float(target_pos * 0.5), float(-target_pos * 0.5)],
            timeout_sec=0.8,
        ):
            self.get_logger().info('⭕ 夹爪动作执行中（gripper_joint1/2）...')
            time.sleep(0.2)
            return True

        # 2) Try Gazebo piper controller naming.
        if self._try_gripper_action(
            ['joint7'],
            [float(target_pos * 0.5)],
            timeout_sec=0.4,
        ):
            self.get_logger().info('⭕ 夹爪动作执行中（joint7）...')
            time.sleep(0.2)
            return True

        # 3) Fallback: publish unified control topic used by real arm and sim bridge.
        self.get_logger().warn('⚠️ 夹爪 action 不可用或被拒绝，回退到 /control/joint_states')
        self._publish_gripper_joint_state(target_pos)
        time.sleep(0.2)
        return True


def main():
    rclpy.init()
    node = MoveItActionClient()
    if not node.wait_for_server(timeout_sec=10.0):
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # ========== IK 分支启用/禁用控制 ==========
    # 修改此处来启用/禁用 Pinocchio IK 求解器分支（默认 False，避免 pinocchio 未安装导致崩溃）
    enable_ik_solver = False  # <-- 设置为 True 来启用 IK 分支
    node.enable_ik = enable_ik_solver
    if node.enable_ik:
        print("✅ Pinocchio IK 分支已启用 - 将在下降/放置步骤中尝试 IK 求解")
    else:
        print("⚠️ Pinocchio IK 分支已禁用 - 使用经典 MoveIt2 规划器")
    
    # 您测量出来的待机位(关节0)
    JOINT_STANDBY = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    GRIPPER_OPEN = 0.090
    GRIPPER_CLOSE = 0.060
    GRIPPER_GRAB = 0.000
    GRIPPER_SETTLE_SEC = 0.30
    GRIPPER_REGRIP_DELTA = 0.006

    def compute_gripper_targets(object_diameter_m):
        # target_pos is the signed opening magnitude in this action server.
        # Use small squeeze margin to hold the object while avoiding deep penetration.
        if object_diameter_m is None:
            return GRIPPER_OPEN, GRIPPER_CLOSE
        d = float(object_diameter_m)
        d = max(0.040, min(0.085, d))
        open_target = max(0.060, min(0.095, d + 0.020))
        close_target = max(0.036, min(0.080, d - 0.006))
        if close_target >= open_target:
            close_target = max(0.038, open_target - 0.006)
        return open_target, close_target
    
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
                node.last_planning_profile_name = ''
                node.last_planning_strategy = ''
                cycle_strategies = []
                cycle_profiles = []
                object_diameter_m = data.get("object_diameter_m", None)
                dynamic_open, dynamic_close = compute_gripper_targets(object_diameter_m)
                
                pick_pose = data["pick"]
                place_pose = data["place"]
                pick_id = data.get("pick_name", "Pick")
                place_id = data.get("place_name", "Place")
                cycle_id = str(data.get("cycle_id", ""))
                
                POSE_PICK = pick_pose.copy()
                # 预留夹爪的物理长度（约10cm），防止法兰盘带着夹爪一头扎进桌面里
                POSE_PICK['z'] += 0.05
                POSE_PICK_UP = POSE_PICK.copy()
                POSE_PICK_UP['z'] += 0.13  # 抬起脱离高 13cm
                POSE_LIFT_SOFT = POSE_PICK.copy()
                POSE_LIFT_SOFT['z'] += 0.035  # 先小幅抬起，降低惯性导致的滑脱
                
                POSE_PLACE = place_pose.copy()
                POSE_PLACE['z'] += 0.05  # 同样预留夹爪的长度 10cm
                POSE_PLACE_PRE = POSE_PLACE.copy()
                POSE_PLACE_PRE['z'] += 0.08  # 放置位上方 8cm (连贯预放点)
                POSE_PLACE_UP = POSE_PLACE.copy()
                POSE_PLACE_UP['z'] += 0.13 # 抬起脱离高 13cm
                
                loop_count += 1
                print(f"\n\n====================== 第 {loop_count} 次分拣 (从 {pick_id} 到 {place_id}) ======================")

                # 以下为具体的机械臂序列
                # 放一个空的队列清理，以免堆积
                while not node.cmd_queue.empty():
                    node.cmd_queue.get_nowait() 
            
                success = True
                grasp_reached_ok = False
                place_reached_ok = False
                for _ in range(1):
                    # 【第零步】 在前往途中或起点提前张开夹爪
                    node.operate_gripper(dynamic_open, "张开夹爪(准备抓取)")

                    # 【第一步】 抓取过渡(防止碰桌面)
                    pick_orientation = node._build_pick_orientation(POSE_PICK)
                    place_orientation = node._build_pick_orientation(POSE_PLACE)

                    if not node.move_arm_cartesian(POSE_PICK_UP, "抓取位上方过渡点", preferred_orientation=pick_orientation, allow_position_only_fallback=False): success = False; break
                    if node.last_planning_strategy:
                        cycle_strategies.append(node.last_planning_strategy)
                    if node.last_planning_profile_name:
                        cycle_profiles.append(node.last_planning_profile_name)
                    
                    # 记录抓取过渡点的关节角，用于百分百安全抬起
                    pick_up_joints = node.current_joints.copy() if node.current_joints else None

                    # 【第二步】 下降抓取 (直线插补为主，遇限位转并发规划)
                    active_ori = getattr(node, 'last_successful_orientation', pick_orientation[0])
                    pose_pick_msg = node._create_pose(POSE_PICK, active_ori)
                    
                    # ========== 可选 IK 分支（当 enable_ik 为 True 时） ==========
                    descent_success = False
                    if node.enable_ik:
                        print("💡 IK 分支已启用，尝试 Pinocchio 求解...")
                        ik_ok, ik_joints = node.move_arm_via_ik(POSE_PICK, [active_ori.x, active_ori.y, active_ori.z, active_ori.w], "下降抓取 (IK 求解)")
                        if ik_ok and ik_joints:
                            print(f"🎯 IK 求解成功，执行关节空间轨迹: {[f'{j:.3f}' for j in ik_joints]}")
                            if node.move_arm_joint(ik_joints, "下降抓取 (IK 方案)", continuous=True):
                                descent_success = True
                                cycle_strategies.append('ik_joint_space')
                                cycle_profiles.append('ik_solution')
                            else:
                                print("⚠️ IK 关节执行失败，回退到 MoveIt2...")
                    
                    if not descent_success:
                        # ========== 原有 MoveIt2 回退方案 ==========
                        if not node.execute_cartesian_path([pose_pick_msg], "下降抓取 (直线插补)"):
                            print("🟡 直线插补受限！启动多路并发退避规划...")
                            if not node.move_arm_cartesian(POSE_PICK, "下降抓取 (退避规划)", continuous=True, preferred_orientation=active_ori, allow_position_only_fallback=True, planning_mode='descend'):
                                success = False; break
                            if node.last_planning_strategy:
                                cycle_strategies.append(node.last_planning_strategy)
                            if node.last_planning_profile_name:
                                cycle_profiles.append(node.last_planning_profile_name)

                    
                    # 【第三步】 闭合夹爪
                    node.operate_gripper(dynamic_close, "闭合夹爪(拿取)")
                    # 闭合后短暂停顿，让物体接触稳定后再上升，减少滑脱/弹飞。
                    time.sleep(GRIPPER_SETTLE_SEC)

                    # 二次补压：在稳定接触后再微量夹紧，提升竖直抬升抗滑能力。
                    regrip_target = max(0.030, dynamic_close - GRIPPER_REGRIP_DELTA)
                    node.operate_gripper(regrip_target, "二次补压(防滑)")
                    time.sleep(0.25)
                    
                    # 【第四步】 抬起脱离 (一键无缝连贯倒回之前的安全关节快照空间)
                    if pick_up_joints:
                        sorted_joints = [pick_up_joints[f'joint{i}'] for i in range(1, 7)]
                        if not node.move_arm_joint(sorted_joints, "抬起脱离 (直接原路逆向)", continuous=True): success = False; break
                        grasp_reached_ok = True
                    else:
                        # 如果没有快照兜底使用之前的倒回方式
                        POSE_LIFT = POSE_PICK.copy()
                        POSE_LIFT['z'] = POSE_PICK_UP['z']
                        if not node.move_arm_cartesian(POSE_LIFT, "抬起脱离 (退避规划)", continuous=True, preferred_orientation=active_ori, allow_position_only_fallback=True, planning_mode='retreat'):
                            success = False; break
                        if node.last_planning_strategy:
                            cycle_strategies.append(node.last_planning_strategy)
                        if node.last_planning_profile_name:
                            cycle_profiles.append(node.last_planning_profile_name)
                        grasp_reached_ok = True
                    
                    # 【第五步】 移动到放置位上方5cm
                    if not node.move_arm_cartesian(POSE_PLACE_PRE, "进入放置预备位 (上方5cm)", continuous=True, preferred_orientation=place_orientation, allow_position_only_fallback=True): success = False; break
                    if node.last_planning_strategy:
                        cycle_strategies.append(node.last_planning_strategy)
                    if node.last_planning_profile_name:
                        cycle_profiles.append(node.last_planning_profile_name)
                    
                    # 记录放置过渡点的关节角，用于百分百安全抬起
                    place_up_joints = node.current_joints.copy() if node.current_joints else None

                    # 【第五点五步】 连贯微降到放置位 (直线插补为主，遇限位转并发规划)
                    active_ori = getattr(node, 'last_successful_orientation', place_orientation[0])
                    pose_place_msg = node._create_pose(POSE_PLACE, active_ori)
                    
                    # ========== 可选 IK 分支（当 enable_ik 为 True 时） ==========
                    place_success = False
                    if node.enable_ik:
                        print("💡 IK 分支已启用，尝试 Pinocchio 求解...")
                        ik_ok, ik_joints = node.move_arm_via_ik(POSE_PLACE, [active_ori.x, active_ori.y, active_ori.z, active_ori.w], "进入放置位 (IK 求解)")
                        if ik_ok and ik_joints:
                            print(f"🎯 IK 求解成功，执行关节空间轨迹: {[f'{j:.3f}' for j in ik_joints]}")
                            if node.move_arm_joint(ik_joints, "进入放置位 (IK 方案)", continuous=True):
                                place_reached_ok = True
                                place_success = True
                                cycle_strategies.append('ik_joint_space')
                                cycle_profiles.append('ik_solution')
                            else:
                                print("⚠️ IK 关节执行失败，回退到 MoveIt2...")
                    
                    if not place_success:
                        # ========== 原有 MoveIt2 回退方案 ==========
                        if not node.execute_cartesian_path([pose_place_msg], "进入放置位 (直线插补)"):
                            print("🟡 直线插补受限！启动多路并发退避规划...")
                            if not node.move_arm_cartesian(POSE_PLACE, "进入放置位 (退避规划)", preferred_orientation=active_ori, allow_position_only_fallback=True, planning_mode='descend'):
                                success = False; break
                            if node.last_planning_strategy:
                                cycle_strategies.append(node.last_planning_strategy)
                            if node.last_planning_profile_name:
                                cycle_profiles.append(node.last_planning_profile_name)
                            place_reached_ok = True
                        else:
                            place_reached_ok = True

                    
                    # 【第六步】 松开夹爪释放物品
                    node.operate_gripper(dynamic_open, "松开夹爪(释放)")
                    
                    # 【第七步】 撤出放置位回到上方过渡点 (一键无缝连贯倒回之前的安全关节快照空间)
                    if place_up_joints:
                        sorted_joints = [place_up_joints[f'joint{i}'] for i in range(1, 7)]
                        if not node.move_arm_joint(sorted_joints, "撤出物品上方 (直接原路逆向)", continuous=True): success = False; break
                    else:
                        if not node.move_arm_cartesian(POSE_PLACE_UP, "撤出物品上方 (退避规划)", preferred_orientation=active_ori, allow_position_only_fallback=True, planning_mode='retreat'):
                            success = False; break
                        if node.last_planning_strategy:
                            cycle_strategies.append(node.last_planning_strategy)
                        if node.last_planning_profile_name:
                            cycle_profiles.append(node.last_planning_profile_name)

                final_strategy = 'cartesian_only'
                if 'free_yaw' in cycle_strategies:
                    final_strategy = 'free_yaw'
                elif 'relaxed' in cycle_strategies:
                    final_strategy = 'relaxed'
                elif 'direct' in cycle_strategies:
                    final_strategy = 'direct'
                elif 'position_only' in cycle_strategies:
                    final_strategy = 'position_only'
                
                if not success:
                    print("\n⚠️ 执行失败 (错误码可能为 99999 或其他)！启动自动退回防护...")
                    # 发送 error 状态通知客户端弹窗
                    status_msg.data = 'error'
                    node.status_pub.publish(status_msg)
                    result_msg = String()
                    result_msg.data = json.dumps({
                        'cycle_success': 0,
                        'cycle_id': cycle_id,
                        'grasp_reached_ok': int(grasp_reached_ok),
                        'place_reached_ok': int(place_reached_ok),
                        'planning_strategy': final_strategy,
                        'planning_profile': cycle_profiles[-1] if cycle_profiles else '',
                        'planning_profiles': cycle_profiles,
                        'object_diameter_m': object_diameter_m,
                        'gripper_open_target': round(dynamic_open, 4),
                        'gripper_close_target': round(dynamic_close, 4),
                    }, ensure_ascii=False)
                    node.cycle_result_pub.publish(result_msg)
                    
                    node.operate_gripper(GRIPPER_OPEN, "防碰撞提前张开夹爪")
                    node.move_arm_joint(JOINT_STANDBY, "自动回到待机点")
                    node.operate_gripper(GRIPPER_GRAB, "闭合夹爪(待机)")
                    
                    time.sleep(0.5)
                    continue

                print(f"🎉 第 {loop_count} 次回合顺利完成！休整一下...")
                result_msg = String()
                result_msg.data = json.dumps({
                    'cycle_success': 1,
                    'cycle_id': cycle_id,
                    'grasp_reached_ok': int(grasp_reached_ok),
                    'place_reached_ok': int(place_reached_ok),
                    'planning_strategy': final_strategy,
                    'planning_profile': cycle_profiles[-1] if cycle_profiles else '',
                    'planning_profiles': cycle_profiles,
                    'object_diameter_m': object_diameter_m,
                    'gripper_open_target': round(dynamic_open, 4),
                    'gripper_close_target': round(dynamic_close, 4),
                }, ensure_ascii=False)
                node.cycle_result_pub.publish(result_msg)
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n⏹️ 收到 Ctrl+C，正在复位机械臂...")
        try:
            node.move_arm_joint(JOINT_STANDBY, "退出前回到待机位")
            node.operate_gripper(GRIPPER_GRAB, "退出前闭合夹爪")
        except Exception:
            pass
        print("⏹️ 动作停止，安全退出。")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            # ignore repeated shutdown in interrupted contexts
            pass

if __name__ == "__main__":
    main()