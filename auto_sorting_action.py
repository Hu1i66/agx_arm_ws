
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import threading
import queue
import json
import math
import sys
import os
import numpy as np
from std_msgs.msg import String
from rclpy.logging import LoggingSeverity, set_logger_level

from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPlanningScene
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume, MotionPlanRequest, WorkspaceParameters, PlanningScene as PlanningSceneMsg, PlanningSceneComponents, AllowedCollisionEntry
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# Pinocchio/CasADi path injection
# CRITICAL: cmeel has the REAL pinocchio (3.9.0), .venv_ik has a fake stub (0.1).
# cmeel must be at sys.path[0] to override .venv_ik's broken pinocchio.
_venv_ik_path = "/home/lxf/agx_arm_ws/.venv_ik/lib/python3.10/site-packages"
if os.path.exists(_venv_ik_path) and _venv_ik_path not in sys.path:
    sys.path.insert(0, _venv_ik_path)
_cmeel_path = "/home/lxf/.local/lib/python3.10/site-packages/cmeel.prefix/lib/python3.10/site-packages"
if os.path.exists(_cmeel_path):
    if _cmeel_path in sys.path:
        sys.path.remove(_cmeel_path)
    sys.path.insert(0, _cmeel_path)
_ws_root = '/home/lxf/agx_arm_ws'
if _ws_root not in sys.path:
    sys.path.insert(0, _ws_root)

import numpy as np
import traceback
from typing import Tuple, Optional

# ---- Pinocchio IKSolver (inlined to avoid ROS2 msg dependency in pinocchio_ik_node) ----

# State variables for lazy-loading
_PINOCCHIO_AVAILABLE = False
_CASADI_AVAILABLE = False
_pin = None

def _try_load_pinocchio():
    global _PINOCCHIO_AVAILABLE, _CASADI_AVAILABLE, _pin
    try:
        import pinocchio as pin
        _pin = pin
        _PINOCCHIO_AVAILABLE = True
    except ImportError:
        _PINOCCHIO_AVAILABLE = False
    try:
        import casadi
        _CASADI_AVAILABLE = True
    except ImportError:
        _CASADI_AVAILABLE = False


class PinocchioIKSolver:
    """Core IK solver using Pinocchio iterative DLS (no CasADi dependency at runtime)"""

    def __init__(self, urdf_path: str, end_effector_name: str = "link6"):
        _try_load_pinocchio()
        if not _PINOCCHIO_AVAILABLE:
            raise RuntimeError("Pinocchio not available")
        if not _CASADI_AVAILABLE:
            raise RuntimeError("CasADi not available")

        self.urdf_path = urdf_path
        self.ee_name = end_effector_name

        self.model = _pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.ee_frame_id = self.model.getFrameId(end_effector_name)

        self.joint_names = []
        self.joint_ids = []
        self.joint_q_indices = []
        for i in range(1, 7):
            joint_name = f"joint{i}"
            try:
                jid = self.model.getJointId(joint_name)
                self.joint_names.append(joint_name)
                self.joint_ids.append(jid)
                self.joint_q_indices.append(self.model.joints[jid].idx_q)
            except RuntimeError:
                pass

        if len(self.joint_ids) < 6:
            raise RuntimeError(f"Expected 6 DOF joints, found {len(self.joint_ids)}")

        self.ndof = len(self.joint_ids)
        self._lo = self.model.lowerPositionLimit
        self._hi = self.model.upperPositionLimit
        print(f"✅ Pinocchio model loaded: {self.ndof} DOF, EE frame: {end_effector_name}")

    def get_ik_solution(
        self,
        target_pos: np.ndarray,
        target_quat: np.ndarray,
        initial_guess: Optional[np.ndarray] = None,
        max_iter: int = 500,
        tol: float = 1e-4
    ) -> Tuple[bool, np.ndarray, float, float]:
        """Random sampling + random walk IK.
        Optimizes position accuracy + z-axis direction (downward).
        Yaw angle around z is free — suitable for grasping round objects.
        """
        t0 = time.time()

        try:
            target_pos_arr = np.array([float(target_pos[0]), float(target_pos[1]), float(target_pos[2])])
            # Target z-axis direction (extracted from target_quat rotation)
            tq = np.array(target_quat, dtype=float)
            tq = tq / max(np.linalg.norm(tq), 1e-9)
            target_se3 = _pin.XYZQUATToSE3(np.concatenate([target_pos_arr, tq]))
            target_z = target_se3.rotation[:, 2].copy()  # Desired EE z-axis in world frame

            lo6 = self._lo[:6]
            hi6 = self._hi[:6]

            def _combined_error(q_in):
                """pos_err (m) + w * z_axis_angle_error (rad)"""
                _pin.forwardKinematics(self.model, self.data, q_in)
                _pin.updateFramePlacements(self.model, self.data)
                pe = float(np.linalg.norm(
                    self.data.oMf[self.ee_frame_id].translation - target_pos_arr))
                actual_z = self.data.oMf[self.ee_frame_id].rotation[:, 2]
                cos_angle = float(np.clip(np.dot(actual_z, target_z), -1.0, 1.0))
                ze = float(np.arccos(cos_angle))  # 0=aligned, pi=opposite
                return pe + 0.06 * ze, pe, ze

            best_q = None
            best_err = float('inf')

            if initial_guess is not None:
                q0 = _pin.neutral(self.model)
                for i, idx in enumerate(self.joint_q_indices):
                    q0[idx] = float(initial_guess[i])
                ce, _, _ = _combined_error(q0)
                if ce < best_err:
                    best_err = ce
                    best_q = q0.copy()

            # Phase 1: Random uniform sampling
            n_samples = 500
            for _ in range(n_samples):
                q = np.zeros(self.model.nq)
                arm_q = np.random.uniform(lo6, hi6)
                for i, idx in enumerate(self.joint_q_indices):
                    q[idx] = arm_q[i]
                ce, pe, ze = _combined_error(q)
                if ce < best_err:
                    best_err = ce
                    best_q = q.copy()
                    if pe < 0.005 and ze < 0.15:
                        break

            if best_q is None:
                t1 = time.time()
                return False, np.zeros(self.ndof), 999.0, (t1 - t0)

            # Phase 2: Random walk refinement
            q = best_q.copy()
            n_walk = 6000
            for _ in range(n_walk):
                qt = q.copy()
                sigma = max(0.003, best_err * 6.0)
                noise = np.random.normal(0, sigma, 6)
                for i, idx in enumerate(self.joint_q_indices):
                    qt[idx] += noise[i]
                np.clip(qt, self._lo, self._hi, out=qt)

                ce, pe, ze = _combined_error(qt)
                if ce < best_err:
                    best_err = ce
                    q = qt.copy()
                    if pe < 0.005 and ze < 0.15:
                        break

            _, final_pe, final_ze = _combined_error(q)
            q_sol = np.array([q[idx] for idx in self.joint_q_indices], dtype=float)
            success = bool(final_pe < 0.015)

            t1 = time.time()
            return success, q_sol, final_pe, (t1 - t0)

        except Exception as e:
            print(f"❌ IK solver error: {e}")
            traceback.print_exc()
            t1 = time.time()
            return False, np.zeros(self.ndof), 999.0, (t1 - t0)



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
        # PlanningScene diff 话题发布器：用于临时禁用/恢复 control_box 等碰撞体的碰撞检测
        # 通过 AllowedCollisionMatrix (ACM) diff 实现，不影响场景中物体的实际存在
        self._planning_scene_pub = self.create_publisher(PlanningSceneMsg, '/planning_scene', 10)
        # GetPlanningScene 服务客户端：用于获取当前 ACM，避免 diff 替换整个 ACM 破坏自碰撞对
        self._get_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
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

        # ── 两阶段精定位：订阅 YOLO 检测结果 ──
        self._latest_detection = None
        self._detection_event = threading.Event()
        self.det_sub = self.create_subscription(String, '/detection_info', self._detection_cb, 10)
        
        # ========== IK Solver 集成（内嵌 Pinocchio）==========
        self.enable_ik = True
        self.ik_solver = None
        self._init_ik_solver()

    def _joint_states_cb(self, msg):
        if self.current_joints is None:
            self.current_joints = {}
        self.current_joints.update(dict(zip(msg.name, msg.position)))

    def _detection_cb(self, msg):
        """缓存 YOLO 检测结果，唤醒等待线程。"""
        try:
            self._latest_detection = json.loads(msg.data)
            self._detection_event.set()
        except Exception:
            pass

    def _init_ik_solver(self):
        """初始化内嵌 Pinocchio IK 求解器（无需 ROS2 topic 通信）"""
        try:
            _try_load_pinocchio()
            if not _PINOCCHIO_AVAILABLE or not _CASADI_AVAILABLE:
                self.get_logger().warn("Pinocchio or CasADi not available, IK disabled")
                self.enable_ik = False
                return

            urdf_path = '/home/lxf/agx_arm_ws/src/agx_arm_ros/src/agx_arm_moveit/config/test2.urdf'
            if not os.path.exists(urdf_path):
                self.get_logger().warn(f"URDF not found: {urdf_path}, IK disabled")
                self.enable_ik = False
                return

            self.ik_solver = PinocchioIKSolver(urdf_path, 'link6')
            self.get_logger().info("✅ Embedded Pinocchio IK solver ready")
        except Exception as e:
            import traceback
            self.get_logger().warn(f"Failed to init embedded IK solver: {e}\n{traceback.format_exc()}")
            self.enable_ik = False

    def _suppress_tf_old_data_logs(self):
        # Suppress noisy TF_OLD_DATA warnings in this process only.
        for logger_name in ('tf2', 'tf2_ros', 'tf2_buffer', 'tf2_ros_buffer'):
            try:
                set_logger_level(logger_name, LoggingSeverity.ERROR)
            except Exception:
                pass

    def _set_bin_collision_allowed(self, allowed=True, obj_ids=None):
        """临时允许/恢复指定碰撞体与机械臂所有连杆的碰撞检测。

        通过 /planning_scene 话题发布 AllowedCollisionMatrix (ACM) diff 实现。
        料框放置位规划失败时调用此方法临时绕过传送带碰撞体，
        让 RRTConnect 能在原本"碰撞"的目标状态采样成功。

        ⚠️ 关键：必须先调用 /get_planning_scene 获取当前 ACM，只修改 obj_ids 相关条目，
        保留所有原有碰撞对（包括夹爪自碰撞对 gripper_base-gripper_link1 等）。
        否则 ACM diff 会替换整个 ACM，导致夹爪自碰撞检测被启用，所有规划都失败。

        ⚠️ 料框1放置位有多个碰撞体（move_group 日志确认）：
        - 'control_box' (Object, conveyor.scene) 与 link5 碰撞
        - 'rail_ny' (Object, conveyor.scene) 与 gripper_link2 碰撞
        - 'conveyor_control_box' (Robot link, conveyor_cell.urdf.xacro) 与 gripper_link1 碰撞
        因此需要同时禁用所有传送带碰撞体。

        Args:
            allowed: True=禁用碰撞检测(允许碰撞), False=恢复碰撞检测
            obj_ids: 要操作的碰撞体名称列表，默认包含所有传送带碰撞体
        """
        if obj_ids is None:
            # 传送带所有碰撞体：conveyor.scene 中的 Object + conveyor_cell.urdf.xacro 中的 Robot link
            obj_ids = [
                # conveyor.scene 中的 Object（场景物体）
                'belt_deck', 'rail_py', 'rail_ny',
                'leg_fr', 'leg_fl', 'leg_br', 'leg_bl',
                'platform', 'control_box',
                # conveyor_cell.urdf.xacro 中的 Robot link（机器人连杆）
                'conveyor_structure', 'conveyor_belt_col', 'conveyor_control_box',
            ]
        try:
            # 1. 等待 GetPlanningScene 服务
            if not self._get_scene_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('❌ /get_planning_scene 服务不可用，无法安全修改 ACM')
                return

            # 2. 获取当前 PlanningScene 的 ACM 组件
            req = GetPlanningScene.Request()
            req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            future = self._get_scene_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result() is None:
                self.get_logger().error('❌ 获取 PlanningScene 超时，无法安全修改 ACM')
                return
            current_acm = future.result().scene.allowed_collision_matrix

            # 3. 构建新 ACM：基于当前 ACM，只修改 obj_ids 相关条目
            new_entry_names = list(current_acm.entry_names)
            # 添加缺失的 obj_ids
            for obj_id in obj_ids:
                if obj_id not in new_entry_names:
                    new_entry_names.append(obj_id)

            n = len(new_entry_names)
            # 构建当前 ACM 的查找表（name -> index）
            cur_idx = {name: i for i, name in enumerate(current_acm.entry_names)}

            new_entry_values = []
            for i, name_i in enumerate(new_entry_names):
                entry = AllowedCollisionEntry()
                row = []
                for j, name_j in enumerate(new_entry_names):
                    if i == j:
                        # 对角线：自身永远允许
                        row.append(True)
                    elif name_i in obj_ids or name_j in obj_ids:
                        # 任一端是 obj_id：按 allowed 设置（禁用/恢复碰撞检测）
                        row.append(bool(allowed))
                    else:
                        # 其他碰撞对：保持当前 ACM 中的值（保留夹爪自碰撞对等）
                        if name_i in cur_idx and name_j in cur_idx:
                            row.append(bool(current_acm.entry_values[cur_idx[name_i]].enabled[cur_idx[name_j]]))
                        else:
                            # 当前 ACM 中没有的条目：默认禁止碰撞
                            row.append(False)
                entry.enabled = row
                new_entry_values.append(entry)

            # 4. 发布修改后的 ACM diff
            ps = PlanningSceneMsg()
            ps.is_diff = True
            ps.robot_state.is_diff = True
            ps.allowed_collision_matrix.entry_names = new_entry_names
            ps.allowed_collision_matrix.entry_values = new_entry_values
            self._planning_scene_pub.publish(ps)
            # 给 move_group 一点时间合并 ACM diff
            time.sleep(0.15)
            action = "禁用" if allowed else "恢复"
            self.get_logger().info(
                f"🛡️ 已{action}碰撞检测: {obj_ids} (ACM 条目数: {n}, 保留原有 {len(current_acm.entry_names)} 条)")
        except Exception as e:
            self.get_logger().error(f"❌ 设置 ACM 失败: {e}")

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

    def _two_stage_refine(self, obs_x, obs_y, verify_height=0.25, timeout_s=5.0):
        """两阶段精定位：走到物体正上方做二次检测，近距离精度远高于远距离。
        只要二次检测到物体，直接使用其结果，不再与观察位对比。
        返回: (success, refined_xyz_or_error_msg)
        """
        verify_pose = {'x': obs_x, 'y': obs_y, 'z': verify_height}
        print(f"\n🔍 [两阶段] 移动到验证位姿: ({verify_pose['x']:.4f},{verify_pose['y']:.4f},{verify_pose['z']:.4f})")
        if not self.move_arm_pose(verify_pose, "两阶段验证位(物体正上方)", continuous=False):
            return False, "移动到验证位姿失败"

        self._detection_event.clear()
        self._latest_detection = None
        print(f"🔍 [两阶段] 等待二次检测... (超时={timeout_s}s)")

        waited = 0.0
        dt = 0.1
        detection = None
        while waited < timeout_s:
            if rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
            if self._detection_event.is_set():
                detection = self._latest_detection
                break
            time.sleep(dt)
            waited += dt

        if detection is None:
            return False, "超时未收到检测信息"
        if not detection.get('detected'):
            return False, "二次检测未发现物体"
        bp = detection.get('base_position_m', {})
        if not bp or not all(k in bp for k in ('x', 'y', 'z')):
            return False, "二次检测缺少 base_position_m"
        ver_x, ver_y, ver_z = float(bp['x']), float(bp['y']), float(bp['z'])

        print(f"✅ [两阶段] 二次检测成功! 近距离精定位坐标=({ver_x:.4f},{ver_y:.4f},{ver_z:.4f})")
        return True, {'x': ver_x, 'y': ver_y, 'z': ver_z}

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

    def _build_pick_orientations_multi(self, target, num_yaw_samples=8):
        """采样多个 yaw 角度生成候选姿态（供 IK 快速尝试）"""
        orientations = []
        z_axis = [0.0, 0.0, -1.0]

        # 优先保留径向姿态（x 指向外侧），平滑抓取
        radial_xy = [float(target['x']), float(target['y']), 0.0]
        x_radial = self._normalize(radial_xy)
        if abs(x_radial[0]) > 1e-6 or abs(x_radial[1]) > 1e-6:
            y_radial = self._cross(z_axis, x_radial)
            y_radial = self._normalize(y_radial)
            orientations.append(self._matrix_to_quaternion(x_radial, y_radial, z_axis))
            x_rev = [-v for v in x_radial]
            y_rev = [-v for v in y_radial]
            orientations.append(self._matrix_to_quaternion(x_rev, y_rev, z_axis))
        else:
            orientations.append(self._matrix_to_quaternion([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], z_axis))
            orientations.append(self._matrix_to_quaternion([-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], z_axis))

        # 均匀采样 yaw 角度
        for i in range(num_yaw_samples):
            angle = (2.0 * math.pi * i) / num_yaw_samples
            x_axis = [math.cos(angle), math.sin(angle), 0.0]
            y_axis = self._cross(z_axis, x_axis)
            y_axis = self._normalize(y_axis)
            orientations.append(self._matrix_to_quaternion(x_axis, y_axis, z_axis))

        return orientations

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
    
    def move_arm_via_ik(self, pose_dict, orientations, desc, continuous=False):
        """
        内嵌 Pinocchio IK 求解 + 关节空间执行（同步，无需 ROS2 topic）

        Args:
            pose_dict: {'x', 'y', 'z'} 目标位置
            orientations: List[Quaternion] 候选姿态列表
            desc: 动作描述
            continuous: 连贯模式

        Returns:
            (success, joint_angles) 或 (False, None)
        """
        if not self.enable_ik or self.ik_solver is None:
            return False, None

        # 用当前关节角做初始猜测
        initial_guess = None
        if self.current_joints:
            initial_guess = np.array([
                self.current_joints.get(f'joint{i}', 0.0) for i in range(1, 7)
            ])

        target_pos = np.array([float(pose_dict['x']), float(pose_dict['y']), float(pose_dict['z'])])
        best_q = None
        best_error = float('inf')
        best_time = 0.0

        for ori in orientations:
            if isinstance(ori, Quaternion):
                qx, qy, qz, qw = ori.x, ori.y, ori.z, ori.w
            else:
                qx, qy, qz, qw = float(ori[0]), float(ori[1]), float(ori[2]), float(ori[3])

            target_quat = np.array([qx, qy, qz, qw])
            ok, q_sol, err, comp_t = self.ik_solver.get_ik_solution(
                target_pos, target_quat, initial_guess=initial_guess
            )

            if ok and err < best_error:
                best_error = err
                best_q = list(q_sol)
                best_time = comp_t
                if err < 0.001:
                    break

        if best_q is None:
            print(f"⚠️ IK 无解 (尝试了 {len(orientations)} 种姿态)")
            return False, None

        print(f"✅ IK 求解成功: error={best_error:.4f}, time={best_time*1000:.1f}ms")
        if not self.move_arm_joint(best_q, f"{desc} (IK求解, err={best_error:.4f})", continuous=continuous):
            print("⚠️ IK 关节执行失败")
            return False, None

        # Save actual EE orientation from IK solution (for subsequent steps)
        try:
            import pinocchio as _pin_save
            q_full = np.zeros(self.ik_solver.model.nq)
            for i, idx in enumerate(self.ik_solver.joint_q_indices):
                q_full[idx] = best_q[i]
            _pin_save.forwardKinematics(self.ik_solver.model, self.ik_solver.data, q_full)
            _pin_save.updateFramePlacements(self.ik_solver.model, self.ik_solver.data)
            se3 = self.ik_solver.data.oMf[self.ik_solver.ee_frame_id]
            xyzquat = _pin_save.SE3ToXYZQUAT(se3)
            self.last_successful_orientation = Quaternion(
                x=float(xyzquat[3]), y=float(xyzquat[4]),
                z=float(xyzquat[5]), w=float(xyzquat[6]))
        except Exception:
            pass

        return True, best_q

    def move_arm_pose(self, pose_dict, desc, continuous=False, planning_mode='normal'):
        """
        统一运动方法：优先使用内嵌 IK + 关节空间，失败回退到 Cartesion 规划

        Args:
            pose_dict: {'x', 'y', 'z'} 目标位置
            desc: 动作描述
            continuous: 连贯模式
            planning_mode: 传递给 Cartesian 兜底的规划模式
        """
        if self.enable_ik and self.ik_solver is not None:
            orientations = self._build_pick_orientations_multi(pose_dict)
            ik_ok, _ = self.move_arm_via_ik(pose_dict, orientations, desc, continuous=continuous)
            if ik_ok:
                return True
            print("🟡 IK 路径失败，回退到 MoveIt2 笛卡尔规划...")

        return self.move_arm_cartesian(
            pose_dict, desc, continuous=continuous,
            preferred_orientation=None,
            allow_position_only_fallback=True,
            planning_mode=planning_mode,
        )

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
    # enable_ik 已在 __init__ 中通过 _init_ik_solver() 自动设置
    # 若 Pinocchio/CasADi 未安装则自动降级为 False
    enable_ik_solver = True
    node.enable_ik = enable_ik_solver and node.ik_solver is not None
    if node.enable_ik:
        print("✅ 内嵌 Pinocchio IK 已启用 - 所有运动步骤优先使用 IK 求解")
    else:
        print("⚠️ Pinocchio IK 不可用 - 回退到经典 MoveIt2 规划器")
    
    # 您测量出来的待机位(关节0)
    JOINT_STANDBY = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # 您测量出来的观察位(关节空间，用于相机观察传送带)
    # joint1=0, joint2=1.0669, joint3=-1.1637, joint4=0, joint5=1.2185, joint6=0
    # 对应末端位姿: x=0.1864, y=0.0, z=0.3782, q=(~0, 0.9643, ~0, 0.2650)
    JOINT_OBSERVE = [0.0, 1.0669023184516138, -1.1636808254746993, 0.0, 1.2184841639873214, 0.0]

    # ========== 料框关节点位 (用户标定, 关节空间) ==========
    # 抓取后流程: pick_up_joints -> bin_above -> bin_place -> 开夹爪 -> bin_above -> observe
    # 料框1上方 (末端 x=-0.086, y=0.317, z=0.173)
    JOINT_BIN1_ABOVE = [1.8195755583741684, 1.7968862780982422, -1.0943388942929646,
                        0.07860962950982461, 0.8454898495436131, 0.10438814256178085]
    # 料框1放置位 (末端 x=-0.109, y=0.329, z=-0.107)
    JOINT_BIN1_PLACE = [1.8810635079219287, 2.641887435451297, -1.1993728086779836,
                        0.2364746603527117, 0.1650732406536237, 0.060702551384362785]
    # 料框2上方 (末端 x=-0.141, y=-0.314, z=0.106)
    JOINT_BIN2_ABOVE = [-1.980896341136004, 1.9832350823336766, -1.06501736285946,
                        -0.07853981633974483, 0.6292086486364757, 0.0]
    # 料框2放置位 (末端 x=-0.145, y=-0.333, z=-0.093)
    JOINT_BIN2_PLACE = [-1.980721808210805, 2.621257643692724, -1.2797103141472823,
                        0.0, 0.31895892080196375, 0.0]
    # 料框编号 -> (上方关节, 放置位关节)
    BIN_JOINTS = {
        1: (JOINT_BIN1_ABOVE, JOINT_BIN1_PLACE),
        2: (JOINT_BIN2_ABOVE, JOINT_BIN2_PLACE),
    }

    GRIPPER_OPEN = 0.090
    GRIPPER_CLOSE = 0.060
    GRIPPER_GRAB = 0.000
    GRIPPER_SETTLE_SEC = 0.30

    # ── 抓取点 z 补偿: link6 原点到夹爪夹持点的 z 距离 ──
    # 实测 (2026-07-20): 青苹果 det_z=0.07, 正确夹住时 link6_z=0.136, offset=0.066
    # 当前值 0.072 = 实测 0.066 + 微调 0.006 (用户从 0.066 上调)
    # 原值 0.11 偏高 4.4cm 导致空夹; 若仍空夹→减小, 若碰桌面→增大, 步长 0.005
    GRIPPER_PICK_Z_OFFSET = 0.064

    # ── 按物体类别的 z 抓取修正 (单位: 米) ──
    # 在 GRIPPER_PICK_Z_OFFSET 基础上叠加; 正值=抬高(防碰台面), 负值=降低(防空夹)
    # 物体类别从 pick_name 提取 (如 "lemon (detected)" → "lemon")
    # 未列出的类别用 0 (无修正); 步长 0.005, 调到刚好夹住为止
    PER_OBJECT_Z_CORRECTION = {
        'lemon': 0.025,        # 柠檬 height_m +20% 后深度估算偏高 → 下探过多碰台面, 抬高 2cm 补偿
        'green apple': -0.015, # 青苹果三维 +10% 后深度估算偏低 → 夹取不够深, 多下降 1.5cm
        'strawberry': 0.010,   # 草莓深度估算偏高 → 下探碰台面, 抬高 1cm 补偿
    }

    def compute_gripper_targets(object_diameter_m):
        # target_pos is the signed opening magnitude in this action server.
        # 一次性闭合: 直接夹到目标力度, 无二次补压。
        # 挤压余量 0.020 (比物体直径小 20mm), 兼顾夹紧力与防过载。
        if object_diameter_m is None:
            return GRIPPER_OPEN, GRIPPER_CLOSE
        d = float(object_diameter_m)
        d = max(0.040, min(0.085, d))
        open_target = max(0.060, min(0.095, d + 0.020))
        close_target = max(0.020, min(0.080, d - 0.020))
        if close_target >= open_target:
            close_target = max(0.022, open_target - 0.028)
        return open_target, close_target
    
    loop_count = 0
    try:
        print("\n\n======== 🔵 自动分拣服务端已启动 (等待 GUI 客户端指令) ========\n")
        
        # 初始运行时到观察位待机 (用户要求: 程序一启动就到观察位)
        node.move_arm_joint(JOINT_OBSERVE, "初始回到观察位 (Observe)")
        node.operate_gripper(GRIPPER_OPEN, "初始化张开夹爪(待机)")
        
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
                node.move_arm_joint(JOINT_OBSERVE, "退出前回到观察位")
                node.operate_gripper(GRIPPER_OPEN, "退出前张开夹爪")
                break

            elif data.get("cmd") == "reset":
                node.is_busy = True
                status_msg.data = 'busy'
                node.status_pub.publish(status_msg)

                print("\n🔄 收到客户端指令：一键回到观察位并关闭夹爪")
                node.move_arm_joint(JOINT_OBSERVE, "回到观察位")
                node.operate_gripper(GRIPPER_OPEN, "张开夹爪(待机)")

            elif data.get("cmd") == "observe":
                node.is_busy = True
                status_msg.data = 'busy'
                node.status_pub.publish(status_msg)

                print("\n👁️ 收到客户端指令：回到观察位")
                # 关节空间直接运动到观察位，不改变夹爪状态
                node.move_arm_joint(JOINT_OBSERVE, "回到观察位 (关节空间)")
                
            elif data.get("cmd") in ("sort", "sort_verify"):
                two_stage = (data.get("cmd") == "sort_verify")
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
                pick_id = data.get("pick_name", "Pick")
                cycle_id = str(data.get("cycle_id", ""))

                # 从 pick_name 提取物体类别 (如 "lemon (detected)" → "lemon"), 查 z 修正表
                obj_class = pick_id.split(' (')[0].strip().lower()
                z_correction = PER_OBJECT_Z_CORRECTION.get(obj_class, 0.0)
                if z_correction:
                    print(f"📏 物体='{obj_class}', z 修正={z_correction:+.3f}m (叠加在 GRIPPER_PICK_Z_OFFSET 上)")

                # 料框编号 (1 或 2), 决定放置关节点位
                try:
                    bin_num = int(data.get("bin", 1))
                except (TypeError, ValueError):
                    bin_num = 1
                if bin_num not in BIN_JOINTS:
                    print(f"⚠️ 无效的 bin={bin_num}, 默认使用料框1")
                    bin_num = 1
                BIN_ABOVE_JOINTS, BIN_PLACE_JOINTS = BIN_JOINTS[bin_num]
                place_id = f"料框{bin_num}"

                POSE_PICK = pick_pose.copy()
                # link6 原点到夹爪夹持点的 z 距离 (实测 0.066, 见 GRIPPER_PICK_Z_OFFSET)
                # + 按物体类别的 z 修正 (PER_OBJECT_Z_CORRECTION, 柠檬等深度估算偏差大的物体)
                POSE_PICK['z'] += GRIPPER_PICK_Z_OFFSET + z_correction
                POSE_PICK_UP = POSE_PICK.copy()
                POSE_PICK_UP['z'] += 0.13  # 抬起脱离高 13cm
                POSE_LIFT_SOFT = POSE_PICK.copy()
                POSE_LIFT_SOFT['z'] += 0.035  # 先小幅抬起，降低惯性导致的滑脱

                loop_count += 1
                mode_label = " [两阶段精定位]" if two_stage else ""
                print(f"\n\n====================== 第 {loop_count} 次分拣{mode_label} (从 {pick_id} 到 {place_id}) ======================")

                # 以下为具体的机械臂序列
                # 放一个空的队列清理，以免堆积
                while not node.cmd_queue.empty():
                    node.cmd_queue.get_nowait()

                success = True
                grasp_reached_ok = False
                place_reached_ok = False
                for _ in range(1):
                    # ── 两阶段精定位 (可选) ──
                    if two_stage:
                        vh = float(data.get('verify_height_m', 0.25))
                        to = float(data.get('verify_timeout_s', 5.0))
                        # 先张开夹爪，避免遮挡相机视野
                        node.operate_gripper(dynamic_open, "张开夹爪(两阶段检测准备)")
                        print("🔍 [两阶段] 进入精定位流程...")
                        refine_ok, refine_result = node._two_stage_refine(
                            float(pick_pose['x']), float(pick_pose['y']),
                            verify_height=vh, timeout_s=to,
                        )
                        if not refine_ok:
                            print(f"❌ 两阶段精定位失败: {refine_result}")
                            node.move_arm_joint(JOINT_OBSERVE, "两阶段失败→回到观察位")
                            node.operate_gripper(GRIPPER_OPEN, "张开夹爪(两阶段失败)")
                            success = False; break
                        # 用精定位坐标更新相关位姿
                        for p in (POSE_PICK, POSE_PICK_UP, POSE_LIFT_SOFT):
                            p['x'] = refine_result['x']
                            p['y'] = refine_result['y']

                    # 【第零步】 在前往途中或起点提前张开夹爪
                    node.operate_gripper(dynamic_open, "张开夹爪(准备抓取)")

                    # 【第一步】 抓取过渡(防止碰桌面) — IK 优先
                    # 两阶段精定位：已在物体正上方(验证位)，直接下降抓取，跳过预备位
                    if two_stage:
                        print("⚡ 两阶段模式：已在物体正上方，直接下降抓取")
                    elif not node.move_arm_pose(POSE_PICK_UP, "抓取位上方过渡点", continuous=False, planning_mode='normal'):
                        success = False; break
                    if node.last_planning_strategy:
                        cycle_strategies.append(node.last_planning_strategy)
                    if node.last_planning_profile_name:
                        cycle_profiles.append(node.last_planning_profile_name)

                    # 记录抓取过渡点的关节角，用于百分百安全抬起
                    pick_up_joints = node.current_joints.copy() if node.current_joints else None

                    # ⚠️ 下降前禁用所有传送带碰撞体: z 补偿降低后夹爪手指(gripper_link1/2)会穿透
                    # belt_deck 台面和 conveyor_belt_col 侧面碰撞模型，导致抬起时起点被判碰撞 → -2。
                    # 日志确认两种碰撞: belt_deck↔gripper_link1, conveyor_belt_col↔gripper_link1。
                    # 物理上物体在台面上方 7cm，夹爪不会真撞台面，仅碰撞模型偏保守。
                    # 恢复在回观察位之后(下方错误处理/成功路径)。
                    node._set_bin_collision_allowed(allowed=True)

                    # 【第二步】 下降抓取 — 优先直线插补（短距），IK 作为备用
                    # 使用 PICK_UP 时 IK 算出的实际末端姿态，确保姿态匹配
                    pick_fallback_ori = node._build_pick_orientation(POSE_PICK)[0]
                    active_ori = getattr(node, 'last_successful_orientation', pick_fallback_ori)
                    pose_pick_msg = node._create_pose(POSE_PICK, active_ori)

                    descent_success = False
                    # 先尝试直线插补（下降仅13cm，阈值放宽到50%）
                    if node.execute_cartesian_path([pose_pick_msg], "下降抓取 (直线插补)", fraction_threshold=0.50):
                        descent_success = True
                    elif node.enable_ik and node.ik_solver is not None:
                        # 直线插补失败，尝试 IK 多姿态求解（以当前关节角为种子，保证解靠近当前构型）
                        pick_orientations = node._build_pick_orientations_multi(POSE_PICK)
                        ik_ok, ik_joints = node.move_arm_via_ik(POSE_PICK, pick_orientations, "下降抓取 (IK多姿态)", continuous=True)
                        if ik_ok:
                            descent_success = True
                            cycle_strategies.append('ik_joint_space')
                            cycle_profiles.append('ik_solution')

                    if not descent_success:
                        print("🟡 直线插补+IK均受限！启动多路并发退避规划...")
                        if not node.move_arm_cartesian(POSE_PICK, "下降抓取 (退避规划)", continuous=True, preferred_orientation=active_ori, allow_position_only_fallback=True, planning_mode='descend'):
                            success = False; break
                        if node.last_planning_strategy:
                            cycle_strategies.append(node.last_planning_strategy)
                        if node.last_planning_profile_name:
                            cycle_profiles.append(node.last_planning_profile_name)

                    
                    # 【第三步】 一次性闭合夹爪 (无二次补压, 直接夹到位)
                    node.operate_gripper(dynamic_close, "闭合夹爪(拿取)")
                    # 闭合后短暂停顿，让物体接触稳定后再上升，减少滑脱/弹飞。
                    time.sleep(GRIPPER_SETTLE_SEC)

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
                    
                    # 【第五步】 关节空间移动到料框上方 (用户标定点位)
                    # ⚠️ 临时禁用传送带所有碰撞体：料框放置位机械臂与多个传送带碰撞体几何相交
                    # (control_box↔link5, rail_ny↔gripper_link2, conveyor_control_box↔gripper_link1 等)，
                    # 导致 RRTConnect 无法在目标状态采样 (错误码 99999)。通过 ACM diff 临时允许碰撞，第九步后恢复。
                    node._set_bin_collision_allowed(allowed=True)
                    if not node.move_arm_joint(BIN_ABOVE_JOINTS, f"移动到{place_id}上方 (关节空间)", continuous=False):
                        success = False; break

                    # 【第六步】 关节空间移动到料框放置位 (用户标定点位)
                    # ⚠️ continuous=False: 上一步执行后需 0.5s 稳定时间，否则起点偏差 > 0.01 触发 -4 错误
                    if not node.move_arm_joint(BIN_PLACE_JOINTS, f"移动到{place_id}放置位 (关节空间)", continuous=False):
                        success = False; break
                    place_reached_ok = True

                    # 【第七步】 松开夹爪释放物品
                    node.operate_gripper(dynamic_open, f"松开夹爪(在{place_id}放置位释放)")
                    time.sleep(GRIPPER_SETTLE_SEC)

                    # 【第八步】 关节空间回到料框上方 (抬起脱离放置位)
                    if not node.move_arm_joint(BIN_ABOVE_JOINTS, f"从{place_id}放置位抬起 (关节空间)", continuous=False):
                        success = False; break

                    # 【第九步】 关节空间回到观察位 (等待下一次抓取)
                    if not node.move_arm_joint(JOINT_OBSERVE, f"回到观察位 (关节空间)", continuous=False):
                        success = False; break

                # ⚠️ 碰撞检测恢复已下移至"回观察位之后"。
                # 原来在此处恢复会导致: 若抓取/抬起失败时机械臂仍在低位，
                # 恢复后下方错误处理中的"回观察位"也会因起点碰撞(belt_deck↔gripper_link1)而失败(-2)。

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
                    node.move_arm_joint(JOINT_OBSERVE, "自动回到观察位")
                    # 安全回到观察位后恢复碰撞检测 (无论回观察位是否成功都恢复，避免影响下一轮)
                    node._set_bin_collision_allowed(allowed=False)
                    node.operate_gripper(GRIPPER_OPEN, "张开夹爪(待机)")
                    
                    time.sleep(0.5)
                    continue

                # 成功完成分拣后恢复碰撞检测
                node._set_bin_collision_allowed(allowed=False)

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
            node.move_arm_joint(JOINT_OBSERVE, "退出前回到观察位")
            node.operate_gripper(GRIPPER_OPEN, "退出前张开夹爪")
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