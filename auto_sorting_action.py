
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
from sensor_msgs.msg import JointState, RegionOfInterest
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header
from builtin_interfaces.msg import Time as TimeMsg

# ── 夹爪力矩检测: 订阅 GripperStatus 获取实时 width/force 反馈 ──
from agx_arm_msgs.msg import GripperStatus

# ── TF 坐标变换 ──
from geometry_msgs.msg import TransformStamped
import tf2_ros

# ── GraspNet 抓取: sort_graspnet 命令通过独立服务节点 graspnet_service_node.py 生成 6DoF 位姿 ──
# 通信: /graspnet/req (JSON 请求) → /graspnet/result (JSON 结果), 无 agx_arm_msgs 依赖

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
                """pos_err + w_z * z_axis_err + w_yaw * yaw_err + w_joint * joint_dist

                joint_dist 惩罚远离 initial_guess 的解 (打破冗余关节的平局,
                避免 IK 选择 joint6 旋转 90° 的解而不是关节空间接近的解).
                w_joint=0.03 使 90° 偏离比 ~0.002m 位置误差多 ~0.05, 足以破平局.
                """
                _pin.forwardKinematics(self.model, self.data, q_in)
                _pin.updateFramePlacements(self.model, self.data)
                pe = float(np.linalg.norm(
                    self.data.oMf[self.ee_frame_id].translation - target_pos_arr))
                R_act = self.data.oMf[self.ee_frame_id].rotation
                actual_z = R_act[:, 2]
                cos_angle = float(np.clip(np.dot(actual_z, target_z), -1.0, 1.0))
                ze = float(np.arccos(cos_angle))  # 0=aligned, pi=opposite
                actual_x = R_act[:, 0]
                target_x = target_se3.rotation[:, 0]
                cos_yaw = float(np.clip(abs(np.dot(actual_x, target_x)), -1.0, 1.0))
                yaw_err = float(np.arccos(cos_yaw))
                total = pe + 0.06 * ze + 0.05 * yaw_err
                if initial_guess is not None:
                    # 只惩罚 joint6 偏差 (冗余关节中最容易翻转的腕关节).
                    # 全关节惩罚会误伤 step1 (待机位→抓取点 关节空间距离大),
                    # 导致 IK 无解回退笛卡尔. joint6 偏差 90°(1.57rad)→罚 0.016,
                    # 与典型位置误差 0.002-0.005 同量级, 足以破平局.
                    dq6 = abs(float(q_in[self.joint_q_indices[5]])
                              - float(initial_guess[5]))
                    total += 0.01 * dq6
                return total, pe, ze, yaw_err

            best_q = None
            best_err = float('inf')

            if initial_guess is not None:
                q0 = _pin.neutral(self.model)
                for i, idx in enumerate(self.joint_q_indices):
                    q0[idx] = float(initial_guess[i])
                ce, _, _, _ = _combined_error(q0)
                if ce < best_err:
                    best_err = ce
                    best_q = q0.copy()

            # Phase 1: Biased random sampling — 70% near initial guess, 30% uniform
            # 局部优先避免 joint6 大幅旋转: 下降抓取时当前关节角就是合理的种子,
            # 附近采样找到的解 joint6 角度接近当前值, 全随机可能找到 joint6 翻转 90°+ 的解
            n_samples = 500
            n_local = int(n_samples * 0.7) if initial_guess is not None else 0
            n_global = n_samples - n_local
            for si in range(n_samples):
                q = np.zeros(self.model.nq)
                if si < n_local:
                    # 局部采样: 高斯分布在当前关节角附近, sigma 逐渐增大
                    sigma = 0.15 * (1.0 + float(si) / max(n_local, 1))  # 0.15 → 0.30 rad
                    perturb = np.random.normal(0, sigma, 6)
                    arm_q = initial_guess + perturb
                    np.clip(arm_q, lo6, hi6, out=arm_q)
                else:
                    arm_q = np.random.uniform(lo6, hi6)  # 全局兜底
                for i, idx in enumerate(self.joint_q_indices):
                    q[idx] = arm_q[i]
                ce, pe, ze, yaw_err = _combined_error(q)
                if ce < best_err:
                    best_err = ce
                    best_q = q.copy()
                    if pe < 0.005 and ze < 0.15 and yaw_err < 0.25:
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

                ce, pe, ze, yaw_err = _combined_error(qt)
                if ce < best_err:
                    best_err = ce
                    q = qt.copy()
                    if pe < 0.005 and ze < 0.15 and yaw_err < 0.25:
                        break

            _, final_pe, final_ze, _ = _combined_error(q)
            q_sol = np.array([q[idx] for idx in self.joint_q_indices], dtype=float)
            # ⚠️ 安全约束: 位置<15mm 且 Z轴(approach)方向误差<30°(0.52rad)
            # 防止 IK 返回位置对但末端朝上的危险解:
            # 当物体超出工作空间时, 只有末端朝上的构型可达 (位置准但 Z 轴朝上),
            # 若只判位置会接受该解 → 机械臂朝上抓 (危险!)。
            # 加 final_ze 约束强制拒绝朝上解, 物体超出工作空间时 IK 返回失败而非朝上解。
            success = bool(final_pe < 0.015 and final_ze < 0.52)

            t1 = time.time()
            return success, q_sol, final_pe, (t1 - t0)

        except Exception as e:
            print(f"❌ IK solver error: {e}")
            traceback.print_exc()
            t1 = time.time()
            return False, np.zeros(self.ndof), 999.0, (t1 - t0)

    def get_frame_position(self, q_joints, frame_name):
        """用 FK 计算指定 frame 在 base 坐标系中的位置。

        Args:
            q_joints: 6 维关节角 (joint1-6, 弧度)
            frame_name: frame 名称 (如 'gripper_link1', 'gripper_link2', 'link6')

        Returns:
            np.array([x, y, z]) 或 None (frame 不存在)
        """
        if not _PINOCCHIO_AVAILABLE:
            return None
        try:
            frame_id = self.model.getFrameId(frame_name)
            if frame_id >= len(self.model.frames):
                return None
            q = _pin.neutral(self.model)
            for i, idx in enumerate(self.joint_q_indices):
                if i < len(q_joints):
                    q[idx] = q_joints[i]
            _pin.forwardKinematics(self.model, self.data, q)
            _pin.updateFramePlacements(self.model, self.data)
            return self.data.oMf[frame_id].translation.copy()
        except Exception as e:
            print(f"❌ FK error for {frame_name}: {e}")
            return None



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
        self.robot_status_pub = self.create_publisher(String, '/sorting/robot_status', 10)
        self.is_busy = False
        self.last_planning_profile_name = ''
        self.last_planning_strategy = ''

        # ── 两阶段精定位：订阅 YOLO 检测结果 ──
        self._latest_detection = None
        self._detection_event = threading.Event()
        self.det_sub = self.create_subscription(String, '/detection_info', self._detection_cb, 10)

        # ── GraspNet 服务客户端 (sort_graspnet 命令) ──
        # 通过话题通信请求独立服务节点 graspnet_service_node.py 生成 6DoF 抓取位姿
        # (该节点运行在 orange_dataset/.venv, 避免与本节点 .venv_ik 的 pinocchio 冲突)
        self._graspnet_req_pub = self.create_publisher(String, '/graspnet/req', 10)
        self._latest_graspnet_result = None
        self._graspnet_event = threading.Event()
        self._graspnet_result_sub = self.create_subscription(
            String, '/graspnet/result', self._graspnet_result_cb, 10)

        # ── 急停 / 返回待机位: 独立 topic 信号, 分拣循环内可响应 ──
        # /sorting_cmds 只在主循环顶部 poll, 分拣序列内收不到.
        # /emergency_stop 通过回调设置标志位, 序列中检查点检测后执行安全停机.
        self._emergency_flag = False
        self._emergency_req_type = ""  # "estop" or "standby"
        self._emergency_sub = self.create_subscription(
            String, "/emergency_stop", self._emergency_cb, 10)
        # 用于取消正在执行的 MoveIt 轨迹
        self._active_goal_handle = None

        # ── 夹爪力矩检测: 订阅 GripperStatus 获取实时 width/force 反馈 ──
        # /feedback/gripper_status 由 agx_ctrl_single_node 发布, 含 width(m) 和 force(N)
        self._latest_gripper_status = None
        self._gripper_status_sub = self.create_subscription(
            GripperStatus, "/feedback/gripper_status", self._gripper_status_cb, 10)
        # 力矩参数 (GUI 可调, 范围 0.5-3.0N, 由 AgxGripperWrapper.FORCE_MIN/MAX 约束)
        self._gripper_force = 1.5         # 当前夹持力 (N), GUI 可调
        self._GRIPPER_FORCE_MIN = 0.5     # 安全下限
        self._GRIPPER_FORCE_MAX = 3.0     # 安全上限 (AgxGripperWrapper.FORCE_MAX)
        self._GRIPPER_FORCE_DANGER = 3.5  # 危险阈值 (超过即判定失败)
        self._pre_grasp_width = None      # 闭合前夹爪宽度 (check_grasp_success 用)

        # ========== IK Solver 集成（内嵌 Pinocchio）==========
        self.enable_ik = True
        self.ik_solver = None
        self._init_ik_solver()

        # ── TF2 监听器 ──
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _gripper_status_cb(self, msg):
        """缓存夹爪状态反馈 (width, force, 安全标志)."""
        self._latest_gripper_status = msg

    def _emergency_cb(self, msg):
        """急停/返回待机位/复位回调 — 设置中断标志位, 分拣序列中检查点检测."""
        try:
            data = json.loads(msg.data)
            req_type = data.get("req", "estop")
            if req_type == "reset":
                # 复位: 仅清除急停锁, 由下次 _check_emergency 执行回待机位
                if self._emergency_flag and self._emergency_req_type == "estop":
                    self._emergency_flag = False
                    self._emergency_req_type = "reset"
                    self.get_logger().info(
                        f"🔄 收到复位信号, 清除急停锁 "
                        f"time={time.strftime('%H:%M:%S')}")
                return
            self._emergency_req_type = req_type
            self._emergency_flag = True
            self.get_logger().error(
                f"🚨 收到中断信号: req={self._emergency_req_type} "
                f"time={time.strftime('%H:%M:%S')}")
        except Exception:
            self._emergency_flag = True
            self._emergency_req_type = "estop"

    def _check_emergency(self, step_name=""):
        """检查是否有急停/返回待机位/复位信号. 若有则执行对应操作.

        在分拣序列中每步之后调用. 返回 True 表示已触发中断, 调用者应 break.

        req="estop":  取消轨迹 + 张开夹爪 + 停在原地 (保持锁定, 不清除标志)
        req="standby": 取消轨迹 + 张开夹爪 + 回待机位 + 恢复碰撞 + 清除标志
        req="reset":  回待机位 + 恢复碰撞 + 清除标志 (急停后复位用)
        """
        # 需要 spin_once 刷新订阅回调 (分拣序列内不会自动 spin)
        rclpy.spin_once(self, timeout_sec=0.01)
        if not self._emergency_flag and self._emergency_req_type != "reset":
            return False

        req = self._emergency_req_type or "estop"

        # ── 复位: 急停锁定后用户按下复位按钮触发 ──
        if req == "reset":
            self.get_logger().info(
                f"🔄 执行复位: 回待机位 step={step_name} "
                f"time={time.strftime('%Y-%m-%dT%H:%M:%S')}")
            try:
                self.move_arm_joint([0.0]*6, "复位-回待机位")
                self._set_bin_collision_allowed(allowed=False)
                self.get_logger().info("✅ 复位完成, 机械臂已回待机位")
            except Exception as e:
                self.get_logger().error(f"❌ 复位异常: {e}")
            self._emergency_flag = False
            self._emergency_req_type = ""
            return True

        # ── 急停 / 返回待机位 ──
        self.get_logger().error(
            f"🚨 急停触发! req={req} step={step_name} "
            f"time={time.strftime('%Y-%m-%dT%H:%M:%S')}")

        try:
            # 1) 取消当前 MoveIt 轨迹
            if self._active_goal_handle is not None:
                self.get_logger().info("🛑 取消当前 MoveIt 轨迹...")
                try:
                    self._active_goal_handle.cancel_goal_async()
                except Exception:
                    pass
                self._active_goal_handle = None

            # 2) 张开夹爪 (防碰撞)
            self.operate_gripper(0.090, "急停-张开夹爪")

            if req == "estop":
                # 急停: 停在当前位置, 保持锁定, 等待复位
                self.get_logger().error(
                    "⛔ 急停锁定! 机械臂已冻结在当前位置. "
                    "按【复位】按钮回待机位.")
                # 不清除 _emergency_flag, 机械臂保持冻结
            else:
                # standby: 回待机位 + 恢复碰撞 + 清除标志
                self.move_arm_joint([0.0]*6, "急停-回待机位")
                self._set_bin_collision_allowed(allowed=False)
                self.get_logger().info(f"✅ 安全停机完成 (req={req})")
                self._emergency_flag = False
                self._emergency_req_type = ""
        except Exception as e:
            self.get_logger().error(f"❌ 安全停机异常: {e}")
            if req != "estop":
                self._emergency_flag = False
                self._emergency_req_type = ""

        return True

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

    def _get_fresh_detection(self, target_name, timeout_s=2.0):
        """等待视觉系统返回目标物体的最新坐标.

        用于抓取失败重试: 物体可能因夹爪触碰而位移, 需要重新获取坐标.
        通过 rclpy.spin_once 轮询 /detection_info, 匹配 target_name 相同的物体.

        Args:
            target_name: 物体名称 (如 "banana (detected)")
            timeout_s: 最长等待时间

        Returns:
            dict with 'x','y','z' on success, None on timeout
        """
        self._detection_event.clear()
        deadline = time.time() + timeout_s
        target_key = target_name.split(' (')[0].strip().lower()

        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._detection_event.is_set():
                det = self._latest_detection
                if det and det.get('detected'):
                    objects = det.get('objects', []) or []
                    for obj in objects:
                        name = (obj.get('name', '') or '').lower().strip()
                        if target_key in name or name in target_key:
                            bp = obj.get('base_position_m')
                            if bp and all(k in bp for k in ('x', 'y', 'z')):
                                self._detection_event.clear()
                                self.get_logger().info(
                                    f"📡 重试: 获取到 {name} 新坐标 "
                                    f"x={bp['x']:.3f} y={bp['y']:.3f} z={bp['z']:.3f}")
                                return {'x': float(bp['x']),
                                        'y': float(bp['y']),
                                        'z': float(bp['z'])}
                    # 检测到了但没匹配到目标物体, 清事件继续等
                    self._detection_event.clear()
            time.sleep(0.1)

        self.get_logger().warn(f"⏰ 重试: {timeout_s}s 内未获取到 {target_name} 新坐标")
        return None

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

        # 刷新关节状态: 处理待处理的 /joint_states 消息, 确保 start_state 反映真实状态
        # 防止夹爪动作/轨迹执行后 current_joints 过时 → 起始状态与控制器不匹配 → -4 (CONTROL_FAILED)
        # timeout 0.20s: 夹爪动作/轨迹执行后需足够时间收到新鲜 /joint_states (原 0.05s 不足导致 -4)
        rclpy.spin_once(self, timeout_sec=0.20)

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
                time.sleep(0.15) 
            return True
        else:
            self.get_logger().error(f'⚠️ 执行失败，错误码: {result.error_code.val}')
            if plan_only:
                return False, None
            return False

    def _two_stage_refine(self, obs_x, obs_y, verify_height=0.25, timeout_s=5.0):
        """两阶段精定位：走到物体正上方做二次检测。

        ⚠️ eye-to-hand 注意: 相机固定, 机械臂移动到验证位不会改变相机视角,
        二次检测精度与待机位相同 (不像 eye-in-hand 能近距离检测)。
        此方法在 eye-to-hand 下效果有限, 主要用于 sort_verify 命令兼容。
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
        # 质心 z (D455 深度点云估算, 不依赖预设尺寸); fallback 表面 z
        ver_centroid_z = detection.get('centroid_base_z')
        if ver_centroid_z is None:
            ver_centroid_z = ver_z
        else:
            ver_centroid_z = float(ver_centroid_z)

        print(f"✅ [两阶段] 二次检测成功! 坐标=({ver_x:.4f},{ver_y:.4f},{ver_z:.4f}) 质心z={ver_centroid_z:.4f}")
        return True, {'x': ver_x, 'y': ver_y, 'z': ver_z, 'centroid_z': ver_centroid_z}

    # ── GraspNet 服务通信 ──
    def _graspnet_result_cb(self, msg):
        """缓存 GraspNet 服务结果, 唤醒等待线程。"""
        try:
            self._latest_graspnet_result = json.loads(msg.data)
        except Exception as e:
            self._latest_graspnet_result = {"success": False, "error": f"bad_json: {e}"}
        self._graspnet_event.set()

    def call_graspnet_service(self, pick_pose, timeout_s=15.0):
        """请求 GraspNet 服务获取 6DoF 抓取位姿 (base 系)。

        通过 /graspnet/req → /graspnet/result 话题通信。
        服务节点 graspnet_service_node.py 在待机位快照最新深度图推理
        (eye-to-hand: T_cam_to_base 是常量, 无需 tcp_pose)。
        失败返回 {"success": False, "error": ...}。

        Args:
            pick_pose: {'x','y','z'} 待机位检测到的物体 base 坐标 (用作邻近过滤提示)
            timeout_s: 等待结果超时 (推理约 2-3s)
        """
        req = {
            "cmd": "get_grasp",
            "pick_x": float(pick_pose['x']),
            "pick_y": float(pick_pose['y']),
            "pick_z": float(pick_pose['z']),
        }
        self._graspnet_event.clear()
        self._latest_graspnet_result = None
        msg = String()
        msg.data = json.dumps(req)
        self._graspnet_req_pub.publish(msg)
        self.get_logger().info(f"📡 已请求 GraspNet 服务 (pick={req['pick_x']:.3f},{req['pick_y']:.3f},{req['pick_z']:.3f}), 等待结果 (超时 {timeout_s}s)...")

        waited = 0.0
        dt = 0.1
        while waited < timeout_s:
            if rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
            if self._graspnet_event.is_set():
                break
            time.sleep(dt)
            waited += dt

        if self._latest_graspnet_result is None:
            return {"success": False, "error": f"timeout_{timeout_s}s"}
        return self._latest_graspnet_result

    def move_arm_joint(self, joints, desc, continuous=False):
        self.get_logger().info(f"🚀 正在规划(关节空间) -> {desc}")
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
        
    def execute_cartesian_path(self, waypoints, desc, fraction_threshold=0.85, jump_threshold=1.6):
        self.get_logger().info(f"🚀 正在快速计算(笛卡尔直线连线) -> {desc}")
        if not self._cartesian_client.wait_for_service(timeout_sec=1.5):
            self.get_logger().error("⚠️ GetCartesianPath 服务不可用")
            return False

        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = 'arm'
        # 刷新关节状态 (同 send_goal): 防止 current_joints 过时导致起始状态不匹配
        rclpy.spin_once(self, timeout_sec=0.20)
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
        req.jump_threshold = jump_threshold  # j6 突变检测阈值 (默认 1.6, 下降时可降到 0.8)
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
            self.get_logger().info(f"⚠️ IK 无解 (尝试了 {len(orientations)} 种姿态)")
            return False, None

        self.get_logger().info(f"✅ IK 求解成功: error={best_error:.4f}, time={best_time*1000:.1f}ms")
        if not self.move_arm_joint(best_q, f"{desc} (IK求解, err={best_error:.4f})", continuous=continuous):
            self.get_logger().info("⚠️ IK 关节执行失败")
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

    def move_arm_pose(self, pose_dict, desc, continuous=False, planning_mode='normal',
                       preferred_orientation=None):
        """
        统一运动方法：优先使用内嵌 IK + 关节空间，失败回退到 Cartesion 规划

        Args:
            pose_dict: {'x', 'y', 'z'} 目标位置
            desc: 动作描述
            continuous: 连贯模式
            planning_mode: 传递给 Cartesian 兜底的规划模式
            preferred_orientation: 优先姿态 ([x,y,z,w] 四元数), 传给 IK 和 Cartesian 兜底
        """
        if self.enable_ik and self.ik_solver is not None:
            orientations = self._build_pick_orientations_multi(pose_dict)
            # 处理 preferred_orientation: 可能是 list([x,y,z,w]) 或 Quaternion 对象
            if preferred_orientation is not None:
                if isinstance(preferred_orientation, Quaternion):
                    preferred_orientation = [preferred_orientation.x,
                                             preferred_orientation.y,
                                             preferred_orientation.z,
                                             preferred_orientation.w]
                if isinstance(preferred_orientation, (list, tuple)) and len(preferred_orientation) == 4:
                    orientations = [list(preferred_orientation)] + orientations
            ik_ok, _ = self.move_arm_via_ik(pose_dict, orientations, desc, continuous=continuous)
            if ik_ok:
                return True
            print("🟡 IK 路径失败，回退到 MoveIt2 笛卡尔规划...")

        return self.move_arm_cartesian(
            pose_dict, desc, continuous=continuous,
            preferred_orientation=preferred_orientation,
            allow_position_only_fallback=True if preferred_orientation is None else False,
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
        self.get_logger().info(f"🚀 正在规划(笛卡尔位置) -> {desc}")

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

        # ⚠️ 精简 profile 列表: 删除"极大放宽"(0.40rad=23°), 由安全兜底(0.78rad=45°)覆盖;
        # 缩减 attempts/time 加速求解 (原 6-7 profile × 4-5s 并发 → MoveIt 内部串行 → 16s+)
        if planning_mode == 'descend':
            add_profile('下探位姿(稳健)', 0.008, (0.10, 0.10, 0.15), 3, 2.5)
            add_profile('下探位姿(放宽)', 0.015, (0.25, 0.25, 0.50), 4, 3.0)
        elif planning_mode == 'retreat':
            add_profile('抬升位姿(稳健)', 0.010, (0.14, 0.14, 0.20), 3, 2.5)
            add_profile('抬升位姿(放宽)', 0.020, (0.30, 0.30, 0.60), 4, 3.0)
        else:
            # ⚠️ 收紧误差: 原 pos_tol=0.008/0.015/0.020 太松,
            # 笛卡尔规划回退时位姿偏差大 → joint6 非预期旋转
            add_profile('直解位姿(快速)', 0.004, (0.06, 0.06, 0.08), 2, 2.0)
            add_profile('直解位姿(放宽)', 0.008, (0.12, 0.12, 0.20), 3, 2.5)
            # 自由偏航兜底: z 轴收紧到 25° (原 180° 意味着任意姿态可接受)
            planning_profiles.append({
                'name': '直解位姿(自由偏航)',
                'orientation': orientations_to_try[0],
                'pos_tol': 0.010,
                'ori_tol': (0.20, 0.20, 0.44),  # z=25°, 原 3.14=180° 太松
                'attempts': 3,
                'time': 2.5,
            })

        if allow_position_only_fallback:
            # ⚠️ 安全兜底: 原 orientation=None (无姿态约束=任意朝向) 会导致末端朝上/水平等
            # 危险姿态 ("朝上抓"事故根因), 且无约束搜索空间巨大 → 求解慢.
            # 改为朝下姿态 + 45°倾斜容差 + 自由偏航:
            #   1. Z轴(approach)限制在竖直向下 ±45° 内, 杜绝朝上/水平抓取
            #   2. 比原极大放宽(23°)更宽松, 提高工作空间边缘可解性
            #   3. 偏航(yaw)完全自由, 不限制夹爪绕竖直轴旋转
            #   4. FK安全检查(ee_z[2]<-0.5, 即<60°倾斜)作为最终硬防线
            # 若此 profile 也失败 → 位置确实无法安全到达 → 整体规划失败(安全优先)
            planning_profiles.append({
                'name': '位置+宽姿态(安全兜底)',
                'orientation': orientations_to_try[0],
                'pos_tol': 0.020,
                'ori_tol': (0.78, 0.78, 3.14159),  # 45°倾斜 + 自由偏航
                'attempts': 3,
                'time': 3.0,
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
                    # ⚠️ 安全检查: 验证轨迹终点的末端 Z 轴朝下 (防朝上抓)
                    # 安全兜底 profile 虽已约束 Z轴±45°, 但 MoveIt 偶尔返回超出容差的解;
                    # 此 FK 检查作为最终硬防线: 拒绝末端 Z 轴 z分量 > -0.5 (倾斜>60°) 的轨迹.
                    if self.enable_ik and self.ik_solver is not None and _pin is not None:
                        try:
                            last_pt = traj.joint_trajectory.points[-1]
                            jm = dict(zip(traj.joint_trajectory.joint_names, last_pt.positions))
                            # ⚠️ model.nq=8 (joint1-6 + gripper_joint1/2), 必须用 neutral 向量
                            # 再按 joint_q_indices 填入 joint1-6, 否则 forwardKinematics 报
                            # "wrong argument size: expected 8, got 6" → 安全检查失效
                            q_full = _pin.neutral(self.ik_solver.model)
                            for i, idx in enumerate(self.ik_solver.joint_q_indices):
                                q_full[idx] = jm.get(f'joint{i}', 0.0)
                            _pin.forwardKinematics(self.ik_solver.model, self.ik_solver.data, q_full)
                            _pin.updateFramePlacements(self.ik_solver.model, self.ik_solver.data)
                            ee_z = self.ik_solver.data.oMf[self.ik_solver.ee_frame_id].rotation[:, 2]
                            if float(ee_z[2]) > -0.5:
                                self.get_logger().error(
                                    f"⛔ 安全拒绝: profile '{p['name']}' 轨迹终点末端 Z 轴朝上 "
                                    f"(z分量={float(ee_z[2]):.2f}, 需<-0.5 防朝上抓), 跳过此 profile"
                                )
                                continue
                        except Exception as _e:
                            self.get_logger().warn(f"轨迹姿态安全检查异常 ({_e}), 谨慎放行")
                    best_profile = p
                    best_trajectory = traj
                    break

        if best_profile is not None:
            self.get_logger().info(f"\x1b[32m🎉 并发计算成功！选用最优解策略: {best_profile['name']}\x1b[0m")
            # 所有 profile 现在都有非 None orientation (安全兜底也已约束为朝下姿态),
            # 直接写入 last_successful_orientation 供后续下降抓取使用
            if best_profile['orientation'] is not None:
                self.last_successful_orientation = best_profile['orientation']
            self.last_planning_profile_name = str(best_profile.get('name', ''))
            if '自由偏航' in self.last_planning_profile_name:
                self.last_planning_strategy = 'free_yaw'
            elif '安全兜底' in self.last_planning_profile_name:
                self.last_planning_strategy = 'position_only'
            elif '放宽' in self.last_planning_profile_name:
                self.last_planning_strategy = 'relaxed'
            elif '快速' in self.last_planning_profile_name or '稳健' in self.last_planning_profile_name:
                self.last_planning_strategy = 'direct'
            else:
                self.last_planning_strategy = 'unknown'
            
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
                    time.sleep(0.15)
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

    def _publish_gripper_joint_state(self, target_pos, force=None):
        # Unified fallback for both real arm and simulation bridge.
        # effort 字段 = 夹持力 (N), agx_ctrl_single_node 读取后传给 gripper.move(width, force)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['gripper']
        msg.position = [float(target_pos)]
        msg.velocity = []
        msg.effort = [force if force is not None else self._gripper_force]
        self._joint_states_pub.publish(msg)

    def operate_gripper(self, target_pos, desc):
        self.get_logger().info(f"✊ 正在执行夹爪动作 -> {desc} (target: {target_pos})")

        # 记录操作前宽度 (抓取检测用, 在调用处设置 _pre_grasp_width)

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

    def _get_current_tcp_mm(self):
        """通过 TF lookup 获取末端执行器坐标 (mm)."""
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link', 'link6', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            return [
                round(t.transform.translation.x * 1000.0, 1),
                round(t.transform.translation.y * 1000.0, 1),
                round(t.transform.translation.z * 1000.0, 1),
            ]
        except Exception as e:
            self.get_logger().warn(f"TF lookup base_link→link6 失败: {e}")
            return [0.0, 0.0, 0.0]

    def _get_bin_place_position(self, bin_place_joints):
        """通过 FK 计算料框放置位坐标字符串."""
        try:
            if self.ik_solver is None or _pin is None:
                return "(0.000, 0.000, 0.000)"
            q_full = _pin.neutral(self.ik_solver.model)
            for i, idx in enumerate(self.ik_solver.joint_q_indices):
                if i < len(bin_place_joints):
                    q_full[idx] = bin_place_joints[i]
            _pin.forwardKinematics(self.ik_solver.model, self.ik_solver.data, q_full)
            _pin.updateFramePlacements(self.ik_solver.model, self.ik_solver.data)
            pos = self.ik_solver.data.oMf[self.ik_solver.ee_frame_id].translation
            return f"({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})"
        except Exception:
            return "(0.000, 0.000, 0.000)"

    def _publish_robot_status(self, success, cargo_type, confidence, source_pos_dict, target_joints, grasp_ok, error_code):
        """收集分拣周期数据并通过 /sorting/robot_status 发布."""
        # source_position 坐标字符串
        src = source_pos_dict
        source_position = f"({src['x']:.3f}, {src['y']:.3f}, {src['z']:.3f})"

        # target_position 通过 FK 计算
        target_position = self._get_bin_place_position(target_joints)

        # actual_position
        actual_position = target_position if success else "NONE"

        # action
        action = "sort_complete" if success else "sort_failed"

        # joint_angles (弧度→度, 6个关节)
        joint_angles = []
        if self.current_joints:
            for j in range(1, 7):
                key = f'joint{j}'
                if key in self.current_joints:
                    joint_angles.append(round(math.degrees(self.current_joints[key]), 2))
                else:
                    joint_angles.append(0.0)
        else:
            joint_angles = [0.0] * 6

        # tcp_position (mm)
        tcp_position = self._get_current_tcp_mm()

        payload = {
            "action": action,
            "cargo_type": cargo_type,
            "confidence": confidence,
            "source_position": source_position,
            "target_position": target_position,
            "actual_position": actual_position,
            "grasp_success": grasp_ok,
            "joint_angles": joint_angles,
            "tcp_position": tcp_position,
            "error_code": error_code,
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.robot_status_pub.publish(msg)
        self.get_logger().info(f"📡 robot_status: action={action} cargo={cargo_type} success={grasp_ok} err={error_code}")

    def check_grasp_success(self, timeout_s=1.5):
        """抬起后检测抓取是否成功. 基于夹爪物理宽度判断:

        逻辑 (夹爪目标为完全闭合 GRIPPER_GRAB=0.000):
          - width > 阈值 (夹爪未完全闭合) → 夹到物体 ✅
          - width ≈ 0 (夹爪完全闭合) → 空抓 ❌

        无需力矩判断: 夹爪完全闭合时 force 接近 0, 夹到物体时 force 也因
        力矩控制固件限制而不可靠. 物理宽度是最直接的证据.
        """
        deadline = time.time() + timeout_s
        last_width = None
        stable_count = 0

        # 等待 width 稳定 (连续 2 次变化 < 1mm)
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._latest_gripper_status is not None:
                w = self._latest_gripper_status.width
                if last_width is not None and abs(w - last_width) < 0.001:
                    stable_count += 1
                    if stable_count >= 2:
                        break
                else:
                    stable_count = 0
                last_width = w
            time.sleep(0.1)

        if self._latest_gripper_status is None:
            return False, 0.0, 0.0, "no_feedback"

        width = self._latest_gripper_status.width
        force = self._latest_gripper_status.force

        self.get_logger().info(
            f"📏 夹爪状态: width={width*1000:.1f}mm force={force:.2f}N "
            f"overcurrent={self._latest_gripper_status.driver_overcurrent}")

        # 判断: 夹爪目标为完全闭合(0mm), 若 width > 5mm 说明有物体阻挡
        WIDTH_EMPTY = 0.005  # 5mm: 完全闭合容许误差
        if width > WIDTH_EMPTY:
            return True, width, force, "success"
        else:
            return False, width, force, "empty_grasp"

    def _publish_grasp_status(self, success, width, force, reason):
        """发布抓取结果到 GUI (/sorting_status)."""
        msg = String()
        msg.data = json.dumps({
            "type": "grasp_result",
            "success": success,
            "width": round(width, 4),
            "force": round(force, 2),
            "reason": reason,
        })
        self.status_pub.publish(msg)

    # ====================================================================
    # Task 2.1: 蓝方块分拣 (sort_blue_block)
    # 顶面中位抓取 + 分层姿态约束 + 物块尺寸感知夹爪控制
    # ====================================================================

    def _compute_blue_block_pick_pose(self, pick, surface_z_m, block_height_m,
                                       block_width_m, min_grasp_z, gripper_pick_z_offset):
        """📐 计算蓝方块顶面抓取位姿 (Task 2.1)。

        下降深度策略: 夹爪夹物块顶面 (surface_z), 安全下限 MIN_GRASP_Z。
        POSE_PICK.z = clamping_z + GRIPPER_PICK_Z_OFFSET (link6 原点到夹爪夹持点的 z 距离)。

        注: 原设计用物块中位 (surface_z - height/2), 但对矮物块 (2-3cm) 中位太低,
        夹爪手指底部会触碰台面。改为顶面抓取, 夹爪从顶面开始夹住物块上部。

        Args:
            pick: {'x','y','z'} 命令中的 pick 坐标 (base 系)
            surface_z_m: 物块顶面高度 (base 系, 米)
            block_height_m: 物块高度 (米)
            block_width_m: 物块短边宽度 (米, 用于夹爪闭合目标)
            min_grasp_z: 夹持点 z 安全下限 (米, 复用 MIN_GRASP_Z)
            gripper_pick_z_offset: link6 原点到夹爪夹持点的 z 距离 (米, 复用 GRIPPER_PICK_Z_OFFSET)

        Returns:
            dict: {
                'POSE_PICK': {'x','y','z'},         # link6 目标位姿 (base 系)
                'POSE_PICK_UP': {'x','y','z'},      # 抬起 12cm
                'clamping_z': float,                # 夹持点 z (米)
                'gripper_close_width': float,       # 闭合目标宽度 (米) = 0.0 (完全闭合)
            }
        """
        # 顶面抓取: clamping_z = surface_z (物块顶面高度, 基座坐标系)
        # 夹爪从顶面开始夹住物块上部, 避免手指底部触碰台面
        clamping_z = float(surface_z_m)
        # 安全下限 (复用 MIN_GRASP_Z)
        if clamping_z < min_grasp_z:
            self.get_logger().info(
                f"⚠️ 蓝方块 clamping_z={clamping_z:.3f}m 低于安全下限 "
                f"{min_grasp_z}m, 钳位至 {min_grasp_z}m (防碰台面)")
            clamping_z = min_grasp_z

        # POSE_PICK: link6 目标位姿 (夹持点 z + link6 偏置)
        POSE_PICK = {
            'x': float(pick['x']),
            'y': float(pick['y']),
            'z': clamping_z + gripper_pick_z_offset,
        }
        # POSE_PICK_UP: 抬起 12cm (脱离物块)
        POSE_PICK_UP = POSE_PICK.copy()
        POSE_PICK_UP['z'] += 0.12

        # 夹爪闭合目标: 完全闭合 (0.0m)
        # 检测宽度偏大 (HSV 掩膜扩张), 用 block_width-2mm 会导致夹爪未夹紧
        # 完全闭合时夹爪碰到物块会自然停住 (机械限位), 确保夹紧
        gripper_close_width = 0.0

        self.get_logger().info(
            f"📐 蓝方块抓取位姿: surface_z={surface_z_m:.3f}m "
            f"height={block_height_m:.3f}m → clamping_z={clamping_z:.3f}m "
            f"link6_z={POSE_PICK['z']:.3f}m pick=({POSE_PICK['x']:.3f},{POSE_PICK['y']:.3f}) "
            f"gripper_close={gripper_close_width*1000:.1f}mm")

        return {
            'POSE_PICK': POSE_PICK,
            'POSE_PICK_UP': POSE_PICK_UP,
            'clamping_z': clamping_z,
            'gripper_close_width': gripper_close_width,
        }

    def _plan_blue_block_orientation(self, pose_pick, block_rotation_deg):
        """📐 分层姿态约束预规划 (Task 2.1)。

        分层回退策略, 平衡求解率和姿态精度:
          第一层 (宽松垂直约束): RadialDownwardOri (复用 _build_pick_orientations_multi),
                                IK 候选池复用现有 get_ik_solution (内部 500 采样 + 6000 游走,
                                覆盖任务要求的 200 采样 + 50 游走),
                                final_ze < 0.52 (Z轴偏离地心 <30°), 位置误差 <0.015m
          第二层 (严格短轴对齐, 第一层失败时回退):
                                从 block_rotation_deg 构建目标四元数 (夹爪 Y 轴/闭合方向对齐物块短边),
                                IK 候选数量减少 (任务要求 50, 复用现有 solver),
                                首位插入 PCA 短轴对齐朝向 + yaw 约束引导 (±10°/±20°),
                                仍要求 final_ze < 0.52 和位置误差 <0.015m

        Args:
            pose_pick: {'x','y','z'} 目标位置 (base 系)
            block_rotation_deg: 物块旋转角 (度, [0,180), 长边方向绕 Z 轴)

        Returns:
            active_ori (list [qx,qy,qz,qw]) 或 None (两层均失败)
        """
        # IK 不可用时回退到径向朝下姿态 (无验证)
        if not self.enable_ik or self.ik_solver is None:
            ori = self._build_pick_orientation(pose_pick)[0]
            return [ori.x, ori.y, ori.z, ori.w]

        target_pos = np.array([float(pose_pick['x']), float(pose_pick['y']), float(pose_pick['z'])])
        initial_guess = None
        if self.current_joints:
            initial_guess = np.array([
                self.current_joints.get(f'joint{i}', 0.0) for i in range(1, 7)
            ])

        # ═══════ 第一层: 物块旋转角对齐 (从 block_rotation_deg 计算) ═══════
        # 蓝方块专用: 根据视觉检测的物块旋转角规划夹爪朝向
        # 物块长边方向 = block_rotation_deg, 短边方向 = 长边 + 90°
        # 夹爪 Y 轴 (闭合方向) 对齐短边方向, 确保夹住侧面而非棱边
        # 生成候选: 目标朝向 + 90° 倍数 (立方体侧面对称) + ±10°/±20° 微调
        self.get_logger().info(
            f"📐 蓝方块姿态预规划 - 第一层 (物块旋转角对齐, rotation={block_rotation_deg:.1f}°)")

        z_axis = [0.0, 0.0, -1.0]
        # 短边方向 = 长边 + 90° (夹爪 Y 轴对齐短边)
        short_axis_yaw = math.radians(float(block_rotation_deg)) + math.pi / 2.0

        # 生成候选朝向列表:
        # 1. 目标朝向 (短边方向) + 90° 倍数 (立方体侧面对称, 4 个等价方向)
        # 2. 每个方向 ±10°/±20° 微调 (适应旋转角检测误差)
        candidate_yaws = []
        for base_yaw in [short_axis_yaw, short_axis_yaw + math.pi/2,
                         short_axis_yaw + math.pi, short_axis_yaw + 3*math.pi/2]:
            for delta in [0.0, math.radians(10), math.radians(-10),
                          math.radians(20), math.radians(-20)]:
                yaw = base_yaw + delta
                # 归一化到 [0, 2π)
                yaw = yaw % (2 * math.pi)
                if yaw not in candidate_yaws:
                    candidate_yaws.append(yaw)

        # 构建朝向四元数列表 (夹爪 Y 轴 = 候选 yaw 方向)
        ori_lists = []
        for yaw in candidate_yaws:
            # Y 轴方向 (闭合方向, 对齐物块短边)
            y_axis = [math.cos(yaw), math.sin(yaw), 0.0]
            # X 轴 = Y × Z (右手系)
            x_axis = self._cross(y_axis, z_axis)
            x_axis = self._normalize(x_axis)
            y_axis = self._normalize(y_axis)
            q = self._matrix_to_quaternion(x_axis, y_axis, z_axis)
            ori_lists.append([q.x, q.y, q.z, q.w])

        # 兜底: 径向朝向 + 均匀 yaw 采样 (如果旋转角对齐全部失败)
        layer1_quats = self._build_pick_orientations_multi(pose_pick, num_yaw_samples=16)
        for ori in layer1_quats:
            if isinstance(ori, Quaternion):
                ori_lists.append([ori.x, ori.y, ori.z, ori.w])
            else:
                ori_lists.append(list(ori))

        rotation_aligned_count = len(candidate_yaws)
        for idx, ori_list in enumerate(ori_lists):
            target_quat = np.array(ori_list)
            ok, q_sol, err, comp_t = self.ik_solver.get_ik_solution(
                target_pos, target_quat, initial_guess=initial_guess)
            # 复用现有安全约束: final_ze < 0.52 (IK solver 内部已检查), 放宽位置误差到 0.020
            if ok and err < 0.020:
                src = "旋转角对齐" if idx < rotation_aligned_count else "径向/yaw兜底"
                self.get_logger().info(
                    f"✅ 第一层姿态求解成功: err={err:.4f} time={comp_t*1000:.1f}ms "
                    f"({src}, final_ze<0.52)")
                return ori_list

        self.get_logger().warn("⚠️ 第一层 (宽松垂直) 无有效解, 回退到第二层 (严格短轴对齐)")

        # ═══════ 第二层: 严格短轴对齐 (从 block_rotation_deg 构建) ═══════
        # 夹爪 Y 轴 (闭合方向) 对齐物块短边
        # 物块长边方向 = block_rotation_deg (绕 Z 轴), 短边方向 = 长边 + 90°
        self.get_logger().info(
            f"📐 蓝方块姿态预规划 - 第二层 (严格短轴对齐, rotation={block_rotation_deg:.1f}°)")

        z_axis = [0.0, 0.0, -1.0]
        yaw_rad = math.radians(float(block_rotation_deg))
        # 短边方向 = 长边 + 90°
        short_axis_yaw = yaw_rad + math.pi / 2.0
        y_axis = [math.cos(short_axis_yaw), math.sin(short_axis_yaw), 0.0]
        # x = y × z (右手系: y × z = x)
        x_axis = self._cross(y_axis, z_axis)
        x_axis = self._normalize(x_axis)
        y_axis = self._normalize(y_axis)
        strict_quat = self._matrix_to_quaternion(x_axis, y_axis, z_axis)
        strict_ori = [strict_quat.x, strict_quat.y, strict_quat.z, strict_quat.w]

        # 首位插入 PCA 短轴对齐朝向 (从 _latest_detection 读取, 若有效)
        # 视觉节点 blue_block_detector.py 发布格式: {"detections": [{"grasp_orientation": {"qx":..,"qy":..,"qz":..,"qw":..}}]}
        layer2_ori_lists = []
        latest_det = getattr(self, '_latest_detection', None)
        if latest_det:
            pca_ori = None
            detections = latest_det.get('detections', [])
            if detections and isinstance(detections, list):
                pca_ori = detections[0].get('grasp_orientation')
            else:
                pca_ori = latest_det.get('grasp_orientation')
            # 视觉节点发布 dict {qx,qy,qz,qw}, 转为 list
            if isinstance(pca_ori, dict) and all(k in pca_ori for k in ('qx', 'qy', 'qz', 'qw')):
                pca_list = [float(pca_ori['qx']), float(pca_ori['qy']),
                            float(pca_ori['qz']), float(pca_ori['qw'])]
                # 排除单位四元数 (视觉节点默认值 qx=qy=qz=0, qw=1, 非真实 PCA)
                if abs(pca_list[0]) + abs(pca_list[1]) + abs(pca_list[2]) > 1e-6:
                    layer2_ori_lists.append(pca_list)
                    self.get_logger().info("📐 第二层: 首位插入 PCA 短轴对齐朝向 (来自视觉节点)")

        # 严格短轴对齐朝向
        layer2_ori_lists.append(strict_ori)
        # yaw 约束引导: ±10°, ±20° 偏移 (松弛约束, 提高求解率)
        for delta_deg in (10.0, -10.0, 20.0, -20.0):
            delta_yaw = short_axis_yaw + math.radians(delta_deg)
            y2 = [math.cos(delta_yaw), math.sin(delta_yaw), 0.0]
            x2 = self._cross(y2, z_axis)
            x2 = self._normalize(x2)
            y2 = self._normalize(y2)
            q2 = self._matrix_to_quaternion(x2, y2, z_axis)
            layer2_ori_lists.append([q2.x, q2.y, q2.z, q2.w])

        # 第二层 IK 候选数量减少 (任务要求 50, 复用现有 solver)
        for ori_list in layer2_ori_lists:
            target_quat = np.array(ori_list)
            ok, q_sol, err, comp_t = self.ik_solver.get_ik_solution(
                target_pos, target_quat, initial_guess=initial_guess)
            if ok and err < 0.015:
                self.get_logger().info(
                    f"✅ 第二层姿态求解成功: err={err:.4f} time={comp_t*1000:.1f}ms "
                    f"(严格短轴对齐, final_ze<0.52)")
                return ori_list

        self.get_logger().error("❌ 蓝方块姿态预规划: 两层均无有效解")
        return None

    def _execute_blue_block_sort(self, data, config):
        """🟦 执行蓝方块分拣序列 (Task 2.1, step0-step9)。

        完全复用现有 sort 序列结构和参数, 差异调整:
          - step0:   开夹爪
          - step0.5: 分层姿态预规划 (调用 _plan_blue_block_orientation)
          - step1:   关节空间移动到 POSE_PICK_UP, continuous=True
          - step2:   笛卡尔直线下降到 POSE_PICK, 禁用传送带碰撞 (复用 _set_bin_collision_allowed)
          - step3:   闭合夹爪, 目标宽度 = block_width_m - 0.002m (不是 GRIPPER_GRAB=0)
          - step4:   上提至 POSE_PICK_UP, continuous=True
          - step4.5: 抓取成功判定 = 闭合宽度 < block_width_m * 0.7 (不是固定 5mm)
          - step5-9: 关节空间移动到料框放置位置 + 释放 + 回待机位
          - 碰撞检测恢复 (成功和错误路径均, 复用现有逻辑)
          - 失败时重试最多 2 次 (复用现有重试逻辑)

        Args:
            data: 命令 JSON dict, 含 pick/surface_z_m/block_width_m/block_height_m/block_rotation_deg
            config: 常量 dict {
                'JOINT_STANDBY', 'JOINT_OBSERVE', 'BIN_JOINTS',
                'GRIPPER_OPEN', 'GRIPPER_SETTLE_SEC',
                'GRIPPER_PICK_Z_OFFSET', 'MIN_GRASP_Z', 'loop_count'
            }

        Returns:
            dict: {'success': bool, 'cycle_id': str}
        """
        JOINT_STANDBY = config['JOINT_STANDBY']
        JOINT_OBSERVE = config['JOINT_OBSERVE']
        BIN_JOINTS = config['BIN_JOINTS']
        GRIPPER_OPEN = config['GRIPPER_OPEN']
        GRIPPER_SETTLE_SEC = config['GRIPPER_SETTLE_SEC']
        GRIPPER_PICK_Z_OFFSET = config['GRIPPER_PICK_Z_OFFSET']
        MIN_GRASP_Z = config['MIN_GRASP_Z']
        # 蓝方块专用安全下限: 水果的 0.045m 对 1-5cm 物块过高
        # 桌面(传送带台面)在 z≈-0.015m, 蓝方块中位 z≈0.003m (3cm 物块)
        # 设 0.0m: 比桌面高 15mm, 夹爪手指有足够离地间隙
        MIN_GRASP_Z = 0.0
        # 蓝方块专用 Z 偏移: FK 实测 gripper_link1/2 在 link6 下方 135.5mm
        # (非水果模式的 64mm). 用 0.1355m 让手指对齐物块顶面, 避免触碰台面.
        # FK 诊断: link6_z=0.0998 → gripper_link1/2_z=-0.0357, 距离=0.1355m
        GRIPPER_PICK_Z_OFFSET = 0.1355
        loop_count = config.get('loop_count', 0)
        MAX_BLUE_BLOCK_RETRIES = 2  # 蓝方块重试上限 (任务要求 2 次)

        # ── 解析命令参数 ──
        pick = data["pick"]
        surface_z_m = float(data.get("surface_z_m", pick.get('z', 0.05)))
        block_width_m = float(data.get("block_width_m", 0.03))
        block_height_m = float(data.get("block_height_m", 0.02))
        block_rotation_deg = float(data.get("block_rotation_deg", 0.0))
        pick_id = data.get("pick_name", "blue_block (detected)")
        cycle_id = str(data.get("cycle_id", ""))
        try:
            bin_num = int(data.get("bin", 1))
        except (TypeError, ValueError):
            bin_num = 1
        if bin_num not in BIN_JOINTS:
            self.get_logger().warn(f"⚠️ 蓝方块: 无效 bin={bin_num}, 默认使用料框1")
            bin_num = 1
        BIN_ABOVE_JOINTS, BIN_PLACE_JOINTS = BIN_JOINTS[bin_num]
        place_id = f"料框{bin_num}"
        obj_class = "blue_block"

        # ── 计算抓取位姿 (顶面中位抓取) ──
        pose_data = self._compute_blue_block_pick_pose(
            pick, surface_z_m, block_height_m, block_width_m,
            MIN_GRASP_Z, GRIPPER_PICK_Z_OFFSET)
        POSE_PICK = pose_data['POSE_PICK']
        POSE_PICK_UP = pose_data['POSE_PICK_UP']
        gripper_close_width = pose_data['gripper_close_width']

        print(f"\n\n====================== 第 {loop_count} 次分拣 [🟦蓝方块] "
              f"(从 {pick_id} 到 {place_id}) ======================")

        # 清空命令队列 (避免堆积)
        while not self.cmd_queue.empty():
            self.cmd_queue.get_nowait()

        success = True
        grasp_reached_ok = False
        place_reached_ok = False
        cycle_strategies = []
        cycle_profiles = []
        self.last_planning_profile_name = ''
        self.last_planning_strategy = ''

        for _ in range(1):
            # 【第零步】 在前往途中或起点提前张开夹爪
            self.operate_gripper(GRIPPER_OPEN, "蓝方块-张开夹爪(准备抓取)")
            if self._check_emergency("blue_block-step0-张开夹爪"):
                success = False; break

            # ── step0.5: 分层姿态预规划 (第一层宽松垂直 → 第二层严格短轴对齐) ──
            active_ori = self._plan_blue_block_orientation(POSE_PICK, block_rotation_deg)
            if active_ori is None:
                # 两层均失败: 返回 CONTROL_FAILED, 机械臂回待机位
                self.get_logger().error("❌ 蓝方块姿态预规划失败 (两层均无解, CONTROL_FAILED), 回待机位")
                self.move_arm_joint(JOINT_STANDBY, "蓝方块-姿态失败回待机位")
                success = False; break

            # 【第一步】 关节空间移动到 POSE_PICK_UP, continuous=True
            if not self.move_arm_pose(POSE_PICK_UP, "蓝方块-抓取位上方过渡点",
                                       continuous=True, planning_mode='normal',
                                       preferred_orientation=active_ori):
                success = False; break
            if self.last_planning_strategy:
                cycle_strategies.append(self.last_planning_strategy)
            if self.last_planning_profile_name:
                cycle_profiles.append(self.last_planning_profile_name)

            # 记录抓取过渡点的关节角, 用于安全抬起
            pick_up_joints = self.current_joints.copy() if self.current_joints else None
            if self._check_emergency("blue_block-step1-上方过渡点"):
                success = False; break

            # ── 下降前禁用所有传送带碰撞体 (复用 _set_bin_collision_allowed) ──
            self._set_bin_collision_allowed(allowed=True)

            # ═══════ 抓取重试循环 (最多 MAX_BLUE_BLOCK_RETRIES 次) ═══════
            grasp_ok = False
            gw = gf = 0.0
            reason = ""
            for retry in range(MAX_BLUE_BLOCK_RETRIES):
                if retry > 0:
                    # ── 重试前准备: 回观察位 → 视觉刷新 → 重新规划姿态 ──
                    self.get_logger().warn(
                        f"🔄 蓝方块抓取重试 {retry+1}/{MAX_BLUE_BLOCK_RETRIES}...")
                    self.operate_gripper(GRIPPER_OPEN, f"蓝方块-重试{retry+1}-张开夹爪")
                    time.sleep(0.2)
                    # 回观察位 (相机视野无遮挡, 蓝方块主要用命令参数坐标, 此处备用刷新)
                    self.get_logger().info("📡 蓝方块重试: 回观察位...")
                    self.move_arm_joint(JOINT_OBSERVE, f"蓝方块-重试{retry+1}-回观察位")
                    time.sleep(1.0)
                    # ── 从最新检测刷新 POSE_PICK (物块可能被上次抓取推走) ──
                    latest_det = getattr(self, '_latest_detection', None)
                    if latest_det:
                        detections = latest_det.get('detections', [])
                        if detections and isinstance(detections, list):
                            det = detections[0]
                            bc = det.get('base_coords', {})
                            if bc and all(k in bc for k in ('x', 'y', 'z')):
                                new_x = float(bc['x'])
                                new_y = float(bc['y'])
                                new_surf = det.get('surface_z_m', surface_z_m)
                                old_x, old_y = POSE_PICK['x'], POSE_PICK['y']
                                # 仅当坐标变化 >1cm 时更新 (避免噪声导致频繁变化)
                                if abs(new_x - old_x) > 0.01 or abs(new_y - old_y) > 0.01:
                                    self.get_logger().info(
                                        f"📡 蓝方块重试: 检测坐标更新 "
                                        f"({old_x:.3f},{old_y:.3f})→({new_x:.3f},{new_y:.3f})")
                                    POSE_PICK['x'] = new_x
                                    POSE_PICK['y'] = new_y
                                    # 重新计算 clamping_z 和 link6_z
                                    clamping_z = max(float(new_surf), MIN_GRASP_Z)
                                    POSE_PICK['z'] = clamping_z + GRIPPER_PICK_Z_OFFSET
                                    POSE_PICK_UP = POSE_PICK.copy()
                                    POSE_PICK_UP['z'] += 0.12
                                    # 更新 block 参数
                                    surface_z_m = float(new_surf)
                                    block_rotation_deg = det.get('block_rotation_deg', block_rotation_deg)
                                    block_width_m = det.get('block_width_m', block_width_m)
                                    block_height_m = det.get('block_height_m', block_height_m)
                                    self.get_logger().info(
                                        f"📐 蓝方块重试: 位姿更新 surface_z={surface_z_m:.3f}m "
                                        f"clamping_z={clamping_z:.3f}m link6_z={POSE_PICK['z']:.3f}m")
                    # 重新规划姿态 (坐标可能已更新)
                    active_ori = self._plan_blue_block_orientation(POSE_PICK, block_rotation_deg)
                    if active_ori is None:
                        self.get_logger().warn("⚠️ 蓝方块重试: 姿态预规划失败, 跳过本次重试")
                        continue
                    # 从观察位移到 POSE_PICK_UP
                    if not self.move_arm_pose(POSE_PICK_UP, f"蓝方块-重试{retry+1}-上方过渡点",
                                               continuous=False, planning_mode='normal',
                                               preferred_orientation=active_ori):
                        self.get_logger().warn("⚠️ 蓝方块重试: 上方过渡点移动失败, 跳过")
                        continue
                    pick_up_joints = self.current_joints.copy() if self.current_joints else None
                    self._set_bin_collision_allowed(allowed=True)
                    if self._check_emergency(f"blue_block-retry{retry+1}-上方过渡点"):
                        success = False; break

                # 【第二步】 IK 关节空间下降到 POSE_PICK (禁用传送带碰撞)
                # 蓝方块专用: IK 关节空间优先 (固定 joint6, 避免笛卡尔下降时 j6 大幅旋转)
                # 第一层姿态已用 initial_guess 求解, IK 解接近当前关节角, j6 不会大幅旋转
                descent_success = False
                if self.enable_ik and self.ik_solver is not None:
                    pick_orientations = [active_ori] + self._build_pick_orientations_multi(POSE_PICK)
                    ik_ok, ik_joints = self.move_arm_via_ik(
                        POSE_PICK, pick_orientations,
                        "蓝方块-下降抓取 (IK关节空间)", continuous=True)
                    if ik_ok:
                        descent_success = True
                        cycle_strategies.append('ik_joint_space')
                        cycle_profiles.append('ik_solution')
                        self.get_logger().info("✅ 蓝方块 IK 关节空间下降完成 (joint6 稳定)")
                        # FK 诊断: 计算 gripper_link1/2 实际位置, 确认手指 z 坐标
                        if ik_joints is not None and self.ik_solver is not None:
                            for fname in ['link6', 'gripper_link1', 'gripper_link2']:
                                pos = self.ik_solver.get_frame_position(ik_joints, fname)
                                if pos is not None:
                                    self.get_logger().info(
                                        f"📍 FK 诊断: {fname} 位置 = "
                                        f"({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})  "
                                        f"目标 surface_z={surface_z_m:.4f} 台面≈{surface_z_m - block_height_m:.4f}")

                if not descent_success:
                    self.get_logger().info("🟡 蓝方块-IK失败, 回退到笛卡尔直线 (jump_threshold=0.8)...")
                    pose_pick_msg = self._create_pose(POSE_PICK, active_ori)
                    # 笛卡尔兜底: 降低 jump_threshold 到 0.8, 检测 j6 突变 (>45°)
                    if self.execute_cartesian_path(
                            [pose_pick_msg],
                            f"蓝方块-下降抓取 (笛卡尔直线兜底,retry{retry+1})",
                            fraction_threshold=0.50,
                            jump_threshold=0.8):
                        descent_success = True
                        cycle_strategies.append('cartesian_line')
                        cycle_profiles.append('cartesian')
                        self.get_logger().info("✅ 蓝方块笛卡尔直线下降完成 (兜底)")

                if not descent_success:
                    self.get_logger().info("🟡 蓝方块-直线插补受限, 启动多路并发退避规划...")
                    if not self.move_arm_cartesian(
                            POSE_PICK, f"蓝方块-下降抓取 (退避规划,retry{retry+1})",
                            continuous=True, preferred_orientation=active_ori,
                            allow_position_only_fallback=True, planning_mode='descend'):
                        continue  # 下降失败跳过本次重试
                    if self.last_planning_strategy:
                        cycle_strategies.append(self.last_planning_strategy)
                    if self.last_planning_profile_name:
                        cycle_profiles.append(self.last_planning_profile_name)

                if self._check_emergency(f"blue_block-step2-下降抓取,retry{retry+1}"):
                    success = False; break

                # 【第三步】 闭合夹爪 — 目标宽度 = block_width_m - 0.002m (不是 GRIPPER_GRAB=0)
                self._pre_grasp_width = (
                    self._latest_gripper_status.width
                    if self._latest_gripper_status is not None else None
                )
                self.operate_gripper(
                    gripper_close_width,
                    f"蓝方块-闭合夹爪(目标宽度={gripper_close_width*1000:.1f}mm)")
                time.sleep(GRIPPER_SETTLE_SEC)
                time.sleep(0.05)  # planning_scene 更新
                if self._check_emergency(f"blue_block-step3-闭合夹爪,retry{retry+1}"):
                    success = False; break

                # 【第四步】 上提至 POSE_PICK_UP, continuous=True (途径点, 不停留)
                _lift_ok = False
                if pick_up_joints:
                    sorted_joints = [pick_up_joints[f'joint{i}'] for i in range(1, 7)]
                    if self.move_arm_joint(sorted_joints, "蓝方块-抬起→夹取上方 (途径点)", continuous=True):
                        _lift_ok = True
                    else:
                        self.get_logger().warn("⚠️ 蓝方块-抬起(关节空间)失败(-2), 回退到笛卡尔")
                if not _lift_ok:
                    POSE_LIFT = POSE_PICK.copy()
                    POSE_LIFT['z'] = POSE_PICK_UP['z']
                    _lift_pose_msg = self._create_pose(POSE_LIFT, active_ori)
                    if self.execute_cartesian_path([_lift_pose_msg], "蓝方块-抬起脱离 (直线插补)", fraction_threshold=0.50):
                        _lift_ok = True
                    elif not self.move_arm_cartesian(
                            POSE_LIFT, "蓝方块-抬起脱离 (退避规划)",
                            continuous=True, preferred_orientation=active_ori,
                            allow_position_only_fallback=True, planning_mode='retreat'):
                        continue  # 抬起失败跳过本次重试
                    if self.last_planning_strategy:
                        cycle_strategies.append(self.last_planning_strategy)
                    if self.last_planning_profile_name:
                        cycle_profiles.append(self.last_planning_profile_name)
                grasp_reached_ok = True
                if self._check_emergency(f"blue_block-step4-抬起,retry{retry+1}"):
                    success = False; break

                # 【第四步半】 抓取成功检测
                # 判定逻辑: 夹爪目标闭合宽度 = block_width_m - 0.002 (略小于物块宽度)
                #   - 抓到物块: 物块阻止夹爪完全闭合, 实际宽度 ≈ block_width_m
                #   - 没抓到: 夹爪完全闭合, 宽度 ≈ 0
                # 因此: width > block_width_m * 0.7 → 物块在两指之间 → 成功
                #       width < block_width_m * 0.7 → 夹爪空闭 → 失败
                time.sleep(0.1)  # 等待夹爪宽度稳定
                rclpy.spin_once(self, timeout_sec=0.1)
                if self._latest_gripper_status is not None:
                    gw = self._latest_gripper_status.width
                    gf = self._latest_gripper_status.force
                else:
                    gw, gf = 0.0, 0.0
                success_threshold = block_width_m * 0.7
                if gw > success_threshold:
                    grasp_ok = True
                    reason = "blue_block_success"
                else:
                    grasp_ok = False
                    reason = "blue_block_empty"
                self.get_logger().info(
                    f"🔍 蓝方块抓取检测 (retry {retry+1}/{MAX_BLUE_BLOCK_RETRIES}): "
                    f"success={grasp_ok} width={gw*1000:.1f}mm "
                    f"force={gf:.2f}N threshold={success_threshold*1000:.1f}mm "
                    f"(width>{success_threshold*1000:.1f}mm→成功) reason={reason}")
                self._publish_grasp_status(grasp_ok, gw, gf, reason)
                if self._check_emergency(f"blue_block-step4.5-抓取检测,retry{retry+1}"):
                    success = False; break

                if grasp_ok:
                    self.get_logger().info(
                        f"✅ 蓝方块抓取成功 (retry {retry+1}/{MAX_BLUE_BLOCK_RETRIES})")
                    break  # 成功! 跳出重试循环

                if retry < MAX_BLUE_BLOCK_RETRIES - 1:
                    self.get_logger().warn(
                        f"⚠️ 蓝方块抓取失败 (retry {retry+1}/{MAX_BLUE_BLOCK_RETRIES}, "
                        f"reason={reason}), 准备重试...")
                    continue

            # ═══════ 重试循环结束 ═══════

            if not grasp_ok:
                self.get_logger().error(
                    f"❌ 蓝方块 {MAX_BLUE_BLOCK_RETRIES}次抓取全部失败 "
                    f"(最后reason={reason}), 跳过本轮")
                self.operate_gripper(GRIPPER_OPEN, "蓝方块-重试耗尽-张开夹爪")
                self.move_arm_joint(JOINT_STANDBY, "蓝方块-重试耗尽-回待机位")
                if self._check_emergency("blue_block-retry_exhausted-回待机位"):
                    pass  # 已安全停机
                success = False; break

            # 【第五步】 关节空间到料框上方 (需稳定后再下降)
            # ⚠️ continuous=False: BIN_ABOVE完成后等0.15s稳定, 否则BIN_PLACE起点偏差→-4
            self._set_bin_collision_allowed(allowed=True)
            if not self.move_arm_joint(BIN_ABOVE_JOINTS, f"蓝方块-移动到{place_id}上方 (途径点)", continuous=False):
                success = False; break
            if self._check_emergency("blue_block-step5-料框上方"):
                success = False; break

            # 【第六步】 关节空间到料框放置位
            if not self.move_arm_joint(BIN_PLACE_JOINTS, f"蓝方块-移动到{place_id}放置位 (关节空间)", continuous=True):
                success = False; break
            place_reached_ok = True
            if self._check_emergency("blue_block-step6-料框放置位"):
                success = False; break

            # 【第七步】 松开夹爪释放物品
            self.operate_gripper(GRIPPER_OPEN, f"蓝方块-松开夹爪(在{place_id}放置位释放)")
            time.sleep(GRIPPER_SETTLE_SEC)
            if self._check_emergency("blue_block-step7-松开夹爪"):
                success = False; break

            # 【第八步】 关节空间回到料框上方 (抬起脱离放置位, 途径点)
            if not self.move_arm_joint(BIN_ABOVE_JOINTS, f"蓝方块-从{place_id}放置位抬起 (途径点)", continuous=True):
                success = False; break
            if self._check_emergency("blue_block-step8-料框抬起"):
                success = False; break

            # 【第九步】 关节空间回到待机位 (等待下一次抓取)
            if not self.move_arm_joint(JOINT_STANDBY, "蓝方块-回到待机位 (关节空间)", continuous=True):
                success = False; break
            if self._check_emergency("blue_block-step9-回待机位"):
                success = False; break

        # ── 碰撞检测恢复 (成功和错误路径均) ──
        final_strategy = 'cartesian_only'
        if 'ik_joint_space' in cycle_strategies:
            final_strategy = 'ik_joint_space'
        elif 'free_yaw' in cycle_strategies:
            final_strategy = 'free_yaw'
        elif 'relaxed' in cycle_strategies:
            final_strategy = 'relaxed'
        elif 'direct' in cycle_strategies:
            final_strategy = 'direct'
        elif 'position_only' in cycle_strategies:
            final_strategy = 'position_only'

        if not success:
            # ── 急停锁定: 跳过自动回待机位, 等待复位 ──
            if self._emergency_flag and self._emergency_req_type == "estop":
                self.get_logger().error(
                    "⛔ 急停锁定中, 跳过自动退回. 按【复位】按钮回待机位.")
                status_msg = String(); status_msg.data = 'estop'
                self.status_pub.publish(status_msg)
                self.get_logger().info("⏳ 等待复位信号...")
                while rclpy.ok() and self._emergency_flag:
                    rclpy.spin_once(self, timeout_sec=0.2)
                    if self._emergency_req_type == "reset":
                        self._check_emergency("blue_block-error_handler-reset")
                        break
                return {'success': False, 'cycle_id': cycle_id}

            self.get_logger().info("⚠️ 蓝方块执行失败！启动自动退回防护...")
            status_msg = String(); status_msg.data = 'error'
            self.status_pub.publish(status_msg)
            result_msg = String()
            result_msg.data = json.dumps({
                'cycle_success': 0,
                'cycle_id': cycle_id,
                'grasp_reached_ok': int(grasp_reached_ok),
                'place_reached_ok': int(place_reached_ok),
                'planning_strategy': final_strategy,
                'planning_profile': cycle_profiles[-1] if cycle_profiles else '',
                'planning_profiles': cycle_profiles,
                'block_width_m': block_width_m,
                'gripper_close_target': round(gripper_close_width, 4),
            }, ensure_ascii=False)
            self.cycle_result_pub.publish(result_msg)

            self.operate_gripper(GRIPPER_OPEN, "蓝方块-防碰撞提前张开夹爪")
            self.move_arm_joint(JOINT_STANDBY, "蓝方块-自动回到待机位")

            # ── 发布机械臂状态 (失败) ──
            self._publish_robot_status(
                success=False, cargo_type=obj_class, confidence=0.95,
                source_pos_dict={"x": float(POSE_PICK['x']), "y": float(POSE_PICK['y']),
                                 "z": float(POSE_PICK['z'])},
                target_joints=BIN_PLACE_JOINTS, grasp_ok=False, error_code=99)

            # 安全回到待机位后恢复碰撞检测 (无论回待机位是否成功都恢复)
            self._set_bin_collision_allowed(allowed=False)
            self.operate_gripper(GRIPPER_OPEN, "蓝方块-张开夹爪(待机)")
            time.sleep(0.5)
            return {'success': False, 'cycle_id': cycle_id}

        # 成功完成分拣后恢复碰撞检测
        self._set_bin_collision_allowed(allowed=False)

        # ── 发布机械臂状态 (成功) ──
        self._publish_robot_status(
            success=True, cargo_type=obj_class, confidence=0.95,
            source_pos_dict={"x": float(POSE_PICK['x']), "y": float(POSE_PICK['y']),
                             "z": float(POSE_PICK['z'])},
            target_joints=BIN_PLACE_JOINTS, grasp_ok=True, error_code=0)

        print(f"🎉 第 {loop_count} 次蓝方块分拣顺利完成！休整一下...")
        result_msg = String()
        result_msg.data = json.dumps({
            'cycle_success': 1,
            'cycle_id': cycle_id,
            'grasp_reached_ok': int(grasp_reached_ok),
            'place_reached_ok': int(place_reached_ok),
            'planning_strategy': final_strategy,
            'planning_profile': cycle_profiles[-1] if cycle_profiles else '',
            'planning_profiles': cycle_profiles,
            'block_width_m': block_width_m,
            'gripper_close_target': round(gripper_close_width, 4),
        }, ensure_ascii=False)
        self.cycle_result_pub.publish(result_msg)
        time.sleep(0.5)
        return {'success': True, 'cycle_id': cycle_id}


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
    
    # ── 默认待机位 (eye-to-hand: 相机固定, 用待机位作为默认位置) ──
    # 程序启动/任务完成/错误恢复 都回到此位置
    JOINT_STANDBY = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # ── 观察位坐标记录 (保留供未来调用, eye-to-hand 下不使用) ──
    # 眼在手内配置用: 相机装在末端, 需到此位置观察传送带
    # joint1=0, joint2=1.0669, joint3=-1.1637, joint4=0, joint5=1.2185, joint6=0
    # 对应末端位姿: x=0.1864, y=0.0, z=0.3782, q=(~0, 0.9643, ~0, 0.2650)
    JOINT_OBSERVE = [0.0, 1.0669023184516138, -1.1636808254746993, 0.0, 1.2184841639873214, 0.0]

    # ========== 料框关节点位 (用户标定, 关节空间) ==========
    # 抓取后流程: pick_up_joints -> bin_above -> bin_place -> 开夹爪 -> bin_above -> standby
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
    GRIPPER_SETTLE_SEC = 0.05
    MAX_GRASP_RETRIES = 3       # 抓取失败最大重试次数

    # ── 抓取点 z 补偿: link6 原点到夹爪夹持点的 z 距离 ──
    # 实测 (2026-07-20): 青苹果 det_z=0.07, 正确夹住时 link6_z=0.136, offset=0.066
    # 当前值 0.072 = 实测 0.066 + 微调 0.006 (用户从 0.066 上调)
    # 原值 0.11 偏高 4.4cm 导致空夹; 若仍空夹→减小, 若碰桌面→增大, 步长 0.005
    GRIPPER_PICK_Z_OFFSET = 0.064

    # ── 夹爪夹持点 z 安全下限 (base 系, 米) ──
    # 防止 D455 深度偏差导致夹爪夹持点低于台面 → 碰撞.
    # 物体高度 4.6-7.1cm, 中心约 2.3-3.5cm; 夹爪手指延伸到夹持点下方,
    # 设 0.045m 确保夹持点不低于台面上 4.5cm, 给手指留出离地间隙.
    # (0.025→0.035→0.045 逐步上调; Z 补偿已在 YOLO 节点 z_offset_m 源头修正,
    #  此值为最后安全兜底; 若仍碰→增大, 若空夹→减小, 步长 0.005)
    MIN_GRASP_Z = 0.045

    # ── eye-to-hand 标定系统性 X/Y 偏移补偿 (单位: 米) ──
    # ⚠️ 已在 realsense_yolo_node.py 源头补偿 (x_offset_m/y_offset_m 参数),
    # 此处设 0 避免双重修正. 如需额外微调, 改为非 0 值叠加补偿.
    PICK_X_OFFSET = 0.0   # 已在 YOLO 节点源头补偿
    PICK_Y_OFFSET = 0.0   # 已在 YOLO 节点源头补偿

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
        
        # 初始运行时到待机位待机 (用户要求: 程序一启动就到待机位)
        node.move_arm_joint(JOINT_STANDBY, "初始回到待机位 (Standby)")
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
                node.move_arm_joint(JOINT_STANDBY, "退出前回到待机位")
                node.operate_gripper(GRIPPER_OPEN, "退出前张开夹爪")
                break

            elif data.get("cmd") == "reset":
                node.is_busy = True
                status_msg.data = 'busy'
                node.status_pub.publish(status_msg)

                print("\n🔄 收到客户端指令：一键回到待机位并关闭夹爪")
                node.move_arm_joint(JOINT_STANDBY, "回到待机位")
                node.operate_gripper(GRIPPER_OPEN, "张开夹爪(待机)")

            elif data.get("cmd") == "observe":
                node.is_busy = True
                status_msg.data = 'busy'
                node.status_pub.publish(status_msg)

                print("\n👁️ 收到客户端指令：回到待机位")
                # 关节空间直接运动到待机位，不改变夹爪状态
                node.move_arm_joint(JOINT_STANDBY, "回到待机位 (关节空间)")
                
            elif data.get("cmd") == "set_gripper_force":
                # 力矩调节 (GUI 滑块/预设按钮发送), 不需要 is_busy
                force = float(data.get("force", 1.5))
                force = max(node._GRIPPER_FORCE_MIN, min(node._GRIPPER_FORCE_MAX, force))
                node._gripper_force = force
                node.get_logger().info(f"🔧 夹爪力矩已设置为 {force:.2f}N")

            elif data.get("cmd") in ("sort", "sort_verify", "sort_graspnet"):
                cmd_type = data.get("cmd")
                # sort: 直接用 pick_pose (无精定位)
                # sort_verify: 两阶段精定位 (eye-to-hand: 固定相机远距离检测 + PCA 短轴对齐)
                # sort_graspnet: GraspNet 主导 (待机位深度图 → 6DoF 位姿, 失败直接跳过不回退)
                two_stage = (cmd_type == "sort_verify")  # eye-to-hand: sort_verify 仍可用两阶段
                use_graspnet = (cmd_type == "sort_graspnet")
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
                # ── eye-to-hand 标定 X/Y 系统性偏差补偿 ──
                # base_position_m 的 X/Y 偏大 2-3cm, 减去补偿量修正
                POSE_PICK['x'] -= PICK_X_OFFSET
                POSE_PICK['y'] -= PICK_Y_OFFSET
                print(f"📐 X/Y 补偿: dx={-PICK_X_OFFSET:+.3f} dy={-PICK_Y_OFFSET:+.3f} "
                      f"→ x={POSE_PICK['x']:.3f} y={POSE_PICK['y']:.3f}")
                # link6 原点到夹爪夹持点的 z 距离 (GRIPPER_PICK_Z_OFFSET=0.064)
                # 抓取高度优先用质心 z (D455 深度点云估算, eye-to-hand 配置下深度可靠);
                # fallback: 表面 z + 按类别的经验 z 修正 (PER_OBJECT_Z_CORRECTION)
                # 安全钳位: 夹爪夹持点 z 不得低于台面上方 MIN_GRASP_Z (防深度偏差碰台面)
                centroid_z = None
                latest_det = getattr(node, '_latest_detection', None)
                if latest_det:
                    centroid_z = latest_det.get('centroid_base_z')
                if centroid_z is not None:
                    print(f"📐 质心补偿: centroid_z={centroid_z:.3f}m (D455点云估算, 替代表面z+经验修正)")
                    clamping_z = float(centroid_z)
                else:
                    clamping_z = POSE_PICK['z'] + z_correction
                    if z_correction:
                        print(f"📏 无质心数据, 用表面z+经验修正: {obj_class} z_corr={z_correction:+.3f}m")
                if clamping_z < MIN_GRASP_Z:
                    print(f"⚠️ 夹持点 z={clamping_z:.3f}m 低于安全下限 "
                          f"{MIN_GRASP_Z}m, 钳位至 {MIN_GRASP_Z}m (防碰台面)")
                    clamping_z = MIN_GRASP_Z
                POSE_PICK['z'] = clamping_z + GRIPPER_PICK_Z_OFFSET
                POSE_PICK_UP = POSE_PICK.copy()
                POSE_PICK_UP['z'] += 0.13  # 抬起脱离高 13cm
                POSE_LIFT_SOFT = POSE_PICK.copy()
                POSE_LIFT_SOFT['z'] += 0.035  # 先小幅抬起，降低惯性导致的滑脱

                loop_count += 1
                if use_graspnet:
                    mode_label = " [GraspNet主导]"
                elif two_stage:
                    mode_label = " [两阶段精定位]"
                else:
                    mode_label = ""
                print(f"\n\n====================== 第 {loop_count} 次分拣{mode_label} (从 {pick_id} 到 {place_id}) ======================")

                # 以下为具体的机械臂序列
                # 放一个空的队列清理，以免堆积
                while not node.cmd_queue.empty():
                    node.cmd_queue.get_nowait()

                success = True
                grasp_reached_ok = False
                place_reached_ok = False
                for _ in range(1):
                    # ── 两阶段精定位 (sort_verify) ──
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
                            node.move_arm_joint(JOINT_STANDBY, "两阶段失败→回到待机位")
                            node.operate_gripper(GRIPPER_OPEN, "张开夹爪(两阶段失败)")
                            success = False; break
                        # 用精定位坐标更新相关位姿
                        for p in (POSE_PICK, POSE_PICK_UP, POSE_LIFT_SOFT):
                            p['x'] = refine_result['x']
                            p['y'] = refine_result['y']
                        # 两阶段质心补偿: 用二次检测的质心 z 重算抓取高度
                        # (替代表面 z + 经验修正, 夹爪夹物体腰部/质心而非顶部)
                        # ⚠️ 仅当 refine_centroid_z 为有效正值时才覆盖 z:
                        # 验证位(verify_height=0.25m)相机离物体<0.4m, D455 近距离深度
                        # 严重失真(系统性偏大), 导致 object_base_z=robot_z-depth 变负
                        # (如 0.25-0.85=-0.60m). 负值被 MIN_GRASP_Z 钳位到 0.025m, 且
                        # 遮/不遮 IR 都一样(质心z和表面z都负→都钳位), 表现为"双目无变化".
                        # 修复: 非正值时保留待机位(远距离 D455 正常, z≈0.054m)算出的 z.
                        refine_centroid_z = refine_result.get('centroid_z')
                        if refine_centroid_z is not None and float(refine_centroid_z) > 0.0:
                            rcz = max(float(refine_centroid_z), MIN_GRASP_Z)
                            POSE_PICK['z'] = rcz + GRIPPER_PICK_Z_OFFSET
                            POSE_PICK_UP['z'] = POSE_PICK['z'] + 0.13
                            POSE_LIFT_SOFT['z'] = POSE_PICK['z'] + 0.035
                            print(f"📐 两阶段质心补偿: centroid_z={refine_centroid_z:.3f}m "
                                  f"→ link6_z={POSE_PICK['z']:.3f}m")
                        else:
                            print(f"⚠️ 两阶段质心z无效({refine_centroid_z})≤0, 保留待机位 z "
                                  f"(D455 近距离失真), link6_z={POSE_PICK['z']:.3f}m")

                    # ── GraspNet 主导抓取 (sort_graspnet) ──
                    # 在待机位运行 GraspNet (D455 在 0.5-0.8m 深度可靠), 用 6DoF 位姿覆盖 POSE_PICK
                    if use_graspnet:
                        # a. 确保待机位 + 张开夹爪 (避免夹爪遮挡, 保证深度数据新鲜)
                        node.move_arm_joint(JOINT_STANDBY, "GraspNet: 回到待机位")
                        node.operate_gripper(dynamic_open, "GraspNet: 张开夹爪")
                        time.sleep(0.5)  # 等待待机位深度数据刷新

                        # b. 请求 GraspNet 服务 (pick_pose 作为邻近过滤提示)
                        result = node.call_graspnet_service(pick_pose, timeout_s=15.0)

                        # c. 失败: 报错跳过 (无回退, 与用户决策一致)
                        if not result.get("success"):
                            node.get_logger().info(f"❌ GraspNet 失败 ({result.get('error')}), 跳过本轮")
                            node.move_arm_joint(JOINT_STANDBY, "GraspNet失败→回待机位")
                            node.operate_gripper(GRIPPER_OPEN, "GraspNet失败→张开夹爪")
                            success = False; break

                        # d. 成功: 用 GraspNet base 系位姿覆盖 POSE_PICK / PICK_UP / LIFT_SOFT
                        gn_t = result["translation"]
                        gn_q = result["quaternion"]
                        gn_w = float(result.get("width", 0.06))
                        for p in (POSE_PICK, POSE_PICK_UP, POSE_LIFT_SOFT):
                            p['x'] = float(gn_t[0]); p['y'] = float(gn_t[1])
                        # GraspNet translation.z 是物体抓取点 base 系 z, 叠加 link6→夹持点偏置
                        POSE_PICK['z'] = max(float(gn_t[2]), MIN_GRASP_Z) + GRIPPER_PICK_Z_OFFSET
                        POSE_PICK_UP['z'] = POSE_PICK['z'] + 0.13
                        POSE_LIFT_SOFT['z'] = POSE_PICK['z'] + 0.035
                        # GraspNet 6DoF 姿态 (机器人约定: R[:,0]=opening, R[:,1]=ortho, R[:,2]=approach)
                        graspnet_quat = [float(gn_q[0]), float(gn_q[1]), float(gn_q[2]), float(gn_q[3])]
                        # 用 GraspNet 估计宽度重算夹爪开合目标
                        dynamic_open, dynamic_close = compute_gripper_targets(gn_w)
                        node.get_logger().info(
                            f"🎯 GraspNet 位姿: t=({gn_t[0]:.3f},{gn_t[1]:.3f},{gn_t[2]:.3f}) "
                            f"score={result.get('score',0):.3f} width={gn_w:.3f} "
                            f"(候选{result.get('n_candidates','?')}个) → link6_z={POSE_PICK['z']:.3f}m")

                    # 【第零步】 在前往途中或起点提前张开夹爪
                    node.operate_gripper(dynamic_open, "张开夹爪(准备抓取)")
                    if node._check_emergency("step0-张开夹爪"):
                        success = False; break

                    # ── 姿态预规划: 在第一步(移动到抓取点上方)之前计算好目标姿态 ──
                    # 确保第一步到位时 link6 已对齐, 第二步下降时无需旋转 → joint6 零旋转
                    if use_graspnet:
                        active_ori = graspnet_quat
                        node.get_logger().info("📐 姿态预规划: GraspNet 6DoF 姿态")
                    else:
                        pick_fallback_ori = node._build_pick_orientation(POSE_PICK)[0]
                        pca_ori = None
                        if getattr(node, '_latest_detection', None):
                            pca_ori = node._latest_detection.get('grasp_orientation')
                        if pca_ori is not None and len(pca_ori) == 4:
                            active_ori = pca_ori
                            node.get_logger().info("📐 姿态预规划: PCA 短轴对齐姿态")
                        else:
                            active_ori = getattr(node, 'last_successful_orientation', pick_fallback_ori)
                            node.get_logger().info("📐 姿态预规划: 径向朝下姿态")

                    # 【第一步】 抓取过渡(防止碰桌面) — IK 优先, 传入预规划姿态
                    if two_stage:
                        node.get_logger().info("⚡ 两阶段模式：已在物体正上方，直接下降抓取")
                    elif not node.move_arm_pose(POSE_PICK_UP, "抓取位上方过渡点",
                                                  continuous=True, planning_mode='normal',
                                                  preferred_orientation=active_ori):
                        success = False; break
                    if node.last_planning_strategy:
                        cycle_strategies.append(node.last_planning_strategy)
                    if node.last_planning_profile_name:
                        cycle_profiles.append(node.last_planning_profile_name)

                    # 记录抓取过渡点的关节角，用于百分百安全抬起
                    pick_up_joints = node.current_joints.copy() if node.current_joints else None
                    if node._check_emergency("step1-上方过渡点"):
                        success = False; break

                    # ⚠️ 下降前禁用所有传送带碰撞体: z 补偿降低后夹爪手指(gripper_link1/2)会穿透
                    # belt_deck 台面和 conveyor_belt_col 侧面碰撞模型，导致抬起时起点被判碰撞 → -2。
                    # 日志确认两种碰撞: belt_deck↔gripper_link1, conveyor_belt_col↔gripper_link1。
                    # 物理上物体在台面上方 7cm，夹爪不会真撞台面，仅碰撞模型偏保守。
                    # 恢复在回待机位之后(下方错误处理/成功路径)。
                    node._set_bin_collision_allowed(allowed=True)

                    # ═══════ 抓取重试循环 (最多 MAX_GRASP_RETRIES 次) ═══════
                    grasp_ok = False
                    gw = gf = 0.0
                    reason = ""
                    for retry in range(MAX_GRASP_RETRIES):
                        if retry > 0:
                            # ── 重试前准备: 回观察位 → 视觉刷新 → 更新坐标 ──
                            node.get_logger().warn(
                                f"🔄 抓取重试 {retry+1}/{MAX_GRASP_RETRIES}...")
                            # 张开夹爪 (释放可能夹住的物体)
                            node.operate_gripper(GRIPPER_OPEN, f"重试{retry+1}-张开夹爪")
                            time.sleep(0.2)
                            # 移动到观察位 (相机视野无遮挡, 获取清晰视觉数据)
                            node.get_logger().info("📡 重试: 回观察位获取清晰视觉...")
                            node.move_arm_joint(JOINT_OBSERVE, f"重试{retry+1}-回观察位")
                            time.sleep(1.0)  # 等待视觉数据刷新 (YOLO ~10Hz, 1s=10帧)
                            # 获取视觉系统最新坐标
                            fresh_pose = node._get_fresh_detection(pick_id, timeout_s=2.0)
                            if fresh_pose:
                                old_x, old_y = POSE_PICK['x'], POSE_PICK['y']
                                for p in (POSE_PICK, POSE_PICK_UP, POSE_LIFT_SOFT):
                                    p['x'] = fresh_pose['x']
                                    p['y'] = fresh_pose['y']
                                node.get_logger().info(
                                    f"📡 重试: 观察位坐标 "
                                    f"({old_x:.3f},{old_y:.3f}) → "
                                    f"({fresh_pose['x']:.3f},{fresh_pose['y']:.3f})")
                            else:
                                node.get_logger().warn(
                                    "⚠️ 重试: 未获取到新坐标, 使用原始坐标继续")
                            # 重新计算姿态 (坐标变了, 径向朝下方向也变了)
                            pick_fallback_ori = node._build_pick_orientation(POSE_PICK)[0]
                            active_ori = getattr(node, 'last_successful_orientation', pick_fallback_ori)
                            node.get_logger().info("📐 重试: 姿态已按新坐标重新计算")
                            # 从观察位移到新 POSE_PICK_UP (重走第一步)
                            node.get_logger().info("📡 重试: 移动到新坐标上方...")
                            if not node.move_arm_pose(POSE_PICK_UP, f"重试{retry+1}-上方过渡点",
                                                      continuous=False, planning_mode='normal',
                                                      preferred_orientation=active_ori):
                                node.get_logger().warn("⚠️ 重试: 上方过渡点移动失败, 跳过本次重试")
                                continue
                            # 记录新过渡点关节角 + 禁用碰撞
                            pick_up_joints = (node.current_joints.copy()
                                              if node.current_joints else None)
                            node._set_bin_collision_allowed(allowed=True)
                            node.get_logger().info("📡 重试: 已到达新坐标上方, 准备下降")
                            if node._check_emergency(f"retry{retry+1}-上方过渡点"):
                                success = False; break

                        # 【第二步】 下降抓取 — IK 关节空间优先 (固定 joint6), 笛卡尔兜底
                        # active_ori 已在姿态预规划中计算, IK 直接求解保证 joint6 不旋转
                        descent_success = False
                        if node.enable_ik and node.ik_solver is not None:
                            pick_orientations = [active_ori] + node._build_pick_orientations_multi(POSE_PICK)
                            ik_ok, ik_joints = node.move_arm_via_ik(POSE_PICK, pick_orientations, "下降抓取 (IK关节空间)", continuous=True)
                            if ik_ok:
                                descent_success = True
                                cycle_strategies.append('ik_joint_space')
                                cycle_profiles.append('ik_solution')
                                node.get_logger().info("✅ IK 关节空间下降完成 (joint6 固定)")

                        if not descent_success:
                            # IK 失败, 回退到笛卡尔路径 (可能有 joint6 旋转)
                            node.get_logger().info("🟡 IK失败, 回退到笛卡尔直线插补...")
                            pose_pick_msg = node._create_pose(POSE_PICK, active_ori)
                            if node.execute_cartesian_path([pose_pick_msg], f"下降抓取 (直线插补,retry{retry+1})", fraction_threshold=0.50):
                                descent_success = True

                        if not descent_success:
                            node.get_logger().info("🟡 直线插补+IK均受限！启动多路并发退避规划...")
                            if not node.move_arm_cartesian(POSE_PICK, f"下降抓取 (退避规划,retry{retry+1})", continuous=True, preferred_orientation=active_ori, allow_position_only_fallback=True, planning_mode='descend'):
                                continue  # 下降失败跳过本次重试
                            if node.last_planning_strategy:
                                cycle_strategies.append(node.last_planning_strategy)
                            if node.last_planning_profile_name:
                                cycle_profiles.append(node.last_planning_profile_name)

                        if node._check_emergency(f"step2-下降抓取,retry{retry+1}"):
                            success = False; break

                        # 【第三步】 闭合夹爪 — 目标完全闭合 (0mm)
                        node._pre_grasp_width = (
                            node._latest_gripper_status.width
                            if node._latest_gripper_status is not None else None
                        )
                        node.operate_gripper(GRIPPER_GRAB, "闭合夹爪(拿取,完全闭合)")
                        time.sleep(GRIPPER_SETTLE_SEC)
                        time.sleep(0.05)  # planning_scene 更新
                        if node._check_emergency(f"step3-闭合夹爪,retry{retry+1}"):
                            success = False; break

                        # 【第四步】 抬起脱离 → 回到夹取位上方 (途径点, 不停留)
                        _lift_ok = False
                        if pick_up_joints:
                            sorted_joints = [pick_up_joints[f'joint{i}'] for i in range(1, 7)]
                            if node.move_arm_joint(sorted_joints, "抬起脱离→夹取上方 (途径点)", continuous=True):
                                _lift_ok = True
                            else:
                                node.get_logger().warn("⚠️ 抬起(关节空间)失败(-2), 回退到笛卡尔规划")
                        if not _lift_ok:
                            POSE_LIFT = POSE_PICK.copy()
                            POSE_LIFT['z'] = POSE_PICK_UP['z']
                            _lift_pose_msg = node._create_pose(POSE_LIFT, active_ori)
                            if node.execute_cartesian_path([_lift_pose_msg], "抬起脱离 (直线插补)", fraction_threshold=0.50):
                                _lift_ok = True
                            elif not node.move_arm_cartesian(POSE_LIFT, "抬起脱离 (退避规划)", continuous=True, preferred_orientation=active_ori, allow_position_only_fallback=True, planning_mode='retreat'):
                                continue  # 抬起失败跳过本次重试
                            if node.last_planning_strategy:
                                cycle_strategies.append(node.last_planning_strategy)
                            if node.last_planning_profile_name:
                                cycle_profiles.append(node.last_planning_profile_name)
                        grasp_reached_ok = True
                        if node._check_emergency(f"step4-抬起→夹取上方,retry{retry+1}"):
                            success = False; break

                        # 【第四步半】 抓取成功检测 — 抬起后立即评估
                        # 夹爪在 step3 已闭合, 经 GRIPPER_SETTLE(0.05s) + lift(~1.5s) 已稳定 >1.5s,
                        # check_grasp_success 的 0.3s 超时在第一轮轮询即返回 (width 已稳定).
                        grasp_ok, gw, gf, reason = node.check_grasp_success(timeout_s=0.3)
                        node.get_logger().info(
                            f"🔍 抓取检测 (retry {retry+1}/{MAX_GRASP_RETRIES}): "
                            f"success={grasp_ok} width={gw*1000:.1f}mm "
                            f"force={gf:.2f}N reason={reason}")
                        node._publish_grasp_status(grasp_ok, gw, gf, reason)
                        if node._check_emergency(f"step4.5-抓取检测,retry{retry+1}"):
                            success = False; break

                        if grasp_ok:
                            node.get_logger().info(
                                f"✅ 抓取成功 (retry {retry+1}/{MAX_GRASP_RETRIES})")
                            break  # 成功! 跳出重试循环

                        if retry < MAX_GRASP_RETRIES - 1:
                            node.get_logger().warn(
                                f"⚠️ 抓取失败 (retry {retry+1}/{MAX_GRASP_RETRIES}, "
                                f"reason={reason}), 准备重试...")
                            continue

                    # ═══════ 重试循环结束 ═══════

                    if not grasp_ok:
                        node.get_logger().error(
                            f"❌ {MAX_GRASP_RETRIES}次抓取全部失败 (最后reason={reason}), "
                            f"跳过本轮")
                        node.operate_gripper(GRIPPER_OPEN, "重试耗尽-张开夹爪")
                        node.move_arm_joint(JOINT_STANDBY, "重试耗尽-回待机位")
                        if node._check_emergency("retry_exhausted-回待机位"):
                            pass  # 已安全停机
                        success = False; break

                    # 【第五步】 关节空间到料框上方 (需稳定后再下降)
                    # ⚠️ continuous=False: BIN_ABOVE完成后等0.15s稳定, 否则BIN_PLACE起点偏差→-4
                    node._set_bin_collision_allowed(allowed=True)
                    if not node.move_arm_joint(BIN_ABOVE_JOINTS, f"移动到{place_id}上方 (途径点)", continuous=False):
                        success = False; break
                    if node._check_emergency("step5-料框上方"):
                        success = False; break

                    # 【第六步】 关节空间到料框放置位
                    if not node.move_arm_joint(BIN_PLACE_JOINTS, f"移动到{place_id}放置位 (关节空间)", continuous=True):
                        success = False; break
                    place_reached_ok = True
                    if node._check_emergency("step6-料框放置位"):
                        success = False; break

                    # 【第七步】 松开夹爪释放物品
                    node.operate_gripper(dynamic_open, f"松开夹爪(在{place_id}放置位释放)")
                    time.sleep(GRIPPER_SETTLE_SEC)
                    if node._check_emergency("step7-松开夹爪"):
                        success = False; break

                    # 【第八步】 关节空间回到料框上方 (抬起脱离放置位, 途径点)
                    if not node.move_arm_joint(BIN_ABOVE_JOINTS, f"从{place_id}放置位抬起 (途径点)", continuous=True):
                        success = False; break
                    if node._check_emergency("step8-料框抬起"):
                        success = False; break

                    # 【第九步】 关节空间回到待机位 (等待下一次抓取)
                    if not node.move_arm_joint(JOINT_STANDBY, f"回到待机位 (关节空间)", continuous=True):
                        success = False; break
                    if node._check_emergency("step9-回待机位"):
                        success = False; break

                # ⚠️ 碰撞检测恢复已下移至"回待机位之后"。
                # 原来在此处恢复会导致: 若抓取/抬起失败时机械臂仍在低位，
                # 恢复后下方错误处理中的"回待机位"也会因起点碰撞(belt_deck↔gripper_link1)而失败(-2)。

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
                    # ── 急停锁定: 跳过自动回待机位, 等待复位 ──
                    if node._emergency_flag and node._emergency_req_type == "estop":
                        node.get_logger().error(
                            "⛔ 急停锁定中, 跳过自动退回. 按【复位】按钮回待机位.")
                        status_msg.data = 'estop'
                        node.status_pub.publish(status_msg)
                        # 等待复位信号 (非阻塞轮询, 允许其他 topic 回调)
                        node.get_logger().info("⏳ 等待复位信号...")
                        while rclpy.ok() and node._emergency_flag:
                            rclpy.spin_once(node, timeout_sec=0.2)
                            if node._emergency_req_type == "reset":
                                node._check_emergency("error_handler-reset")
                                break
                        continue

                    node.get_logger().info("⚠️ 执行失败 (错误码可能为 99999 或其他)！启动自动退回防护...")
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
                    node.move_arm_joint(JOINT_STANDBY, "自动回到待机位")

                    # ── 发布机械臂状态 (失败) ──
                    conf = 0.0
                    latest_det = getattr(node, '_latest_detection', None)
                    if latest_det:
                        conf = float(latest_det.get('confidence', 0.0))
                        det_name = latest_det.get('object_name', obj_class)
                        cargo_name = det_name.split(' (')[0].strip().lower()
                    else:
                        cargo_name = obj_class

                    node._publish_robot_status(
                        success=False,
                        cargo_type=cargo_name,
                        confidence=conf,
                        source_pos_dict={"x": float(POSE_PICK['x']), "y": float(POSE_PICK['y']), "z": float(POSE_PICK['z'])},
                        target_joints=BIN_PLACE_JOINTS,
                        grasp_ok=False,
                        error_code=99,
                    )

                    # 安全回到待机位后恢复碰撞检测 (无论回待机位是否成功都恢复，避免影响下一轮)
                    node._set_bin_collision_allowed(allowed=False)
                    node.operate_gripper(GRIPPER_OPEN, "张开夹爪(待机)")
                    
                    time.sleep(0.5)
                    continue

                # 成功完成分拣后恢复碰撞检测
                node._set_bin_collision_allowed(allowed=False)

                # ── 发布机械臂状态 (成功) ──
                conf = 0.0
                latest_det = getattr(node, '_latest_detection', None)
                if latest_det:
                    conf = float(latest_det.get('confidence', 0.0))
                    det_name = latest_det.get('object_name', obj_class)
                    cargo_name = det_name.split(' (')[0].strip()
                else:
                    cargo_name = obj_class

                node._publish_robot_status(
                    success=True,
                    cargo_type=cargo_name,
                    confidence=conf,
                    source_pos_dict={"x": float(POSE_PICK['x']), "y": float(POSE_PICK['y']), "z": float(POSE_PICK['z'])},
                    target_joints=BIN_PLACE_JOINTS,
                    grasp_ok=True,
                    error_code=0,
                )

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

            elif data.get("cmd") == "sort_blue_block":
                # 🟦 蓝方块分拣 (Task 2.1): 顶面中位抓取 + 分层姿态约束
                # 命令 JSON: {cmd, pick, surface_z_m, block_width_m, block_length_m,
                #             block_height_m, block_rotation_deg}
                node.is_busy = True
                status_msg.data = 'busy'
                node.status_pub.publish(status_msg)
                loop_count += 1
                blue_block_config = {
                    'JOINT_STANDBY': JOINT_STANDBY,
                    'JOINT_OBSERVE': JOINT_OBSERVE,
                    'BIN_JOINTS': BIN_JOINTS,
                    'GRIPPER_OPEN': GRIPPER_OPEN,
                    'GRIPPER_SETTLE_SEC': GRIPPER_SETTLE_SEC,
                    'GRIPPER_PICK_Z_OFFSET': GRIPPER_PICK_Z_OFFSET,
                    'MIN_GRASP_Z': MIN_GRASP_Z,
                    'loop_count': loop_count,
                }
                node._execute_blue_block_sort(data, blue_block_config)

    except KeyboardInterrupt:
        print("\n⏹️ 收到 Ctrl+C，正在复位机械臂...")
        try:
            node.move_arm_joint(JOINT_STANDBY, "退出前回到待机位")
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