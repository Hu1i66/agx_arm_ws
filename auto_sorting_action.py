
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
import uuid
import runtime_config as runtime_config
from runtime_geometry import assert_visual_target_fresh, tcp_transform, tcp_to_flange, transform, assert_robot_tcp, short_axis_tool_rotation
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from rcl_interfaces.srv import GetParameters
from rclpy.logging import LoggingSeverity, set_logger_level

from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPlanningScene, GetPositionIK, GetStateValidity
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
_ws_root = os.path.dirname(os.path.abspath(__file__))
if _ws_root not in sys.path:
    sys.path.insert(0, _ws_root)

import numpy as np
import traceback
from typing import Tuple, Optional


# ---- Pinocchio IKSolver (内嵌副本, 独立节点 pinocchio_ik_node.py 已移除) ----

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
        self.tcp_transform = tcp_transform()

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
            target_tcp = transform(target_pos_arr,tq)
            flange = tcp_to_flange(target_tcp,self.tcp_transform)
            target_pos_arr = flange[:3,3]
            target_se3 = _pin.SE3(flange[:3,:3],flange[:3,3])
            target_z = target_tcp[:3, 2].copy()  # Desired EE z-axis in world frame

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
                actual_flange = self.data.oMf[self.ee_frame_id]
                actual_tcp = np.eye(4)
                actual_tcp[:3,:3] = actual_flange.rotation @ self.tcp_transform[:3,:3]
                actual_tcp[:3,3] = actual_flange.translation + actual_flange.rotation @ self.tcp_transform[:3,3]
                pe = float(np.linalg.norm(actual_tcp[:3,3] - target_tcp[:3,3]))
                R_act = actual_tcp[:3,:3]
                actual_z = R_act[:, 2]
                cos_angle = float(np.clip(np.dot(actual_z, target_z), -1.0, 1.0))
                ze = float(np.arccos(cos_angle))  # 0=aligned, pi=opposite
                actual_x = R_act[:, 0]
                target_x = target_tcp[:3, 0]
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



from workbench.action_control import WorkbenchControl

class MoveItActionClient(WorkbenchControl, Node):
    def __init__(self):
        super().__init__('piper_moveit_action_client')
        self._suppress_tf_old_data_logs()
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self._cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self._plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self._execute_action_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        
        self._gripper_action_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        # 手臂关节轨迹控制器 (方案 X): 用于"多航点关节轨迹一次下发", 不经过 MoveIt 单目标规划.
        # 日志4 证明 GetCartesianPath 笛卡尔直线在下降区 0.0% 失败, 而关节空间逐航点 Pinocchio IK
        # 100% 可达 (offline_verify_jointspace_multiwaypoint.py). 故下降改为: 逐航点 IK 解关节角
        # → 打包成整条 JointTrajectory → 一次下发 arm_controller, 既消除笛卡尔奇异性又消分段死区.
        self._arm_jt_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self._valid_state_client = self.create_client(GetStateValidity,'/check_state_validity')
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
        self._pause_after_cycle = False
        self._workbench_recover = False
        self._workbench_sub = self.create_subscription(String, '/workbench/control', self._workbench_control, 10)
        self.status_pub = self.create_publisher(String, '/sorting_status', 10)
        self.cycle_result_pub = self.create_publisher(String, '/sorting/cycle_result', 10)
        self.robot_status_pub = self.create_publisher(String, '/sorting/robot_status', 10)
        self.is_busy = False
        self.last_planning_profile_name = ''
        self.last_planning_strategy = ''

        # ── 两阶段精定位：订阅 YOLO 检测结果 ──
        self._latest_detection = None
        self._detection_event = threading.Event()
        self._detections_by_camera = {}
        self.det_sub = self.create_subscription(String, '/detection_info', lambda m:self._detection_cb(m, 'd455'), 10)
        self.dabai_det_sub = self.create_subscription(String, '/dabai/detection_info', lambda m:self._detection_cb(m, 'dabai'), 10)

        # ── 动态抓取: 订阅传送带状态 (速度/方向/物体检测) ──
        self._belt_speed = 0.0
        self._belt_direction = 1
        self._dynamic_tracker = None
        self._dynamic_actual_joints = None
        self._dynamic_actual_at = -100.
        self._dynamic_feedback_sub = self.create_subscription(
            JointState, "/feedback/joint_states", self._dynamic_feedback_cb, 10)
        self._dynamic_calib = None
        self._conveyor_status_sub = self.create_subscription(
            String, '/conveyor_status', self._conveyor_status_cb, 10)
        self._load_dynamic_calib()

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
        self._workbench_emergency = False
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
        # 优先使用内嵌 Pinocchio IK 求解, 失败回退到 MoveIt2 (TRAC_IKKinematicsPlugin)
        self.enable_ik = True
        self.ik_solver = None
        self._init_ik_solver()

        # ── TF2 监听器 ──
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _gripper_status_cb(self, msg):
        """缓存夹爪状态反馈 (width, force, 安全标志)."""
        self._latest_gripper_status = msg
        self._dynamic_gripper_at = time.monotonic()

    def _emergency_cb(self, msg):
        """急停/返回待机位/复位回调 — 设置中断标志位, 分拣序列中检查点检测."""
        try:
            data = json.loads(msg.data)
            req_type = data.get("req", "estop")
            self._workbench_emergency = bool(data.get("workbench"))
            self._workbench_idle_emergency_handled = False
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
        if self.workbench_interrupted():
            return True  # recovery cancels only: no release or home motion
        if not self._emergency_flag and self._emergency_req_type != "reset":
            return False

        req = self._emergency_req_type or "estop"

        # ── 复位: 急停锁定后用户按下复位按钮触发 ──
        if req == "reset":
            self.get_logger().info(
                f"🔄 执行复位: 回待机位 step={step_name} "
                f"time={time.strftime('%Y-%m-%dT%H:%M:%S')}")
            try:
                self.move_arm_joint(self.placement_snapshot("", "d455")["normal_return_joints_rad"] if self._workbench_emergency else [0.0]*6, "复位-回待机位")
                self._set_conveyor_collision_allowed(allowed=False)
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
                self.move_arm_joint(self.placement_snapshot("", "d455")["normal_return_joints_rad"] if self._workbench_emergency else [0.0]*6, "急停-回待机位")
                self._set_conveyor_collision_allowed(allowed=False)
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

    def _conveyor_status_cb(self, msg):
        """缓存传送带状态 (STM32 滤波后速度), 供动态抓取预测."""
        try:
            d = json.loads(msg.data)
            if d.get('type') == 'conveyor_status':
                self._belt_speed = float(d.get('speed', 0.0))
                self._belt_direction = int(d.get('motor_direction', 1))
                self._belt_status_at = time.monotonic()-max(0., float(d.get('status_age_s', 0.)))
                self._belt_connected = bool(d.get('connected', True))
                self._belt_running = bool(d.get('motor_running', True))
                self._belt_commanded_running = d.get('commanded_running')
        except Exception:
            pass

    def _load_dynamic_calib(self):
        """加载传送带方向标定；不应用历史速度比例。"""
        import os
        path = runtime_config.resolve_resource('conveyor_calib')
        if not os.path.exists(path):
            self.get_logger().warn(f"未找到传送带方向文件 {path}，动态抓取暂不可用；静止抓取不受影响")
            return
        try:
            with open(path, 'r') as f:
                calib = json.load(f)
            self._dynamic_calib = calib
            self.get_logger().info(
                f"✅ 传送带方向标定已加载: d̂=({calib['dx']:.4f},{calib['dy']:.4f}) "
                f"angle={calib.get('angle_rad', 0.0):.3f}rad")
        except Exception as e:
            self.get_logger().warn(f"⚠️ 传送带标定加载失败: {e}")

    def _dynamic_settings(self):
        """动态抓取参数（缓存），与 dynamic_grasp_runtime 读同一份配置。"""
        if getattr(self, '_workbench_task', False) or getattr(self, '_dyn_cfg_cache', None) is None:
            root = runtime_config.workspace_root()
            self._dyn_cfg_cache = json.loads((root/'config/dynamic_grasp.json').read_text())
        return self._dyn_cfg_cache

    def _detection_cb(self, msg, camera="d455"):
        """缓存 YOLO 检测结果，唤醒等待线程。"""
        try:
            data = json.loads(msg.data)
            self.cache_detection(camera, data)
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
                        name = (obj.get('object_name') or obj.get('name') or '').lower().strip()
                        if name and (target_key in name or name in target_key):
                            bp = obj.get('base_position_m')
                            if bp and all(k in bp for k in ('x', 'y', 'z')):
                                self._detection_event.clear()
                                self.get_logger().info(
                                    f"📡 重试: 获取到 {name} 新坐标 "
                                    f"x={bp['x']:.3f} y={bp['y']:.3f} z={bp['z']:.3f}")
                                return {'x': float(bp['x']),
                                        'y': float(bp['y']),
                                        'z': float(bp['z']), 'grasp_geometry': obj.get('grasp_geometry')}
                    # 检测到了但没匹配到目标物体, 清事件继续等
                    self._detection_event.clear()
            time.sleep(0.1)

        self.get_logger().warn(f"⏰ 重试: {timeout_s}s 内未获取到 {target_name} 新坐标")
        return None

    def _dynamic_feedback_cb(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if stamp > 0 and stamp <= getattr(self, '_dynamic_actual_stamp', 0.):
            return  # SDK旧状态重复发布不能刷新真实反馈新鲜度。
        values = dict(zip(msg.name, msg.position))
        names = [f'joint{i}' for i in range(1, 7)]
        if all(name in values for name in names):
            joints = np.array([values[name] for name in names], dtype=float)
            if np.isfinite(joints).all():
                self._dynamic_actual_joints = joints.tolist()
                self._dynamic_actual_at = time.monotonic()
                self._dynamic_actual_stamp = stamp

    def _init_ik_solver(self):
        """初始化内嵌 Pinocchio IK 求解器（无需 ROS2 topic 通信）"""
        try:
            _try_load_pinocchio()
            if not _PINOCCHIO_AVAILABLE or not _CASADI_AVAILABLE:
                self.get_logger().warn("Pinocchio or CasADi not available, IK disabled")
                self.enable_ik = False
                return

            urdf_path = str(runtime_config.resolve_resource('ik_urdf'))
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

    def _set_conveyor_collision_allowed(self, allowed=True, obj_ids=None):
        """恢复指定碰撞体检测；标定抓取不允许绕过碰撞场景。

        通过 /planning_scene 话题发布 AllowedCollisionMatrix (ACM) diff 实现。
        放置目标点规划失败时调用此方法临时绕过传送带碰撞体，
        让 RRTConnect 能在原本"碰撞"的目标状态采样成功。

        ⚠️ 关键：必须先调用 /get_planning_scene 获取当前 ACM，只修改 obj_ids 相关条目，
        保留所有原有碰撞对（包括夹爪自碰撞对 gripper_base-gripper_link1 等）。
        否则 ACM diff 会替换整个 ACM，导致夹爪自碰撞检测被启用，所有规划都失败。

        ⚠️ 放置目标点附近有多个碰撞体（move_group 日志确认）：
        - 'control_box' (Object, conveyor.scene) 与 link5 碰撞
        - 'rail_ny' (Object, conveyor.scene) 与 gripper_link2 碰撞
        - 'conveyor_control_box' (Robot link, conveyor_cell.urdf.xacro) 与 gripper_link1 碰撞
        因此需要同时禁用所有传送带碰撞体。

        Args:
            allowed: True=禁用碰撞检测(允许碰撞), False=恢复碰撞检测
            obj_ids: 要操作的碰撞体名称列表，默认包含所有传送带碰撞体
        """
        if allowed:
            self.get_logger().warning('标定抓取保留传送带碰撞检测，不绕过失败规划')
            return
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
        try:
            command=json.loads(msg.data);camera=command.get('camera_id','d455')
            self.validate_workbench_command(command)
            if command.get("workbench") and command.get("start_intent"):
                self._pause_after_cycle=False
            assert_visual_target_fresh(command)
            if camera not in ('d455','dabai'):raise ValueError('未知抓取相机')
            if camera=='dabai' and command.get('cmd')=='sort_dynamic':raise ValueError('Dabai首轮仅支持静态抓取')
        except (ValueError,TypeError) as exc:
            self.get_logger().error(str(exc));self.workbench_ack(locals().get('command',{}),False,str(exc));return
        self.cmd_queue.put(msg.data);self.workbench_ack(command,True)
        
    def _check_calibrated_tool(self):
        if time.monotonic()-getattr(self,'_tool_check_at',-100.)>2.:
            assert_robot_tcp(self)
            self._tool_check_at=time.monotonic()

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
        if self.workbench_interrupted():return False
        self._check_calibrated_tool()
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
        goal_msg.request.max_velocity_scaling_factor = getattr(self,"_static_cycle_settings",{}).get("joint_velocity_scaling",0.65)
        goal_msg.request.max_acceleration_scaling_factor = getattr(self,"_static_cycle_settings",{}).get("joint_acceleration_scaling",0.65)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        goal_msg.planning_options.plan_only = plan_only
        
        self.get_logger().info(f'发送规划目标...')
        # print("正在请求...")
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        if not self._wait_control_future(send_goal_future):return False
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('❌ 规划目标被拒绝！(不可达或干涉)')
            return False

        self.get_logger().info('⭕ 目标被接受，正在执行...')
        self._active_goal_handle = goal_handle
        if self.workbench_interrupted():
            goal_handle.cancel_goal_async();return False
        get_result_future = goal_handle.get_result_async()
        if not self._wait_control_future(get_result_future):return False
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
        ver_centroid_z = ver_z  # 使用点云估计的三维中心，不加类别尺寸/经验高度
        if ver_centroid_z is None:
            ver_centroid_z = ver_z
        else:
            ver_centroid_z = float(ver_centroid_z)

        print(f"✅ [两阶段] 二次检测成功! 坐标=({ver_x:.4f},{ver_y:.4f},{ver_z:.4f}) 质心z={ver_centroid_z:.4f}")
        return True, {'x': ver_x, 'y': ver_y, 'z': ver_z, 'centroid_z': ver_centroid_z,
                      'grasp_geometry': detection.get('grasp_geometry')}

    # ── GraspNet 服务通信 ──
    def _graspnet_result_cb(self, msg):
        try:result=json.loads(msg.data)
        except (ValueError,TypeError):return
        pending=getattr(self,'_pending_grasp_request',None)
        if not pending or result.get('request_id')!=pending['request_id'] or result.get('camera_id')!=pending['camera_id']:return
        self._latest_graspnet_result=result;self._graspnet_event.set()

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
            "request_id": uuid.uuid4().hex,
            "camera_id": getattr(self,"_active_camera","d455"),
            "cmd": "get_grasp",
            "pick_semantics": "estimated_centroid",
            "pick_x": float(pick_pose['x']),
            "pick_y": float(pick_pose['y']),
            "pick_z": float(pick_pose['z']),
        }
        req['target_track_id'] = getattr(self,'_visual_target_meta',{}).get('target_track_id')
        self._pending_grasp_request = req
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
            if self._emergency_flag:
                self._pending_grasp_request = None
                return {"success":False,"error":"interrupted"}
            if self._graspnet_event.is_set():
                break
            time.sleep(dt)
            waited += dt

        self._pending_grasp_request = None
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

    def _joint_waypoints_valid(self, joint_waypoints, start_joints=None, report_failure=True):
        """复用原有插值碰撞检查；仅计算，不发送运动。"""
        self._last_state_validity_failure = None
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        if not self._valid_state_client.wait_for_service(timeout_sec=2.):return False
        current=self.current_joints or {}
        if not all(n in current for n in joint_names):return False
        previous=np.array(start_joints if start_joints is not None else [current[n] for n in joint_names])
        for waypoint in joint_waypoints:
            q=np.asarray(waypoint,dtype=float)
            if q.shape!=(6,) or not np.isfinite(q).all():return False
            steps=max(1,int(np.ceil(np.max(np.abs(q-previous))/.03)))
            for alpha in np.linspace(0.,1.,steps+1):
                if self._emergency_flag:return False
                request=GetStateValidity.Request();request.group_name='arm';request.robot_state.is_diff=True
                mapping=dict(current);mapping.update(zip(joint_names,((1-alpha)*previous+alpha*q).tolist()))
                request.robot_state.joint_state.name=list(mapping);request.robot_state.joint_state.position=list(mapping.values())
                future=self._valid_state_client.call_async(request)
                rclpy.spin_until_future_complete(self,future,timeout_sec=2.)
                if not future.done() or future.exception() or not future.result().valid:
                    response = future.result() if future.done() and not future.exception() else None
                    self._last_state_validity_failure = dict(reason='状态检查未通过/超时',
                        contacts=[] if response is None else [dict(a=c.contact_body_1, b=c.contact_body_2, depth_m=c.depth)
                                                              for c in response.contacts])
                    if report_failure:
                        self.get_logger().error('直接关节轨迹碰撞/限位检查失败，未派发')
                    return False
            previous=q
        return True

    def execute_multiwaypoint_joint_trajectory(self, joint_waypoints, desc, per_point_dur=0.30):
        """方案 X: 多航点关节轨迹一次下发 (不经过 MoveIt 单目标规划).

        日志4 根因: GetCartesianPath 笛卡尔刚性直线在 Piper 悬停(z=0.32)->近桌面(z≈0.109)
        扩展位形下奇异性不可达, 完整度 0.0%-0.2% 全失败; 而关节空间逐航点 Pinocchio IK
        100% 可达 (offline_verify_jointspace_multiwaypoint.py 验证). 本函数把逐航点 IK 解出的
        关节角序列打包成整条 JointTrajectory, 一次下发 arm_controller 连续执行, 既消除
        笛卡尔奇异性, 又无"分段式每段间计算/启动死区"导致的落后.

        Args:
            joint_waypoints: list[[j1..j6], ...] 依次为下降轨迹各航点的关节角
            desc: 动作描述
            per_point_dur: 每段耗时 (s), 决定轨迹时间戳分配
        Returns:
            bool: 是否成功
        """
        if self.workbench_interrupted():return False
        self._check_calibrated_tool()
        if not joint_waypoints:
            return False
        if not self._arm_jt_client.wait_for_server(timeout_sec=1.5):
            self.get_logger().error("⚠️ /arm_controller/follow_joint_trajectory 服务不可用 (方案X)")
            return False

        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        if not self._joint_waypoints_valid(joint_waypoints):return False
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names
        # 逐点构造 JointTrajectoryPoint, 时间戳递增.
        # ⚠️ 不额外插入"起点点": FollowJointTrajectory 会从当前实际位姿平滑过渡到首点,
        # 机械臂自然从影子位姿 (即 vma 第0点附近) 起步, 无需重复起点 (重复反而产生零长段).
        # 抖动根因 (日志5): 相邻航点 IK 解在关节空间可能有较大跳变 (离线 Δq_max 0.8-1.4rad),
        # FollowJointTrajectory 线性推进放大末端抖动. 对相邻航点做关节空间线性插值
        # (每段中间插 SMOOTH_SUB 个子点), 平滑轨迹消除末端抖动. 关节空间线性插值安全
        # (凸组合, 不易越界), 且不改变起点终点.
        SMOOTH_SUB = 2          # 每段插入子点数
        smooth_points = []
        for i, q in enumerate(joint_waypoints):
            smooth_points.append([float(v) for v in q])
            if i < len(joint_waypoints) - 1:
                qn = joint_waypoints[i + 1]
                for s in range(1, SMOOTH_SUB + 1):
                    alpha = s / (SMOOTH_SUB + 1.0)
                    interp = [float(a + alpha * (b - a)) for a, b in zip(q, qn)]
                    smooth_points.append(interp)
        # 插值后按时间均匀分配 (总时长= len(waypoints_orig)*per_point_dur, 保持下降总耗时不变)
        total_dur = len(joint_waypoints) * per_point_dur
        n_pts = len(smooth_points)
        for k, q in enumerate(smooth_points):
            point = JointTrajectoryPoint()
            point.positions = q
            t = (k + 1) * (total_dur / n_pts)
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            goal_msg.trajectory.points.append(point)

        self.get_logger().info(f"🚀 连续关节轨迹一次下发 -> {desc} ({len(joint_waypoints)}航点→{n_pts}插入平滑点, 总量={total_dur:.2f}s)")
        send_goal_future = self._arm_jt_client.send_goal_async(goal_msg)
        if not self._wait_control_future(send_goal_future):return False
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("❌ 关节轨迹被拒绝执行")
            return False
        self._active_goal_handle = goal_handle
        if self.workbench_interrupted():
            goal_handle.cancel_goal_async();return False
        get_result_future = goal_handle.get_result_async()
        if not self._wait_control_future(get_result_future):return False
        result = get_result_future.result().result
        if result and result.error_code == 0:  # FollowJointTrajectory 成功返回 error_code 0
            self.get_logger().info(f"✅ 连续关节轨迹执行完毕: {desc}")
            # 直发成功后同步 current_joints 到轨迹终点: move_group 会忽略客户端 start_state,
            # 但 warm_q/种子/IK 起点依赖 current_joints; 同步避免后续起算点滞后.
            if joint_waypoints and self.current_joints is None:
                self.current_joints = {}
            if joint_waypoints and isinstance(self.current_joints, dict):
                end = joint_waypoints[-1]
                for i in range(6):
                    self.current_joints[f'joint{i + 1}'] = float(end[i])
            return True
        err = result.error_code if result else "None"
        self.get_logger().error(f"❌ 轨迹执行失败, 错误码 {err}")
        return False

    def _solve_single_ik(self, target_pos, ori_wp, warm_q, time_budget=None, err_accept=0.012):
        """实时解 1 个 IK (当前关节 warm start). 返回 (best_q or None, best_err).

        Args:
            time_budget: 求解时间上限 (s). None=不限 (调用方无延迟敏感). 超时立即返回当前最优.
            err_accept: 组合误差接受阈值 (越组位置误差 + 姿态误差). 抬起等需要"大概率解出"的
                       场景可放宽 (0.03/0.05), 降低失败率.
        """
        if self.ik_solver is None:
            return None, float('inf')
        if isinstance(ori_wp, Quaternion):
            q_quat = np.array([ori_wp.x, ori_wp.y, ori_wp.z, ori_wp.w])
        else:
            q_quat = np.array([float(ori_wp[0]), float(ori_wp[1]), float(ori_wp[2]), float(ori_wp[3])])
        seeds = []
        if warm_q is not None:
            seeds.append(np.asarray(warm_q, dtype=float))
        seeds.append(np.zeros(6))
        if warm_q is not None:
            for _ in range(3):
                seeds.append((np.asarray(warm_q) + np.random.uniform(-0.5, 0.5, 6)).astype(float))
        for _ in range(2):
            seeds.append(np.random.uniform(-np.pi, np.pi, 6).astype(float))
        best_q, best_err = None, float('inf')
        _t0 = time.time()
        for _sd in seeds:
            if time_budget is not None and time.time() - _t0 > time_budget:
                break
            ok, q_sol, err, _ct = self.ik_solver.get_ik_solution(
                target_pos, q_quat, initial_guess=_sd)
            if ok and err < best_err:
                best_err = err
                best_q = list(q_sol)
                if err < err_accept:
                    break
        return best_q, best_err

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
        pc.link_name = 'tcp_link'
        
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
            oc.link_name = 'tcp_link'
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
        if self.workbench_interrupted():return False
        self._check_calibrated_tool()
        self.get_logger().info(f"🚀 正在快速计算(笛卡尔直线连线) -> {desc}")
        if not self._cartesian_client.wait_for_service(timeout_sec=1.5):
            self.get_logger().error("⚠️ GetCartesianPath 服务不可用")
            return False

        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = 'arm'
        req.link_name = 'tcp_link'
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
            req.start_state.is_diff = True
            req.start_state.joint_state.name = names
            req.start_state.joint_state.position = positions
        req.waypoints = waypoints
        req.max_step = 0.010      # 10mm 一步 (之前5mm, 改为10mm减少路径点数量, 见2026-08-17分析)
        req.jump_threshold = jump_threshold  # j6 突变检测阈值 (默认 1.6, 下降时可降到 0.8)
        req.avoid_collisions = True
        fraction_threshold = 1.0
        
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
        if not self._wait_control_future(future_exec):return False
        goal_handle = future_exec.result()
        if not goal_handle.accepted:
            self.get_logger().error("轨迹被拒绝执行")
            return False
            
        self._active_goal_handle = goal_handle
        if self.workbench_interrupted():
            goal_handle.cancel_goal_async();return False
        res_future = goal_handle.get_result_async()
        if not self._wait_control_future(res_future):return False
        res_exec = res_future.result()
        if res_exec and res_exec.result.error_code.val == 1:
            self.get_logger().info(f"\x1b[32m✅ 直线插补执行完毕: {desc}\x1b[0m")
            return True
            
        self.get_logger().error(f"❌ 轨迹执行失败，错误码 {res_exec.result.error_code.val}")
        return False
    
    def _compute_ik_via_service(self, pose_dict, orientation, seed_joints=None, timeout_s=0.5):
        """
        通过 MoveIt2 /compute_ik 服务调用 TRAC-IK 求解 IK.
        
        Args:
            pose_dict: {'x', 'y', 'z'} 目标位置
            orientation: Quaternion 或 [qx,qy,qz,qw] 目标姿态
            seed_joints: [6] 关节角度种子 (可选, 默认当前关节)
            timeout_s: IK 超时 (秒)
            
        Returns:
            (success, [j1..j6]) 或 (False, None)
        """
        # ── 诊断1: 检查服务可用性 ──
        if not self._ik_client.service_is_ready():
            self.get_logger().info(
                f"🔍 DIAG: 等待 /compute_ik 服务... (timeout={timeout_s})")
            if not self._ik_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("⚠️ /compute_ik 服务不可用, 跳过 TRAC-IK 直接求解")
                return False, None
        
        # ── 诊断2: 构建目标姿态 ──
        if isinstance(orientation, Quaternion):
            q = orientation
        else:
            q = Quaternion(x=float(orientation[0]), y=float(orientation[1]),
                           z=float(orientation[2]), w=float(orientation[3]))
        target_str = f"目标=({pose_dict['x']:.3f},{pose_dict['y']:.3f},{pose_dict['z']:.3f}) " \
                     f"ori=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f})"
        self.get_logger().info(f"🔍 DIAG: TRAC-IK 求解开始, {target_str}, timeout={timeout_s}s")
        
        # ── 诊断3: 构建种子候选 ──
        # 策略: 显式种子 → 当前关节 → 待机位 → 随机偏移 → 全空间随机
        seed_candidates = []
        if seed_joints is not None:
            seed_candidates.append(('显式种子', list(seed_joints)))
        if self.current_joints:
            cur = [self.current_joints.get(f'joint{i}', 0.0) for i in range(1, 7)]
            seed_candidates.append(('当前关节', cur))
        # 待机位 [0,0,0,0,0,0] 作为通用种子
        standby = [0.0] * 6
        if not any(np.allclose(standby, s[1], atol=0.01) for s in seed_candidates):
            seed_candidates.append(('待机位', standby))
        # 当前关节大范围随机偏移 (±0.5rad)
        if self.current_joints:
            cur = [self.current_joints.get(f'joint{i}', 0.0) for i in range(1, 7)]
            np.random.seed(int(time.time() * 1000) % 10000)
            for _ in range(3):
                noisy = [c + np.random.uniform(-0.5, 0.5) for c in cur]
                seed_candidates.append(('大偏移', noisy))
        # 全空间均匀随机 (2个, 扩大搜索范围)
        np.random.seed(int(time.time() * 1000) % 10000 + 1)
        for _ in range(2):
            full_random = [np.random.uniform(-np.pi, np.pi) for _ in range(6)]
            seed_candidates.append(('全随机', full_random))
        
        # ── 诊断4: 遍历所有种子, 记录每个种子的结果 ──
        seed_debug = "; ".join(f"{n}=({','.join(f'{v:.2f}' for v in s)})" for n, s in seed_candidates)
        self.get_logger().info(f"🔍 DIAG: 种子候选 ({len(seed_candidates)}个): {seed_debug}")
        
        _t0 = self.get_clock().now().nanoseconds * 1e-9
        for seed_name, seed_pos in seed_candidates:
            req = GetPositionIK.Request()
            req.ik_request.group_name = 'arm'
            req.ik_request.ik_link_name = 'tcp_link'
            req.ik_request.pose_stamped.header.frame_id = 'base_link'
            req.ik_request.pose_stamped.pose.position.x = float(pose_dict['x'])
            req.ik_request.pose_stamped.pose.position.y = float(pose_dict['y'])
            req.ik_request.pose_stamped.pose.position.z = float(pose_dict['z'])
            req.ik_request.pose_stamped.pose.orientation = q
            req.ik_request.timeout.sec = int(timeout_s)
            req.ik_request.timeout.nanosec = int((timeout_s - int(timeout_s)) * 1e9)
            req.ik_request.avoid_collisions = True
            req.ik_request.robot_state.is_diff = True
            req.ik_request.robot_state.joint_state.name = [f'joint{i}' for i in range(1, 7)]
            req.ik_request.robot_state.joint_state.position = list(seed_pos)
            
            _t_seed = self.get_clock().now().nanoseconds * 1e-9
            future = self._ik_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s + 0.5)
            _t_seed_end = self.get_clock().now().nanoseconds * 1e-9
            seed_elapsed = (_t_seed_end - _t_seed) * 1000
            
            if not future.done():
                self.get_logger().info(
                    f"🔍 DIAG: 种子<{seed_name}> 服务超时 ({seed_elapsed:.0f}ms)")
                continue
            
            res = future.result()
            if res is None:
                self.get_logger().info(
                    f"🔍 DIAG: 种子<{seed_name}> 返回 None ({seed_elapsed:.0f}ms)")
                continue
                
            err_code = res.error_code.val
            if err_code == 1:
                # 提取 joint1-6
                jnames = res.solution.joint_state.name
                jpos = res.solution.joint_state.position
                q_sol = [0.0] * 6
                for i in range(6):
                    name = f'joint{i+1}'
                    for j, n in enumerate(jnames):
                        if n == name:
                            q_sol[i] = jpos[j]
                            break
                _t1 = self.get_clock().now().nanoseconds * 1e-9
                self.get_logger().info(
                    f"✅ TRAC-IK 求解成功 ({(_t1-_t0)*1000:.1f}ms, "
                    f"种子={seed_name}, error_code=1)")
                return True, q_sol
            else:
                self.get_logger().info(
                    f"🔍 DIAG: 种子<{seed_name}> 失败 (error_code={err_code}, "
                    f"{seed_elapsed:.0f}ms)")
        
        self.get_logger().info(
            f"🔴 TRAC-IK 求解失败 (尝试了 {len(seed_candidates)} 个种子, "
            f"总耗时 {(self.get_clock().now().nanoseconds*1e-9 - _t0)*1000:.0f}ms "
            f"for {target_str})")
        return False, None
    
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

        # 动态抓取代 IK 总时间预算: 超过则该点视为不可解并立即采用当前最优解
        # (若有), 防止 10姿态×8种子全扫耗时数十秒, 运行中的物体早已跑出工作空间.
        _ik_budget = 2.5  # 秒

        # ⚠️ 多种子策略 (RC-4 修复): Pinocchio 随机优化只用"当前关节角"单一初始猜测,
        # 在待机位(折叠态)与伸展+手腕朝下抓取构型属于不同吸引域, 70% 局部采样够不到,
        # 30% 全局采样在 6D 空间过于稀疏 → 频繁误报"无解".
        # 参照 _compute_ik_via_service 的 TRAC-IK 8 种子策略:
        #   当前关节 → 待机位 → 大偏移(±0.5rad) → 全空间随机, 逐种子求解取最优.
        # 所有姿态候选共享同一组种子, 遍历顺序: 先姿态, 后种子 (外层 ori 保持不变,
        # 内层并行尝试多种子, 保留第一个足够好的解以缩短耗时).
        seed_candidates = []
        if initial_guess is not None:
            seed_candidates.append(('当前关节', initial_guess.astype(float)))
        standby = np.zeros(6)
        seed_candidates.append(('待机位', standby.astype(float)))
        if initial_guess is not None:
            np.random.seed(int(time.time() * 1000) % 10000)
            for _ in range(3):
                noisy = initial_guess + np.random.uniform(-0.5, 0.5, 6)
                seed_candidates.append(('大偏移', noisy.astype(float)))
        np.random.seed(int(time.time() * 1000) % 10000 + 1)
        for _ in range(2):
            full_random = np.random.uniform(-np.pi, np.pi, 6)
            seed_candidates.append(('全随机', full_random.astype(float)))

        _t_ik0 = time.time()
        for ori in orientations:
            if time.time() - _t_ik0 > _ik_budget:
                self.get_logger().info(f"⏱️ IK 时间预算已达 ({_ik_budget:.1f}s), 采用当前最优 err={best_error:.4f}")
                break
            if isinstance(ori, Quaternion):
                qx, qy, qz, qw = ori.x, ori.y, ori.z, ori.w
            else:
                qx, qy, qz, qw = float(ori[0]), float(ori[1]), float(ori[2]), float(ori[3])

            target_quat = np.array([qx, qy, qz, qw])
            for seed_name, seed_val in seed_candidates:
                ok, q_sol, err, comp_t = self.ik_solver.get_ik_solution(
                    target_pos, target_quat, initial_guess=seed_val
                )
                if ok and err < best_error:
                    best_error = err
                    best_q = list(q_sol)
                    best_time = comp_t
                    # 提前接受阈值: 动态抓取对求解速度敏感, 组合误差 <0.010 已足够
                    # (对应位置误差<1cm 级 + 合理姿态), 避免为追求 0.001 全扫 8 种子
                    # 导致单次 IK 数秒~数十秒, 动态目标早跑出视野. 后续仍会留更多姿态
                    # 校正, 无需极致精确. (原 0.001 阈值对真实可达点几乎不触发, 每次都
                    # 全扫 10姿态×8种子 → 20s+, 动态抓取不可用, 见离线 IKM test.)
                    if err < 0.010:
                        break
            if best_error < 0.010:
                break

        if best_q is None:
            self.get_logger().info(f"⚠️ IK 无解 (尝试了 {len(orientations)} 种姿态 × {len(seed_candidates)} 种子)")
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

    def _fixed_tcp_fk(self, joints):
        solver = self.ik_solver
        q = _pin.neutral(solver.model)
        for index, value in zip(solver.joint_q_indices, joints):
            q[index] = float(value)
        _pin.forwardKinematics(solver.model, solver.data, q)
        _pin.updateFramePlacements(solver.model, solver.data)
        flange = solver.data.oMf[solver.ee_frame_id]
        pose = np.eye(4)
        pose[:3, :3] = flange.rotation
        pose[:3, 3] = flange.translation
        return pose @ solver.tcp_transform

    def _fixed_tcp_ik(self, pose, orientation, seed=None):
        """复用Pinocchio模型及SciPy求完整TCP位姿，不用随机误差折中倾斜夹爪。"""
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation
        solver = self.ik_solver
        target = transform([pose[k] for k in ('x', 'y', 'z')], orientation)
        indices = solver.joint_q_indices
        lower = solver.model.lowerPositionLimit[indices]
        upper = solver.model.upperPositionLimit[indices]

        def residual(joints):
            actual = self._fixed_tcp_fk(joints)
            rotation = Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).as_rotvec()
            return np.r_[actual[:3, 3] - target[:3, 3], 0.1 * rotation]

        seeds = ([seed] if seed is not None else []) + [[0., 1.8, -1.3, 0., 1.1, 0.], [0.] * 6]
        for initial in seeds:
            fit = least_squares(residual, np.clip(initial, lower + 1e-9, upper - 1e-9),
                                bounds=(lower, upper), max_nfev=200)
            error = residual(fit.x)
            # 分开判断位置与完整姿态，不能用“位置误差小”代替姿态准确。
            if np.linalg.norm(error[:3]) <= 0.002 and np.linalg.norm(error[3:]) <= 0.0035:
                return fit.x.tolist()
        return None

    def _fixed_tcp_waypoints(self, start, target, orientation, seed):
        """固定姿态直线路径；用于抓取前预解与实际接近。"""
        start = np.asarray(start, dtype=float)
        end = np.array([target[k] for k in ('x', 'y', 'z')], dtype=float)
        count = max(1, int(np.ceil(np.linalg.norm(end - start) / 0.003)))
        waypoints = [seed]
        for alpha in np.linspace(0., 1., count + 1)[1:]:
            if self._emergency_flag:
                return None
            position = dict(zip(('x', 'y', 'z'), (start + alpha * (end - start)).tolist()))
            seed = self._fixed_tcp_ik(position, orientation, seed)
            if seed is None:
                return None
            if np.max(np.abs(np.asarray(seed) - waypoints[-1])) > 0.15:
                return None
            waypoints.append(seed)
        return waypoints

    def _execute_fixed_tcp_descent(self, target, orientation, desc, extra_depth=0.0, per_point_dur=0.12):
        if not self.current_joints or self.ik_solver is None:
            return False
        seed = [self.current_joints[f'joint{i}'] for i in range(1, 7)]
        start = self._fixed_tcp_fk(seed)[:3, 3]
        axis = transform([0., 0., 0.], orientation)[:3, 2]
        end = np.array([target[k] for k in ('x', 'y', 'z')]) + extra_depth * axis
        grasp_target = dict(target, **dict(zip(('x', 'y', 'z'), end.tolist())))
        waypoints = self._fixed_tcp_waypoints(start, grasp_target, orientation, seed)
        if waypoints is None:
            self.get_logger().warning(f'{self._active_camera}工具轴接近路径未完整求出，未派发接近轨迹')
            return False
        return self.execute_multiwaypoint_joint_trajectory(waypoints, desc, per_point_dur=per_point_dur)

    def _plan_tool_axis_approach(self, target, seed, preferred_orientation, automatic, extra_depth=None,
                                 path_filter=None, diagnostics=None, pregrasp_m=0.06, depth_ladder=False):
        """优先小倾角、少转J6；接近线沿工具+Z，抓后仍走原搬运流程。"""
        from scipy.spatial.transform import Rotation
        extra_depth = (0.02 if automatic else 0.0) if extra_depth is None else extra_depth
        end = np.array([target[k] for k in ('x', 'y', 'z')], dtype=float)
        radial_yaw = np.degrees(np.arctan2(end[1], end[0]))
        geometry = getattr(self, '_grasp_geometry', None) or {}
        short_axis = None
        try:
            if automatic and geometry.get('frame') == 'base_link' and float(geometry.get('elongation_ratio', 0)) >= 1.5:
                axis = np.asarray(geometry['short_axis_base'], dtype=float)
                if axis.shape == (3,) and np.isfinite(axis).all() and np.linalg.norm(axis[:2]) > 1e-6:
                    short_axis = axis
        except (KeyError, TypeError, ValueError):
            pass

        if automatic:
            groups = []
            for tilt in (0., 5., 10., 15., 20., 25., 30.):
                q = Rotation.from_euler('xyz', [180., tilt, radial_yaw + 180.], degrees=True).as_quat()
                if short_axis is not None:
                    z_axis = Rotation.from_quat(q).as_matrix()[:, 2]
                    q = Rotation.from_matrix(short_axis_tool_rotation(short_axis, z_axis)).as_quat()

                # 对称姿态保持同一工具Z轴，不能把倾斜方向也反过来。
                groups.append((tilt, [q, [q[1], -q[0], q[3], -q[2]]]))
        else:
            q = preferred_orientation
            groups = [(None, [q, [q[1], -q[0], q[3], -q[2]]])]
        # 动态优先在同一小倾角内退让深入量，避免为已碰台面的2cm方案扫描所有姿态。
        depths = [extra_depth]
        if (path_filter is not None or depth_ladder) and automatic:
            while depths[-1] > 0.:
                depths.append(max(0., round(depths[-1]-.005, 6)))

        def record(stage, tilt, depth, pose, detail=None):
            if diagnostics is not None:
                diagnostics.append(dict(stage=stage, tilt_deg=tilt, depth_m=depth, pose=pose, detail=detail))

        for tilt, orientations, candidate_depth in [(t, o, d) for t, o in groups for d in depths]:
            if self._emergency_flag:
                return None
            candidates = []
            for orientation in orientations:
                if self._emergency_flag:
                    return None
                axis = Rotation.from_quat(orientation).as_matrix()[:, 2]
                start = end - pregrasp_m * axis
                pre_pose = dict(zip(('x', 'y', 'z'), start.tolist()))
                joints = self._fixed_tcp_ik(pre_pose, orientation, seed)
                if joints is None:
                    record('预抓取IK失败', tilt, candidate_depth, pre_pose)
                    continue
                # 预抓取点不变；额外深度只沿工具轴加一次，并在移动前核查。
                grasp_end = end + candidate_depth * axis
                grasp_target = dict(target, **dict(zip(('x', 'y', 'z'), grasp_end.tolist())))
                path = self._fixed_tcp_waypoints(start, grasp_target, orientation, joints)
                if path is None:
                    record('工具轴路径IK失败', tilt, candidate_depth, pre_pose)
                    continue
                score = abs(joints[5] - seed[5]) + 0.1 * np.linalg.norm(np.asarray(joints) - seed)
                candidates.append((score, pre_pose, orientation.tolist() if hasattr(orientation, 'tolist') else list(orientation), joints, path))
            for _, pre_pose, orientation, joints, path in sorted(candidates, key=lambda item: item[0]):
                if not self._joint_waypoints_valid(path, start_joints=joints, report_failure=False):
                    record('静态路径状态检查失败', tilt, candidate_depth, pre_pose,
                           getattr(self, '_last_state_validity_failure', None))
                    continue
                if path_filter is not None and not path_filter(pre_pose, orientation, joints, candidate_depth):
                    record('完整移动路径预解失败', tilt, candidate_depth, pre_pose)
                    continue
                self.last_grasp_extra_depth = candidate_depth
                self.last_grasp_axis_mode = '短轴夹持' if short_axis is not None else '径向夹持'
                record('通过', tilt, candidate_depth, pre_pose)
                return pre_pose, orientation, joints, tilt
        if (automatic and path_filter is None and not depth_ladder
                and extra_depth > 0.0 and not self._emergency_flag):
            next_depth = max(0.0, round(extra_depth - 0.005, 6))
            self.get_logger().warning(
                f'{self._active_camera}额外深入{extra_depth * 100:.1f}cm的路径未通过，'
                f'改试额外深入{next_depth * 100:.1f}cm')
            return self._plan_tool_axis_approach(target, seed, preferred_orientation, True,
                extra_depth=next_depth, path_filter=path_filter, diagnostics=diagnostics,
                pregrasp_m=pregrasp_m)
        return None

    def move_arm_pose(self, pose_dict, desc, continuous=False, planning_mode='normal',
                       preferred_orientation=None, fixed_orientation=False, approach_target_z=None,
                       auto_grasp=False):
        """
        统一运动方法：优先使用 IK + 关节空间，失败回退到 Cartesion 规划

        IK 优先级:
          1. enable_ik=True → Pinocchio 内嵌 IK (move_arm_via_ik)
          2. enable_ik=False → TRAC-IK 服务 (/compute_ik, _compute_ik_via_service)
          3. 兜底 → MoveIt2 OMPL 规划 (move_arm_cartesian)

        Args:
            pose_dict: {'x', 'y', 'z'} 目标位置
            desc: 动作描述
            continuous: 连贯模式
            planning_mode: 传递给 Cartesian 兜底的规划模式
            preferred_orientation: 优先姿态 ([x,y,z,w] 四元数), 传给 IK 和 Cartesian 兜底
            fixed_orientation: 固定姿态、沿工具Z轴接近
            auto_grasp: 双相机普通水果抓取自动选择径向小倾角；GraspNet保留网络姿态
        """
        if fixed_orientation:
            from scipy.spatial.transform import Rotation
            if preferred_orientation is None or self.ik_solver is None:
                return False
            q = preferred_orientation
            original = ([q.x, q.y, q.z, q.w] if isinstance(q, Quaternion) else list(q))
            seed = ([self.current_joints.get(f'joint{i}', 0.0) for i in range(1, 7)]
                    if self.current_joints else None)
            if seed is None:
                return False
            if approach_target_z is not None:
                target = dict(pose_dict, z=float(approach_target_z))
                plan = self._plan_tool_axis_approach(target, seed, original, auto_grasp, pregrasp_m=getattr(self,'_static_cycle_settings',{}).get('approach_height_m',.06))
                if plan is None:
                    self.get_logger().warning(f'{self._active_camera}未找到可达且无碰撞的工具轴接近路径，目标={target}')
                    return False
                candidate, orientation, joints, tilt = plan
            else:
                candidate, orientation, tilt = dict(pose_dict), original, None
                joints = self._fixed_tcp_ik(candidate, orientation, seed)
                if joints is None:
                    return False
            self.last_planning_strategy = 'pinocchio_fixed_tcp'
            self.last_planning_profile_name = 'tool_z_approach'
            if not self.move_arm_joint(joints, f'{desc} (工具轴预抓取点)', continuous=continuous):
                return False
            self.last_successful_orientation = orientation
            self.last_approach_pose = candidate
            rpy = Rotation.from_quat(orientation).as_euler('xyz', degrees=True)
            self.get_logger().info(f'{self._active_camera}工具轴接近: 自动={auto_grasp} 倾角={tilt}deg '
                                   f'RPY={np.round(rpy, 1).tolist()} J6={np.degrees(joints[5]):.1f}deg '
                                   f'方向={getattr(self, "last_grasp_axis_mode", "固定姿态")} '
                                   f'预抓取点={candidate}; 预留距离={getattr(self, "_static_cycle_settings", {}).get("approach_height_m",.06)}m 额外深入={self.last_grasp_extra_depth if approach_target_z is not None else 0.0}m')
            return True

        if self.enable_ik and self.ik_solver is not None:
            # ── 路径 1: Pinocchio 内嵌 IK ──
            orientations = self._build_pick_orientations_multi(pose_dict)
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
            print("🟡 Pinocchio IK 路径失败，回退到 MoveIt2 笛卡尔规划...")
        else:
            # ── 路径 2: TRAC-IK 服务直接求解 (通过 /compute_ik) ──
            # 构建候选姿态列表: 优先 preferred_orientation, 再径向朝下
            trac_ik_oris = []
            if preferred_orientation is not None:
                if isinstance(preferred_orientation, Quaternion):
                    trac_ik_oris.append(preferred_orientation)
                elif isinstance(preferred_orientation, (list, tuple)) and len(preferred_orientation) == 4:
                    trac_ik_oris.append(Quaternion(
                        x=float(preferred_orientation[0]),
                        y=float(preferred_orientation[1]),
                        z=float(preferred_orientation[2]),
                        w=float(preferred_orientation[3])))
            # 补充径向朝下姿态
            radial_ori = self._build_pick_orientation(pose_dict)
            if radial_ori:
                for o in radial_ori:
                    if o not in trac_ik_oris:
                        trac_ik_oris.append(o)

            # 尝试每个候选姿态 + 反向
            seed = None
            if self.current_joints:
                seed = [self.current_joints.get(f'joint{i}', 0.0) for i in range(1, 7)]
            _trac_t0 = self.get_clock().now().nanoseconds * 1e-9
            for ori in trac_ik_oris:
                ok, q_sol = self._compute_ik_via_service(pose_dict, ori, seed_joints=seed, timeout_s=0.2)
                if ok and q_sol:
                    if self.move_arm_joint(q_sol, f"{desc} (TRAC-IK)", continuous=continuous):
                        _trac_t1 = self.get_clock().now().nanoseconds * 1e-9
                        self.get_logger().info(
                            f"📊 TRAC-IK 路径总耗时: {(_trac_t1-_trac_t0)*1000:.0f}ms "
                            f"(IK+关节执行)")
                        # 保存成功姿态供后续下降使用
                        self.last_successful_orientation = ori
                        return True
                    else:
                        self.get_logger().warn("⚠️ TRAC-IK 关节执行失败, 尝试下一姿态")
            
            self.get_logger().info("🟡 TRAC-IK 直接求解失败, 回退到 MoveIt2 OMPL 规划...")

        # ⚠️ 全程仅用 Pinocchio IK (用户要求): 不再回退到 MoveIt2 OMPL/笛卡尔规划.
        # 原因: move_group 单一规划器下同时发出多个 profile 请求会互相抢占资源,
        # 既慢又常全部失败; 而 Pinocchio IK + 多种子 + 提前接受阈值对动态抓取已足够.
        # Pinocchio IK 失败直接返回 False, 交由调用方回退 (如动态抓取回退静态、
        # 静态流程回待机位), 不做 MoveIt 兜底.
        if self.enable_ik and self.ik_solver is not None:
            self.get_logger().warn(f"🟡 Pinocchio IK 失败, 不再回退 MoveIt (全程皮诺曹), 目标: {desc}")
            return False

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
        if self.workbench_interrupted():return False
        self._check_calibrated_tool()
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

        # ⚠️ 禁用 MoveIt 并行求解: move_group 是单一规划器, 同时 call_async 多个请求
        # 会互相抢占 BFM 线程/规划资源, 导致每个 profile 都超慢且通常全部失败.
        # 改为串行: 按优先级逐个发送, 单请求等待完成后若不满足再尝试下一档 (更快速且稳定).
        self.get_logger().info(f"⚡ 开始串行求解 {len(planning_profiles)} 种位姿解 (禁用并行)...")

        best_profile = None
        best_trajectory = None

        # 按优先级顺序 (0 为最严格/最优) 串行尝试
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
            req.motion_plan_request.max_velocity_scaling_factor = getattr(self,"_static_cycle_settings",{}).get("joint_velocity_scaling",0.65)
            req.motion_plan_request.max_acceleration_scaling_factor = getattr(self,"_static_cycle_settings",{}).get("joint_acceleration_scaling",0.65)
            
            c = self._build_goal_constraints(
                pose_dict,
                orientation=profile['orientation'],
                pos_tolerance_radius=profile['pos_tol'],
                ori_tolerances=profile['ori_tol'],
            )
            req.motion_plan_request.goal_constraints.append(c)

            # 串行: 发送单个请求并等待其完成
            future = self._plan_client.call_async(req)
            while rclpy.ok() and not future.done():
                rclpy.spin_once(self, timeout_sec=0.01)
            res = future.result()

            if not (res and res.motion_plan_response.error_code.val == 1):
                self.get_logger().info(f"⏳ 串行: profile '{profile['name']}' 未解得, 尝试下一档")
                continue

            traj = res.motion_plan_response.trajectory
            if len(traj.joint_trajectory.points) == 0:
                self.get_logger().info(f"⏳ 串行: profile '{profile['name']}' 返回空轨迹, 尝试下一档")
                continue

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
                            f"⛔ 安全拒绝: profile '{profile['name']}' 轨迹终点末端 Z 轴朝上 "
                            f"(z分量={float(ee_z[2]):.2f}, 需<-0.5 防朝上抓), 跳过此 profile"
                        )
                        continue
                except Exception as _e:
                    self.get_logger().warn(f"轨迹姿态安全检查异常 ({_e}), 谨慎放行")
            best_profile = profile
            best_trajectory = traj
            break

        if best_profile is not None:
            self.get_logger().info(f"\x1b[32m🎉 串行求解成功！选用最优解策略: {best_profile['name']}\x1b[0m")
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
            if not self._wait_control_future(future_exec):return False
            goal_handle = future_exec.result()
            if not goal_handle.accepted:
                self.get_logger().error("被拒绝执行")
                return False
                
            self._active_goal_handle = goal_handle
            if self.workbench_interrupted():
                goal_handle.cancel_goal_async();return False
            res_future = goal_handle.get_result_async()
            if not self._wait_control_future(res_future):return False
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
        if self.workbench_interrupted():return False
        if not self._gripper_action_client.wait_for_server(timeout_sec=timeout_sec):
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        # 夹爪提速 (连贯性优化): 原 1s 偏慢, 全周期 2 次夹爪动作累加耗时长; 降到 0.6s.
        # ⚠️ 实机标定旋钮: 夹不紧(太快/未到位)→调大, 卡顿→再调小.
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 600_000_000
        goal_msg.trajectory.points.append(point)

        send_goal_future = self._gripper_action_client.send_goal_async(goal_msg)
        if not self._wait_control_future(send_goal_future):return False
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False

        self._active_goal_handle = goal_handle
        if self.workbench_interrupted():
            goal_handle.cancel_goal_async();return False
        get_result_future = goal_handle.get_result_async()
        if not self._wait_control_future(get_result_future):return False
        result = get_result_future.result()
        if result is None:
            return False
        return True

    def _publish_gripper_joint_state(self, target_pos, force=None):
        # Unified fallback for both real arm and simulation bridge.
        # effort 字段 = 夹持力 (N), agx_ctrl_single_node 读取后传给 gripper.move(width, force)
        if self.workbench_interrupted():return False
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['gripper']
        msg.position = [float(target_pos)]
        msg.velocity = []
        msg.effort = [force if force is not None else self._gripper_force]
        self._joint_states_pub.publish(msg)

    def operate_gripper(self, target_pos, desc):
        if self.workbench_interrupted():return False
        self.get_logger().info(f"✊ 正在执行夹爪动作 -> {desc} (target: {target_pos})")

        # 记录操作前宽度 (抓取检测用, 在调用处设置 _pre_grasp_width)

        # 1) Try common MoveIt/gripper controller naming first.
        if self._try_gripper_action(
            ['gripper_joint1', 'gripper_joint2'],
            [float(target_pos * 0.5), float(-target_pos * 0.5)],
            timeout_sec=0.8,
        ):
            self.get_logger().info('⭕ 夹爪动作执行中（gripper_joint1/2）...')
            time.sleep(0.05)  # 连贯性优化: 原 0.2s 冗余等待, 降为 0.05
            return True

        # 2) Try Gazebo piper controller naming.
        if self._try_gripper_action(
            ['joint7'],
            [float(target_pos * 0.5)],
            timeout_sec=0.4,
        ):
            self.get_logger().info('⭕ 夹爪动作执行中（joint7）...')
            time.sleep(0.05)  # 连贯性优化: 原 0.2s 冗余等待, 降为 0.05
            return True

        # 3) Fallback: publish unified control topic used by real arm and sim bridge.
        self.get_logger().warn('⚠️ 夹爪 action 不可用或被拒绝，回退到 /control/joint_states')
        self._publish_gripper_joint_state(target_pos)
        time.sleep(0.05)  # 连贯性优化: 原 0.2s 冗余等待, 降为 0.05
        return True

    def _get_current_tcp_mm(self):
        """通过 TF lookup 获取末端执行器坐标 (mm)."""
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link', 'tcp_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            return [
                round(t.transform.translation.x * 1000.0, 1),
                round(t.transform.translation.y * 1000.0, 1),
                round(t.transform.translation.z * 1000.0, 1),
            ]
        except Exception as e:
            self.get_logger().warn(f"TF lookup base_link→link6 失败: {e}")
            return [0.0, 0.0, 0.0]

    def _get_place_position(self, place_joints):
        """通过 FK 计算放置目标点坐标字符串."""
        try:
            if self.ik_solver is None or _pin is None:
                return "(0.000, 0.000, 0.000)"
            q_full = _pin.neutral(self.ik_solver.model)
            for i, idx in enumerate(self.ik_solver.joint_q_indices):
                if i < len(place_joints):
                    q_full[idx] = place_joints[i]
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
        target_position = self._get_place_position(target_joints)

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
        """指尖中点直接对准pick中的三维中心；表面高度只保留作诊断。"""
        position=np.array([pick[k] for k in ('x','y','z')],dtype=float)
        if not np.isfinite(position).all() or position[2]<min_grasp_z:
            raise ValueError('蓝方块TCP中心无效或低于安全下限')
        pose=dict(zip(('x','y','z'),position.tolist()))
        lifted=dict(pose,z=pose['z']+.12)
        self.get_logger().info(f"蓝方块TCP中心: {position.tolist()}；不添加工具长度/顶面偏置")
        return dict(POSE_PICK=pose,POSE_PICK_UP=lifted,clamping_z=pose['z'],gripper_close_width=0.0)

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
        # 尝试从视觉节点获取 PCA 短轴朝向 (优先于径向朝下)
        pca_ori = None
        if hasattr(self, '_latest_detection') and self._latest_detection:
            pca_ori = self._latest_detection.get('grasp_orientation')
        if pca_ori is not None and len(pca_ori) == 4:
            # PCA 短轴朝向来自视觉节点 (D455 点云 PCA)
            self.get_logger().info(
                f"📐 蓝方块姿态预规划: PCA 短轴对齐 (rotation={block_rotation_deg:.1f}°)")
            return list(pca_ori)
        
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
          - step2:   笛卡尔直线下降到 POSE_PICK, 禁用传送带碰撞 (复用 _set_conveyor_collision_allowed)
          - step3:   闭合夹爪, 目标宽度 = block_width_m - 0.002m (不是 GRIPPER_GRAB=0)
          - step4:   上提至 POSE_PICK_UP, continuous=True
          - step4.5: 抓取成功判定 = 闭合宽度 < block_width_m * 0.7 (不是固定 5mm)
          - step5:   关节空间直接移动到放置目标点 + 释放 + 回待机位
          - 碰撞检测恢复 (成功和错误路径均, 复用现有逻辑)
          - 失败时重试最多 2 次 (复用现有重试逻辑)

        Args:
            data: 命令 JSON dict, 含 pick/surface_z_m/block_width_m/block_height_m/block_rotation_deg
            config: 常量 dict {
                'JOINT_STANDBY', 'JOINT_OBSERVE', 'JOINT_PLACE_TARGET',
                'GRIPPER_PLACE_OPEN', 'GRIPPER_OPEN', 'GRIPPER_SETTLE_SEC',
                'GRIPPER_PICK_Z_OFFSET', 'MIN_GRASP_Z', 'loop_count'
            }

        Returns:
            dict: {'success': bool, 'cycle_id': str}
        """
        JOINT_STANDBY = config['JOINT_STANDBY']
        JOINT_OBSERVE = config['JOINT_OBSERVE']
        JOINT_PLACE_TARGET = config['JOINT_PLACE_TARGET']
        GRIPPER_PLACE_OPEN = config['GRIPPER_PLACE_OPEN']
        GRIPPER_OPEN = config['GRIPPER_OPEN']
        GRIPPER_SETTLE_SEC = config['GRIPPER_SETTLE_SEC']
        GRIPPER_PICK_Z_OFFSET = config['GRIPPER_PICK_Z_OFFSET']
        MIN_GRASP_Z = config['MIN_GRASP_Z']
        # 蓝方块专用安全下限: 水果的 0.045m 对 1-5cm 物块过高
        # 桌面(传送带台面)在 z≈-0.015m, 蓝方块中位 z≈0.003m (3cm 物块)
        # 设 0.0m: 比桌面高 15mm, 夹爪手指有足够离地间隙
        MIN_GRASP_Z = 0.004
        # 蓝方块专用 Z 偏移: 二分法调试
        # 0.064 → 夹到了(width=54-69mm) 但 gripper_link1 z=-0.036 触碰台面
        # 0.1355 → gripper_link1 z=0.038 对齐顶面 但 空夹(width=0mm)
        # 0.10 → 折中测试 (gripper_link1 z≈-0.018, 接近台面但不深入)
        GRIPPER_PICK_Z_OFFSET = 0.0
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
        place_id = "放置点"
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

            # ── 下降前禁用所有传送带碰撞体 (复用 _set_conveyor_collision_allowed) ──
            self._set_conveyor_collision_allowed(allowed=True)

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
                                    # 更新 block 参数
                                    surface_z_m = float(new_surf)
                                    block_rotation_deg = det.get('block_rotation_deg', block_rotation_deg)
                                    block_width_m = det.get('block_width_m', block_width_m)
                                    block_height_m = det.get('block_height_m', block_height_m)
                                    # 重新计算动态偏移和 clamping_z
                                    clamping_z = float(new_surf)
                                    if clamping_z < MIN_GRASP_Z:raise ValueError("蓝方块TCP目标低于安全下限")
                                    new_height = float(block_height_m) if block_height_m else 0.04
                                    dynamic_offset = 0.0
                                    POSE_PICK['z'] = clamping_z + dynamic_offset
                                    POSE_PICK_UP = POSE_PICK.copy()
                                    POSE_PICK_UP['z'] += 0.12
                                    self.get_logger().info(
                                        f"📐 蓝方块重试: 位姿更新 surface_z={surface_z_m:.3f}m "
                                        f"height={block_height_m:.3f}m offset={dynamic_offset*1000:.1f}mm "
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
                    self._set_conveyor_collision_allowed(allowed=True)
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

            # 【第五步】 关节空间直接移动到放置目标点 (抓取抬起后直达, 无需中间途径点)
            # ⚠️ continuous=False: 到达后等 0.15s 稳定, 再开夹爪
            self._set_conveyor_collision_allowed(allowed=True)
            if not self.move_arm_joint(JOINT_PLACE_TARGET, f"蓝方块-移动到{place_id} (关节空间)", continuous=False):
                success = False; break
            place_reached_ok = True
            if self._check_emergency("blue_block-step5-放置点"):
                success = False; break

            # 【第六步】 松开夹爪释放物品
            self.operate_gripper(GRIPPER_PLACE_OPEN, f"蓝方块-松开夹爪(在{place_id}释放)")
            time.sleep(GRIPPER_SETTLE_SEC)
            if self._check_emergency("blue_block-step6-松开夹爪"):
                success = False; break

            # 【第七步】 关节空间回到待机位 (等待下一次抓取)
            if not self.move_arm_joint(JOINT_STANDBY, "蓝方块-回到待机位 (关节空间)", continuous=True):
                success = False; break
            if self._check_emergency("blue_block-step7-回待机位"):
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
                target_joints=JOINT_PLACE_TARGET, grasp_ok=False, error_code=99)

            # 安全回到待机位后恢复碰撞检测 (无论回待机位是否成功都恢复)
            self._set_conveyor_collision_allowed(allowed=False)
            self.operate_gripper(GRIPPER_OPEN, "蓝方块-张开夹爪(待机)")
            time.sleep(0.5)
            return {'success': False, 'cycle_id': cycle_id}

        # 成功完成分拣后恢复碰撞检测
        self._set_conveyor_collision_allowed(allowed=False)

        # ── 发布机械臂状态 (成功) ──
        self._publish_robot_status(
            success=True, cargo_type=obj_class, confidence=0.95,
            source_pos_dict={"x": float(POSE_PICK['x']), "y": float(POSE_PICK['y']),
                             "z": float(POSE_PICK['z'])},
            target_joints=JOINT_PLACE_TARGET, grasp_ok=True, error_code=0)

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
    # ⚠️ 2026-08-17: TRAC-IK 测试阶段, 临时禁用内嵌 Pinocchio IK
    # 使用内嵌 Pinocchio IK 求解, 失败回退到 MoveIt2 (TRAC_IKKinematicsPlugin)
    enable_ik_solver = True
    node.enable_ik = True
    if node.enable_ik:
        print("✅ 内嵌 Pinocchio IK 已启用 - 优先使用 IK 求解, 失败回退 MoveIt2")
    else:
        print("⚠️ Pinocchio IK 已禁用 - 所有运动步骤走 MoveIt2")
    
    # ── 默认待机位 (eye-to-hand: 相机固定, 用待机位作为默认位置) ──
    # 程序启动/任务完成/错误恢复 都回到此位置
    JOINT_STANDBY = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # ── 观察位坐标记录 (保留供未来调用, eye-to-hand 下不使用) ──
    # 眼在手内配置用: 相机装在末端, 需到此位置观察传送带
    # joint1=0, joint2=1.0669, joint3=-1.1637, joint4=0, joint5=1.2185, joint6=0
    # 对应末端位姿: x=0.1864, y=0.0, z=0.3782, q=(~0, 0.9643, ~0, 0.2650)
    JOINT_OBSERVE = [0.0, 1.0669023184516138, -1.1636808254746993, 0.0, 1.2184841639873214, 0.0]

    # ========== 放置目标点 (用户标定, 关节空间) ==========
    # 抓取后流程: pick_up_joints -> 直接移动到放置目标点 -> 开夹爪 -> standby
    # 关节顺序: joint1, joint2, joint3, joint4, joint5, joint6
    # (gripper_joint1/2 由 GRIPPER_PLACE_OPEN 控制, 对应 +0.04475 / -0.04475)
    JOINT_PLACE_TARGET = [1.9992746581595044, 1.5727510955571302, -1.0801144608892108,
                          -0.008045967851693859, 1.0517179539592632, 0.022270401255447644]
    # 放置点开夹爪宽度: gripper_joint1 = 0.0895*0.5 = 0.04475, gripper_joint2 = -0.04475
    GRIPPER_PLACE_OPEN = 0.0895

    GRIPPER_OPEN = 0.090
    GRIPPER_CLOSE = 0.060
    GRIPPER_GRAB = 0.000
    GRIPPER_SETTLE_SEC = 0.05
    MAX_GRASP_RETRIES = 3       # 抓取失败最大重试次数

    # 兼容既有蓝方块调用参数；实际TCP目标不再添加这个偏移。
    GRIPPER_PICK_Z_OFFSET = 0.0
    # conveyor.scene中传送带顶面z=0；低于安全下限时拒绝，不移动目标坐标。
    MIN_GRASP_Z = 0.004  # conveyor.scene的belt_deck顶面z=0，保留4mm几何安全间隙

    # 动态流程读取 config/dynamic_grasp.json；静止抓取常量不变。
    DYNAMIC_R_MAX = 0.55

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

    def _run_dynamic_grasp(self, target):
        from dynamic_grasp_runtime import DynamicGraspRunner
        plan = self.placement_snapshot(target.get('name',''), 'd455') if getattr(self, '_workbench_task', False) else None
        return DynamicGraspRunner(self, target, plan['release_joints_rad'] if plan else JOINT_PLACE_TARGET, placement=plan).run()
    node._run_dynamic_grasp = _run_dynamic_grasp

    def _pick_next_target(self, excluded=(), requested=None):
        det = self._latest_detection or {}
        if det.get('camera') != 'd455' or not det.get('motion_session'):
            return None
        cfg = self._dynamic_settings()
        s_max = float(cfg['intercept_s_max_m'])
        candidates = []
        for obj in det.get('objects', []):
            track_id = obj.get('track_id')
            if track_id is None or track_id in excluded or (requested is not None and track_id != requested):
                continue
            obs = obj.get('motion_observation') or {}
            bp = obs.get('position_m')
            if not bp or not all(k in bp for k in ('x', 'y', 'z')):
                continue
            stamp = obs.get('stamp_s', 0.)
            age = self.get_clock().now().nanoseconds*1e-9-float(stamp)
            if not 0 <= age <= 1. or math.hypot(float(bp['x']), float(bp['y'])) > DYNAMIC_R_MAX:
                continue
            calib = self._dynamic_calib or {}
            from dynamic_grasp import conveyor_velocity
            direction = conveyor_velocity(calib, 1., self._belt_direction)
            downstream = float(bp['x'])*direction[0] + float(bp['y'])*direction[1]
            # 只挑还没越过物理可抓上界的物体：固定选最下游那个会把拦截点推到工作区外。
            if downstream > s_max:
                continue
            candidates.append((downstream, dict(name=obj.get('object_name', 'object'),
                track_id=track_id, motion_session=det['motion_session'], base_position_m=bp,
                diameter=obj.get('estimated_diameter_m'))))
        return max(candidates, key=lambda item:item[0])[1] if candidates else None
    node._dynamic_pick_next_target = _pick_next_target

    LEGACY_PLACE_TARGET = list(JOINT_PLACE_TARGET)
    LEGACY_STANDBY = list(JOINT_STANDBY)
    LEGACY_OBSERVE = list(JOINT_OBSERVE)

    # 动态连续续抓安全护栏: 一次 sort_dynamic 最多连续抓取个数, 到限即返回待机让位,
    # 给 GUI 停止/急停机会 (避免传送带持续供料时服务端无限抓取、GUI 无法打断).
    DYNAMIC_MAX_CONSECUTIVE = 6
    # 单次 sort_dynamic 内连续失败上限: 失败只跳过本轮目标并等下一个物体, 超过上限才交回 GUI.
    DYNAMIC_MAX_ATTEMPT_FAILS = 5

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
            if getattr(node, "_workbench_emergency", False) and not getattr(node, "_workbench_idle_emergency_handled", False) and (node._emergency_flag or node._emergency_req_type == "reset"):
                node._workbench_idle_emergency_handled = True
                node._check_emergency("工作台空闲控制")
                if node._emergency_flag:continue
            
            try:
                cmd_data_str = node.cmd_queue.get_nowait()
            except queue.Empty:
                continue
                
            try:
                data = json.loads(cmd_data_str)
                node.validate_workbench_command(data)
                assert_visual_target_fresh(data)  # 出队重新查戳，不能只在ROS接收时检查
            except Exception as e:
                print(f"❌ 收到无法解析的指令: {cmd_data_str}, 错误: {e}")
                node.workbench_ack(locals().get("data",{}),False,str(e))
                continue
                
            node._workbench_cycle_epoch=getattr(node,"_workbench_epoch",0)
            camera = data.get("camera_id", "d455")
            if camera != getattr(node,'_active_camera','d455'):
                node._latest_detection = None
                node._pending_grasp_request = None
                node._graspnet_event.clear()
            node._active_camera = camera
            node._latest_detection = node._detections_by_camera.get(camera)
            node._static_cycle_settings = {}
            node._workbench_task = bool(data.get('workbench'))
            JOINT_STANDBY, JOINT_OBSERVE, JOINT_PLACE_TARGET = list(LEGACY_STANDBY), list(LEGACY_OBSERVE), list(LEGACY_PLACE_TARGET)
            if node._workbench_task:
                refs = __import__('workbench.placement',fromlist=['PlacementStore']).PlacementStore(runtime_config.workspace_root()/'config/workbench/placements.json').get()['reference_poses']
                JOINT_STANDBY = refs['standby_joints_rad']
                JOINT_OBSERVE = refs['dabai_observe_joints_rad']
            node._visual_target_meta = data if data.get('vision_target') else {}

            if data.get("cmd") == "select_camera":
                continue

            elif data.get("cmd") == "quit":
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

                print(f"\n👁️ 收到客户端指令：到 {node._active_camera} 相机观察位")
                # 通过 MoveIt 到当前相机观察位，不改变夹爪状态
                observation = (getattr(node, '_dynamic_ready_joints', None) if node._active_camera == 'd455' and data.get('dynamic') else None)
                if observation is None and node._workbench_task and data.get('dynamic'):
                    node.get_logger().error('当前尚无动态准备位，请先运行原动态准备流程');continue
                if observation is None:
                    observation = (JOINT_OBSERVE if camera=='dabai' else JOINT_STANDBY) if node._workbench_task else runtime_config.load_runtime_config()['cameras'][node._active_camera].get('observation_joints',JOINT_STANDBY)
                if len(observation)!=6 or not np.isfinite(observation).all():raise ValueError('观察位关节参数无效')
                node.move_arm_joint(observation, "到当前相机观察位 (关节空间)")
                
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
                node._grasp_geometry = data.get('grasp_geometry') if not use_graspnet else None
                object_diameter_m = data.get("object_diameter_m", None)
                geometry = node._grasp_geometry or {}
                if geometry.get('elongation_ratio', 0) >= 1.5:
                    object_diameter_m = geometry.get('width_m', object_diameter_m)
                    node.get_logger().info(
                        f"长条物体短轴夹持: 长约{geometry['length_m'] * 1000:.1f}mm "
                        f"宽约{object_diameter_m * 1000:.1f}mm，沿短轴调整夹爪方向与开度")

                dynamic_open, dynamic_close = compute_gripper_targets(object_diameter_m)

                pick_pose = data["pick"]
                pick_id = data.get("pick_name", "Pick")
                obj_class = str(data.get("object_name") or pick_id).split(' (')[0].strip()
                cycle_id = str(data.get("cycle_id", ""))

                place_id = "放置点"
                placement = node.placement_snapshot(obj_class, camera) if node._workbench_task else None
                JOINT_PLACE_TARGET = placement['release_joints_rad'] if placement else JOINT_PLACE_TARGET
                normal_return = placement['normal_return_joints_rad'] if placement else JOINT_STANDBY
                static_settings = json.loads((runtime_config.workspace_root()/'config/workbench/settings.json').read_text()).get('static',{}) if node._workbench_task else {}
                node._static_cycle_settings = dict(static_settings)
                node._gripper_force = static_settings.get('gripper_force_n', node._gripper_force)
                GRIPPER_SETTLE_SEC = static_settings.get('gripper_settle_s', 0.05)
                POSE_PICK = pick_pose.copy()
                # 全部目标为指尖中点TCP；不添加旧XYZ、类别或夹爪长度补偿。
                # base_link的Z=0不是实测台面；不以固定4mm阈值裁剪或抬高水果目标。
                try:
                    coordinates = {k: float(POSE_PICK[k]) for k in ('x', 'y', 'z')}
                    if not all(math.isfinite(v) for v in coordinates.values()):
                        raise ValueError('XYZ包含非有限数值')
                except (KeyError, TypeError, ValueError) as exc:
                    node.get_logger().error(f'本次抓取坐标无效，跳过并继续接收指令: {exc}')
                    node.workbench_ack(data,False,str(exc))
                    continue
                POSE_PICK.update(coordinates)
                POSE_PICK_UP = POSE_PICK.copy()
                approach_height = 0.13 if node._active_camera == 'dabai' and use_graspnet else static_settings.get('approach_height_m', 0.06)
                POSE_PICK_UP['z'] += approach_height  # 普通水果抓取随后按工具轴重算预抓取XYZ
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
                        node._grasp_geometry = refine_result.get('grasp_geometry')
                        # 用精定位坐标更新相关位姿
                        for p in (POSE_PICK, POSE_PICK_UP, POSE_LIFT_SOFT):
                            p['x'] = refine_result['x']
                            p['y'] = refine_result['y']
                        # 二次检测输出物体三维中心；不使用尺寸推断的质心或旧偏置。
                        measured_z = float(refine_result['z'])
                        if not math.isfinite(measured_z):
                            node.get_logger().error('精定位TCP目标Z不是有限数值，本次抓取失败')
                            success = False; break
                        POSE_PICK['z'] = measured_z
                        POSE_PICK_UP['z'] = measured_z + approach_height
                        POSE_LIFT_SOFT['z'] = measured_z + 0.035

                    # ── GraspNet 主导抓取 (sort_graspnet) ──
                    # 同帧RGB-D重估选中物体中心；网络提供姿态和宽度。
                    if use_graspnet:
                        # a. 确保待机位 + 张开夹爪 (避免夹爪遮挡, 保证深度数据新鲜)
                        if node._active_camera == 'd455':
                            if not node.move_arm_joint(JOINT_STANDBY, "GraspNet: 回到待机位"):
                                success = False;break
                        # Dabai保持当前已看到物体的观察位，不先退到零点造成失去视野。
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
                        # 服务返回的translation已经是物体中心TCP目标，不再加长度。
                        if result.get('target_frame') != 'tcp_link' or result.get('reference_point') != 'midpoint_between_fingertips' or result.get('grasp_point_type') != 'estimated_centroid':
                            node.get_logger().error('GraspNet返回未知目标参考点，本次抓取失败')
                            success = False; break
                        if not np.isfinite(np.asarray(gn_t, dtype=float)).all():
                            node.get_logger().error('GraspNet TCP目标不是有限数值，本次抓取失败')
                            success = False; break
                        POSE_PICK['z'] = float(gn_t[2])
                        POSE_PICK_UP['z'] = POSE_PICK['z'] + approach_height
                        POSE_LIFT_SOFT['z'] = POSE_PICK['z'] + 0.035
                        # GraspNet 6DoF 姿态 (机器人约定: R[:,0]=opening, R[:,1]=ortho, R[:,2]=approach)
                        graspnet_quat = [float(gn_q[0]), float(gn_q[1]), float(gn_q[2]), float(gn_q[3])]
                        # 用 GraspNet 估计宽度重算夹爪开合目标
                        dynamic_open, dynamic_close = compute_gripper_targets(gn_w)
                        node.get_logger().info(
                            f"🎯 GraspNet 位姿: t=({gn_t[0]:.3f},{gn_t[1]:.3f},{gn_t[2]:.3f}) "
                            f"score={result.get('score',0):.3f} width={gn_w:.3f} "
                            f"(候选{result.get('n_candidates','?')}个) → TCP中心z={POSE_PICK['z']:.3f}m")

                    # 【第零步】 在前往途中或起点提前张开夹爪
                    node.operate_gripper(dynamic_open, "张开夹爪(准备抓取)")
                    if node._check_emergency("step0-张开夹爪"):
                        success = False; break

                    # ── 姿态预规划: 在第一步(移动到抓取点上方)之前计算好目标姿态 ──
                    # 双相机普通水果抓取共用自动姿态；GraspNet保留网络姿态。
                    tool_axis_grasp = (node._active_camera == 'd455'
                                       or (node._active_camera == 'dabai' and not use_graspnet))
                    if use_graspnet:
                        active_ori = graspnet_quat
                        node.get_logger().info("📐 姿态预规划: GraspNet 6DoF 姿态")
                    elif tool_axis_grasp:
                        active_ori = ([float(POSE_PICK[k]) for k in ('qx', 'qy', 'qz', 'qw')]
                                      if all(k in POSE_PICK for k in ('qx', 'qy', 'qz', 'qw'))
                                      else node._build_pick_orientation(POSE_PICK)[0])
                        node.get_logger().info(f'{node._active_camera}自动选择抓取姿态，GUI中的RPY不再用于普通抓取')
                    else:
                        pick_fallback_ori = node._build_pick_orientation(POSE_PICK)[0]
                        # 从视觉节点获取 PCA 短轴对齐朝向
                        # realsense_yolo_node.py 发布格式: dict {qx,qy,qz,qw}
                        # blue_block_detector.py 发布格式: dict {qx,qy,qz,qw}
                        pca_ori = None
                        raw_ori = None
                        if getattr(node, '_latest_detection', None):
                            raw_ori = node._latest_detection.get('grasp_orientation')
                        if raw_ori is not None:
                            # 处理 dict 格式 {qx, qy, qz, qw}
                            if isinstance(raw_ori, dict) and all(k in raw_ori for k in ('qx','qy','qz','qw')):
                                # 排除单位四元数 (视觉节点默认值 qx=qy=qz=0, qw=1, 非真实 PCA)
                                if abs(float(raw_ori['qx'])) + abs(float(raw_ori['qy'])) + abs(float(raw_ori['qz'])) > 1e-6:
                                    pca_ori = [float(raw_ori['qx']), float(raw_ori['qy']),
                                               float(raw_ori['qz']), float(raw_ori['qw'])]
                                    node.get_logger().info(
                                        f"📐 姿态预规划: PCA 短轴对齐姿态 (来自视觉节点 dict)")
                            # 处理 list 格式 [qx, qy, qz, qw]
                            elif isinstance(raw_ori, (list, tuple)) and len(raw_ori) == 4:
                                if abs(raw_ori[0]) + abs(raw_ori[1]) + abs(raw_ori[2]) > 1e-6:
                                    pca_ori = list(raw_ori)
                                    node.get_logger().info(
                                        f"📐 姿态预规划: PCA 短轴对齐姿态 (来自视觉节点 list)")
                        if pca_ori is not None:
                            active_ori = pca_ori
                        else:
                            active_ori = getattr(node, 'last_successful_orientation', pick_fallback_ori)
                            if raw_ori is None:
                                node.get_logger().info("📐 姿态预规划: 径向朝下姿态 (无 PCA 数据)")
                            else:
                                node.get_logger().info("📐 姿态预规划: 径向朝下姿态 (PCA 数据无效)")

                    # 【第一步】 抓取过渡(防止碰桌面) — IK 优先, 传入预规划姿态
                    if two_stage and not tool_axis_grasp:
                        node.get_logger().info("⚡ 两阶段模式：已在物体正上方，直接下降抓取")
                    elif not node.move_arm_pose(POSE_PICK_UP, "抓取位上方过渡点",
                                                  continuous=True, planning_mode='normal',
                                                  preferred_orientation=active_ori,
                                                  fixed_orientation=tool_axis_grasp,
                                                  approach_target_z=POSE_PICK['z'] if tool_axis_grasp else None,
                                                  auto_grasp=tool_axis_grasp and not use_graspnet):
                        success = False; break
                    if tool_axis_grasp:
                        active_ori = node.last_successful_orientation
                        POSE_PICK_UP.update(node.last_approach_pose)
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
                    node._set_conveyor_collision_allowed(allowed=True)

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
                                old_x, old_y, old_z = (POSE_PICK[k] for k in ('x', 'y', 'z'))
                                node._grasp_geometry = fresh_pose.get('grasp_geometry')
                                for p in (POSE_PICK, POSE_PICK_UP, POSE_LIFT_SOFT):
                                    p['x'] = fresh_pose['x']
                                    p['y'] = fresh_pose['y']
                                POSE_PICK['z'] = fresh_pose['z']
                                POSE_PICK_UP['z'] = fresh_pose['z'] + approach_height
                                POSE_LIFT_SOFT['z'] = fresh_pose['z'] + 0.035
                                node.get_logger().info(
                                    f"📡 重试: 观察位坐标 "
                                    f"({old_x:.3f},{old_y:.3f},{old_z:.3f}) → "
                                    f"({fresh_pose['x']:.3f},{fresh_pose['y']:.3f},{fresh_pose['z']:.3f})")
                            else:
                                node.get_logger().warn(
                                    "⚠️ 重试: 未获取到新坐标, 使用原始坐标继续")
                            # 重新计算姿态 (坐标变了, 径向朝下方向也变了)
                            if not tool_axis_grasp:
                                pick_fallback_ori = node._build_pick_orientation(POSE_PICK)[0]
                                active_ori = getattr(node, 'last_successful_orientation', pick_fallback_ori)
                            node.get_logger().info(f'{node._active_camera}重试: 重新选择工具轴接近姿态' if tool_axis_grasp
                                                   else '📐 重试: 姿态已按新坐标重新计算')
                            # 从观察位移到新 POSE_PICK_UP (重走第一步)
                            node.get_logger().info("📡 重试: 移动到新坐标上方...")
                            if not node.move_arm_pose(POSE_PICK_UP, f"重试{retry+1}-上方过渡点",
                                                      continuous=False, planning_mode='normal',
                                                      preferred_orientation=active_ori,
                                                      fixed_orientation=tool_axis_grasp,
                                                      approach_target_z=POSE_PICK['z'] if tool_axis_grasp else None,
                                                      auto_grasp=tool_axis_grasp and not use_graspnet):
                                node.get_logger().warn("⚠️ 重试: 上方过渡点移动失败, 跳过本次重试")
                                continue
                            if tool_axis_grasp:
                                active_ori = node.last_successful_orientation
                                POSE_PICK_UP.update(node.last_approach_pose)
                            # 记录新过渡点关节角 + 禁用碰撞
                            pick_up_joints = (node.current_joints.copy()
                                              if node.current_joints else None)
                            node._set_conveyor_collision_allowed(allowed=True)
                            node.get_logger().info("📡 重试: 已到达新坐标上方, 准备下降")
                            if node._check_emergency(f"retry{retry+1}-上方过渡点"):
                                success = False; break

                        # 【第二步】双相机普通水果抓取共用固定姿态、工具轴接近。
                        descent_success = False
                        if tool_axis_grasp:
                            node.get_logger().info(f'{node._active_camera}沿夹爪工具+Z轴直线接近，保持抓取姿态')
                            descent_success = node._execute_fixed_tcp_descent(
                                POSE_PICK, active_ori, f'{node._active_camera}工具轴接近,retry{retry+1}',
                                extra_depth=getattr(node, 'last_grasp_extra_depth', 0.0),
                                per_point_dur=0.12 if use_graspnet else 0.05)
                            if descent_success:
                                cycle_strategies.append('fixed_tcp_waypoints')
                                cycle_profiles.append('pinocchio_full_pose')
                        if not tool_axis_grasp and node.enable_ik and node.ik_solver is not None:
                            pick_orientations = [active_ori] + node._build_pick_orientations_multi(POSE_PICK)
                            ik_ok, ik_joints = node.move_arm_via_ik(POSE_PICK, pick_orientations, "下降抓取 (IK关节空间)", continuous=True)
                            if ik_ok:
                                descent_success = True
                                cycle_strategies.append('ik_joint_space')
                                cycle_profiles.append('ik_solution')
                                node.get_logger().info("✅ IK 关节空间下降完成 (joint6 固定)")

                        if not descent_success and not tool_axis_grasp:
                            # 未接入工具轴流程的GraspNet分支保留原有笛卡尔兜底。
                            node.get_logger().info('🟡 IK失败, 回退到笛卡尔直线插补...')
                            pose_pick_msg = node._create_pose(POSE_PICK, active_ori)
                            if node.execute_cartesian_path([pose_pick_msg], f"下降抓取 (直线插补,retry{retry+1})", fraction_threshold=0.50):
                                descent_success = True

                        if not descent_success and not tool_axis_grasp:
                            node.get_logger().info("🟡 直线插补+IK均受限！启动多路并发退避规划...")
                            if not node.move_arm_cartesian(POSE_PICK, f"下降抓取 (退避规划,retry{retry+1})", continuous=True, preferred_orientation=active_ori, allow_position_only_fallback=True, planning_mode='descend'):
                                continue  # 下降失败跳过本次重试
                            if node.last_planning_strategy:
                                cycle_strategies.append(node.last_planning_strategy)
                            if node.last_planning_profile_name:
                                cycle_profiles.append(node.last_planning_profile_name)

                        if not descent_success:
                            success = False
                            break

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

                    # 【第五步】 关节空间直接移动到放置目标点 (抓取抬起后直达, 无需中间途径点)
                    # ⚠️ continuous=False: 到达后等 0.15s 稳定, 再开夹爪
                    node._set_conveyor_collision_allowed(allowed=True)
                    via_ok = True
                    for index, via in enumerate(placement['via_joints_rad'] if placement else []):
                        if not node.move_arm_joint(via, f'放置途经点 {index+1}', continuous=False) or node._check_emergency('放置途经点'):
                            via_ok = False; break
                    if not via_ok:
                        success = False; break
                    if not node.move_arm_joint(JOINT_PLACE_TARGET, f"移动到{place_id} (关节空间)", continuous=False):
                        success = False; break
                    place_reached_ok = True
                    if node._check_emergency("step5-放置点"):
                        success = False; break

                    # 【第六步】 松开夹爪释放物品
                    node.operate_gripper(GRIPPER_PLACE_OPEN, f"松开夹爪(在{place_id}释放)")
                    time.sleep(GRIPPER_SETTLE_SEC)
                    if node._check_emergency("step6-松开夹爪"):
                        success = False; break

                    # 【第七步】 关节空间回到待机位 (等待下一次抓取)
                    if not node.move_arm_joint(normal_return, f"释放后返回参考位 (关节空间)", continuous=True):
                        success = False; break
                    if node._check_emergency("step7-回待机位"):
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
                    
                    if node.workbench_interrupted():
                        node.is_busy = False
                        continue
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
                        target_joints=JOINT_PLACE_TARGET,
                        grasp_ok=False,
                        error_code=99,
                    )

                    # 安全回到待机位后恢复碰撞检测 (无论回待机位是否成功都恢复，避免影响下一轮)
                    node._set_conveyor_collision_allowed(allowed=False)
                    node.operate_gripper(GRIPPER_OPEN, "张开夹爪(待机)")
                    
                    time.sleep(0.5)
                    continue

                # 成功完成分拣后恢复碰撞检测
                node._set_conveyor_collision_allowed(allowed=False)

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
                    target_joints=JOINT_PLACE_TARGET,
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

            elif data.get("cmd") == "sort_dynamic":
                # 动态抓取: 传送带持续运行, 服务端自主连续循环拦截移动物体 (统一放置到同一目标点).
                # 依赖: conveyor_calib.json (方向标定) + /conveyor_status (STM32 速度)
                # 流程: 检测工作区有物 → 抓放 → 再检测；暂时无目标交回GUI等待，不退出动态模式。
                # 达单批上限后GUI可继续派发；失败/中断不追加运动。
                node.is_busy = True
                status_msg.data = 'busy'
                node.status_pub.publish(status_msg)
                node.last_planning_profile_name = ''
                node.last_planning_strategy = ''
                success_all = True
                grab_count = 0
                fail_count = 0
                total_fails = 0
                failed_tracks = []
                attempted_tracks = set()
                try:
                    while rclpy.ok() and not node._emergency_flag and not node.workbench_interrupted() and not node._pause_after_cycle:
                        # 安全护栏: 达连续抓取上限即返回待机让位 (供 GUI 停止/急停).
                        if grab_count >= DYNAMIC_MAX_CONSECUTIVE:
                            node.get_logger().info(
                                f"⏹️ 动态续抓达上限 {DYNAMIC_MAX_CONSECUTIVE}, 返回待机让位")
                            break
                        # 暂时无目标时交回GUI检测循环，动态模式继续等待，不报失败。
                        rclpy.spin_once(node, timeout_sec=0.02)
                        if node._pause_after_cycle or node.workbench_interrupted():break
                        det = node._latest_detection or {}
                        if data.get('motion_session') and data['motion_session'] != det.get('motion_session'):
                            raise ValueError('视觉会话已变化，结束本次动态任务')
                        # 一次失败后不再锁定 GUI 最初指定的目标: 该物体可能已经越过可达区,
                        # 交给服务端在剩余候选里挑下一个 (通常是更靠上方的物体).
                        tgt = node._dynamic_pick_next_target(node, attempted_tracks,
                            data.get('target_track_id') if (grab_count == 0 and fail_count == 0) else None)
                        if tgt is None:
                            # 空场时先回准备位(=上一轮待命关节)，下一个物体来了就不用从放置点长途起飞。
                            ready = getattr(node, '_dynamic_ready_joints', None)
                            if ready is not None:
                                try:
                                    node.move_arm_joint(list(ready), '动态抓取：回到准备位等待下一物体')
                                except Exception as exc:
                                    node.get_logger().warn(f'回到准备位失败: {exc}')
                            break
                        ok = node._run_dynamic_grasp(node, tgt)
                        attempted_tracks.add(tgt['track_id'])
                        if ok:
                            grab_count += 1
                            fail_count = 0
                        else:
                            # 本轮失败只放弃这个目标, 动态模式与传送带继续运行, 等下一个物体.
                            fail_count += 1
                            total_fails += 1
                            failed_tracks.append(tgt['track_id'])
                            reason = getattr(node, '_last_dynamic_error', '') or '本轮未成功'
                            node.get_logger().warn(
                                f"⚠️ 动态本轮失败（{fail_count}/{DYNAMIC_MAX_ATTEMPT_FAILS}）：{reason}；"
                                f"已放弃该目标，继续等待下一物体")
                        if fail_count >= DYNAMIC_MAX_ATTEMPT_FAILS:
                            node.get_logger().warn(
                                f"⏹️ 动态连续失败达上限 {DYNAMIC_MAX_ATTEMPT_FAILS}, 返回待机等待下一物体")
                            break
                        # 消化检测回调, 拉取最新视觉
                        rclpy.spin_once(node, timeout_sec=0.05)
                except Exception as e:
                    print(f"❌ 动态抓取异常: {e}")
                    import traceback; traceback.print_exc()
                    success_all = False
                finally:
                    # 动态失败/中断只取消，不追加回零、回待机或松爪；正常放置后也直接待命。
                    node._dynamic_tracker = None
                    if node._emergency_flag:
                        success_all = False
                # 单轮抓取失败不再让 GUI 退出动态模式: 只上报本轮未成功, 传送带保持运行.
                result = {'ok': success_all,
                          'attempt_failed': success_all and total_fails > 0,
                          'failed_track_ids': failed_tracks,
                          'waiting_for_target': success_all and grab_count == 0 and total_fails == 0,
                          'cmd': 'sort_dynamic',
                          'cycle_id': data.get('cycle_id'),
                          'msg': f'连续抓取 {grab_count} 个' if success_all and grab_count > 0
                                  else ('动态抓取异常' if not success_all
                                        else (f'本轮失败 {total_fails} 次, 继续等待下一物体'
                                              if total_fails > 0 else '等待下一物体'))}
                result_msg = String()
                result_msg.data = json.dumps(result)
                node.cycle_result_pub.publish(result_msg)
                node.is_busy = False
                status_msg.data = 'idle' if result['ok'] else 'error'
                node.status_pub.publish(status_msg)
                loop_count += 1

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
                    'JOINT_PLACE_TARGET': JOINT_PLACE_TARGET,
                    'GRIPPER_PLACE_OPEN': GRIPPER_PLACE_OPEN,
                    'GRIPPER_OPEN': GRIPPER_OPEN,
                    'GRIPPER_SETTLE_SEC': GRIPPER_SETTLE_SEC,
                    'GRIPPER_PICK_Z_OFFSET': GRIPPER_PICK_Z_OFFSET,
                    'MIN_GRASP_Z': MIN_GRASP_Z,
                    'loop_count': loop_count,
                }
                node._execute_blue_block_sort(data, blue_block_config)

    except KeyboardInterrupt:
        print("\n⏹️ 收到 Ctrl+C，停止动作程序...")
        try:
            if not (os.environ.get("PIPER_WORKBENCH_MANAGED")=="1" or getattr(node,"_workbench_task",False) or node.workbench_interrupted()):
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
