#!/usr/bin/env python3
"""
Joint-space sorting action node for ROS 2.

This node replaces the Cartesian-path centric sorter with a joint-space planning
pipeline:
- /sorting_cmds subscription
- /sorting_status and /sorting/cycle_result publishing
- gripper control keeps the original priority order:
  FollowJointTrajectory action first, fallback to /control/joint_states
- all large motions use joint-space planning
- only terminal IK is used to convert each target Pose into a joint target

The IK strategy uses MoveIt's compute_ik service with multiple tool-Z yaw candidates.
The chosen solution is the one with the smallest weighted motion from current joints.
"""

import json
import math
import queue
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from rclpy.action import ActionClient
from rclpy.logging import LoggingSeverity, set_logger_level
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header, String
from trajectory_msgs.msg import JointTrajectoryPoint

from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    RobotState,
    RobotTrajectory,
    OrientationConstraint,
)
from moveit_msgs.srv import GetMotionPlan, GetPositionIK


class MoveGroupInterface:
    """Small ROS 2 wrapper that mimics the common MoveGroupInterface workflow."""

    def __init__(self, node: Node, group_name: str = 'arm', base_frame: str = 'base_link', ee_link: str = 'link6'):
        self.node = node
        self.group_name = group_name
        self.base_frame = base_frame
        self.ee_link = ee_link
        self.target_joint_names = [f'joint{i}' for i in range(1, 7)]
        self.target_joint_values: Optional[List[float]] = None
        self._last_trajectory: Optional[RobotTrajectory] = None

        self._ik_client = self.node.create_client(GetPositionIK, '/compute_ik')
        self._plan_client = self.node.create_client(GetMotionPlan, '/plan_kinematic_path')
        self._execute_client = ActionClient(self.node, ExecuteTrajectory, '/execute_trajectory')

        self.joint_lower_limits = [-math.pi] * 6
        self.joint_upper_limits = [math.pi] * 6

    def wait_for_servers(self, timeout_sec: float = 5.0) -> bool:
        ok = True
        ok = self._ik_client.wait_for_service(timeout_sec=timeout_sec) and ok
        ok = self._plan_client.wait_for_service(timeout_sec=timeout_sec) and ok
        ok = self._execute_client.wait_for_server(timeout_sec=timeout_sec) and ok
        return ok

    def set_joint_value_target(self, joint_values: Sequence[float]) -> None:
        self.target_joint_values = [float(v) for v in joint_values]

    def _current_joint_map(self) -> Dict[str, float]:
        current = self.node.current_joints or {}
        return dict(current)

    def _build_robot_state(self, seed_joints: Optional[Sequence[float]] = None) -> RobotState:
        state = RobotState()
        state.joint_state.name = list(self.target_joint_names)
        if seed_joints is None:
            current = self._current_joint_map()
            state.joint_state.position = [float(current.get(name, 0.0)) for name in self.target_joint_names]
        else:
            state.joint_state.position = [float(v) for v in seed_joints]
        return state

    def get_ik(self, target_pose: Pose, seed_joints: Optional[Sequence[float]] = None) -> Optional[List[float]]:
        if not self._ik_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().error('❌ /compute_ik 服务不可用')
            return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.ee_link
        req.ik_request.pose_stamped = PoseStamped(
            header=Header(frame_id=self.base_frame, stamp=self.node.get_clock().now().to_msg()),
            pose=target_pose,
        )
        req.ik_request.robot_state = self._build_robot_state(seed_joints)
        req.ik_request.timeout = Duration(sec=3, nanosec=0)
        req.ik_request.avoid_collisions = False

        future = self._ik_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        res = future.result()
        if res is None:
            self.node.get_logger().warning('⚠️ IK 返回空结果')
            return None

        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.node.get_logger().warning(f'⚠️ IK 失败: error_code={res.error_code.val}')
            return None

        joint_map = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        joint_values = []
        for name in self.target_joint_names:
            if name not in joint_map:
                return None
            joint_values.append(float(joint_map[name]))

        if not self._within_joint_limits(joint_values):
            return None
        return joint_values

    def _within_joint_limits(self, joint_values: Sequence[float]) -> bool:
        for idx, value in enumerate(joint_values):
            lower = self.joint_lower_limits[idx]
            upper = self.joint_upper_limits[idx]
            if value < lower or value > upper:
                return False
        return True

    def _build_goal_constraints(self, joint_values: Sequence[float]) -> Constraints:
        constraints = Constraints()
        for name, value in zip(self.target_joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return constraints

    def plan(self) -> Optional[RobotTrajectory]:
        if self.target_joint_values is None:
            self.node.get_logger().error('❌ 未设置关节目标')
            return None
        if not self._plan_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error('❌ /plan_kinematic_path 服务不可用')
            return None

        req = GetMotionPlan.Request()
        motion_request: MotionPlanRequest = req.motion_plan_request
        motion_request.workspace_parameters.header.frame_id = self.base_frame
        motion_request.workspace_parameters.header.stamp = self.node.get_clock().now().to_msg()
        motion_request.group_name = self.group_name
        motion_request.num_planning_attempts = 1
        motion_request.allowed_planning_time = 0.4
        motion_request.max_velocity_scaling_factor = 0.8
        motion_request.max_acceleration_scaling_factor = 0.8
        motion_request.start_state = self._build_robot_state()
        motion_request.goal_constraints.append(self._build_goal_constraints(self.target_joint_values))

        future = self._plan_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        res = future.result()
        if res is None or res.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None

        trajectory = res.motion_plan_response.trajectory
        self._last_trajectory = trajectory
        return trajectory

    def adjust_trajectory_start(self, trajectory: RobotTrajectory) -> RobotTrajectory:
        if trajectory is None or trajectory.joint_trajectory is None:
            return trajectory

        points = trajectory.joint_trajectory.points
        if not points:
            return trajectory

        current = self._current_joint_map()
        if not current:
            return trajectory

        pt0 = points[0]
        pt0.time_from_start.sec = 0
        pt0.time_from_start.nanosec = 0
        if len(pt0.positions) == len(trajectory.joint_trajectory.joint_names):
            for idx, name in enumerate(trajectory.joint_trajectory.joint_names):
                if name in current:
                    pt0.positions[idx] = float(current[name])
        if pt0.velocities:
            pt0.velocities = [0.0] * len(pt0.velocities)
        if pt0.accelerations:
            pt0.accelerations = [0.0] * len(pt0.accelerations)
        return trajectory

    def execute(self, trajectory: RobotTrajectory, wait: bool = True) -> bool:
        if trajectory is None:
            return False
        if not self._execute_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error('❌ /execute_trajectory 动作服务器不可用')
            return False

        goal = ExecuteTrajectory.Goal()
        trajectory.joint_trajectory.header.stamp.sec = 0
        trajectory.joint_trajectory.header.stamp.nanosec = 0
        goal.trajectory = trajectory

        future = self._execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.node.get_logger().error('❌ 轨迹被拒绝执行')
            return False

        result_future = handle.get_result_async()
        if wait:
            rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result()
        if result is None:
            return False
        return result.result.error_code.val == MoveItErrorCodes.SUCCESS


class JointSpaceSortingAction(Node):
    def grasp_orientations_from_pose_dict(self, pose_dict: dict) -> list:
        """
        参照 auto_sorting_action.py 的 _build_pick_orientation，输入 dict，输出两个正反抓取四元数
        """
        z_axis = [0.0, 0.0, -1.0]
        radial_xy = [float(pose_dict['x']), float(pose_dict['y']), 0.0]
        x_axis_1 = self._normalize(radial_xy)
        if abs(x_axis_1[0]) < 1e-6 and abs(x_axis_1[1]) < 1e-6:
            x_axis_1 = [1.0, 0.0, 0.0]
        y_axis_1 = self._cross(z_axis, x_axis_1)
        y_axis_1 = self._normalize(y_axis_1)
        # 构造正向四元数
        rot1 = np.array([
            [x_axis_1[0], y_axis_1[0], z_axis[0]],
            [x_axis_1[1], y_axis_1[1], z_axis[1]],
            [x_axis_1[2], y_axis_1[2], z_axis[2]],
        ], dtype=float)
        quat1 = self._matrix_to_quat(rot1)
        # 反向180度抓取姿态(规避腕关节限位)
        x_axis_2 = [-x for x in x_axis_1]
        y_axis_2 = [-y for y in y_axis_1]
        rot2 = np.array([
            [x_axis_2[0], y_axis_2[0], z_axis[0]],
            [x_axis_2[1], y_axis_2[1], z_axis[1]],
            [x_axis_2[2], y_axis_2[2], z_axis[2]],
        ], dtype=float)
        quat2 = self._matrix_to_quat(rot2)
        return [quat1, quat2]

    def __init__(self):
        super().__init__('joint_space_sorting_action')
        self._suppress_tf_old_data_logs()

        self.move_group = MoveGroupInterface(self, group_name='arm', base_frame='base_link', ee_link='link6')

        self.current_joints = {}
        self.cmd_queue = queue.Queue()
        self.cmd_sub = self.create_subscription(String, '/sorting_cmds', self.cmd_callback, 10)
        self.status_pub = self.create_publisher(String, '/sorting_status', 10)
        self.cycle_result_pub = self.create_publisher(String, '/sorting/cycle_result', 10)

        self._joint_states_sub = self.create_subscription(JointState, '/joint_states', self._joint_states_cb, 10)
        self._control_joint_states_sub = self.create_subscription(JointState, '/control/joint_states', self._joint_states_cb, 10)

        self._gripper_action_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self._joint_states_pub = self.create_publisher(JointState, '/control/joint_states', 10)

        self.is_busy = False
        self.last_planning_profile_name = ''
        self.last_planning_strategy = ''
        self.last_ik_candidate_name = ''

        self.gripper_joint_names = ['gripper_joint1', 'gripper_joint2']
        self.joint_names = [f'joint{i}' for i in range(1, 7)]
        self.joint_standby = [0.0] * 6

        self.ik_yaw_candidates_deg = [0.0, 15.0, -15.0, 30.0, -30.0]
        self.joint_space_retry_wait_sec = 0.05
        self.enable_joint_space_all_moves = True

    def _suppress_tf_old_data_logs(self):
        for logger_name in ('tf2', 'tf2_ros', 'tf2_buffer', 'tf2_ros_buffer'):
            try:
                set_logger_level(logger_name, LoggingSeverity.ERROR)
            except Exception:
                pass

    def _joint_states_cb(self, msg: JointState):
        if self.current_joints is None:
            self.current_joints = {}
        self.current_joints.update(dict(zip(msg.name, msg.position)))

    def cmd_callback(self, msg: String):
        self.cmd_queue.put(msg.data)

    def wait_for_ready(self, timeout_sec: float = 10.0) -> bool:
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            if self.current_joints:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.current_joints:
            self.get_logger().warn('⚠️ 尚未收到 /joint_states，仍然继续启动，但轨迹起点校准会降级')
        return self.move_group.wait_for_servers(timeout_sec=timeout_sec)

    def _normalize(self, v: Sequence[float]) -> List[float]:
        n = math.sqrt(float(v[0]) * float(v[0]) + float(v[1]) * float(v[1]) + float(v[2]) * float(v[2]))
        if n < 1e-9:
            return [1.0, 0.0, 0.0]
        return [float(v[0]) / n, float(v[1]) / n, float(v[2]) / n]

    def _cross(self, a: Sequence[float], b: Sequence[float]) -> List[float]:
        return [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]

    def _quat_normalize(self, q: Sequence[float]) -> List[float]:
        n = math.sqrt(sum(float(x) * float(x) for x in q))
        if n < 1e-9:
            return [0.0, 0.0, 0.0, 1.0]
        return [float(x) / n for x in q]

    def _quat_multiply(self, q1: Sequence[float], q2: Sequence[float]) -> List[float]:
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]

    def _quat_from_axis_angle(self, axis: Sequence[float], angle_rad: float) -> List[float]:
        axis_n = self._normalize(axis)
        half = angle_rad * 0.5
        s = math.sin(half)
        return [axis_n[0] * s, axis_n[1] * s, axis_n[2] * s, math.cos(half)]

    def _quat_to_matrix(self, q: Sequence[float]) -> np.ndarray:
        x, y, z, w = self._quat_normalize(q)
        return np.array([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ], dtype=float)

    def _matrix_to_quat(self, m: np.ndarray) -> List[float]:
        trace = float(m[0, 0] + m[1, 1] + m[2, 2])
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (m[2, 1] - m[1, 2]) / s
            qy = (m[0, 2] - m[2, 0]) / s
            qz = (m[1, 0] - m[0, 1]) / s
        elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            qw = (m[2, 1] - m[1, 2]) / s
            qx = 0.25 * s
            qy = (m[0, 1] + m[1, 0]) / s
            qz = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            qw = (m[0, 2] - m[2, 0]) / s
            qx = (m[0, 1] + m[1, 0]) / s
            qy = 0.25 * s
            qz = (m[1, 2] + m[2, 1]) / s
        else:
            s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            qw = (m[1, 0] - m[0, 1]) / s
            qx = (m[0, 2] + m[2, 0]) / s
            qy = (m[1, 2] + m[2, 1]) / s
            qz = 0.25 * s
        return self._quat_normalize([qx, qy, qz, qw])

    def _build_grasp_base_quaternion(self, pose: Pose) -> List[float]:
        z_axis = [0.0, 0.0, -1.0]
        radial_xy = [float(pose.position.x), float(pose.position.y), 0.0]
        x_axis = self._normalize(radial_xy)
        if abs(x_axis[0]) < 1e-6 and abs(x_axis[1]) < 1e-6:
            x_axis = [1.0, 0.0, 0.0]
        y_axis = self._normalize(self._cross(z_axis, x_axis))
        x_axis = self._normalize(self._cross(y_axis, z_axis))
        rot = np.array([
            [x_axis[0], y_axis[0], z_axis[0]],
            [x_axis[1], y_axis[1], z_axis[1]],
            [x_axis[2], y_axis[2], z_axis[2]],
        ], dtype=float)
        return self._matrix_to_quat(rot)

    def _build_pose_from_dict(self, pose_dict: Dict[str, float], z_offset: float = 0.0) -> Pose:
        pose = Pose()
        pose.position.x = float(pose_dict['x'])
        pose.position.y = float(pose_dict['y'])
        pose.position.z = float(pose_dict['z']) + float(z_offset)
        return pose

    def _candidate_orientations(self, pose: Pose, pose_dict: dict = None) -> List[Quaternion]:
        """
        只返回正反两个抓取姿态，参照 auto_sorting_action.py 的 _build_pick_orientation
        pose_dict 必须传入 {'x','y','z'}
        """
        if pose_dict is None:
            # 从 pose 转 dict
            pose_dict = {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z}
        quat_list = self.grasp_orientations_from_pose_dict(pose_dict)
        return [Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]) for q in quat_list]

    def _pose_with_quaternion(self, pose: Pose, quat: Quaternion) -> Pose:
        out = Pose()
        out.position.x = float(pose.position.x)
        out.position.y = float(pose.position.y)
        out.position.z = float(pose.position.z)
        out.orientation = Quaternion(x=float(quat.x), y=float(quat.y), z=float(quat.z), w=float(quat.w))
        return out

    def ik_with_candidates(self, target_pose: Pose, pose_dict: dict = None) -> Optional[List[float]]:
        """
        只尝试正反两个抓取姿态，参照 auto_sorting_action.py
        """
        candidates = self._candidate_orientations(target_pose, pose_dict)
        current = self.current_joints or {}
        seed = [float(current.get(name, 0.0)) for name in self.joint_names]

        joint_weights = [1.0, 1.0, 1.0, 0.8, 0.8, 0.6]
        joint_limit_margin_rad = 0.05
        joint_limit_penalty = 10.0

        best_solution = None
        best_cost = None
        best_name = ''
        best_idx = -1

        def limit_penalty(solution: Sequence[float]) -> float:
            penalty = 0.0
            for idx, value in enumerate(solution):
                lower = self.move_group.joint_lower_limits[idx]
                upper = self.move_group.joint_upper_limits[idx]
                if value - lower < joint_limit_margin_rad:
                    penalty += joint_limit_penalty
                if upper - value < joint_limit_margin_rad:
                    penalty += joint_limit_penalty
            return penalty

        for cand_idx, quat in enumerate(candidates):
            pose_try = self._pose_with_quaternion(target_pose, quat)
            solution = self.move_group.get_ik(pose_try, seed_joints=seed)
            if solution is None:
                continue

            move_cost = 0.0
            for j_idx, joint_name in enumerate(self.joint_names):
                cur = float(current.get(joint_name, 0.0))
                move_cost += joint_weights[j_idx] * abs(float(solution[j_idx]) - cur)

            total_cost = move_cost + limit_penalty(solution)

            if best_cost is None or total_cost < best_cost:
                best_cost = total_cost
                best_solution = solution
                best_idx = cand_idx
                best_name = f'cand_{cand_idx}'

        self.last_ik_candidate_name = best_name
        if best_solution is not None:
            self.get_logger().info(
                f'✅ IK候选命中: idx={best_idx}, cost={best_cost:.3f}'
            )
        return best_solution

    def move_arm_joint_space(self, target_pose: Pose) -> bool:
        joint_angles = self.ik_with_candidates(target_pose)
        if joint_angles is None:
            self.get_logger().error('❌ 终点 IK 失败，无法生成关节空间目标')
            return False

        self.move_group.set_joint_value_target(joint_angles)
        trajectory = self.move_group.plan()
        if trajectory is None:
            self.get_logger().error('❌ 关节空间规划失败')
            return False

        trajectory = self.move_group.adjust_trajectory_start(trajectory)
        ok = self.move_group.execute(trajectory, wait=True)
        if not ok:
            self.get_logger().error('❌ 轨迹执行失败')
            return False
        time.sleep(self.joint_space_retry_wait_sec)
        return True

    def move_arm_joint(self, joints: Sequence[float], desc: str, continuous: bool = False) -> bool:
        print(f'\n🚀 正在规划(关节空间) -> {desc}')
        self.move_group.set_joint_value_target(joints)
        trajectory = self.move_group.plan()
        if trajectory is None:
            self.get_logger().error('❌ 关节目标规划失败')
            return False
        trajectory = self.move_group.adjust_trajectory_start(trajectory)
        ok = self.move_group.execute(trajectory, wait=True)
        if ok:
            time.sleep(0.05 if continuous else 0.35)
            return True
        return False

    def _try_gripper_action(self, joint_names: Sequence[str], positions: Sequence[float], timeout_sec: float = 1.0) -> bool:
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
        return result is not None

    def _publish_gripper_joint_state(self, target_pos: float):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['gripper']
        msg.position = [float(target_pos)]
        msg.velocity = []
        msg.effort = [1.5]
        self._joint_states_pub.publish(msg)

    def operate_gripper(self, target_pos: float, desc: str) -> bool:
        print(f'✊ 正在执行夹爪动作 -> {desc} (target: {target_pos})')

        if self._try_gripper_action(
            ['gripper_joint1', 'gripper_joint2'],
            [float(target_pos * 0.5), float(-target_pos * 0.5)],
            timeout_sec=0.8,
        ):
            self.get_logger().info('⭕ 夹爪动作执行中（gripper_joint1/2）...')
            time.sleep(0.15)
            return True

        if self._try_gripper_action(
            ['joint7'],
            [float(target_pos * 0.5)],
            timeout_sec=0.4,
        ):
            self.get_logger().info('⭕ 夹爪动作执行中（joint7）...')
            time.sleep(0.15)
            return True

        self.get_logger().warn('⚠️ 夹爪 action 不可用或被拒绝，回退到 /control/joint_states')
        self._publish_gripper_joint_state(target_pos)
        time.sleep(0.15)
        return True

    def _compute_gripper_targets(self, object_diameter_m: Optional[float]) -> Tuple[float, float]:
        gripper_open = 0.090
        gripper_close = 0.060
        if object_diameter_m is None:
            return gripper_open, gripper_close
        d = float(object_diameter_m)
        d = max(0.040, min(0.085, d))
        open_target = max(0.060, min(0.095, d + 0.020))
        close_target = max(0.036, min(0.080, d - 0.006))
        if close_target >= open_target:
            close_target = max(0.038, open_target - 0.006)
        return open_target, close_target

    def _recover_to_standby(self, standby_joints: Sequence[float], gripper_grab: float):
        try:
            self.operate_gripper(0.090, '防碰撞提前张开夹爪')
        except Exception:
            pass
        try:
            self.move_arm_joint(standby_joints, '自动回到待机点')
        except Exception:
            pass
        try:
            self.operate_gripper(gripper_grab, '闭合夹爪(待机)')
        except Exception:
            pass

    def _publish_result(self, cycle_success: int, cycle_id: str, grasp_reached_ok: int, place_reached_ok: int,
                        planning_strategy: str, planning_profile: str, planning_profiles: List[str],
                        object_diameter_m: Optional[float], gripper_open_target: float, gripper_close_target: float):
        result_msg = String()
        result_msg.data = json.dumps({
            'cycle_success': cycle_success,
            'cycle_id': cycle_id,
            'grasp_reached_ok': int(grasp_reached_ok),
            'place_reached_ok': int(place_reached_ok),
            'planning_strategy': planning_strategy,
            'planning_profile': planning_profile,
            'planning_profiles': planning_profiles,
            'object_diameter_m': object_diameter_m,
            'gripper_open_target': round(float(gripper_open_target), 4),
            'gripper_close_target': round(float(gripper_close_target), 4),
        }, ensure_ascii=False)
        self.cycle_result_pub.publish(result_msg)

    def handle_sort(self, data: Dict) -> bool:
        self.is_busy = True
        status_msg = String()
        status_msg.data = 'busy'
        self.status_pub.publish(status_msg)

        self.last_planning_profile_name = ''
        self.last_planning_strategy = ''
        cycle_profiles: List[str] = []
        cycle_strategies: List[str] = []

        object_diameter_m = data.get('object_diameter_m', None)
        dynamic_open, dynamic_close = self._compute_gripper_targets(object_diameter_m)

        pick_pose_dict = data['pick']
        place_pose_dict = data['place']
        pick_id = data.get('pick_name', 'Pick')
        place_id = data.get('place_name', 'Place')
        cycle_id = str(data.get('cycle_id', ''))

        pose_pick = self._build_pose_from_dict(pick_pose_dict, z_offset=0.05)
        pose_pick_up = self._build_pose_from_dict(pick_pose_dict, z_offset=0.05 + 0.13)
        pose_pick_soft_lift = self._build_pose_from_dict(pick_pose_dict, z_offset=0.05 + 0.035)

        pose_place = self._build_pose_from_dict(place_pose_dict, z_offset=0.05)
        pose_place_pre = self._build_pose_from_dict(place_pose_dict, z_offset=0.05 + 0.08)
        pose_place_up = self._build_pose_from_dict(place_pose_dict, z_offset=0.05 + 0.13)

        loop_count = int(data.get('loop_count', 0)) + 1
        print(f'\n\n====================== 第 {loop_count} 次分拣 (从 {pick_id} 到 {place_id}) ======================')

        while not self.cmd_queue.empty():
            try:
                self.cmd_queue.get_nowait()
            except queue.Empty:
                break

        success = True
        grasp_reached_ok = False
        place_reached_ok = False

        # 1) open gripper
        self.operate_gripper(dynamic_open, '张开夹爪(准备抓取)')

        # 2) move above pick
        if not self.move_arm_joint_space(pose_pick_up):
            success = False
        else:
            cycle_strategies.append('joint_space_ik')
            cycle_profiles.append(f'pick_up_{self.last_ik_candidate_name}')

        # 3) descend to pick
        if success and not self.move_arm_joint_space(pose_pick):
            success = False
        else:
            if success:
                cycle_strategies.append('joint_space_ik')
                cycle_profiles.append(f'pick_down_{self.last_ik_candidate_name}')

        # 4) close gripper
        if success:
            self.operate_gripper(dynamic_close, '闭合夹爪(拿取)')
            time.sleep(0.30)
            regrip_target = max(0.030, dynamic_close - 0.006)
            self.operate_gripper(regrip_target, '二次补压(防滑)')
            time.sleep(0.20)

        # 5) lift
        if success:
            if not self.move_arm_joint_space(pose_pick_up):
                success = False
            else:
                grasp_reached_ok = True
                cycle_strategies.append('joint_space_ik')
                cycle_profiles.append(f'lift_{self.last_ik_candidate_name}')

        # 6) go to place pre
        if success:
            if not self.move_arm_joint_space(pose_place_pre):
                success = False
            else:
                cycle_strategies.append('joint_space_ik')
                cycle_profiles.append(f'place_pre_{self.last_ik_candidate_name}')

        # 7) descend to place
        if success:
            if not self.move_arm_joint_space(pose_place):
                success = False
            else:
                place_reached_ok = True
                cycle_strategies.append('joint_space_ik')
                cycle_profiles.append(f'place_down_{self.last_ik_candidate_name}')

        # 8) release object
        if success:
            self.operate_gripper(dynamic_open, '松开夹爪(释放)')

        # 9) retreat
        if success:
            if not self.move_arm_joint_space(pose_place_up):
                success = False
            else:
                cycle_strategies.append('joint_space_ik')
                cycle_profiles.append(f'retreat_{self.last_ik_candidate_name}')

        final_strategy = 'joint_space_ik'
        final_profile = cycle_profiles[-1] if cycle_profiles else self.last_ik_candidate_name
        if success:
            self._publish_result(
                1,
                cycle_id,
                int(grasp_reached_ok),
                int(place_reached_ok),
                final_strategy,
                final_profile,
                cycle_profiles,
                object_diameter_m,
                dynamic_open,
                dynamic_close,
            )
            print(f'🎉 第 {loop_count} 次回合顺利完成！')
            time.sleep(0.35)
            return True

        print('\n⚠️ 执行失败！启动自动回退防护...')
        status_msg = String()
        status_msg.data = 'error'
        self.status_pub.publish(status_msg)
        self._publish_result(
            0,
            cycle_id,
            int(grasp_reached_ok),
            int(place_reached_ok),
            final_strategy,
            final_profile,
            cycle_profiles,
            object_diameter_m,
            dynamic_open,
            dynamic_close,
        )
        self._recover_to_standby(self.joint_standby, 0.0)
        time.sleep(0.35)
        return False


def main():
    rclpy.init()
    node = JointSpaceSortingAction()
    if not node.wait_for_ready(timeout_sec=10.0):
        node.destroy_node()
        rclpy.shutdown()
        return

    print('\n\n======== 🔵 关节空间分拣服务端已启动 (等待 GUI 客户端指令) ========\n')

    # 初始运行时回到待机位
    node.move_arm_joint(node.joint_standby, '初始回到零点/待机位 (Standby)')
    node.operate_gripper(0.0, '初始化闭合夹爪(待机)')

    try:
        while rclpy.ok():
            status_msg = String()
            status_msg.data = 'idle'
            node.status_pub.publish(status_msg)

            rclpy.spin_once(node, timeout_sec=0.1)

            try:
                cmd_data_str = node.cmd_queue.get_nowait()
            except queue.Empty:
                continue

            try:
                data = json.loads(cmd_data_str)
            except Exception as exc:
                print(f'❌ 收到无法解析的指令: {cmd_data_str}, 错误: {exc}')
                continue

            cmd = data.get('cmd')
            if cmd == 'quit':
                print('\n🛑 收到来自客户端的退出指令，准备复位...')
                node.move_arm_joint(node.joint_standby, '退出前回到待机位')
                node.operate_gripper(0.0, '退出前闭合夹爪')
                break

            if cmd == 'reset':
                print('\n🔄 收到客户端指令：一键回到待机点并关闭夹爪')
                node.is_busy = True
                busy_msg = String()
                busy_msg.data = 'busy'
                node.status_pub.publish(busy_msg)
                node.move_arm_joint(node.joint_standby, '回到待机位')
                node.operate_gripper(0.0, '闭合夹爪(待机)')
                continue

            if cmd == 'sort':
                node.is_busy = True
                node.handle_sort(data)
                continue

    except KeyboardInterrupt:
        print('\n⏹️ 收到 Ctrl+C，正在复位机械臂...')
        try:
            node.move_arm_joint(node.joint_standby, '退出前回到待机位')
            node.operate_gripper(0.0, '退出前闭合夹爪')
        except Exception:
            pass
        print('⏹️ 动作停止，安全退出。')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
