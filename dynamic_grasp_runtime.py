"""D455 动态抓取：保留控制器时间戳，闭合期间继续随动。仅由 action 命令调用。"""
import json
import math
import time
import uuid

import numpy as np
import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation

import runtime_config
from dynamic_grasp import MotionTrack, moving_tcp_path, joint_spline, smoothstep, conveyor_velocity
from runtime_geometry import transform


class DynamicGraspRunner:
    def __init__(self, node, target, place_joints):
        self.n, self.target, self.place = node, target, place_joints
        root = runtime_config.workspace_root()
        self.settings = json.loads((root/'config/dynamic_grasp.json').read_text())
        cfg = self.settings
        signed = {'intercept_s_pref_m', 'intercept_s_max_m'}  # 沿带坐标原点在机械臂基座，允许为负值
        for key, value in cfg.items():
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'动态配置 {key} 必须为有限数值')
            if value <= 0 and key not in signed:
                raise ValueError(f'动态配置 {key} 必须为有限正数')
        if cfg['follow_s'] < cfg['close_s']+0.1:
            raise ValueError('follow_s 至少比 close_s 长0.1秒')
        if cfg['velocity_scaling'] > 1 or cfg['acceleration_scaling'] > 1:
            raise ValueError('动态速度/加速度比例不能超过1')
        limits = yaml.safe_load((root/'src/agx_arm_ros/src/agx_arm_moveit/config/joint_limits.yaml').read_text())['joint_limits']
        self.vmax = np.array([limits[f'joint{i}']['max_velocity'] for i in range(1, 7)])*cfg['velocity_scaling']
        self.amax = np.array([limits[f'joint{i}']['max_acceleration'] for i in range(1, 7)])*cfg['acceleration_scaling']
        # 硬件真实限值：舒适阈值只用于优先保证平滑，超了但没破硬件就不能白白丢掉目标。
        self.hard_vmax = np.array([limits[f'joint{i}']['max_velocity'] for i in range(1, 7)])
        self.hard_amax = np.array([limits[f'joint{i}']['max_acceleration'] for i in range(1, 7)])
        self.track = MotionTrack(target['track_id'], target['motion_session'])
        self.track.observe(node._latest_detection or {})
        self.handles = []
        self.stopped = False
        self.motion_anchor = None
        self.prediction_frozen = False
        self.velocity_residual_mps = 0.
        self.report = dict(target=target, settings=cfg.copy(), speed_source='STM32原值 + 正常视觉沿带残差',
                           physical_accuracy='未完成物理精度验证', preflight=[], previews=[], planning_inputs=[], plans=[], feedback=[], success=False)
        self.path = root/'data/diagnostics/dynamic_grasp'/f'{time.strftime("%Y%m%d-%H%M%S")}_{uuid.uuid4().hex[:8]}.json'

    def now(self):
        return self.n.get_clock().now().nanoseconds*1e-9

    def spin(self):
        rclpy.spin_once(self.n, timeout_sec=0.01)
        if not rclpy.ok() or self.n._emergency_flag:
            raise RuntimeError('动态抓取已中断，不再派发后续动作')

    def actual_joints(self):
        state = getattr(self.n, '_dynamic_actual_joints', None)
        if state is None or time.monotonic()-self.n._dynamic_actual_at > self.settings['feedback_max_age_s']:
            raise RuntimeError('真实关节反馈过期（/feedback/joint_states）')
        stamp = getattr(self.n, '_dynamic_actual_stamp', 0.)
        if stamp > 0 and not -0.1 <= self.now()-stamp <= self.settings['feedback_max_age_s']:
            raise RuntimeError('真实关节采样时间过期或时钟不一致')
        return np.array(state, dtype=float)

    def velocity(self):
        if not getattr(self.n, '_belt_connected', False):
            raise RuntimeError('传送带未连接')
        if getattr(self.n, '_belt_commanded_running', None) is False:
            raise RuntimeError('已收到GUI停止传送带的确认，本轮动态抓取停止')
        if time.monotonic()-getattr(self.n, '_belt_status_at', -100) > self.settings['belt_max_age_s']:
            raise RuntimeError('传送带状态过期')
        speed = self.n._belt_speed
        if not math.isfinite(speed) or speed <= 0:
            raise RuntimeError('传送带当前无正速度，动态抓取等待接带后使用')
        measured = conveyor_velocity(self.n._dynamic_calib or {}, speed, self.n._belt_direction)
        self.report['stm32_velocity_mps'] = measured.tolist()
        # 只修正本轮水果预测，不改串口读数、GUI速度或历史标定比例。
        return measured*(max(0., speed+self.velocity_residual_mps)/speed)

    def snapshot(self):
        if self.motion_anchor is not None:
            if self.track.samples:
                latest, observed = self.track.samples[-1]
                self.report['last_visual_observation_after_lock'] = [latest, observed.tolist()]
                if not self.prediction_frozen and latest > self.motion_anchor[0]:
                    velocity = self.velocity()
                    old_stamp, old_xyz = self.motion_anchor
                    radius = max(.015, .5*float(self.target.get('diameter') or .06))
                    clean = []
                    for stamp, xyz in self.track.samples:
                        predicted = old_xyz+velocity*(stamp-old_stamp)
                        # 水果在水平皮带上，夹爪遮挡常造成Z跳高；只忽略异常帧，不中止运动模型。
                        if (0 <= self.now()-stamp <= self.settings['observation_max_age_s']
                                and latest-stamp <= .4 and abs(xyz[2]-old_xyz[2]) <= .015
                                and np.linalg.norm((xyz-predicted)[:2]) <= radius):
                            clean.append((stamp, xyz))
                    if clean and clean[-1][0] > old_stamp:
                        stamp = clean[-1][0]
                        # 先按STM32速度对齐到同一图像时间再取中位数，避免普通平均带来的运动滞后。
                        aligned = [xyz[:2]+velocity[:2]*(stamp-t) for t, xyz in clean]
                        xyz = np.r_[np.median(aligned, axis=0), old_xyz[2]]
                        correction = xyz-(old_xyz+velocity*(stamp-old_stamp))
                        self.motion_anchor = (stamp, xyz)
                        self.report.setdefault('position_updates', []).append(dict(
                            stamp_s=stamp, xy_correction_mm=(correction[:2]*1000).tolist(),
                            accepted_samples=len(clean), anchor=xyz.tolist()))
                    else:
                        self.report['last_ignored_observation'] = [latest, observed.tolist()]
            stamp, xyz = self.motion_anchor
            return stamp, xyz.copy()  # 保留原采样时间，不能冒充新鲜视觉帧。
        return self.track.snapshot(self.now(), self.settings['observation_max_age_s'])

    def wait_future(self, future, timeout):
        deadline = time.monotonic()+timeout
        while not future.done():
            self.spin()
            if time.monotonic() > deadline:
                raise RuntimeError('动作通信/执行超时')
        return future.result()

    def send(self, client, goal):
        self.spin()
        if not client.wait_for_server(timeout_sec=0.5):
            raise RuntimeError('动作服务未就绪')
        trajectory = getattr(goal, 'trajectory', None)
        if trajectory is not None:
            stamp = trajectory.header.stamp.sec+trajectory.header.stamp.nanosec*1e-9
            if stamp > 0 and self.now() >= stamp-0.05:
                raise RuntimeError('发送前预测启动时刻已临近/过期，本轮不派发')
        future = client.send_goal_async(goal)
        abandoned = [False]

        def accepted_late(done):
            handle = done.result()
            if handle and handle.accepted:
                if handle not in self.handles:
                    self.handles.append(handle)
                if abandoned[0] or self.stopped or self.n._emergency_flag:
                    handle.cancel_goal_async()
        future.add_done_callback(accepted_late)
        try:
            handle = self.wait_future(future, 3.)
        except BaseException:
            abandoned[0] = True
            raise
        if handle is None or not handle.accepted:
            raise RuntimeError('动作目标被拒绝')
        if handle not in self.handles:
            self.handles.append(handle)
        self.n._active_goal_handle = handle
        return handle

    def move_joints(self, joints, description, planning_time=2.0, attempts=2, scaling=None):
        """预定位/放置走既有 MoveIt 链路，加入可取消和超时处理。"""
        self.n._check_calibrated_tool()
        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.allowed_planning_time = planning_time
        goal.request.num_planning_attempts = attempts
        goal.request.max_velocity_scaling_factor = (float(scaling) if scaling is not None
                                                    else self.settings['velocity_scaling'])
        goal.request.max_acceleration_scaling_factor = (min(1., float(scaling)*1.2) if scaling is not None
                                                        else self.settings['acceleration_scaling'])
        goal.request.start_state.is_diff = True
        goal.request.start_state.joint_state.name = [f'joint{i}' for i in range(1, 7)]
        goal.request.start_state.joint_state.position = self.actual_joints().tolist()
        constraints = Constraints()
        for i, value in enumerate(joints, 1):
            joint = JointConstraint(joint_name=f'joint{i}', position=float(value),
                                    tolerance_above=0.01, tolerance_below=0.01, weight=1.)
            constraints.joint_constraints.append(joint)
        goal.request.goal_constraints.append(constraints)
        self.n.get_logger().info(description)
        handle = self.send(self.n._action_client, goal)
        result = self.wait_future(handle.get_result_async(), 30.)
        if result.status != 4 or result.result.error_code.val != 1:
            raise RuntimeError(f'{description}失败，不继续后续动作')

    def gripper_goal(self, opening):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['gripper_joint1', 'gripper_joint2']
        point = JointTrajectoryPoint(positions=[float(opening/2), float(-opening/2)])
        self.duration(point, self.settings['close_s'])
        goal.trajectory.points.append(point)
        return goal

    @staticmethod
    def duration(point, seconds):
        ns = round(seconds*1e9)
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 1000000000)

    def gripper_wait(self, opening):
        handle = self.send(self.n._gripper_action_client, self.gripper_goal(opening))
        result = self.wait_future(handle.get_result_async(), 3.)
        if result.status != 4 or result.result.error_code != 0:
            raise RuntimeError('夹爪动作失败')

    def kinematics(self, anchor, velocity, start, initial, orientation, cfg):
        actual_pose = self.n._fixed_tcp_fk(initial)
        actual = actual_pose[:3, 3]
        initial_rotation = Rotation.from_matrix(actual_pose[:3, :3])
        rotation_delta = (Rotation.from_quat(orientation)*initial_rotation.inv()).as_rotvec()
        axis = transform([0., 0., 0.], orientation)[:3, 2]
        times, positions, close_at = moving_tcp_path(anchor, velocity, start, actual, axis, cfg)
        joints = [initial]
        for t, position in zip(times[1:], positions[1:]):
            self.spin()
            # 高位对齐时平滑消除MoveIt到位容差，接近/闭合阶段保持完整姿态不变。
            current_orientation = ((Rotation.from_rotvec(smoothstep(t/cfg['align_s'])*rotation_delta)*initial_rotation).as_quat()
                                   if t < cfg['align_s'] else orientation)
            q = self.n._fixed_tcp_ik(dict(zip(('x', 'y', 'z'), position.tolist())), current_orientation, joints[-1])
            if q is None:
                raise ValueError(f'动态IK无解，t={t:.3f}s，xyz={position.tolist()}')
            if np.max(np.abs(np.array(q)-joints[-1])) > 0.15:
                raise ValueError(f'动态IK关节跳变，t={t:.3f}s')
            joints.append(np.array(q))
        spline, scale, peak_v, peak_a = joint_spline(times, joints, self.vmax, self.amax)
        # ROS Duration 的精度是纳秒；先量化再去重，不能派发四舍五入后同戳的点。
        dense = np.unique(np.round(np.r_[np.arange(0., times[-1], 0.02), times], 9))
        path = spline(dense)
        _, expected, _ = moving_tcp_path(anchor, velocity, start, actual, axis, cfg, dense)
        deviation = max(np.linalg.norm(self.n._fixed_tcp_fk(q)[:3, 3]-p) for q, p in zip(path, expected))
        return times, positions, close_at, spline, scale, peak_v, peak_a, dense, path, deviation

    def slower(self, cfg, result):
        # 三次样条的加速度在每段为一次函数，峰值在结点；只延长超限阶段。
        # 不能用全局倍数套到四个阶段: 远距离起步只有对齐阶段超限, 把接近/随动一起拉长
        # 会让拦截点整体后移十几厘米, 反而在可达区外才闭合。
        times, spline = result[0], result[3]
        boundaries = np.cumsum([0., cfg['align_s'], cfg['approach_s'], cfg['follow_s'], cfg['lift_s']])
        for key, begin, end in zip(('align_s', 'approach_s', 'follow_s', 'lift_s'), boundaries[:-1], boundaries[1:]):
            knots = np.unique(np.r_[begin, times[(times > begin) & (times < end)], end])
            velocity_scale = float(np.max(np.abs(spline(knots, 1))/self.vmax))
            acceleration_scale = float(np.sqrt(np.max(np.abs(spline(knots, 2))/self.amax)))
            factor = max(1., velocity_scale, acceleration_scale)
            if factor > 1.001:
                # 单次最多放慢1.5倍，且总时长不超过配置值2.5倍。旧实现一次可乘3倍，
                # 会把approach_s撑到7s以上，物体在下降途中又走十几厘米，永远收敛不了。
                cfg[key] = min(cfg[key]*min(1.5, factor)*1.05, self.settings[key]*3.5)
                if key == 'align_s':
                    cfg['align_min_s'] = cfg[key]  # slower() 拉长后不再被自适应缩短

    def set_align_time(self, cfg, actual_tcp, center):
        """对齐阶段时长按起步点到预抓取点的距离确定。

        固定0.35s时, 机械臂从远处起步会被关节限速放大约2倍, 整条轨迹被拉长,
        水果在拦截前又往前走了十几厘米 (日志中 t≈4.1s 时 y≈0.29m 即由此而来).
        但残差很小时不能死守长对齐: 待命点已经就是预抓取点时, 0.55s 的空等
        看上去就是"停住一会", 所以下限收到0.3s, 超限再由 slower() 延长。
        """
        distance = float(np.linalg.norm(np.asarray(center)-np.asarray(actual_tcp)))
        # 静止到静止补距离d的时间按加速度反推: a≈6d/T², 取1.2m/s² ⇒ T≈sqrt(6d/1.2)。
        # 用固定下限0.3s时, 3cm残差就要6.7m/s², 正是日志里倍率2.59的来源。
        # 1.2m/s²的TCP加速度在手腕附近会被放大到20+ rad/s², 取0.3m/s²留足余量。
        wanted = min(1.4, max(.3, math.sqrt(max(distance, 0.)*6./.3)))
        cfg['align_s'] = max(float(cfg.get('align_min_s', .3)), wanted)

    def delay_for_intercept(self, anchor, velocity, s_stage):
        """把待命移动推迟到刚好赶得上，避免"先飞过去再干等一会"。

        等待发生在移动之前：机械臂停在上一位置，物体快进窗口时才开始一次连续移动，
        到位后立刻下降。剩余误差仍由 wait_in_window 兜底。
        """
        cfg, speed = self.settings, float(np.linalg.norm(velocity))
        direction = velocity/speed
        lo = s_stage-min(.03, cfg['intercept_window_m'])
        need = ((lo-float(np.asarray(anchor[1])@direction))/speed
                -max(0., self.now()-float(anchor[0])))
        move_s = float(getattr(self.n, '_dynamic_stage_s', cfg['stage_move_s']))
        pre = min(need-move_s*1.1, cfg['max_wait_s'])
        self.report['pre_wait_s'] = max(0., pre)
        if pre <= .05:
            return 0.
        deadline = time.monotonic()+pre
        while time.monotonic() < deadline:
            self.spin()
            time.sleep(.02)
        self.n.get_logger().info(f'动态抓取：等待目标接近{pre:.1f}s，随后一次走到位')
        return pre

    def preview_candidate(self, pre_tcp, orientation, initial, depth):
        """待命位就绪后预解整条随动轨迹，不可达的姿态直接淘汰；不发送运动。"""
        cfg = self.settings.copy()
        cfg['extra_depth_m'] = depth
        axis = transform([0., 0., 0.], orientation)[:3, 2]
        center = np.asarray(pre_tcp, dtype=float)+cfg['pregrasp_m']*axis
        # 已到待命位，对齐段只需消除横向残差；高度差是固定的预抓取偏置，不该计入对齐距离。
        actual = self.n._fixed_tcp_fk(np.array(initial))[:3, 3]
        self.set_align_time(cfg, actual, actual+np.array([center[0]-actual[0], center[1]-actual[1], 0.]))
        pre_pose = dict(zip(('x', 'y', 'z'), np.asarray(pre_tcp, dtype=float).tolist()))
        entry = dict(depth_m=depth, center_m=center.tolist(), orientation_xyzw=list(orientation), attempts=[])
        self.report['previews'].append(entry)
        try:
            velocity = self.velocity()
            for _ in range(int(cfg['max_plan_attempts'])):
                result = self.kinematics((0., center), velocity, 0., np.array(initial), orientation, cfg)
                scale, deviation = result[4], result[-1]
                entry['attempts'].append(dict(scale=float(scale), duration_s=float(result[0][-1]), spline_error_mm=deviation*1000))
                if scale > 1.03:
                    self.slower(cfg, result)
                    continue
                if deviation > .005:
                    cfg['sample_s'] *= .5
                    continue
                self.candidate_settings = cfg
                entry['result'] = 'passed'
                return True
        except ValueError as exc:
            entry['result'] = str(exc)
        if 'result' not in entry:
            entry['result'] = '关节限速/样条误差重算未收敛'
        return False

    def fast_pose(self, target, seed):
        """复用上一次成功的倾角/深入量，省掉每次重扫7个倾角。"""
        hint = getattr(self.n, '_dynamic_pose_hint', None)
        geometry = getattr(self.n, '_grasp_geometry', None) or {}
        if not hint or float(geometry.get('elongation_ratio') or 0) >= 1.5:
            return None
        # 姿态是按上一个物体的径向方向生成的；物体换到别的方位后必须重新扫，
        # 否则拿着错误偏航角的姿态去解，会直接判无解（报告002923即此）。
        try:
            yaw = math.degrees(math.atan2(float(target['y']), float(target['x'])))
            delta = abs((yaw-float(hint['yaw'])+180.) % 360.-180.)
        except (KeyError, TypeError, ValueError):
            return None
        if delta > 12.:
            return None
        try:
            orientation = np.asarray(hint['orientation'], dtype=float)
        except (KeyError, TypeError, ValueError):
            return None
        if orientation.shape != (4,):
            return None
        plan = self.n._plan_tool_axis_approach(
            target, seed, orientation, False,
            extra_depth=float(hint.get('depth', self.settings['extra_depth_m'])), path_filter=None,
            diagnostics=self.report['preflight'], pregrasp_m=self.settings['pregrasp_m'])
        self.report['pose_hint'] = '命中' if plan is not None else '未命中，退回全扫描'
        return plan

    def stage_target(self, anchor, velocity):
        """预定位点固定在可抓窗口内：横向跟随物体，纵向落在拦截区，不追几秒后的位置。

        旧实现用 anchor+velocity*(now+hover) 追未来点，hover 只增不减，预测越推越下游，
        日志中 y=+0.30m 的 IK 失败即由此而来。
        """
        cfg, speed = self.settings, float(np.linalg.norm(velocity))
        direction = velocity/speed
        xyz = np.asarray(anchor[1], dtype=float)
        # anchor 是图像时刻的位置，必须先补上“观测已过去多久”，否则待命点整体滞后，
        # 移动结束时物体已经越过窗口（日志 s=-0.097 > hi=-0.209 即此原因）。
        age = max(0., self.now()-float(anchor[0]))
        move_s = getattr(self.n, '_dynamic_stage_s', cfg['stage_move_s'])
        # 待命点要对准“轨迹真正启动时”物体的位置：移动耗时之外还要加规划提前量，
        # 否则到位后物体又走了5~6cm，对齐段硬补导致倍率2.0并反复重算（表现为停顿）。
        lead = self.plan_lead()
        s_future = float(xyz@direction)+speed*(age+move_s+lead)
        horizon = cfg['align_s']+cfg['approach_s']
        s_hi = cfg['intercept_s_max_m']-speed*horizon
        s_lo = cfg['intercept_s_pref_m']-cfg['intercept_window_m']
        s_stage = float(np.clip(s_future,
                                s_lo, max(s_hi, s_lo)))
        return xyz+(s_stage-float(xyz@direction))*direction, s_stage, s_future

    def plan_lead(self):
        """规划提前量：给IK/碰撞检查留出余量，避免每次都卡在发送时间窗边上。"""
        prepare = min(float(getattr(self.n, '_dynamic_prepare_s', .8)), 1.5)
        return min(1.6, max(.6, prepare*1.2+.3))

    def update_velocity_residual(self, measured):
        """用视觉观测的沿带速度修正STM32读数；GUI设定值与真实速度相差约4%~14%。"""
        speed = float(np.linalg.norm(measured))
        observed = None
        if self.track.samples:
            last, z_ref = self.track.samples[-1][0], self.track.samples[-1][1][2]
            if 0 <= self.now()-last <= self.settings['observation_max_age_s']:
                observed = self.track.longitudinal_speed(measured/speed, z_ref)
        if observed is not None and observed > 0.:
            self.velocity_residual_mps = float(np.clip(observed-speed, -.2*speed, .2*speed))
        self.report['motion_speed_estimate'] = dict(
            stm32_speed_mps=speed, observed_along_belt_mps=observed,
            residual_mps=self.velocity_residual_mps,
            prediction_speed_mps=speed+self.velocity_residual_mps)
        self.n.get_logger().info(
            f"动态预测速度: STM32={speed:.4f}m/s，视觉={'-' if observed is None else f'{observed:.4f}'}m/s，"
            f"修正={self.velocity_residual_mps:+.4f}m/s")

    def wait_in_window(self, s_stage, velocity, lead):
        """在待命位等物体进入可抓窗口；已越过的目标直接放弃，不追到工作区外。"""
        cfg, direction = self.settings, velocity/np.linalg.norm(velocity)
        speed = float(np.linalg.norm(velocity))
        horizon = cfg['align_s']+cfg['approach_s']
        # 只等到目标基本就位(差3cm内)再起步；等太早会让对齐段重新变成大补差。
        lo = s_stage-min(.03, cfg['intercept_window_m'])
        # 夹爪真正闭合时物体还会走 v*horizon，按那一刻是否还在可达区判断。
        hi = cfg['intercept_s_max_m']-speed*horizon
        deadline = time.monotonic()+cfg['max_wait_s']
        while True:
            self.spin()
            stamp, xyz = self.snapshot()
            s_start = float((xyz+velocity*(self.now()+lead-stamp))@direction)
            if s_start > hi:
                raise RuntimeError(
                    f'目标已越过可抓区(抓取点s={s_start+speed*horizon:+.3f}m > '
                    f'{cfg["intercept_s_max_m"]:+.3f}m)，本轮不追')
            if s_start >= lo:
                self.report['intercept_window'] = dict(s_at_start=s_start, lo=lo, hi=hi, lead_s=lead)
                return
            if time.monotonic() > deadline:
                raise RuntimeError(f'等待目标进入可抓区超时(s={s_start:+.3f}m < {lo:+.3f}m)')
            time.sleep(0.02)

    def plan(self, orientation):
        """每次调整时间都重新预测移动目标、求IK并检查；不事后缩放已预测轨迹。"""
        cfg = self.candidate_settings.copy()
        # 同一求解轮固定绝对启动时刻；不能每次放慢轨迹又把启动点向后推，追着目标越算越远。
        lead = self.plan_lead()
        start = self.now()+lead
        last_reason = '未取得可用轨迹'
        for attempt in range(int(cfg['max_plan_attempts'])):
            self.spin()
            anchor = self.snapshot()
            velocity = self.velocity()
            initial = self.actual_joints()
            begin = self.now()
            if start < begin+cfg['send_margin_s']:
                start = begin+lead
            axis = transform([0., 0., 0.], orientation)[:3, 2]
            center = anchor[1]+velocity*(start-anchor[0])-cfg['pregrasp_m']*axis
            self.set_align_time(cfg, self.n._fixed_tcp_fk(initial)[:3, 3], center)
            entry = dict(attempt=attempt, begin_s=begin, start_s=start,
                anchor=[anchor[0], anchor[1].tolist()], velocity_mps=velocity.tolist(),
                initial_joints=initial.tolist(), initial_tcp=self.n._fixed_tcp_fk(initial).tolist(),
                settings=cfg.copy())
            self.report['planning_inputs'].append(entry)
            try:
                result = self.kinematics(anchor, velocity, start, initial, orientation, cfg)
                times, positions, close_at, spline, scale, peak_v, peak_a, dense, path, deviation = result
                entry.update(scale=float(scale), peak_velocity=peak_v.tolist(), peak_acceleration=peak_a.tolist())
            except ValueError as exc:
                entry['error'] = str(exc)
                raise
            # 轨迹越长，物体走得越远。带宽0.048m/s时6s轨迹会让目标再走29cm，
            # 直接落到工作区外（日志 t=6.18s xyz y=+0.21 即此）。宁可放弃也不硬追。
            if times[-1] > cfg['max_trajectory_s']:
                raise RuntimeError(
                    f'动态轨迹过长({times[-1]:.1f}s > {cfg["max_trajectory_s"]:.1f}s)，物体将移出可达区，放弃本轮')
            # 1.03 以内视为可接受: 限值本身已按45%缩放, 3%余量仍远低于电机真实上限,
            # 再重算一次反而让物体多走1cm (日志里 1.0→1.09 的来回抖动即此)。
            if scale > 1.03:
                # 舒适阈值超了, 但只要没破硬件真实限值就照抓; 否则慢一档重算。
                within_hw = (np.all(peak_v <= self.hard_vmax*.98)
                             and np.all(peak_a <= self.hard_amax*.98))
                if not within_hw:
                    self.slower(cfg, result)
                    last_reason = f'关节速度/加速度超限(倍率{scale:.2f})'
                    self.n.get_logger().info(f'动态轨迹按关节速度/加速度重算时间，倍率={scale:.2f}')
                    continue
                self.report.setdefault('hard_limit_accepts', []).append(
                    dict(attempt=attempt, scale=float(scale),
                         peak_velocity=peak_v.tolist(), peak_acceleration=peak_a.tolist()))
                self.n.get_logger().warn(
                    f'动态轨迹超舒适限值(倍率{scale:.2f})但未超硬件限值，按硬件限值执行')
            # 控制器使用同一三次曲线；密采样检查样条本身，而非仅检查原始IK点。
            if deviation > .005 and cfg['sample_s'] >= .03:
                cfg['sample_s'] *= .5
                last_reason = f'样条与IK偏差{deviation*1000:.1f}mm，加密采样'
                continue
            if not self.n._joint_waypoints_valid(path, start_joints=initial, report_failure=False):
                failure = getattr(self.n, '_last_state_validity_failure', None) or {}
                entry['state_validity_failure'] = failure
                deck = {'belt_deck', 'conveyor_belt_col', 'platform', 'conveyor_platform_col'}
                table_contact = any(c.get('a') in deck or c.get('b') in deck for c in failure.get('contacts', []))
                if table_contact and cfg['extra_depth_m'] > 0.:
                    cfg['extra_depth_m'] = max(0., round(cfg['extra_depth_m']-.005, 6))
                    self.n.get_logger().warning(f"动态路径碰台面，额外深入退让至{cfg['extra_depth_m']*100:.1f}cm后重新核查")
                    continue
                raise RuntimeError('动态样条碰撞/限位检查失败，未派发')
            elapsed = self.now()-begin
            self.n._dynamic_prepare_s = elapsed
            self.report['plans'].append(dict(attempt=attempt, start_s=start, prepare_s=elapsed,
                duration_s=float(times[-1]), close_at_s=close_at, lead_s=lead,
                peak_velocity=peak_v.tolist(), peak_acceleration=peak_a.tolist(), spline_error_mm=deviation*1000))
            if self.now() > start-cfg['send_margin_s']:
                lead = max(lead, elapsed*1.2+cfg['send_margin_s']+0.1)
                start = self.now()+lead
                last_reason = f'规划耗时{elapsed:.2f}s超过发送时间窗'
                continue
            new_stamp, new_xyz = self.snapshot()
            old_at_start = anchor[1]+velocity*(start-anchor[0])
            new_at_start = new_xyz+velocity*(start-new_stamp)
            entry['observation_correction_mm'] = ((new_at_start-old_at_start)*1000).tolist()
            # 3mm门限过紧: 每重算一次要0.4s，物体又前进1cm，越修越远。
            # 8mm以内直接由对齐段吸收，只有明显跳变才整条重算。
            if np.linalg.norm(new_at_start-old_at_start) > .008:
                entry['retry_reason'] = '正常视觉修正目标位置'
                last_reason = f'视觉修正{np.linalg.norm(new_at_start-old_at_start)*1000:.1f}mm'
                self.n.get_logger().info(f'动态观测修正{np.linalg.norm(new_at_start-old_at_start)*1000:.1f}mm，重新核查轨迹')
                continue  # 干净视觉已经修正目标位置，重新计算整条路径后再派发。
            if np.linalg.norm(self.velocity()-velocity) > .008:
                last_reason = '传送带速度读数跳变'
                continue
            if np.max(np.abs(self.actual_joints()-initial)) > 0.02:
                last_reason = '真实关节仍在移动'
                continue
            self.report.update(anchor=[anchor[0], anchor[1].tolist()], velocity_mps=velocity.tolist(),
                orientation_xyzw=list(orientation), times_s=times.tolist(), targets_m=positions.tolist(),
                joint_points=path.tolist(), joint_times_s=dense.tolist(), effective_settings=cfg,
                selected_extra_depth_m=cfg['extra_depth_m'],
                visual_velocity_mps=None if self.track.visual_velocity() is None else self.track.visual_velocity().tolist())
            return start, dense, path, spline, close_at, velocity
        raise RuntimeError(f'动态轨迹{int(cfg["max_plan_attempts"])}次规划均未通过({last_reason})，未派发')

    def execute(self, plan):
        self.prediction_frozen = True  # 已派发的单条连续轨迹不跟随遮挡帧重写或反复取消。
        start, times, path, spline, close_at, velocity = plan
        self.n._check_calibrated_tool()
        if self.now() > start-self.settings['send_margin_s']:
            raise RuntimeError('核查后发送时间窗已过，拒绝使用过期预测')
        goal = FollowJointTrajectory.Goal()
        ns = round(start*1e9)
        goal.trajectory.header.stamp.sec, goal.trajectory.header.stamp.nanosec = divmod(ns, 1000000000)
        goal.trajectory.joint_names = [f'joint{i}' for i in range(1, 7)]
        for t, q in zip(times, path):
            point = JointTrajectoryPoint(positions=q.tolist(), velocities=spline(float(t), 1).tolist())
            self.duration(point, float(t))
            goal.trajectory.points.append(point)
        handle = self.send(self.n._arm_jt_client, goal)
        future = handle.get_result_async()
        close_future = None
        close_send_future = None
        next_log = -1.
        close_sent_at = None
        while not future.done():
            self.spin()
            elapsed = self.now()-start
            actual_q = self.actual_joints()
            current_velocity = self.velocity()
            if np.linalg.norm(current_velocity-velocity) > max(0.01, np.linalg.norm(velocity)*0.3):
                raise RuntimeError('传送带速度/方向已明显改变，本条定速轨迹停止')
            if elapsed < close_at:
                self.snapshot()
            if close_send_future is None and elapsed >= close_at:
                if elapsed > close_at+0.1:
                    raise RuntimeError('闭合派发已错过时间窗，取消本轮')
                # 接近主循环不能阻塞等待夹爪接受，否则期间无法及时检查急停/反馈。
                close_send_future = self.n._gripper_action_client.send_goal_async(self.gripper_goal(0.))
                def track_close(done):
                    h = done.result()
                    if h and h.accepted:
                        if h not in self.handles:
                            self.handles.append(h)
                        if self.stopped or self.n._emergency_flag:
                            h.cancel_goal_async()
                close_send_future.add_done_callback(track_close)
                close_sent_at = self.now()
                self.report['close_sent_s'] = close_sent_at
                self.n.get_logger().info('动态抓取：沿带随动中闭合夹爪')
            if close_send_future is not None and close_future is None and close_send_future.done():
                close_handle = close_send_future.result()
                if close_handle is None or not close_handle.accepted:
                    raise RuntimeError('夹爪闭合目标被拒绝')
                if close_handle not in self.handles:
                    self.handles.append(close_handle)
                close_future = close_handle.get_result_async()
            if close_future is not None and close_future.done():
                closed = close_future.result()
                if closed is None or closed.status != 4 or closed.result.error_code != 0:
                    raise RuntimeError('随动闭合失败，取消后续运动')
            elif close_sent_at is not None and self.now()-close_sent_at > self.settings['close_s']+0.15:
                raise RuntimeError('夹爪未及时完成闭合，停止抬起/搬运')
            if elapsed >= next_log:
                sample_t = float(np.clip(elapsed, times[0], times[-1]))
                wanted = self.n._fixed_tcp_fk(spline(sample_t))
                measured = self.n._fixed_tcp_fk(actual_q)
                self.report['feedback'].append(dict(stamp_s=self.now(), trajectory_s=elapsed,
                    feedback_sample_stamp_s=getattr(self.n, '_dynamic_actual_stamp', 0.),
                    gripper_width_m=(None if self.n._latest_gripper_status is None else float(self.n._latest_gripper_status.width)),
                    gripper_feedback_age_s=time.monotonic()-getattr(self.n, '_dynamic_gripper_at', -100.),
                    target_tcp_m=wanted[:3, 3].tolist(), feedback_tcp_m=measured[:3, 3].tolist(),
                    target_rotation=wanted[:3, :3].tolist(), feedback_rotation=measured[:3, :3].tolist(),
                    error_mm=float(np.linalg.norm(wanted[:3, 3]-measured[:3, 3])*1000)))
                next_log = elapsed+0.05
            if elapsed > times[-1]+2.:
                raise RuntimeError('随动轨迹执行超时')
        result = future.result()
        if result is None or result.status != 4 or result.result.error_code != 0 or close_future is None:
            raise RuntimeError('动态轨迹未成功完成，不放置、不松爪')
        closed = self.wait_future(close_future, 1.)
        if closed.status != 4 or closed.result.error_code != 0:
            raise RuntimeError('闭合未成功，不放置')

    def run(self):
        self.n._dynamic_tracker = self.track
        self.n._last_dynamic_error = ''
        try:
            self.spin()
            self.n._check_calibrated_tool()
            for client in (self.n._action_client, self.n._arm_jt_client, self.n._gripper_action_client):
                if not client.wait_for_server(timeout_sec=1.):
                    raise RuntimeError('动态抓取的MoveIt/机械臂/夹爪服务未就绪')
            stamp, xyz = self.snapshot()
            velocity = self.velocity()
            self.report.update(initial_anchor=[stamp, xyz.tolist()], initial_velocity_mps=velocity.tolist(),
                motor_direction=self.n._belt_direction, motor_running_reported=getattr(self.n, '_belt_running', None),
                commanded_running=getattr(self.n, '_belt_commanded_running', None),
                direction_calibration=dict(self.n._dynamic_calib or {}))
            self.motion_anchor = (stamp, xyz.copy())
            self.n._grasp_geometry = self.track.geometry
            # 移动物体留足入爪余量；不让抖动的宽度估计在接近前缩小开口。
            self.report['gripper_open_target_m'] = .095
            opened = self.n._latest_gripper_status
            already_open = (opened is not None and float(opened.width) >= .088
                            and time.monotonic()-getattr(self.n, '_dynamic_gripper_at', -100.) <= .5)
            if already_open:
                # 上一轮放下后夹爪就是张开的, 省掉一次0.65s的重复开爪动作。
                self.report['gripper_open_target_m'] = float(opened.width)
            else:
                self.gripper_wait(.095)
                self.spin()
                opened = self.n._latest_gripper_status
            self.report['gripper_open_feedback_m'] = None if opened is None else float(opened.width)
            # 候选碰撞检查使用张开后的真实/控制关节状态。
            # 1) 先到固定拦截区上方待命。规划失败时机械臂重新就位再解一次：
            # 原地重算会让物体越走越远、要补的距离越来越大，反而永远解不出来。
            rounds = max(1, int(self.settings.get('max_stage_rounds', 2)))
            for stage_round in range(rounds):
                self.spin()
                anchor_now = self.snapshot()
                velocity = self.velocity()
                stage_xyz, s_stage, s_future = self.stage_target(anchor_now, velocity)
                horizon = self.settings['align_s']+self.settings['approach_s']
                s_hi = self.settings['intercept_s_max_m']-float(np.linalg.norm(velocity))*horizon
                if s_future > s_hi+self.settings['intercept_window_m']:
                    raise RuntimeError(f'目标来不及(到位时s={s_future:+.3f}m > {s_hi:+.3f}m)，本轮不移动')
                target = dict(zip(('x', 'y', 'z'), stage_xyz.tolist()))
                seed = self.actual_joints()
                # 第二轮不再复用上一轮姿态：换一套倾角可能刚好避开手腕奇异。
                plan = self.fast_pose(target, seed) if stage_round == 0 else None
                if plan is None:
                    plan = self.n._plan_tool_axis_approach(target, seed, [1., 0., 0., 0.], True,
                        extra_depth=self.settings['extra_depth_m'], path_filter=None,
                        diagnostics=self.report['preflight'], pregrasp_m=self.settings['pregrasp_m'],
                        depth_ladder=True)
                if plan is None:
                    raise RuntimeError('动态待命位不可达；preflight记录具体阶段')
                _, orientation, joints, tilt = plan
                self.settings['extra_depth_m'] = self.n.last_grasp_extra_depth
                # 记住这套待命关节, 空场时先回到这里等下一个物体, 省掉从放置点起飞的时间。
                self.n._dynamic_ready_joints = [float(v) for v in joints]
                self.n._dynamic_pose_hint = dict(orientation=list(orientation),
                                                 depth=float(self.settings['extra_depth_m']),
                                                 yaw=math.degrees(math.atan2(
                                                     float(stage_xyz[1]), float(stage_xyz[0]))))
                self.report.update(tilt_deg=tilt, selected_extra_depth_m=self.settings['extra_depth_m'],
                    stage_target_m=stage_xyz.tolist(), stage_s_m=s_stage, stage_s_future_m=s_future,
                    stage_round=stage_round+1)
                # 规划算完再等目标接近：这段时间物体还在走，不动机械臂，到位后立即下降。
                self.delay_for_intercept(self.snapshot(), velocity, s_stage)
                began = self.now()
                self.move_joints(joints, '动态抓取：到拦截区上方待命' if stage_round == 0
                                 else f'动态抓取：重新待命(第{stage_round+1}轮)', scaling=.7)
                self.n._dynamic_stage_s = self.now()-began
                self.spin()
                # 2) 待命后用视觉校准沿带速度，并等物体进入可抓窗口再同步下降。
                measured = self.velocity()  # 残差尚未生效，先取STM32原值。
                self.motion_anchor = self.snapshot()
                self.update_velocity_residual(measured)
                velocity = self.velocity()
                self.wait_in_window(s_stage, velocity, self.plan_lead())
                self.report.update(prediction_mode='拦截区待命→窗口内同步下降；遮挡区按STM32速度+视觉残差外推',
                    locked_anchor=[self.motion_anchor[0], self.motion_anchor[1].tolist()])
                # 3) 从当前真实姿态生成整条随动轨迹，通过后按绝对时间派发。
                self.candidate_settings = self.settings.copy()
                try:
                    trajectory = self.plan(orientation)
                except RuntimeError as exc:
                    if stage_round+1 >= rounds or '动态轨迹' not in str(exc):
                        raise
                    self.report.setdefault('stage_retries', []).append(str(exc))
                    self.n.get_logger().warn(f'动态规划未通过，重新就位后再试：{exc}')
                    continue
                self.execute(trajectory)
                break
            # 已在随动轨迹末段抬起并停稳。宽度反馈只判断夹持，不冒充物理定位精度。
            self.spin()
            feedback = self.n._latest_gripper_status
            age = time.monotonic()-getattr(self.n, '_dynamic_gripper_at', -100.)
            self.report['gripper_final_feedback'] = dict(age_s=age,
                width_m=None if feedback is None else float(feedback.width),
                force_n=None if feedback is None else float(feedback.force))
            if feedback is None:
                raise RuntimeError('夹爪反馈缺失，本轮未成功')
            if age > .5:
                raise RuntimeError(f'夹爪反馈过期，年龄={age:.3f}s，本轮未成功')
            self.report['gripper_width_m'] = float(feedback.width)
            if feedback.width <= .005:
                raise RuntimeError(f'夹爪空抓，实际宽度={feedback.width*1000:.1f}mm')
            self.move_joints(self.place, '动态抓取：移动到放置点', scaling=.8)
            self.gripper_wait(.09)
            self.report['success'] = True
            return True
        except (Exception, KeyboardInterrupt) as exc:
            self.report['error'] = str(exc) or 'KeyboardInterrupt'
            self.n._last_dynamic_error = self.report['error']
            self.n.get_logger().error(f'动态抓取停止：{self.report["error"]}；不自动回零/退回/松爪')
            return False
        finally:
            self.stopped = True
            if not self.report['success']:
                self.n._dynamic_tracker = None
                futures = [handle.cancel_goal_async() for handle in self.handles if handle.status in (1, 2, 3)]
                # 有限处理取消应答；不调用会追加回待机动作的 _check_emergency。
                deadline = time.monotonic()+1.
                while rclpy.ok() and any(not f.done() for f in futures) and time.monotonic() < deadline:
                    rclpy.spin_once(self.n, timeout_sec=0.01)
                self.report['cancel_acknowledged'] = all(f.done() and not f.exception() and
                    f.result() is not None and f.result().return_code == 0 for f in futures)
            self.n._active_goal_handle = None
            self.n._dynamic_tracker = None
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.path.write_text(json.dumps(self.report, ensure_ascii=False, indent=2))
            self.n.get_logger().info(f'动态抓取报告：{self.path}')
