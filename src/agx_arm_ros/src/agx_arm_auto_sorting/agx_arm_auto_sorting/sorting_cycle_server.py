#!/usr/bin/env python3
import json
import time
import threading
from typing import Dict, List, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import BoundingVolume, Constraints, JointConstraint, OrientationConstraint, PositionConstraint
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from .config_utils import ensure_pose_keys, load_yaml_config


class SortingCycleServer(Node):
    def __init__(self):
        super().__init__('sorting_cycle_server')
        self.cb_group = ReentrantCallbackGroup()

        default_config = get_package_share_directory('agx_arm_auto_sorting') + '/config/sorting_dataset_config.yaml'
        self.declare_parameter('config_file', default_config)
        self.config_file = self.get_parameter('config_file').get_parameter_value().string_value

        all_cfg = load_yaml_config(self.config_file)
        self.cfg = all_cfg['sorting_cycle']

        self.frame_id = self.cfg['frame_id']
        self.group_name = self.cfg['arm_group_name']
        self.workspace = self.cfg['workspace_limits']

        self.status_pub = self.create_publisher(String, self.cfg['status_topic_name'], 10)
        self.result_pub = self.create_publisher(String, self.cfg['result_topic_name'], 10)
        self.cmd_sub = self.create_subscription(
            String,
            self.cfg.get('sorting_cmd_topic_name', '/sorting_cmds'),
            self._cmd_cb,
            20,
            callback_group=self.cb_group,
        )
        self.prepare_srv = self.create_service(
            Trigger,
            self.cfg['prepare_service_name'],
            self._prepare_cb,
            callback_group=self.cb_group,
        )
        self.trigger_srv = self.create_service(
            Trigger,
            self.cfg['trigger_service_name'],
            self._trigger_cb,
            callback_group=self.cb_group,
        )

        self.latest_joint_state = JointState()
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            30,
            callback_group=self.cb_group,
        )

        self.move_client = ActionClient(
            self,
            MoveGroup,
            self.cfg['move_action_name'],
            callback_group=self.cb_group,
        )
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.cfg['gripper_action_name'],
            callback_group=self.cb_group,
        )
        self.gripper8_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper8_controller/follow_joint_trajectory',
            callback_group=self.cb_group,
        )

        self._busy = False
        self._last_result = {}
        self._cycle_lock = threading.Lock()
        self._cycle_thread = None
        self._latest_sort_cmd: Optional[Dict[str, object]] = None

        self.get_logger().info('Waiting MoveIt and gripper action servers...')
        if not self.move_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action server not available.')
        if not self.gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('Gripper FollowJointTrajectory action server not available now.')
        if not self.gripper8_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Gripper8 FollowJointTrajectory action server not available now.')

        self._publish_state('IDLE')
        self.get_logger().info('Sorting cycle server ready.')

    def _joint_state_cb(self, msg: JointState):
        self.latest_joint_state = msg

    def _cmd_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            self.get_logger().warn('Ignore invalid /sorting_cmds json payload.')
            return

        if data.get('cmd') != 'sort':
            return

        pick_pose = data.get('pick')
        place_pose = data.get('place')
        if not isinstance(pick_pose, dict) or not isinstance(place_pose, dict):
            self.get_logger().warn('Ignore sort cmd without valid pick/place pose dict.')
            return

        try:
            ensure_pose_keys(pick_pose)
            ensure_pose_keys(place_pose)
        except Exception as exc:
            self.get_logger().warn(f'Ignore sort cmd due to invalid pose keys: {exc}')
            return

        object_diameter_m = data.get('object_diameter_m', None)
        if object_diameter_m is not None:
            try:
                object_diameter_m = float(object_diameter_m)
            except Exception:
                self.get_logger().warn('Ignore invalid object_diameter_m in sort cmd.')
                object_diameter_m = None

        self._latest_sort_cmd = {
            'pick': {k: float(v) for k, v in pick_pose.items()},
            'place': {k: float(v) for k, v in place_pose.items()},
            'object_diameter_m': object_diameter_m,
        }
        self.get_logger().info('Received sort cmd on /sorting_cmds, pick/place updated for next cycle.')

    def _compute_dynamic_gripper_targets(self, object_diameter_m: Optional[float]) -> Tuple[List[float], List[float]]:
        open_default = [float(v) for v in self.cfg['gripper']['open_positions_rad']]
        close_default = [float(v) for v in self.cfg['gripper']['close_positions_rad']]
        if object_diameter_m is None:
            return open_default, close_default

        mapping = self.cfg['gripper'].get('diameter_to_open', {})
        min_d = float(mapping.get('min_diameter_m', 0.03))
        max_d = float(mapping.get('max_diameter_m', 0.09))
        min_open = float(mapping.get('min_open_rad', min(abs(open_default[0]), abs(open_default[1]))))
        max_open = float(mapping.get('max_open_rad', max(abs(open_default[0]), abs(open_default[1]))))
        close_delta = float(mapping.get('close_delta_rad', 0.02))

        d = max(min_d, min(max_d, float(object_diameter_m)))
        if max_d - min_d < 1e-6:
            open_mag = min_open
        else:
            ratio = (d - min_d) / (max_d - min_d)
            open_mag = min_open + ratio * (max_open - min_open)

        open_targets = [float(open_mag), float(-open_mag)]
        close_mag = max(min_open * 0.35, open_mag - close_delta)
        close_targets = [float(close_mag), float(-close_mag)]
        return open_targets, close_targets

    def _publish_state(self, state: str):
        msg = String()
        msg.data = state
        self.status_pub.publish(msg)

    def _wait_future(self, future, timeout_sec: float = 15.0):
        start = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start > timeout_sec:
                return None
            time.sleep(0.01)
        if not future.done():
            return None
        return future.result()

    def _trigger_cb(self, request: Trigger.Request, response: Trigger.Response):
        del request
        with self._cycle_lock:
            if self._busy:
                response.success = False
                response.message = 'busy'
                return response

            self._busy = True

        self._publish_state('RUNNING')
        self._cycle_thread = threading.Thread(target=self._cycle_worker, daemon=True)
        self._cycle_thread.start()

        response.success = True
        response.message = 'started'
        return response

    def _prepare_cb(self, request: Trigger.Request, response: Trigger.Response):
        del request
        with self._cycle_lock:
            if self._busy:
                response.success = False
                response.message = 'busy'
                return response

            self._busy = True

        self._publish_state('PREPARING')
        self._cycle_thread = threading.Thread(target=self._prepare_worker, daemon=True)
        self._cycle_thread.start()

        response.success = True
        response.message = 'started'
        return response

    def _prepare_worker(self):
        started = time.time()
        try:
            if not self._is_near_joint_goal(self.cfg['standby_joint_values_rad']):
                self.get_logger().info('Prepare step: MOVE_STANDBY')
                if not self._send_move_group(self._build_joint_goal_constraints(self.cfg['standby_joint_values_rad'])):
                    raise RuntimeError('Failed to move to standby joints during prepare.')
            else:
                self.get_logger().info('Prepare step: standby already satisfied')

            self.get_logger().info('Prepare step: GRIPPER_OPEN_PREPARE')
            if not self._gripper_command(self.cfg['gripper']['open_positions_rad']):
                raise RuntimeError('Failed to open gripper during prepare.')

            out = String()
            out.data = json.dumps({
                'prepare_success': True,
                'duration_sec': round(time.time() - started, 4),
            }, ensure_ascii=False)
            self.result_pub.publish(out)
            self.get_logger().info(f'Prepare finished: {out.data}')
        except Exception as exc:
            self.get_logger().error(f'Prepare error: {exc}')
            out = String()
            out.data = json.dumps({
                'prepare_success': False,
                'error': str(exc),
                'duration_sec': round(time.time() - started, 4),
            }, ensure_ascii=False)
            self.result_pub.publish(out)
        finally:
            with self._cycle_lock:
                self._busy = False
            self._publish_state('IDLE')

    def _cycle_worker(self):
        started = time.time()
        try:
            metrics = self._run_full_cycle()
            metrics['duration_sec'] = round(time.time() - started, 4)
            metrics['timestamp'] = self.get_clock().now().to_msg().sec
            self._last_result = metrics

            out = String()
            out.data = json.dumps(metrics, ensure_ascii=False)
            self.result_pub.publish(out)
            self.get_logger().info(f'Cycle finished: {out.data}')
        except Exception as exc:
            self.get_logger().error(f'Cycle error: {exc}')
            fail_result = {
                'cycle_success': False,
                'error': str(exc),
                'duration_sec': round(time.time() - started, 4),
            }
            self._last_result = fail_result
            out = String()
            out.data = json.dumps(fail_result, ensure_ascii=False)
            self.result_pub.publish(out)
        finally:
            with self._cycle_lock:
                self._busy = False
            self._publish_state('IDLE')

    def _within_workspace(self, pose: Dict[str, float]) -> bool:
        x_ok = self.workspace['x'][0] <= pose['x'] <= self.workspace['x'][1]
        y_ok = self.workspace['y'][0] <= pose['y'] <= self.workspace['y'][1]
        z_ok = self.workspace['z'][0] <= pose['z'] <= self.workspace['z'][1]
        return x_ok and y_ok and z_ok

    def _build_pose_goal_constraints(self, pose_dict: Dict[str, float]) -> Constraints:
        ensure_pose_keys(pose_dict)

        if not self._within_workspace(pose_dict):
            raise ValueError(f'Pose out of workspace limits: {pose_dict}')

        constraints = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = self.frame_id
        pc.link_name = 'link6'
        pc.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [float(self.cfg['planning']['position_tolerance_m'])]

        center = Pose()
        center.position.x = float(pose_dict['x'])
        center.position.y = float(pose_dict['y'])
        center.position.z = float(pose_dict['z'])

        bv = BoundingVolume()
        bv.primitives.append(sphere)
        bv.primitive_poses.append(center)
        pc.constraint_region = bv

        oc = OrientationConstraint()
        oc.header.frame_id = self.frame_id
        oc.link_name = 'link6'
        oc.orientation.x = float(pose_dict['qx'])
        oc.orientation.y = float(pose_dict['qy'])
        oc.orientation.z = float(pose_dict['qz'])
        oc.orientation.w = float(pose_dict['qw'])
        tol = float(self.cfg['planning']['orientation_tolerance_rad'])
        oc.absolute_x_axis_tolerance = tol
        oc.absolute_y_axis_tolerance = tol
        oc.absolute_z_axis_tolerance = tol
        oc.weight = 1.0

        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)
        return constraints

    def _build_joint_goal_constraints(self, joints) -> Constraints:
        names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        c = Constraints()
        for name, val in zip(names, joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return c

    def _send_move_group(self, constraints: Constraints) -> bool:
        goal = MoveGroup.Goal()
        goal.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        goal.request.workspace_parameters.header.frame_id = self.frame_id
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = int(self.cfg['planning']['num_planning_attempts'])
        goal.request.allowed_planning_time = float(self.cfg['planning']['allowed_planning_time'])
        goal.request.max_velocity_scaling_factor = float(self.cfg['planning']['velocity_scale'])
        goal.request.max_acceleration_scaling_factor = float(self.cfg['planning']['acceleration_scale'])
        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False

        if not self.move_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('MoveGroup action server unavailable during trigger.')
            return False

        self.get_logger().info(f'Sending MoveGroup goal for {self.group_name}')
        send_future = self.move_client.send_goal_async(goal)
        handle = self._wait_future(send_future, timeout_sec=12.0)
        if handle is None:
            self.get_logger().error('MoveGroup goal handle timeout.')
            return False
        if not handle.accepted:
            self.get_logger().error('MoveGroup goal rejected.')
            return False

        self.get_logger().info('MoveGroup goal accepted.')

        result_future = handle.get_result_async()
        result_wrapped = self._wait_future(result_future, timeout_sec=20.0)
        if result_wrapped is None:
            self.get_logger().error('MoveGroup result timeout.')
            return False

        error_code = result_wrapped.result.error_code.val
        if error_code != 1:
            self.get_logger().error(f'MoveGroup failed with error_code={error_code}')
            return False

        self.get_logger().info('MoveGroup goal succeeded.')
        return True

    def _send_trajectory_goal(self, client, joint_names, positions, timeout_sec: float) -> Tuple[bool, str]:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        sec = float(self.cfg['gripper']['motion_duration_sec'])
        point.time_from_start.sec = int(sec)
        point.time_from_start.nanosec = int((sec - int(sec)) * 1e9)
        goal.trajectory.points.append(point)

        if not client.wait_for_server(timeout_sec=1.0):
            return False, 'action_server_unavailable'

        send_future = client.send_goal_async(goal)
        handle = self._wait_future(send_future, timeout_sec=8.0)
        if handle is None:
            return False, 'goal_handle_timeout'
        if not handle.accepted:
            return False, 'goal_rejected'

        result_future = handle.get_result_async()
        wrapped = self._wait_future(result_future, timeout_sec=timeout_sec)
        if wrapped is None:
            return False, 'result_timeout'

        status = getattr(wrapped, 'status', None)
        result = getattr(wrapped, 'result', None)
        if result is None:
            return False, f'no_result_status_{status}'

        error_code = getattr(result, 'error_code', None)
        if error_code is not None and hasattr(error_code, 'val') and error_code.val != 1:
            error_string = getattr(result, 'error_string', '')
            return False, f'error_code_{error_code.val}:{error_string}'

        return True, 'ok'

    def _gripper_command(self, positions) -> bool:
        latest_names = set(self.latest_joint_state.name or [])

        # Current runtime exposes the gripper as joint7/joint8 on two separate controllers.
        if {'joint7', 'joint8'}.issubset(latest_names):
            ok7, msg7 = self._send_trajectory_goal(
                self.gripper_client,
                ['joint7'],
                [float(positions[0])],
                timeout_sec=15.0,
            )
            ok8, msg8 = self._send_trajectory_goal(
                self.gripper8_client,
                ['joint8'],
                [float(positions[1])],
                timeout_sec=15.0,
            )
            if not ok7:
                self.get_logger().error(f'Gripper joint7 command failed: {msg7}')
            if not ok8:
                self.get_logger().error(f'Gripper joint8 command failed: {msg8}')
            return ok7 and ok8

        ok, msg = self._send_trajectory_goal(
            self.gripper_client,
            list(self.cfg['gripper']['joint_names']),
            positions,
            timeout_sec=15.0,
        )
        if not ok:
            self.get_logger().error(f'Gripper command failed: {msg}')
        return ok

    def _compute_gripper_close_ok(self, close_positions) -> Tuple[bool, float]:
        if not self.latest_joint_state.name:
            return False, 1e9

        err_sum = 0.0
        if {'joint7', 'joint8'}.issubset(set(self.latest_joint_state.name)):
            names = ['joint7', 'joint8']
        else:
            names = list(self.cfg['gripper']['joint_names'])
        for idx, name in enumerate(names):
            if name not in self.latest_joint_state.name:
                return False, 1e9
            js_idx = self.latest_joint_state.name.index(name)
            err_sum += abs(self.latest_joint_state.position[js_idx] - float(close_positions[idx]))

        tol = float(self.cfg['gripper']['close_check_tolerance_rad']) * len(names)
        return err_sum <= tol, err_sum

    def _is_near_joint_goal(self, target_joints, tolerance: float = 0.05) -> bool:
        if not self.latest_joint_state.name:
            return False

        names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        for idx, name in enumerate(names):
            if name not in self.latest_joint_state.name:
                return False
            js_idx = self.latest_joint_state.name.index(name)
            if abs(self.latest_joint_state.position[js_idx] - float(target_joints[idx])) > tolerance:
                return False
        return True

    def _offset_pose(self, pose: Dict[str, float], dz: float) -> Dict[str, float]:
        out = dict(pose)
        out['z'] = float(out['z']) + float(dz)
        return out

    def _go_pose(self, pose: Dict[str, float], state_name: str) -> bool:
        self._publish_state(state_name)
        return self._send_move_group(self._build_pose_goal_constraints(pose))

    def _run_full_cycle(self) -> Dict[str, object]:
        cmd = self._latest_sort_cmd
        if cmd:
            pick_pose = dict(cmd['pick'])
            place_pose = dict(cmd['place'])
            object_diameter_m = cmd.get('object_diameter_m')
        else:
            pick_pose = self.cfg['poses']['pick_pose']
            place_pose = self.cfg['poses']['place_pose']
            object_diameter_m = None
        ensure_pose_keys(pick_pose)
        ensure_pose_keys(place_pose)
        dynamic_open, dynamic_close = self._compute_dynamic_gripper_targets(object_diameter_m)

        pre_pick = self._offset_pose(pick_pose, self.cfg['offsets']['pre_pick_z_offset_m'])
        lift_pose = self._offset_pose(pick_pose, self.cfg['offsets']['lift_z_offset_m'])
        pre_place = self._offset_pose(place_pose, self.cfg['offsets']['pre_place_z_offset_m'])

        result = {
            'cycle_success': False,
            'gripper_close_ok': False,
            'gripper_close_error_sum_rad': None,
            'state_sequence': [],
            'pick_pose_source': 'sorting_cmd' if cmd else 'config_default',
            'object_diameter_m': object_diameter_m,
            'gripper_open_positions_rad': [round(v, 6) for v in dynamic_open],
            'gripper_close_positions_rad': [round(v, 6) for v in dynamic_close],
        }

        def step(name: str):
            result['state_sequence'].append(name)
            self._publish_state(name)
            self.get_logger().info(f'Cycle step: {name}')

        step('MOVE_STANDBY')
        standby_joints = self.cfg['standby_joint_values_rad']
        if not self._is_near_joint_goal(standby_joints):
            if not self._send_move_group(self._build_joint_goal_constraints(standby_joints)):
                raise RuntimeError('Failed to move to standby joints.')
        else:
            self.get_logger().info('Skip standby move because current joint state is already near target.')

        step('GRIPPER_OPEN_PREPARE')
        if not self._gripper_command(dynamic_open):
            raise RuntimeError('Failed to open gripper before picking.')

        if not self._go_pose(pre_pick, 'MOVE_PRE_PICK'):
            raise RuntimeError('Failed at pre-pick pose.')
        if not self._go_pose(pick_pose, 'MOVE_PICK'):
            raise RuntimeError('Failed at pick pose.')

        step('GRIPPER_CLOSE')
        if not self._gripper_command(dynamic_close):
            raise RuntimeError('Failed to close gripper.')

        time.sleep(float(self.cfg['safety']['settle_wait_sec']))
        close_ok, close_err = self._compute_gripper_close_ok(dynamic_close)
        result['gripper_close_ok'] = close_ok
        result['gripper_close_error_sum_rad'] = round(close_err, 6)

        if not self._go_pose(lift_pose, 'MOVE_LIFT'):
            raise RuntimeError('Failed at lift pose.')
        if not self._go_pose(pre_place, 'MOVE_PRE_PLACE'):
            raise RuntimeError('Failed at pre-place pose.')
        if not self._go_pose(place_pose, 'MOVE_PLACE'):
            raise RuntimeError('Failed at place pose.')

        step('GRIPPER_OPEN_RELEASE')
        if not self._gripper_command(dynamic_open):
            raise RuntimeError('Failed to open gripper at place pose.')

        if not self._go_pose(pre_place, 'MOVE_RETREAT'):
            raise RuntimeError('Failed at retreat pose.')

        step('RETURN_STANDBY')
        if not self._is_near_joint_goal(self.cfg['standby_joint_values_rad']):
            if not self._send_move_group(self._build_joint_goal_constraints(self.cfg['standby_joint_values_rad'])):
                raise RuntimeError('Failed to return standby.')
        else:
            self.get_logger().info('Skip return-to-standby because current joint state is already near target.')

        step('DONE')
        result['cycle_success'] = True
        self._latest_sort_cmd = None
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SortingCycleServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
