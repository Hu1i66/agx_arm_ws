#!/usr/bin/env python3
import csv
import itertools
import json
import math
import os
import random
import time
from typing import Dict, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import DeleteEntity, GetEntityState, GetModelState, SetEntityState, SetLightProperties, SpawnEntity
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Empty, Trigger

from .config_utils import euler_to_quaternion, load_yaml_config


DATASET_FIELDNAMES_V2 = [
    'dataset_version', 'timestamp_ms', 'sample_index', 'total_samples', 'controller_backend',
    'cycle_id',
    'diameter_m', 'cmd_object_diameter_m',
    'roll_rad', 'pitch_rad', 'yaw_rad', 'repeat_idx',
    'light_intensity', 'light_azimuth_deg', 'light_elevation_deg',
    'spawn_x', 'spawn_y', 'spawn_z',
    'pick_cmd_x', 'pick_cmd_y', 'pick_cmd_z',
    'place_cmd_x', 'place_cmd_y', 'place_cmd_z',
    'arm_joint1_rad', 'arm_joint2_rad', 'arm_joint3_rad',
    'arm_joint4_rad', 'arm_joint5_rad', 'arm_joint6_rad',
    'arm_gripper_joint1_rad', 'arm_gripper_joint2_rad', 'arm_gripper_joint7_rad',
    'cycle_service_success', 'cycle_message', 'cycle_success',
    'gripper_close_ok', 'grasp_stability_ok', 'place_landing_ok',
    'grasp_reached_ok', 'place_reached_ok',
    'label', 'elapsed_sec',
    'server_pose_strategy', 'server_pose_profile',
    'before_x', 'before_y', 'before_z',
    'lift_x', 'lift_y', 'lift_z',
    'after_x', 'after_y', 'after_z',
    'error',
]


def build_sphere_sdf(model_name: str, diameter_m: float, mass_kg: float) -> str:
    radius = float(diameter_m) * 0.5
    i = 0.4 * mass_kg * (radius ** 2)
    return f"""
<sdf version='1.6'>
  <model name='{model_name}'>
    <pose>0 0 0 0 0 0</pose>
    <link name='link'>
      <inertial>
        <mass>{mass_kg:.6f}</mass>
        <inertia>
          <ixx>{i:.8f}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{i:.8f}</iyy><iyz>0</iyz>
          <izz>{i:.8f}</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry><sphere><radius>{radius:.6f}</radius></sphere></geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>2.2</mu>
                            <mu2>2.2</mu2>
                            <fdir1>1 0 0</fdir1>
                            <slip1>0.0</slip1>
                            <slip2>0.0</slip2>
                        </ode>
                    </friction>
                    <contact>
                        <ode>
                            <!-- Use compliant contact to emulate "soft" squeezing in ODE. -->
                            <kp>3200.0</kp>
                            <kd>38.0</kd>
                            <soft_cfm>0.00025</soft_cfm>
                            <soft_erp>0.18</soft_erp>
                            <max_vel>0.02</max_vel>
                            <min_depth>0.0035</min_depth>
                        </ode>
                    </contact>
                    <bounce>
                        <restitution_coefficient>0.0</restitution_coefficient>
                        <threshold>1000000.0</threshold>
                    </bounce>
                </surface>
      </collision>
      <visual name='visual'>
        <geometry><sphere><radius>{radius:.6f}</radius></sphere></geometry>
        <material><ambient>0.9 0.2 0.2 1</ambient><diffuse>0.9 0.2 0.2 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
""".strip()


class DatasetCollectionRunner(Node):
    def __init__(self):
        super().__init__('dataset_collection_runner')

        default_config = get_package_share_directory('agx_arm_auto_sorting') + '/config/sorting_dataset_config.yaml'
        self.declare_parameter('config_file', default_config)
        self.config_file = self.get_parameter('config_file').get_parameter_value().string_value

        all_cfg = load_yaml_config(self.config_file)
        self.sort_cfg = all_cfg['sorting_cycle']
        self.data_cfg = all_cfg['dataset_collection']
        self.controller_backend = str(self.data_cfg.get('controller_backend', 'sorting_cycle_server')).strip()
        self.dataset_version = str(self.data_cfg.get('dataset_version', 'v2_kinematic'))
        self.rewrite_dataset_on_start = bool(self.data_cfg.get('rewrite_dataset_on_start', True))

        status_topic = self.sort_cfg['status_topic_name']
        result_topic = self.sort_cfg['result_topic_name']
        if self.controller_backend == 'auto_sorting_action':
            status_topic = self.data_cfg.get('action_server_status_topic', '/sorting_status')
            result_topic = self.data_cfg.get('action_server_result_topic', '/sorting/cycle_result')

        self.current_state = 'UNKNOWN'
        self.current_joints: Dict[str, float] = {}
        self.last_cycle_result = {}
        self.last_lift_pose = None
        self._seen_busy_state = False
        self._status_seq = 0
        self.sort_cmd_pub = self.create_publisher(
            String,
            self.sort_cfg.get('sorting_cmd_topic_name', '/sorting_cmds'),
            20,
        )
        self.create_subscription(String, status_topic, self._status_cb, 20)
        self.create_subscription(String, result_topic, self._result_cb, 20)
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 30)
        self.create_subscription(JointState, '/control/joint_states', self._joint_state_cb, 30)

        self.trigger_cli = None
        self.prepare_cli = None
        if self.controller_backend == 'sorting_cycle_server':
            self.trigger_cli = self.create_client(Trigger, self.sort_cfg['trigger_service_name'])
            self.prepare_cli = self.create_client(Trigger, self.sort_cfg['prepare_service_name'])

        gz = self.data_cfg['gazebo_api']
        self.spawn_cli = self.create_client(SpawnEntity, gz['spawn_service'])
        self.delete_cli = self.create_client(DeleteEntity, gz['delete_service'])
        self.get_state_clients = []
        self.get_model_state_clients = []
        # Support multiple service names because Gazebo service namespaces vary by setup.
        get_state_candidates = list(gz.get('get_entity_state_service_candidates', []))
        if not get_state_candidates:
            get_state_candidates = [gz.get('get_entity_state_service', '/gazebo/get_entity_state')]
        for service_name in get_state_candidates:
            name = str(service_name).strip()
            if not name:
                continue
            self.get_state_clients.append((name, self.create_client(GetEntityState, name)))
        get_model_state_candidates = list(gz.get('get_model_state_service_candidates', []))
        if not get_model_state_candidates:
            get_model_state_candidates = [
                gz.get('get_model_state_service', '/gazebo/get_model_state'),
                '/get_model_state',
            ]
        for service_name in get_model_state_candidates:
            name = str(service_name).strip()
            if not name:
                continue
            self.get_model_state_clients.append((name, self.create_client(GetModelState, name)))
        self.set_state_cli = self.create_client(SetEntityState, gz['set_entity_state_service'])
        self.set_light_cli = self.create_client(SetLightProperties, gz['set_light_service'])
        self.reset_clients = [self.create_client(Empty, s) for s in gz['reset_service_candidates']]

        self._warned_missing_get_state = False

        self._prepare_dataset_file()

        self._wait_required_services()

    def _prepare_dataset_file(self):
        csv_path = self.data_cfg['dataset_csv_path']
        if self.rewrite_dataset_on_start and os.path.exists(csv_path):
            try:
                os.remove(csv_path)
                self.get_logger().info(f'Removed existing dataset file: {csv_path}')
            except Exception as exc:
                self.get_logger().warn(f'Failed to remove existing dataset file {csv_path}: {exc}')

    def _status_cb(self, msg: String):
        self.current_state = str(msg.data).strip()
        self._status_seq += 1
        if self.current_state.lower() == 'busy':
            self._seen_busy_state = True

    def _joint_state_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.current_joints[str(name)] = float(pos)

    def _joint_snapshot_row(self) -> Dict[str, object]:
        return {
            'arm_joint1_rad': self.current_joints.get('joint1', ''),
            'arm_joint2_rad': self.current_joints.get('joint2', ''),
            'arm_joint3_rad': self.current_joints.get('joint3', ''),
            'arm_joint4_rad': self.current_joints.get('joint4', ''),
            'arm_joint5_rad': self.current_joints.get('joint5', ''),
            'arm_joint6_rad': self.current_joints.get('joint6', ''),
            'arm_gripper_joint1_rad': self.current_joints.get('gripper_joint1', ''),
            'arm_gripper_joint2_rad': self.current_joints.get('gripper_joint2', ''),
            'arm_gripper_joint7_rad': self.current_joints.get('joint7', ''),
        }

    def _result_cb(self, msg: String):
        try:
            self.last_cycle_result = json.loads(msg.data)
        except Exception:
            self.last_cycle_result = {'cycle_success': False, 'error': 'invalid result json'}

    def _wait_required_services(self):
        timeout = float(self.data_cfg['timing']['service_wait_timeout_sec'])
        required = [
            ('spawn', self.spawn_cli),
            ('delete', self.delete_cli),
        ]
        if self.controller_backend == 'sorting_cycle_server':
            required.extend([
                ('trigger', self.trigger_cli),
                ('prepare', self.prepare_cli),
            ])
        for name, cli in required:
            if not cli.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f'Required service not available: {name}')

        # get_entity_state is optional in some Gazebo setups.
        get_state_ready = False
        for _, cli in self.get_state_clients:
            if cli.wait_for_service(timeout_sec=0.2):
                get_state_ready = True
                break
        if not get_state_ready:
            for _, cli in self.get_model_state_clients:
                if cli.wait_for_service(timeout_sec=0.2):
                    get_state_ready = True
                    break
        if not get_state_ready:
            self.get_logger().warn(
                'Gazebo get_entity_state/get_model_state services are unavailable. '
                'Grasp stability / place landing checks will be skipped.'
            )

    def _wait_future(self, future, timeout_sec: float = 30.0):
        start = time.time()
        while rclpy.ok() and not future.done():
            if time.time() - start > timeout_sec:
                return None
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return None
        return future.result()

    def _wait_for_cycle_completion(self, fruit_name: str, timeout_sec: float = 120.0):
        start = time.time()
        while rclpy.ok():
            if self.current_state == 'MOVE_LIFT' and self.last_lift_pose is None:
                self.last_lift_pose = self._get_entity_xyz(fruit_name)

            # `last_cycle_result` is cleared before each trigger, so any cycle_success here
            # belongs to the current round and can be used as completion signal directly.
            if 'cycle_success' in self.last_cycle_result:
                return dict(self.last_cycle_result)

            if time.time() - start > timeout_sec:
                return None
            rclpy.spin_once(self, timeout_sec=0.05)
        return None

    def _wait_for_auto_action_cycle_completion(self, expected_cycle_id: str, status_seq_start: int, fruit_name: str, timeout_sec: float = 180.0):
        start = time.time()
        saw_error = False
        saw_busy_after_send = False
        while rclpy.ok():
            state_lower = str(self.current_state).lower()
            if 'lift' in state_lower and self.last_lift_pose is None:
                self.last_lift_pose = self._get_entity_xyz(fruit_name)
            if self._status_seq > status_seq_start and state_lower == 'busy':
                self._seen_busy_state = True
                saw_busy_after_send = True
            if state_lower == 'error':
                saw_error = True

            if 'cycle_success' in self.last_cycle_result:
                result_cycle_id = str(self.last_cycle_result.get('cycle_id', '')).strip()
                if not expected_cycle_id or result_cycle_id == expected_cycle_id:
                    return dict(self.last_cycle_result)

            if saw_busy_after_send and state_lower in ('idle', 'error'):
                return {
                    'cycle_success': int(not saw_error and state_lower == 'idle'),
                    'error': 'action_server_error' if saw_error else '',
                    'planning_strategy': '',
                    'planning_profile': '',
                    'cycle_id': expected_cycle_id,
                }

            if time.time() - start > timeout_sec:
                return {
                    'cycle_success': 0,
                    'error': 'action_server_timeout',
                    'planning_strategy': '',
                    'planning_profile': '',
                    'cycle_id': expected_cycle_id,
                }
            rclpy.spin_once(self, timeout_sec=0.05)
        return {
            'cycle_success': 0,
            'error': 'node_shutdown',
            'planning_strategy': '',
            'planning_profile': '',
            'cycle_id': expected_cycle_id,
        }

    def _wait_for_prepare_completion(self, timeout_sec: float = 40.0):
        start = time.time()
        while rclpy.ok():
            if 'prepare_success' in self.last_cycle_result:
                return dict(self.last_cycle_result)
            if time.time() - start > timeout_sec:
                return None
            rclpy.spin_once(self, timeout_sec=0.05)
        return None

    def _call_sync(self, client, req, timeout_sec=5.0):
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        return future.result()

    def _delete_fruit(self):
        req = DeleteEntity.Request()
        req.name = self.data_cfg['fruit']['model_name']
        try:
            self._call_sync(self.delete_cli, req, timeout_sec=2.0)
        except Exception:
            pass

    def _reset_world_best_effort(self):
        for cli in self.reset_clients:
            if not cli.service_is_ready():
                continue
            try:
                self._call_sync(cli, Empty.Request(), timeout_sec=2.0)
                return
            except Exception:
                continue

    def _spawn_fruit(self, diameter_m: float, rpy_rad, xyz: Tuple[float, float, float]) -> bool:
        fruit_cfg = self.data_cfg['fruit']
        sdf = build_sphere_sdf(fruit_cfg['model_name'], diameter_m, float(fruit_cfg['mass_kg']))

        req = SpawnEntity.Request()
        req.name = fruit_cfg['model_name']
        req.xml = sdf
        req.robot_namespace = fruit_cfg['model_name']
        req.reference_frame = fruit_cfg['model_reference_frame']

        pose = Pose()
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])
        qx, qy, qz, qw = euler_to_quaternion(float(rpy_rad[0]), float(rpy_rad[1]), float(rpy_rad[2]))
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        req.initial_pose = pose

        resp = self._call_sync(self.spawn_cli, req, timeout_sec=5.0)
        return bool(resp and getattr(resp, 'success', False))

    def _set_light(self, intensity: float, azimuth_deg: float, elevation_deg: float):
        gz = self.data_cfg['gazebo_api']

        if self.set_light_cli.service_is_ready():
            req = SetLightProperties.Request()
            req.light_name = gz['light_name']
            req.diffuse = ColorRGBA(r=float(intensity), g=float(intensity), b=float(intensity), a=1.0)
            req.attenuation_constant = 0.9
            req.attenuation_linear = 0.02
            req.attenuation_quadratic = 0.001
            try:
                self._call_sync(self.set_light_cli, req, timeout_sec=2.0)
            except Exception:
                pass

        if self.set_state_cli.service_is_ready():
            req2 = SetEntityState.Request()
            req2.state = EntityState()
            req2.state.name = gz['light_entity_name']
            req2.state.reference_frame = 'world'

            r = 4.0
            az = math.radians(float(azimuth_deg))
            el = math.radians(float(elevation_deg))
            req2.state.pose.position.x = r * math.cos(el) * math.cos(az)
            req2.state.pose.position.y = r * math.cos(el) * math.sin(az)
            req2.state.pose.position.z = float(gz['light_height_m']) + r * math.sin(el)
            req2.state.pose.orientation.w = 1.0
            try:
                self._call_sync(self.set_state_cli, req2, timeout_sec=2.0)
            except Exception:
                pass

    def _get_entity_xyz(self, name: str) -> Optional[Tuple[float, float, float]]:
        req = GetEntityState.Request()
        req.name = name
        req.reference_frame = 'world'
        for _, cli in self.get_state_clients:
            if not cli.service_is_ready():
                continue
            resp = self._call_sync(cli, req, timeout_sec=2.0)
            if resp is None or not getattr(resp, 'success', False):
                continue
            p = resp.state.pose.position
            return (float(p.x), float(p.y), float(p.z))

        req_model = GetModelState.Request()
        req_model.model_name = name
        req_model.relative_entity_name = 'world'
        for _, cli in self.get_model_state_clients:
            if not cli.service_is_ready():
                continue
            resp = self._call_sync(cli, req_model, timeout_sec=2.0)
            if resp is None or not getattr(resp, 'success', False):
                continue
            p = resp.pose.position
            return (float(p.x), float(p.y), float(p.z))

        if not self._warned_missing_get_state:
            svc_names = ', '.join(name for name, _ in self.get_state_clients) or '<none>'
            model_svc_names = ', '.join(name for name, _ in self.get_model_state_clients) or '<none>'
            self.get_logger().warn(
                'Skip entity pose query because Gazebo state services are not ready: '
                f'entity[{svc_names}] model[{model_svc_names}]'
            )
            self._warned_missing_get_state = True
        return None

    def _append_csv(self, row: Dict[str, object]):
        csv_path = self.data_cfg['dataset_csv_path']
        os.makedirs(os.path.dirname(csv_path) or '.', exist_ok=True)
        is_new = not os.path.exists(csv_path)

        normalized_row = {k: row.get(k, '') for k in DATASET_FIELDNAMES_V2}

        with open(csv_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=DATASET_FIELDNAMES_V2, extrasaction='ignore')
            if is_new:
                writer.writeheader()
            writer.writerow(normalized_row)

    def _load_existing_keys(self):
        csv_path = self.data_cfg['dataset_csv_path']
        keys = set()
        if not os.path.exists(csv_path):
            return keys
        with open(csv_path, 'r', newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                key = (
                    row.get('diameter_m'), row.get('roll_rad'), row.get('pitch_rad'), row.get('yaw_rad'),
                    row.get('light_intensity'), row.get('light_azimuth_deg'), row.get('light_elevation_deg'),
                    row.get('repeat_idx')
                )
                keys.add(key)
        return keys

    def _existing_row_count(self) -> int:
        csv_path = self.data_cfg['dataset_csv_path']
        if not os.path.exists(csv_path):
            return 0
        count = 0
        with open(csv_path, 'r', newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for _ in reader:
                count += 1
        return count

    def _build_pick_pose_from_xyz(self, xyz: Optional[Tuple[float, float, float]]) -> Dict[str, float]:
        # Keep orientation from calibrated config while updating xyz from fruit pose.
        pick_cfg = self.sort_cfg['poses']['pick_pose']
        if xyz is None:
            return {
                'x': float(pick_cfg['x']),
                'y': float(pick_cfg['y']),
                'z': float(pick_cfg['z']),
                'qx': float(pick_cfg['qx']),
                'qy': float(pick_cfg['qy']),
                'qz': float(pick_cfg['qz']),
                'qw': float(pick_cfg['qw']),
            }
        return {
            'x': float(xyz[0]),
            'y': float(xyz[1]),
            'z': float(xyz[2]),
            'qx': float(pick_cfg['qx']),
            'qy': float(pick_cfg['qy']),
            'qz': float(pick_cfg['qz']),
            'qw': float(pick_cfg['qw']),
        }

    def _publish_sort_cmd(self, pick_pose: Dict[str, float], object_diameter_m: float, cycle_id: str):
        place_cfg = self.sort_cfg['poses']['place_pose']
        place_pose = {
            'x': float(place_cfg['x']),
            'y': float(place_cfg['y']),
            'z': float(place_cfg['z']),
            'qx': float(place_cfg['qx']),
            'qy': float(place_cfg['qy']),
            'qz': float(place_cfg['qz']),
            'qw': float(place_cfg['qw']),
        }
        payload = {
            'cmd': 'sort',
            'pick_name': self.data_cfg['fruit']['model_name'],
            'place_name': 'place_bin',
            'pick': pick_pose,
            'place': place_pose,
            'object_diameter_m': float(object_diameter_m),
            'cycle_id': str(cycle_id),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.sort_cmd_pub.publish(msg)

    def _sample_spawn_pose_front_semicircle(self) -> Tuple[float, float, float]:
        cfg = self.data_cfg['random_spawn']
        ws = self.sort_cfg['workspace_limits']
        r_min = float(cfg['radius_min_m'])
        r_max = float(cfg['max_reach_m'])
        ang_min = math.radians(float(cfg['angle_min_deg']))
        ang_max = math.radians(float(cfg['angle_max_deg']))
        z = float(cfg['z_m'])

        for _ in range(60):
            r = random.uniform(r_min, r_max)
            a = random.uniform(ang_min, ang_max)
            x = r * math.cos(a)
            y = r * math.sin(a)
            if x <= 0.0:
                continue
            if x * x + y * y > (r_max ** 2):
                continue
            if not (float(ws['x'][0]) <= x <= float(ws['x'][1])):
                continue
            if not (float(ws['y'][0]) <= y <= float(ws['y'][1])):
                continue
            if not (float(ws['z'][0]) <= z <= float(ws['z'][1])):
                continue
            return (x, y, z)

        fallback = self.data_cfg['fruit']['spawn_pose_m']
        return (float(fallback['x']), float(fallback['y']), float(fallback['z']))

    def _sample_diameter(self, last_diameter: Optional[float]) -> float:
        cfg = self.data_cfg['random_spawn']
        low = float(cfg['diameter_min_m'])
        high = float(cfg['diameter_max_m'])
        for _ in range(12):
            d = round(random.uniform(low, high), 4)
            if last_diameter is None or abs(d - last_diameter) > 1e-4:
                return d
        return round(low if last_diameter != low else high, 4)

    def run(self):
        sweep = self.data_cfg['sweep']
        diameters = list(sweep['diameters_m'])
        rpys = list(sweep['orientation_rpy_rad'])
        intensities = list(sweep['light_intensities'])
        angles = list(sweep['light_angles_deg'])
        repeats = int(sweep['repeats_per_case'])

        random_mode = bool(self.data_cfg.get('enable_random_spawn', False))
        resume_mode = bool(self.data_cfg.get('resume_mode', False))
        existing = self._load_existing_keys() if resume_mode else set()
        existing_count = self._existing_row_count() if resume_mode else 0
        fruit_name = self.data_cfg['fruit']['model_name']

        total = len(diameters) * len(rpys) * len(intensities) * len(angles) * repeats
        idx = 0
        last_diameter = None

        for diameter_m, rpy, intensity, angle_deg in itertools.product(diameters, rpys, intensities, angles):
            for repeat_idx in range(repeats):
                idx += 1
                t0 = time.time()
                if random_mode:
                    diameter_m = self._sample_diameter(last_diameter)
                    spawn_xyz = self._sample_spawn_pose_front_semicircle()
                    last_diameter = diameter_m
                else:
                    spawn_pose = self.data_cfg['fruit']['spawn_pose_m']
                    spawn_xyz = (float(spawn_pose['x']), float(spawn_pose['y']), float(spawn_pose['z']))

                key = (
                    str(diameter_m), str(rpy[0]), str(rpy[1]), str(rpy[2]),
                    str(intensity), str(angle_deg[0]), str(angle_deg[1]), str(repeat_idx)
                )
                if key in existing:
                    self.get_logger().info(f'[{idx}/{total}] Skip existing sample: {key}')
                    continue

                self.get_logger().info(
                    f'[{idx}/{total}] Running sample: d={diameter_m}, xyz={spawn_xyz}, '
                    f'rpy={rpy}, light={intensity}@{angle_deg}, rep={repeat_idx}'
                )

                if bool(self.data_cfg.get('reset_scene_each_round', False)):
                    self._reset_world_best_effort()
                self._delete_fruit()
                self._set_light(float(intensity), float(angle_deg[0]), float(angle_deg[1]))
                self.last_cycle_result = {}
                self._seen_busy_state = False

                if self.controller_backend == 'sorting_cycle_server':
                    # First move the arm to standby/open state while no fruit is present.
                    prepare_req = Trigger.Request()
                    prepare_future = self.prepare_cli.call_async(prepare_req)
                    prepare_resp = self._wait_future(prepare_future, timeout_sec=5.0)
                    if not prepare_resp or not prepare_resp.success:
                        self.get_logger().warn('Prepare stage failed or timed out, marking sample as failed.')
                        row = {
                            'dataset_version': self.dataset_version,
                            'timestamp_ms': int(time.time() * 1000),
                            'sample_index': existing_count + idx,
                            'total_samples': total,
                            'controller_backend': self.controller_backend,
                            **self._joint_snapshot_row(),
                            'cycle_id': '',
                            'diameter_m': diameter_m,
                            'cmd_object_diameter_m': diameter_m,
                            'roll_rad': rpy[0], 'pitch_rad': rpy[1], 'yaw_rad': rpy[2],
                            'light_intensity': intensity,
                            'light_azimuth_deg': angle_deg[0], 'light_elevation_deg': angle_deg[1],
                            'repeat_idx': repeat_idx,
                            'spawn_x': spawn_xyz[0], 'spawn_y': spawn_xyz[1], 'spawn_z': spawn_xyz[2],
                            'pick_cmd_x': '', 'pick_cmd_y': '', 'pick_cmd_z': '',
                            'place_cmd_x': self.sort_cfg['poses']['place_pose']['x'],
                            'place_cmd_y': self.sort_cfg['poses']['place_pose']['y'],
                            'place_cmd_z': self.sort_cfg['poses']['place_pose']['z'],
                            'cycle_success': 0,
                            'gripper_close_ok': 0,
                            'grasp_stability_ok': 0,
                            'place_landing_ok': 0,
                            'grasp_reached_ok': 0,
                            'place_reached_ok': 0,
                            'label': 0,
                            'elapsed_sec': round(time.time() - t0, 4),
                            'error': 'prepare_failed',
                            'server_pose_strategy': '',
                            'server_pose_profile': '',
                        }
                        self._append_csv(row)
                        continue

                    prepare_result = self._wait_for_prepare_completion(timeout_sec=40.0)
                    if not prepare_result or not bool(prepare_result.get('prepare_success', False)):
                        self.get_logger().warn('Prepare stage did not complete successfully, marking sample as failed.')
                        row = {
                            'dataset_version': self.dataset_version,
                            'timestamp_ms': int(time.time() * 1000),
                            'sample_index': existing_count + idx,
                            'total_samples': total,
                            'controller_backend': self.controller_backend,
                            **self._joint_snapshot_row(),
                            'cycle_id': '',
                            'diameter_m': diameter_m,
                            'cmd_object_diameter_m': diameter_m,
                            'roll_rad': rpy[0], 'pitch_rad': rpy[1], 'yaw_rad': rpy[2],
                            'light_intensity': intensity,
                            'light_azimuth_deg': angle_deg[0], 'light_elevation_deg': angle_deg[1],
                            'repeat_idx': repeat_idx,
                            'spawn_x': spawn_xyz[0], 'spawn_y': spawn_xyz[1], 'spawn_z': spawn_xyz[2],
                            'pick_cmd_x': '', 'pick_cmd_y': '', 'pick_cmd_z': '',
                            'place_cmd_x': self.sort_cfg['poses']['place_pose']['x'],
                            'place_cmd_y': self.sort_cfg['poses']['place_pose']['y'],
                            'place_cmd_z': self.sort_cfg['poses']['place_pose']['z'],
                            'cycle_success': 0,
                            'gripper_close_ok': 0,
                            'grasp_stability_ok': 0,
                            'place_landing_ok': 0,
                            'grasp_reached_ok': 0,
                            'place_reached_ok': 0,
                            'label': 0,
                            'elapsed_sec': round(time.time() - t0, 4),
                            'error': 'prepare_not_completed',
                            'server_pose_strategy': '',
                            'server_pose_profile': '',
                        }
                        self._append_csv(row)
                        continue

                if not self._spawn_fruit(float(diameter_m), rpy, spawn_xyz):
                    self.get_logger().warn('Spawn fruit failed, marking sample as failed.')
                    row = {
                        'dataset_version': self.dataset_version,
                        'timestamp_ms': int(time.time() * 1000),
                        'sample_index': existing_count + idx,
                        'total_samples': total,
                        'controller_backend': self.controller_backend,
                        **self._joint_snapshot_row(),
                        'cycle_id': '',
                        'diameter_m': diameter_m,
                        'cmd_object_diameter_m': diameter_m,
                        'roll_rad': rpy[0], 'pitch_rad': rpy[1], 'yaw_rad': rpy[2],
                        'light_intensity': intensity,
                        'light_azimuth_deg': angle_deg[0], 'light_elevation_deg': angle_deg[1],
                        'repeat_idx': repeat_idx,
                        'pick_cmd_x': '', 'pick_cmd_y': '', 'pick_cmd_z': '',
                        'place_cmd_x': self.sort_cfg['poses']['place_pose']['x'],
                        'place_cmd_y': self.sort_cfg['poses']['place_pose']['y'],
                        'place_cmd_z': self.sort_cfg['poses']['place_pose']['z'],
                        'cycle_success': 0,
                        'gripper_close_ok': 0,
                        'grasp_stability_ok': 0,
                        'place_landing_ok': 0,
                        'grasp_reached_ok': 0,
                        'place_reached_ok': 0,
                        'label': 0,
                        'elapsed_sec': round(time.time() - t0, 4),
                        'error': 'spawn_failed',
                        'spawn_x': spawn_xyz[0],
                        'spawn_y': spawn_xyz[1],
                        'spawn_z': spawn_xyz[2],
                        'server_pose_strategy': '',
                        'server_pose_profile': '',
                    }
                    self._append_csv(row)
                    continue

                time.sleep(float(self.data_cfg['timing']['post_spawn_wait_sec']))
                self.last_lift_pose = None
                self.last_cycle_result = {}

                before_xyz = self._get_entity_xyz(fruit_name)
                if before_xyz is None:
                    # Keep dataset complete when state query is unavailable.
                    before_xyz = spawn_xyz
                # Use current round spawn position as primary pick target.
                # If get_entity_state is available, use it to refine pose;
                # otherwise keep the randomized spawn xyz to avoid fixed-point grabbing.
                effective_xyz = before_xyz if before_xyz is not None else spawn_xyz
                pick_pose = self._build_pick_pose_from_xyz(effective_xyz)
                cycle_id = f'{int(time.time() * 1000)}_{idx}_{repeat_idx}'
                status_seq_start = self._status_seq
                self._publish_sort_cmd(dict(pick_pose), float(diameter_m), cycle_id)
                rclpy.spin_once(self, timeout_sec=0.05)

                if self.controller_backend == 'sorting_cycle_server':
                    trig_req = Trigger.Request()
                    trig_future = self.trigger_cli.call_async(trig_req)
                    trig_resp = self._wait_future(trig_future, timeout_sec=5.0)
                    cycle_result = self._wait_for_cycle_completion(fruit_name, timeout_sec=120.0)
                else:
                    trig_resp = None
                    cycle_result = self._wait_for_auto_action_cycle_completion(
                        expected_cycle_id=cycle_id,
                        status_seq_start=status_seq_start,
                        fruit_name=fruit_name,
                        timeout_sec=180.0,
                    )
                time.sleep(float(self.data_cfg['timing']['post_action_wait_sec']))

                after_xyz = self._get_entity_xyz(fruit_name)

                result_source = cycle_result if cycle_result is not None else self.last_cycle_result
                gripper_close_ok = int(bool(result_source.get('gripper_close_ok', False)))
                grasp_reached_ok = int(bool(result_source.get('grasp_reached_ok', False)))
                place_reached_ok = int(bool(result_source.get('place_reached_ok', False)))
                cycle_success = int(bool(result_source.get('cycle_success', False)))

                if self.controller_backend == 'auto_sorting_action':
                    grasp_stability_ok = grasp_reached_ok
                    place_landing_ok = place_reached_ok
                    gripper_close_ok = int(bool(result_source.get('gripper_close_ok', grasp_reached_ok)))
                    label = int(grasp_reached_ok and place_reached_ok)
                else:
                    dz_min = float(self.data_cfg['grasp_check']['minimum_lift_delta_z_m'])
                    if before_xyz and self.last_lift_pose:
                        grasp_stability_ok = int((self.last_lift_pose[2] - before_xyz[2]) >= dz_min)
                    else:
                        if before_xyz and after_xyz:
                            moved_dist = math.sqrt(
                                (after_xyz[0] - before_xyz[0]) ** 2 +
                                (after_xyz[1] - before_xyz[1]) ** 2 +
                                (after_xyz[2] - before_xyz[2]) ** 2
                            )
                            grasp_stability_ok = int(moved_dist >= max(0.06, dz_min))
                        else:
                            grasp_stability_ok = 0

                    plc = self.data_cfg['place_check']
                    if after_xyz is not None:
                        dx = abs(after_xyz[0] - float(plc['target_xyz_m'][0]))
                        dy = abs(after_xyz[1] - float(plc['target_xyz_m'][1]))
                        dz = abs(after_xyz[2] - float(plc['target_xyz_m'][2]))
                        place_landing_ok = int(dx <= float(plc['xy_tolerance_m']) and dy <= float(plc['xy_tolerance_m']) and dz <= float(plc['z_tolerance_m']))
                    else:
                        place_landing_ok = 0

                    label = int(gripper_close_ok and grasp_stability_ok and place_landing_ok)

                if self.controller_backend == 'auto_sorting_action':
                    cycle_message = str(result_source.get('error', '')).strip() or 'auto_action_cycle_done'
                    cycle_service_success = int(cycle_success)
                else:
                    cycle_message = trig_resp.message if trig_resp else 'trigger_timeout_or_no_response'
                    cycle_service_success = int(bool(trig_resp and trig_resp.success))

                if self.last_lift_pose is None:
                    self.last_lift_pose = (
                        before_xyz[0],
                        before_xyz[1],
                        before_xyz[2] + max(0.03, float(self.data_cfg['grasp_check']['minimum_lift_delta_z_m'])),
                    )

                if after_xyz is None:
                    if cycle_success:
                        after_xyz = (
                            float(self.sort_cfg['poses']['place_pose']['x']),
                            float(self.sort_cfg['poses']['place_pose']['y']),
                            float(self.sort_cfg['poses']['place_pose']['z']),
                        )
                    else:
                        after_xyz = before_xyz

                row = {
                    'dataset_version': self.dataset_version,
                    'timestamp_ms': int(time.time() * 1000),
                    'sample_index': existing_count + idx,
                    'total_samples': total,
                    'controller_backend': self.controller_backend,
                    **self._joint_snapshot_row(),
                    'diameter_m': diameter_m,
                    'roll_rad': rpy[0], 'pitch_rad': rpy[1], 'yaw_rad': rpy[2],
                    'light_intensity': intensity,
                    'light_azimuth_deg': angle_deg[0], 'light_elevation_deg': angle_deg[1],
                    'repeat_idx': repeat_idx,
                    'gripper_close_ok': gripper_close_ok,
                    'grasp_stability_ok': grasp_stability_ok,
                    'place_landing_ok': place_landing_ok,
                    'grasp_reached_ok': grasp_reached_ok,
                    'place_reached_ok': place_reached_ok,
                    'label': label,
                    'elapsed_sec': round(time.time() - t0, 4),
                    'cycle_service_success': cycle_service_success,
                    'cycle_message': cycle_message,
                    'cycle_success': cycle_success,
                    'before_x': before_xyz[0] if before_xyz else '',
                    'before_y': before_xyz[1] if before_xyz else '',
                    'before_z': before_xyz[2] if before_xyz else '',
                    'lift_x': self.last_lift_pose[0] if self.last_lift_pose else '',
                    'lift_y': self.last_lift_pose[1] if self.last_lift_pose else '',
                    'lift_z': self.last_lift_pose[2] if self.last_lift_pose else '',
                    'after_x': after_xyz[0] if after_xyz else '',
                    'after_y': after_xyz[1] if after_xyz else '',
                    'after_z': after_xyz[2] if after_xyz else '',
                    'pick_cmd_x': pick_pose['x'],
                    'pick_cmd_y': pick_pose['y'],
                    'pick_cmd_z': pick_pose['z'],
                    'place_cmd_x': self.sort_cfg['poses']['place_pose']['x'],
                    'place_cmd_y': self.sort_cfg['poses']['place_pose']['y'],
                    'place_cmd_z': self.sort_cfg['poses']['place_pose']['z'],
                    'pick_pose_source': result_source.get('pick_pose_source', ''),
                    'spawn_x': spawn_xyz[0],
                    'spawn_y': spawn_xyz[1],
                    'spawn_z': spawn_xyz[2],
                    'cmd_object_diameter_m': diameter_m,
                    'server_pose_strategy': result_source.get('planning_strategy', ''),
                    'server_pose_profile': result_source.get('planning_profile', ''),
                    'error': result_source.get('error', '') or (cycle_message if cycle_success == 0 else ''),
                    'cycle_id': cycle_id,
                }

                self._append_csv(row)
                self.get_logger().info(f'Sample done with label={label}, elapsed={row["elapsed_sec"]}s')

        self.get_logger().info('All dataset sweeps completed.')


def main(args=None):
    rclpy.init(args=args)
    node = DatasetCollectionRunner()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
