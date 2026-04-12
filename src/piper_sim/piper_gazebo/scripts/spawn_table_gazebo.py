#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dataclasses import dataclass
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity


@dataclass
class SceneObject:
    object_id: str
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    dimensions: tuple[float, float, float]


def _parse_floats(line: str, expected_count: int) -> tuple[float, ...]:
    values = [float(item) for item in line.split()]
    if len(values) != expected_count:
        raise ValueError(f'Expected {expected_count} floats, got {len(values)}: {line}')
    return tuple(values)


def _parse_scene_file(scene_file: str) -> list[SceneObject]:
    raw_lines = [line.strip() for line in Path(scene_file).read_text().splitlines()]
    lines = [line for line in raw_lines if line]
    objects: list[SceneObject] = []

    index = 0
    while index < len(lines):
        line = lines[index]
        if line == '.':
            break
        if not line.startswith('*'):
            index += 1
            continue

        object_id = line.lstrip('*').strip() or f'object_{len(objects)}'
        index += 1
        if index + 3 >= len(lines):
            break

        position = _parse_floats(lines[index], 3)
        index += 1
        orientation = _parse_floats(lines[index], 4)
        index += 1

        while index < len(lines) and lines[index].lower() not in {'box', 'sphere', 'cylinder', 'mesh'}:
            if lines[index] == '.':
                break
            index += 1
        if index >= len(lines) or lines[index] == '.':
            break

        shape = lines[index].lower()
        index += 1
        if shape != 'box':
            raise ValueError(f'Unsupported shape in scene file: {shape}')

        dimensions = _parse_floats(lines[index], 3)
        index += 1

        objects.append(
            SceneObject(
                object_id=object_id,
                position=position,
                orientation=orientation,
                dimensions=dimensions,
            )
        )

        while index < len(lines) and not lines[index].startswith('*') and lines[index] != '.':
            index += 1

    return objects


def _build_sdf(object_id: str, dimensions: tuple[float, float, float]) -> str:
    size_x, size_y, size_z = dimensions
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{object_id}'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <box>
            <size>{size_x} {size_y} {size_z}</size>
          </box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>{size_x} {size_y} {size_z}</size>
          </box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""


class TableGazeboSpawner(Node):
    def __init__(self):
        super().__init__('table_gazebo_spawner')
        self.declare_parameter('scene_file', '/home/lxf/agx_arm_ws/table.scene')
        self.scene_file = self.get_parameter('scene_file').value
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        self.timer = self.create_timer(1.0, self._try_spawn_once)
        self.sent = False

    def _try_spawn_once(self):
        if self.sent:
            return
        if not Path(self.scene_file).exists():
            self.get_logger().error(f'Scene file not found: {self.scene_file}')
            self.sent = True
            self.timer.cancel()
            return
        if not self.client.wait_for_service(timeout_sec=0.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
            return

        try:
            objects = _parse_scene_file(self.scene_file)
            if not objects:
                self.get_logger().warn(f'No spawnable objects parsed from {self.scene_file}')
                self.sent = True
                self.timer.cancel()
                return

            for item in objects:
                request = SpawnEntity.Request()
                request.name = item.object_id
                request.xml = _build_sdf(item.object_id, item.dimensions)
                request.robot_namespace = ''
                request.reference_frame = 'world'
                request.initial_pose = Pose()
                request.initial_pose.position.x, request.initial_pose.position.y, request.initial_pose.position.z = item.position
                request.initial_pose.orientation.x, request.initial_pose.orientation.y, request.initial_pose.orientation.z, request.initial_pose.orientation.w = item.orientation
                future = self.client.call_async(request)
                future.add_done_callback(lambda done_future, object_id=item.object_id: self._on_spawn_result(done_future, object_id))

            self.sent = True
            self.timer.cancel()
        except Exception as exc:
            self.get_logger().error(f'Failed to parse/spawn scene: {exc}')
            self.sent = True
            self.timer.cancel()

    def _on_spawn_result(self, future, object_id: str):
        try:
            response = future.result()
            if response is None or not response.success:
                self.get_logger().error(
                    f'Failed to spawn {object_id} in Gazebo: {getattr(response, "status_message", "unknown error")}')
            else:
                self.get_logger().info(f'Spawned {object_id} in Gazebo')
        except Exception as exc:
            self.get_logger().error(f'Failed to spawn {object_id} in Gazebo: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = TableGazeboSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
