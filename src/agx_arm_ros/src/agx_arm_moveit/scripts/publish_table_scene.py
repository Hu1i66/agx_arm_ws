#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dataclasses import dataclass
from pathlib import Path
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


@dataclass
class SceneObject:
    object_id: str
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    shape: str
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
                shape=shape,
                dimensions=dimensions,
            )
        )

        while index < len(lines) and not lines[index].startswith('*') and lines[index] != '.':
            index += 1

    return objects


class TableScenePublisher(Node):
    def __init__(self):
        super().__init__('table_scene_publisher')
        self.declare_parameter('scene_file', '/home/lxf/agx_arm_ws/table.scene')
        self.scene_file = self.get_parameter('scene_file').value
        self.client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.timer = self.create_timer(1.0, self._try_apply_once)
        self.sent = False

    def _build_scene(self, objects: list[SceneObject]) -> PlanningScene:
        scene = PlanningScene()
        scene.is_diff = True
        for item in objects:
            collision_object = CollisionObject()
            collision_object.id = item.object_id
            collision_object.header.frame_id = 'world'
            collision_object.operation = CollisionObject.ADD

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [item.dimensions[0], item.dimensions[1], item.dimensions[2]]
            collision_object.primitives = [primitive]

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = item.position
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = item.orientation
            collision_object.primitive_poses = [pose]
            scene.world.collision_objects.append(collision_object)
        return scene

    def _try_apply_once(self):
        if self.sent:
            return
        if not Path(self.scene_file).exists():
            self.get_logger().error(f'Scene file not found: {self.scene_file}')
            self.sent = True
            self.timer.cancel()
            return
        if not self.client.wait_for_service(timeout_sec=0.0):
            self.get_logger().info('Waiting for /apply_planning_scene service...')
            return

        try:
            objects = _parse_scene_file(self.scene_file)
            if not objects:
                self.get_logger().warn(f'No collision objects parsed from {self.scene_file}')
                self.sent = True
                self.timer.cancel()
                return

            request = ApplyPlanningScene.Request()
            request.scene = self._build_scene(objects)
            future = self.client.call_async(request)
            future.add_done_callback(self._on_result)
            self.sent = True
            self.timer.cancel()
            self.get_logger().info(f'Applied {len(objects)} object(s) from {self.scene_file}')
        except Exception as exc:
            self.get_logger().error(f'Failed to parse/apply scene: {exc}')
            self.sent = True
            self.timer.cancel()

    def _on_result(self, future):
        try:
            response = future.result()
            if response is None or not response.success:
                self.get_logger().error('ApplyPlanningScene service reported failure')
            else:
                self.get_logger().info('Planning scene updated successfully')
        except Exception as exc:
            self.get_logger().error(f'ApplyPlanningScene request failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = TableScenePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
