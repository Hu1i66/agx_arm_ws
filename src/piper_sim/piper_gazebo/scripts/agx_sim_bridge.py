#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class AgxSimBridge(Node):
    def __init__(self):
        super().__init__("agx_sim_bridge")

        self.declare_parameter("effector_type", "none")
        self.effector_type = self.get_parameter("effector_type").value

        self.arm_joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]

        self.create_subscription(JointState, "/control/joint_states", self._joint_states_cb, 10)
        self.create_subscription(JointState, "/control/move_j", self._move_j_cb, 10)
        self.create_subscription(JointState, "/control/move_js", self._move_j_cb, 10)
        self.create_subscription(JointState, "/joint_states", self._sim_joint_states_cb, 10)

        self.arm_traj_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.gripper_traj_pub = self.create_publisher(
            JointTrajectory, "/gripper_controller/joint_trajectory", 10
        )
        self.feedback_joint_pub = self.create_publisher(JointState, "/feedback/joint_states", 10)

    def _safe_value(self, arr, idx, default=0.0):
        return arr[idx] if idx < len(arr) else default

    def _publish_arm_traj(self, joint_target_map):
        arm_positions = [joint_target_map.get(name, 0.0) for name in self.arm_joint_names]
        traj = JointTrajectory()
        traj.joint_names = list(self.arm_joint_names)
        point = JointTrajectoryPoint()
        point.positions = arm_positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000
        traj.points.append(point)
        self.arm_traj_pub.publish(traj)

    def _publish_gripper_traj(self, width, use_joint_name=False):
        traj = JointTrajectory()
        traj.joint_names = ["joint7"]
        point = JointTrajectoryPoint()
        if use_joint_name:
            point.positions = [width]
        else:
            # Compatibility with agx_arm_ctrl: gripper width maps to joint7 = width * 0.5
            point.positions = [width * 0.5]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000
        traj.points.append(point)
        self.gripper_traj_pub.publish(traj)

    def _move_j_cb(self, msg: JointState):
        joint_target_map = {
            name: self._safe_value(msg.position, idx)
            for idx, name in enumerate(msg.name)
            if name in self.arm_joint_names
        }
        if joint_target_map:
            self._publish_arm_traj(joint_target_map)

    def _joint_states_cb(self, msg: JointState):
        joint_target_map = {
            name: self._safe_value(msg.position, idx)
            for idx, name in enumerate(msg.name)
            if name in self.arm_joint_names
        }
        if joint_target_map:
            self._publish_arm_traj(joint_target_map)

        # Accept both legacy sim joint names and agx gripper name
        if self.effector_type == "agx_gripper":
            name_to_pos = {name: self._safe_value(msg.position, idx) for idx, name in enumerate(msg.name)}
            if "gripper" in name_to_pos:
                self._publish_gripper_traj(name_to_pos["gripper"], use_joint_name=False)
            elif "joint7" in name_to_pos:
                self._publish_gripper_traj(name_to_pos["joint7"], use_joint_name=True)

    def _sim_joint_states_cb(self, msg: JointState):
        # Republish for agx_arm_ros compatibility
        feedback = JointState()
        # Use local ROS clock timestamp to keep MoveIt current state monitor in sync
        # when simulation publishes /joint_states with sim-time stamps.
        feedback.header.stamp = self.get_clock().now().to_msg()
        # Keep only joints that exist in current robot model profile.
        if self.effector_type == "agx_gripper":
            allow_names = set(self.arm_joint_names + ["joint7", "joint8"])
        else:
            allow_names = set(self.arm_joint_names)

        feedback.name = []
        feedback.position = []
        feedback.velocity = []
        feedback.effort = []

        for idx, name in enumerate(msg.name):
            if name not in allow_names:
                continue
            feedback.name.append(name)
            feedback.position.append(self._safe_value(msg.position, idx, 0.0))
            feedback.velocity.append(self._safe_value(msg.velocity, idx, 0.0))
            feedback.effort.append(self._safe_value(msg.effort, idx, 0.0))

        # Add synthetic gripper opening width only for gripper profile.
        if self.effector_type == "agx_gripper" and "joint7" in feedback.name:
            idx = feedback.name.index("joint7")
            joint7 = feedback.position[idx] if idx < len(feedback.position) else 0.0
            feedback.name.append("gripper")
            feedback.position.append(abs(joint7) * 2.0)
            feedback.velocity.append(0.0)
            feedback.effort.append(0.0)

        self.feedback_joint_pub.publish(feedback)


def main(args=None):
    rclpy.init(args=args)
    node = AgxSimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()