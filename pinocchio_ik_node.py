#!/usr/bin/env python3
"""
Pinocchio-based IK Solver Node (ROS2 rclpy)

Provides inverse kinematics solution using Pinocchio 3.6.0 + CasADi optimization.
Input: PoseCmd (target Cartesian pose)
Output: IKSolution (joint angles)

Dependencies: pinocchio, casadi, numpy
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import time
import traceback
from typing import Tuple, Optional, Dict

import sys
import os
_cmeel_path = "/home/lxf/.local/lib/python3.10/site-packages/cmeel.prefix/lib/python3.10/site-packages"
if os.path.exists(_cmeel_path) and _cmeel_path not in sys.path:
    sys.path.insert(0, _cmeel_path)

try:
    import pinocchio as pin
    PINOCCHIO_AVAILABLE = True
except ImportError:
    PINOCCHIO_AVAILABLE = False
    print("⚠️ Warning: pinocchio not installed. Install with: pip install pin")

try:
    import casadi
    CASADI_AVAILABLE = True
except ImportError:
    CASADI_AVAILABLE = False
    print("⚠️ Warning: casadi not installed. Install with: pip install casadi")

from agx_arm_msgs.msg import PoseCmd, IKSolution
from sensor_msgs.msg import JointState


class PinocchioIKSolver:
    """
    Core IK solver using Pinocchio + CasADi
    """
    
    def __init__(self, urdf_path: str, end_effector_name: str = "link6"):
        """
        Initialize IK solver from URDF.
        
        Args:
            urdf_path: Path to robot URDF file
            end_effector_name: Name of end-effector link in URDF
        """
        if not PINOCCHIO_AVAILABLE:
            raise RuntimeError("Pinocchio not available. Install: pip install pinocchio")
        if not CASADI_AVAILABLE:
            raise RuntimeError("CasADi not available. Install: pip install casadi")
        
        self.urdf_path = urdf_path
        self.ee_name = end_effector_name
        
        try:
            # Load robot model from URDF
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
            
            # Get end-effector frame ID
            self.ee_frame_id = self.model.getFrameId(end_effector_name)
            
            # Extract relevant joint names (joints 1-6 only)
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
                    print(f"⚠️ Joint {joint_name} not found in model")
            
            if len(self.joint_ids) < 6:
                raise RuntimeError(f"Expected 6 DOF joints, found {len(self.joint_ids)}")
            
            self.ndof = len(self.joint_ids)
            print(f"✅ Pinocchio model loaded: {self.ndof} DOF, EE frame: {end_effector_name}")
            
        except Exception as e:
            print(f"❌ Failed to load URDF: {e}")
            raise
    
    def get_ik_solution(
        self,
        target_pos: np.ndarray,
        target_quat: np.ndarray,
        initial_guess: Optional[np.ndarray] = None,
        max_iter: int = 200,
        tol: float = 1e-4
    ) -> Tuple[bool, np.ndarray, float, float]:
        """
        Solve IK using CasADi optimization (based on Pinocchio kinematics).
        
        Args:
            target_pos: Target position [x, y, z]
            target_quat: Target quaternion [qx, qy, qz, qw]
            initial_guess: Initial joint angle guess (6D), default to zeros
            max_iter: Maximum CasADi optimizer iterations
            tol: Convergence tolerance
        
        Returns:
            (success, joint_angles, error, computation_time)
        """
        t0 = time.time()
        
        q = pin.neutral(self.model)
        if initial_guess is not None:
            for i, idx in enumerate(self.joint_q_indices):
                q[idx] = float(initial_guess[i])
        
        try:
            # Use Pinocchio iterative IK (damped least squares) for robustness in ROS2 runtime.
            target_quat = np.array(target_quat, dtype=float)
            target_quat = target_quat / max(np.linalg.norm(target_quat), 1e-9)
            target_se3 = pin.XYZQUATToSE3(
                np.array([
                    float(target_pos[0]), float(target_pos[1]), float(target_pos[2]),
                    float(target_quat[0]), float(target_quat[1]), float(target_quat[2]), float(target_quat[3])
                ])
            )

            damping = 1e-6
            step_scale = 0.6
            final_error = 999.0

            for _ in range(max_iter):
                pin.forwardKinematics(self.model, self.data, q)
                pin.updateFramePlacements(self.model, self.data)

                current_se3 = self.data.oMf[self.ee_frame_id]
                err6 = pin.log6(current_se3.actInv(target_se3)).vector
                final_error = float(np.linalg.norm(err6))
                if final_error < tol:
                    break

                j6 = pin.computeFrameJacobian(
                    self.model, self.data, q, self.ee_frame_id, pin.ReferenceFrame.LOCAL
                )
                a = j6 @ j6.T + damping * np.eye(6)
                dq = -j6.T @ np.linalg.solve(a, err6)
                q = pin.integrate(self.model, q, step_scale * dq)

            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            current_pose = self.data.oMf[self.ee_frame_id]
            pos_error = np.linalg.norm(current_pose.translation - target_pos)

            q_sol = np.array([q[idx] for idx in self.joint_q_indices], dtype=float)
            success = bool(pos_error < 0.01 and final_error < 0.05)
            
            t1 = time.time()
            return success, q_sol, final_error, (t1 - t0)
            
        except Exception as e:
            print(f"❌ IK solver error: {e}")
            traceback.print_exc()
            t1 = time.time()
            return False, np.zeros(self.ndof), 999.0, (t1 - t0)


class PinocchioIKNode(Node):
    """
    ROS2 node wrapper for Pinocchio IK solver
    """
    
    def __init__(self):
        super().__init__('pinocchio_ik_solver')
        self.declare_parameter('enable_ik', True)
        self.declare_parameter('urdf_path', '/home/lxf/agx_arm_ws/src/agx_arm_ros/src/agx_arm_moveit/config/test2.urdf')
        self.declare_parameter('ee_frame', 'link6')
        
        # Node configuration
        self.enable_ik = False  # Enable/disable IK solving on this node
        self.solver: Optional[PinocchioIKSolver] = None
        self.current_joints: Dict[str, float] = {}
        
        # Try to initialize solver
        self._init_solver()
        
        # Create publisher and subscriber
        self.ik_solution_pub = self.create_publisher(
            IKSolution, '/ik_solution', 10
        )
        
        cb_group = ReentrantCallbackGroup()
        self.pose_cmd_sub = self.create_subscription(
            PoseCmd, '/pose_cmd', self._pose_cmd_callback, 10,
            callback_group=cb_group
        )
        
        # Joint states subscriber (for current joint seed)
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states',
            self._joint_states_callback, 10
        )
        
        self.get_logger().info("🤖 Pinocchio IK Solver Node initialized")
    
    def _init_solver(self):
        """Initialize Pinocchio IK solver"""
        try:
            if not PINOCCHIO_AVAILABLE or not CASADI_AVAILABLE:
                self.get_logger().warn("⚠️ Pinocchio or CasADi not available, IK disabled")
                return
            
            # Try parameter path first, then common workspace candidates.
            param_urdf = self.get_parameter('urdf_path').value
            ee_frame = self.get_parameter('ee_frame').value
            urdf_paths = [
                param_urdf,
                '/home/lxf/agx_arm_ws/src/agx_arm_ros/src/agx_arm_moveit/config/test2.urdf',
                '/home/lxf/agx_arm_ws/src/piper_isaac_sim/piper_description/urdf/piper_description.urdf',
                'piper_description.urdf'
            ]
            
            urdf_file = None
            for path in urdf_paths:
                import os
                if os.path.exists(path):
                    urdf_file = path
                    break
            
            if urdf_file is None:
                self.get_logger().warn("⚠️ URDF file not found, IK disabled")
                return
            
            self.solver = PinocchioIKSolver(urdf_file, end_effector_name=str(ee_frame))
            self.enable_ik = True
            self.get_logger().info(f"✅ Pinocchio solver ready, urdf={urdf_file}, ee={ee_frame}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize solver: {e}")
            self.enable_ik = False
    
    def _joint_states_callback(self, msg):
        """Update current joint positions"""
        self.current_joints = dict(zip(msg.name, msg.position))
    
    def _pose_cmd_callback(self, msg: PoseCmd):
        """
        Handle incoming pose command and compute IK
        """
        if not self.enable_ik or self.solver is None:
            self.get_logger().debug("IK disabled or solver unavailable")
            return
        
        try:
            # Extract target pose
            target_pos = np.array([msg.x, msg.y, msg.z])
            target_quat = np.array([msg.qx, msg.qy, msg.qz, msg.qw])
            
            # Get initial guess from current joints
            initial_guess = None
            if self.current_joints:
                initial_guess = np.array([
                    self.current_joints.get(f'joint{i}', 0.0)
                    for i in range(1, 7)
                ])
            
            # Solve IK
            success, q_sol, error, comp_time = self.solver.get_ik_solution(
                target_pos, target_quat, initial_guess=initial_guess
            )
            
            # Publish result
            result = IKSolution()
            result.success = success
            result.joint1 = q_sol[0]
            result.joint2 = q_sol[1]
            result.joint3 = q_sol[2]
            result.joint4 = q_sol[3]
            result.joint5 = q_sol[4]
            result.joint6 = q_sol[5]
            result.error = error
            result.computation_time = comp_time
            
            self.ik_solution_pub.publish(result)
            
            self.get_logger().debug(
                f"IK solved: success={success}, error={error:.4f}, "
                f"time={comp_time*1000:.1f}ms"
            )
            
        except Exception as e:
            self.get_logger().error(f"❌ IK callback error: {e}")
            traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)
    node = PinocchioIKNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n⏹️ IK node shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
