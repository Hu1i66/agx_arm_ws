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

try:
    import pinocchio as pin
    PINOCCHIO_AVAILABLE = True
except ImportError:
    PINOCCHIO_AVAILABLE = False
    print("⚠️ Warning: pinocchio not installed. Install with: pip install pinocchio")

try:
    import casadi
    CASADI_AVAILABLE = True
except ImportError:
    CASADI_AVAILABLE = False
    print("⚠️ Warning: casadi not installed. Install with: pip install casadi")

from agx_arm_msgs.msg import PoseCmd, IKSolution


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
            for i in range(1, 7):
                joint_name = f"joint{i}"
                try:
                    jid = self.model.getJointId(joint_name)
                    self.joint_names.append(joint_name)
                    self.joint_ids.append(jid)
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
        max_iter: int = 100,
        tol: float = 1e-5
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
        
        if initial_guess is None:
            q_init = np.zeros(self.ndof)
        else:
            q_init = np.array(initial_guess)
        
        try:
            # Use CasADi for optimization
            q_sym = casadi.SX.sym('q', self.ndof)
            
            # Callback function to compute forward kinematics in CasADi
            # We'll use a numerical callback since Pinocchio doesn't have CasADi integration
            def fk_callback(q_val):
                """Compute FK and return pose error"""
                q_np = np.array(q_val).flatten()
                pin.forwardKinematics(self.model, self.data, q_np)
                pin.updateFramePlacements(self.model, self.data)
                
                # Get current EE pose
                current_pose = self.data.oMf[self.ee_frame_id]
                current_pos = current_pose.translation
                current_rot = current_pose.rotation
                
                # Convert rotation matrix to quaternion
                current_quat = pin.Quaternion(current_rot)
                
                # Compute position error (L2 norm)
                pos_error = np.linalg.norm(current_pos - target_pos)
                
                # Compute orientation error (quaternion distance)
                target_quat_norm = target_quat / np.linalg.norm(target_quat)
                current_quat_arr = np.array([current_quat.x, current_quat.y, current_quat.z, current_quat.w])
                quat_error = 1.0 - np.abs(np.dot(target_quat_norm, current_quat_arr))
                
                total_error = pos_error + 0.5 * quat_error
                return total_error
            
            # Create external function for numerical evaluation
            fk_func = casadi.external('fk', fk_callback)
            
            # Define cost function with joint limits penalty
            cost = fk_func(q_sym)
            
            # Add joint limits penalty (soft constraints)
            q_min = np.array([-np.pi] * self.ndof)
            q_max = np.array([np.pi] * self.ndof)
            
            for i in range(self.ndof):
                # Penalty for violating joint limits
                penalty_factor = 100.0
                if q_sym[i] < q_min[i]:
                    cost += penalty_factor * (q_min[i] - q_sym[i])**2
                elif q_sym[i] > q_max[i]:
                    cost += penalty_factor * (q_sym[i] - q_max[i])**2
            
            # Setup optimizer
            nlp = {'x': q_sym, 'f': cost}
            solver = casadi.nlpsol('solver', 'ipopt', nlp, {
                'ipopt.max_iter': max_iter,
                'ipopt.tol': tol,
                'ipopt.print_level': 0,
                'print_time': False,
                'verbose': False
            })
            
            # Solve
            result = solver(x0=q_init)
            q_sol = np.array(result['x']).flatten()
            
            # Verify solution by forward kinematics
            pin.forwardKinematics(self.model, self.data, q_sol)
            pin.updateFramePlacements(self.model, self.data)
            
            current_pose = self.data.oMf[self.ee_frame_id]
            current_pos = current_pose.translation
            current_rot = current_pose.rotation
            
            pos_error = np.linalg.norm(current_pos - target_pos)
            final_error = float(result['f'])
            
            success = pos_error < 0.01  # Position error < 1cm
            
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
        
        # Joint states subscriber (for odometry)
        self.joint_states_sub = self.create_subscription(
            'sensor_msgs/msg/JointState', '/joint_states',
            self._joint_states_callback, 10
        )
        
        # Parameters
        self.declare_parameter('enable_ik', False)
        self.declare_parameter('urdf_path', '/opt/ros/humble/share/piper_description/urdf/piper_arm.urdf')
        self.declare_parameter('ee_frame', 'link6')
        
        self.get_logger().info("🤖 Pinocchio IK Solver Node initialized")
    
    def _init_solver(self):
        """Initialize Pinocchio IK solver"""
        try:
            if not PINOCCHIO_AVAILABLE or not CASADI_AVAILABLE:
                self.get_logger().warn("⚠️ Pinocchio or CasADi not available, IK disabled")
                return
            
            # Try to find URDF
            urdf_paths = [
                '/opt/ros/humble/share/piper_description/urdf/piper_arm.urdf',
                '/home/lxf/agx_arm_ws/src/agx_arm_ros/src/piper_description/urdf/piper_arm.urdf',
                'piper_arm.urdf'
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
            
            self.solver = PinocchioIKSolver(urdf_file, end_effector_name="link6")
            self.enable_ik = True
            self.get_logger().info("✅ Pinocchio solver ready")
            
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
