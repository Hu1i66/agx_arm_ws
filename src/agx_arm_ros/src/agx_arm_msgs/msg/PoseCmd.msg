# IK Solver Input: Target Pose Command
# Contains Cartesian pose (position + orientation) and gripper command

# Target position (meters)
float64 x
float64 y
float64 z

# Target orientation (quaternion)
float64 qx
float64 qy
float64 qz
float64 qw

# Gripper target (0.0 to 1.0, 0=open, 1=close)
float64 gripper_target
