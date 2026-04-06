import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from piper_sdk import C_PiperInterface

rclpy.init()
node = Node('test_drag')
client = node.create_client(SetBool, '/enable_agx_arm')
client.wait_for_service()
req = SetBool.Request()
req.data = False
node.get_logger().info("Disabling via ROS...")
rclpy.spin_until_future_complete(node, client.call_async(req))
node.destroy_node()
rclpy.shutdown()

print("ROS disabled. Now using CAN to enable and enter teach mode...")
piper = C_PiperInterface("can0")
piper.ConnectPort()
# Enable the arm hardware (so motors have power for gravity comp)
piper.EnableArm(7)
time.sleep(0.5)

# Enter teach mode
piper.MotionCtrl_1(0, 0, 1)
print("Teach mode sent!")
