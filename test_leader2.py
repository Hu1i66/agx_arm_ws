import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time

rclpy.init()
node = Node('temp')
client = node.create_client(SetBool, '/enable_agx_arm')
client.wait_for_service()
req = SetBool.Request()
req.data = False
rclpy.spin_until_future_complete(node, client.call_async(req))
node.destroy_node()
rclpy.shutdown()
print("ROS disable sent.")

from pyAgxArm import create_agx_arm_config, AgxArmFactory
config = create_agx_arm_config(robot='piper', comm='can', channel='can0')
arm = AgxArmFactory.create_arm(config)
arm.connect()

arm.enable()
time.sleep(0.5)
arm.set_leader_mode()
time.sleep(0.5)
print("Leader mode set.")
