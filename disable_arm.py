#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class PiperDisableNode(Node):
    def __init__(self):
        super().__init__('piper_disable_node')
        self.client = self.create_client(SetBool, '/enable_agx_arm')
        self.get_logger().info('等待失能服务 /enable_agx_arm ...')
        self.client.wait_for_service()

    def disable_arm(self):
        req = SetBool.Request()
        req.data = False
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"响应: {future.result().message}")
            self.get_logger().info("🔴 已成功发送失能指令，现在您应该可以拖动机械臂了。")
        else:
            self.get_logger().error("服务调用失败。")

def main(args=None):
    rclpy.init(args=args)
    node = PiperDisableNode()
    node.disable_arm()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
