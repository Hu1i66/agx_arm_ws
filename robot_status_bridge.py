#!/usr/bin/env python3
"""机械臂状态 MQTT 桥接节点.

订阅 auto_sorting_action.py 发布的 /sorting/robot_status 话题,
将 RobotPayload 封装为 RobotData 后通过 MQTT 转发到外部 Broker.

非侵入式设计:
  - 本节点崩溃不影响 auto_sorting_action 的抓取流程
  - MQTT 连接断开时自动重连, 不丢失订阅关系
  - 配置参数全部通过 YAML 文件管理

启动:
  ros2 launch /home/lxf/agx_arm_ws/launch/robot_status_bridge.launch.py

数据模型:
  RobotPayload (来自 /sorting/robot_status JSON)
    └─ RobotData (封装 msg_type, device_id, device_type, timestamp, seq)
       └─ MQTT publish 到配置的 topic

依赖: paho-mqtt, pydantic
"""

import json
import time
import logging
import traceback
from datetime import datetime, timezone, timedelta

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import paho.mqtt.client as mqtt
from pydantic import BaseModel

# ── 数据模型 ──


class RobotPayload(BaseModel):
    action: str              # 动作类型: "sort_complete" / "sort_failed"
    cargo_type: str          # 货物类型
    confidence: float        # 识别置信度 (0-1)
    source_position: str     # 抓取位置 "(x, y, z)"
    target_position: str     # 应放位置 "(x, y, z)"
    actual_position: str     # 实际放置位置 "(x, y, z)" 或 "NONE"
    grasp_success: bool      # 抓取是否成功
    joint_angles: list[float]  # 6 个关节角度 (度)
    tcp_position: list[float]  # 末端执行器坐标 [x, y, z] (mm)
    error_code: int          # 错误码, 0 表示正常


class RobotData(BaseModel):
    msg_type: str            # "robot_action"
    device_id: str           # 设备编号
    device_type: str         # 设备类型 "robot"
    timestamp: str           # ISO 8601 时间
    payload: RobotPayload    # 交付信息
    seq: int                 # Unix 秒级时间戳


# ── ROS2 节点 ──


class RobotStatusBridge(Node):
    """订阅 /sorting/robot_status, 封装为 RobotData, MQTT 发布."""

    def __init__(self):
        super().__init__('robot_status_bridge')

        # ── 声明 ROS2 参数 (可通过 YAML 或命令行覆盖) ──
        self.declare_parameter('broker_host', 'localhost')
        self.declare_parameter('broker_port', 1883)
        self.declare_parameter('topic', 'robot/ROBOT-01/action')
        self.declare_parameter('device_id', 'ROBOT-01')
        self.declare_parameter('device_type', 'robot')
        self.declare_parameter('client_id', 'robot_status_bridge')
        self.declare_parameter('username', '')
        self.declare_parameter('password', '')
        self.declare_parameter('qos', 1)
        self.declare_parameter('keepalive', 60)

        # 读取参数
        self._broker_host = self.get_parameter('broker_host').value
        self._broker_port = self.get_parameter('broker_port').value
        self._mqtt_topic = self.get_parameter('topic').value
        self._device_id = self.get_parameter('device_id').value
        self._device_type = self.get_parameter('device_type').value
        self._client_id = self.get_parameter('client_id').value
        self._username = self.get_parameter('username').value
        self._password = self.get_parameter('password').value
        self._qos = self.get_parameter('qos').value
        self._keepalive = self.get_parameter('keepalive').value

        self.get_logger().info(
            f"MQTT Broker: {self._broker_host}:{self._broker_port} "
            f"topic={self._mqtt_topic} device={self._device_id}"
        )

        # ── MQTT 客户端 ──
        self._mqtt_client = mqtt.Client(
            client_id=self._client_id,
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        )
        if self._username:
            self._mqtt_client.username_pw_set(self._username, self._password)

        # 设置回调
        self._mqtt_client.on_connect = self._on_mqtt_connect
        self._mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self._mqtt_client.on_publish = self._on_mqtt_publish

        # 连接 Broker
        self._mqtt_connected = False
        self._connect_mqtt()

        # ── ROS2 订阅 ──
        self._sub = self.create_subscription(
            String, '/sorting/robot_status', self._robot_status_cb, 10)
        self.get_logger().info("✅ robot_status_bridge 就绪, 监听 /sorting/robot_status")

    def _connect_mqtt(self):
        """连接 MQTT Broker (非阻塞)."""
        try:
            self._mqtt_client.connect_async(self._broker_host, self._broker_port,
                                             keepalive=self._keepalive)
            self._mqtt_client.loop_start()
            self.get_logger().info(f"MQTT 连接中: {self._broker_host}:{self._broker_port}")
        except Exception as e:
            self.get_logger().error(f"MQTT 连接失败: {e}")

    def _on_mqtt_connect(self, client, userdata, flags, reason_code, properties=None):
        """MQTT 连接成功回调."""
        self._mqtt_connected = (reason_code == 0) or (reason_code is None)
        if self._mqtt_connected:
            self.get_logger().info(f"✅ MQTT 已连接: {self._broker_host}:{self._broker_port}")
        else:
            self.get_logger().warn(f"MQTT 连接失败, rc={reason_code}")

    def _on_mqtt_disconnect(self, client, userdata, flags, reason_code, properties=None):
        """MQTT 断连回调."""
        self._mqtt_connected = False
        self.get_logger().warn(f"MQTT 已断开, rc={reason_code}, paho 将自动重连")

    def _on_mqtt_publish(self, client, userdata, mid, reason_code, properties=None):
        """MQTT 发布完成回调: 确认消息是否送达 Broker."""
        if reason_code is None or reason_code == 0:
            self.get_logger().info(f"📨 MQTT 消息已送达 Broker (mid={mid})")
        else:
            self.get_logger().error(f"❌ MQTT 消息送达失败 (mid={mid}, rc={reason_code})")

    def _robot_status_cb(self, msg: String):
        """接收 /sorting/robot_status, 封装并发布到 MQTT."""
        # ── 解析 RobotPayload ──
        try:
            payload_dict = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON 解析失败: {e}")
            return

        try:
            robot_payload = RobotPayload(**payload_dict)
        except Exception as e:
            self.get_logger().error(f"RobotPayload 校验失败: {e}")
            return

        # ── 构建 RobotData ──
        tz = timezone(timedelta(hours=8))  # Asia/Shanghai
        now = datetime.now(tz)
        robot_data = RobotData(
            msg_type="robot_action",
            device_id=self._device_id,
            device_type=self._device_type,
            timestamp=now.isoformat(),
            payload=robot_payload,
            seq=int(time.time()),
        )

        # ── MQTT 发布 ──
        connected = self._mqtt_connected
        try:
            json_str = robot_data.model_dump_json(indent=None)
            info = self._mqtt_client.publish(
                self._mqtt_topic, json_str, qos=self._qos)
            if info.rc == mqtt.MQTT_ERR_SUCCESS:
                p = robot_data.payload
                status_icon = "✅" if connected else "⚠️"
                self.get_logger().info(
                    f"{status_icon} 分拣状态上报: action={p.action} "
                    f"cargo={p.cargo_type} confidence={p.confidence:.2f} "
                    f"grasp={'成功' if p.grasp_success else '失败'} "
                    f"err={p.error_code} | MQTT→{self._broker_host}:{self._broker_port}/{self._mqtt_topic}")
                if not connected:
                    self.get_logger().warn("⚠️ MQTT 未连接, 消息已缓存待重连后发送")
            else:
                self.get_logger().error(f"❌ MQTT 发布失败, rc={info.rc}")
        except Exception as e:
            self.get_logger().error(f"❌ MQTT 发布异常: {e}")

    def destroy_node(self):
        """清理 MQTT 资源."""
        try:
            self._mqtt_client.loop_stop()
            self._mqtt_client.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = RobotStatusBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
