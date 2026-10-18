"""Choose compatible image reliability; prefer retransmission when the camera offers it."""
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy


def image_subscription_qos(node, topic):
    node.declare_parameter('image_reliability', 'auto')
    mode = node.get_parameter('image_reliability').value
    if mode not in ('auto', 'reliable', 'best_effort'):
        raise ValueError('image_reliability须为auto/reliable/best_effort')
    if mode == 'auto':
        deadline = time.monotonic() + 3.
        publishers = node.get_publishers_info_by_topic(topic)
        while not publishers and time.monotonic() < deadline:
            time.sleep(.05)
            publishers = node.get_publishers_info_by_topic(topic)
        if publishers:
            mode = ('best_effort' if any(p.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
                                       for p in publishers) else 'reliable')
        else:
            # Discovery may lag startup; keep the known camera defaults, with an explicit override.
            mode = 'reliable' if node.get_parameter('camera_id').value == 'd455' else 'best_effort'
            node.get_logger().warning(f'尚未发现{topic}发布端，暂按相机默认{mode}订阅；请先启动相机')
    reliability = (ReliabilityPolicy.RELIABLE if mode == 'reliable' else ReliabilityPolicy.BEST_EFFORT)
    node.get_logger().info(f'图像订阅QoS: {mode}, depth=1, topic={topic}')
    return QoSProfile(depth=1, reliability=reliability)
