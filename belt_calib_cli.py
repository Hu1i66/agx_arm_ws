#!/usr/bin/env python3
"""传送带方向动态标定 CLI (提取自 .trae/specs/dynamic-grasping/plan.md Task 7 Step 1)

依赖运行中的节点:
  - realsense_yolo_node.py  → 发布 /detection_info (含 base_position_m, 需已加载最新手眼标定)
  - stm32_conveyor.py       → 发布 /conveyor_status (STM32 速度)

用法:
  cd /home/lxf/agx_arm_ws && source /opt/ros/humble/setup.bash
  python3 belt_calib_cli.py [采集秒数, 默认3]

步骤: 放置特征物体在传送带上 → 以固定速度运行 → 运行本脚本
输出: /home/lxf/agx_arm_ws/conveyor_calib.json
"""
import rclpy
import json
import math
import time
import sys
from rclpy.node import Node
from std_msgs.msg import String

from conveyor_calib import fit_belt_direction, estimate_belt_speed_vision

COLLECT_SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
OUT_PATH = '/home/lxf/agx_arm_ws/conveyor_calib.json'


class Calib(Node):
    def __init__(self):
        super().__init__('belt_calib_cli')
        self.samples = []
        self.speed_samples = []
        self.create_subscription(String, '/detection_info', self._det, 10)
        self.create_subscription(String, '/conveyor_status', self._spd, 10)

    def _det(self, m):
        try:
            d = json.loads(m.data)
            for o in d.get('objects', []):
                bp = o.get('base_position_m')
                hs = o.get('header_stamp', '')
                if bp and 'x' in bp:
                    t = time.time()
                    try:
                        c = hs.replace('T', ' ').replace('Z', '')
                        t = time.mktime(time.strptime(c[:19], '%Y-%m-%d %H:%M:%S'))
                        if len(c) > 20:
                            t += float('0.' + c[20:])
                    except Exception:
                        pass
                    self.samples.append((t, bp['x'], bp['y']))
        except Exception:
            pass

    def _spd(self, m):
        try:
            d = json.loads(m.data)
            if d.get('type') == 'conveyor_status':
                self.speed_samples.append((time.time(), d.get('speed', 0.0)))
        except Exception:
            pass


def main():
    rclpy.init()
    node = Calib()
    print(f"⏳ 正在采集 {COLLECT_SECONDS} 秒轨迹... "
          f"请确保传送带以固定速度运行且物体在视野内")
    end = time.time() + COLLECT_SECONDS
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    samples = node.samples
    if len(samples) < 3:
        print("❌ 采集点不足, 请重试 (检查 YOLO 节点是否发布 /detection_info)")
        rclpy.shutdown()
        raise SystemExit(1)

    xys = [(s[1], s[2]) for s in samples]
    dx, dy, info = fit_belt_direction(xys)
    v_vis = estimate_belt_speed_vision(samples)
    v_stm = max([s[1] for s in node.speed_samples], default=0.0)
    scale = (v_vis / v_stm) if v_stm > 1e-6 else 1.0

    out = {'dx': dx, 'dy': dy, 'speed_scale': scale,
           'angle_rad': info['angle_rad'], 'source': 'belt_calib_cli',
           'v_visual_mps': v_vis, 'v_stm32_mps': v_stm, 'n_points': len(samples)}
    with open(OUT_PATH, 'w') as f:
        json.dump(out, f, indent=2)

    print(f"✅ 标定完成: d̂=({dx:.4f},{dy:.4f}) "
          f"angle={math.degrees(info['angle_rad']):.2f}°")
    print(f"   视觉速度={v_vis:.4f} m/s, STM32速度={v_stm:.4f} m/s, "
          f"比例={scale:.3f} (STM32已为真实速度, scale应≈1.0)")
    print(f"   已写入 {OUT_PATH}")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
