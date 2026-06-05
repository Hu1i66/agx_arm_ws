#!/usr/bin/env python3
"""
Subscribe to /feedback/joint_states and log data to file.
Usage:
  ros2 run agx_arm_auto_sorting joint_state_logger                    # auto-generate filename under /tmp
  ros2 run agx_arm_auto_sorting joint_state_logger --ros-args -p output_path:=/path/to/data.csv
  ros2 run agx_arm_auto_sorting joint_state_logger --ros-args -p format:=json
"""

import csv
import json
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateLogger(Node):
    def __init__(self):
        super().__init__('joint_state_logger')

        self.declare_parameter('output_path', '')
        self.declare_parameter('format', 'csv')  # csv, json, jsonl
        self.declare_parameter('topic', '/feedback/joint_states')

        topic = self.get_parameter('topic').value
        fmt = self.get_parameter('format').value.lower()
        output_path = self.get_parameter('output_path').value

        if not output_path:
            ts = time.strftime('%Y%m%d_%H%M%S')
            output_path = f'/tmp/joint_states_{ts}.{fmt if fmt != "jsonl" else "jsonl"}'

        self.output_path = output_path
        self.format = fmt
        self.msg_count = 0
        self.start_time = time.time()
        self.file = None
        self.csv_writer = None
        self._header_written = False

        self.subscription = self.create_subscription(
            JointState, topic, self.callback, 10)

        self.get_logger().info(
            f'Logging [{topic}] → {output_path} (format={fmt})'
        )

    def callback(self, msg: JointState):
        try:
            if self.file is None:
                self._open_file(msg)

            if self.format in ('csv',):
                self._write_csv(msg)
            elif self.format in ('json',):
                self._write_json(msg)
            elif self.format in ('jsonl',):
                self._write_jsonl(msg)
        except Exception:
            self.get_logger().exception('Callback error')

    def _open_file(self, msg: JointState):
        self.file = open(self.output_path, 'w', newline='')
        if self.format in ('csv',):
            header = ['sec', 'nanosec']
            for name in msg.name:
                header.append(f'{name}_pos')
            for name in msg.name:
                header.append(f'{name}_vel')
            for name in msg.name:
                header.append(f'{name}_eff')
            self.csv_writer = csv.writer(self.file)
            self.csv_writer.writerow(header)
            self.file.flush()
        elif self.format in ('json',):
            self.file.write('[\n')
            self.file.flush()

    def _write_csv(self, msg: JointState):
        row = [msg.header.stamp.sec, msg.header.stamp.nanosec]
        n = len(msg.name)
        positions = list(msg.position) if msg.position else [0.0]*n
        velocities = list(msg.velocity) if msg.velocity else [0.0]*n
        efforts = list(msg.effort) if msg.effort else [0.0]*n
        row.extend(positions)
        row.extend(velocities)
        row.extend(efforts)
        self.csv_writer.writerow(row)
        self.msg_count += 1
        if self.msg_count % 500 == 0:
            self.file.flush()

    def _write_json(self, msg: JointState):
        if self.msg_count > 0:
            self.file.write(',\n')
        record = {
            'stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
            'name': list(msg.name),
            'position': list(msg.position) if msg.position else [],
            'velocity': list(msg.velocity) if msg.velocity else [],
            'effort': list(msg.effort) if msg.effort else [],
        }
        json.dump(record, self.file, indent=None)
        self.msg_count += 1
        if self.msg_count % 500 == 0:
            self.file.flush()

    def _write_jsonl(self, msg: JointState):
        record = {
            'stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
            'name': list(msg.name),
            'position': list(msg.position) if msg.position else [],
            'velocity': list(msg.velocity) if msg.velocity else [],
            'effort': list(msg.effort) if msg.effort else [],
        }
        self.file.write(json.dumps(record, separators=(',', ':')) + '\n')
        self.msg_count += 1
        if self.msg_count % 500 == 0:
            self.file.flush()

    def close(self):
        if self.file is None:
            return
        elapsed = time.time() - self.start_time
        rate = self.msg_count / elapsed if elapsed > 0 else 0
        if self.format in ('json',) and self.msg_count > 0:
            self.file.write('\n]\n')
        self.file.flush()
        self.file.close()
        self.get_logger().info(
            f'Saved {self.msg_count} msgs ({elapsed:.1f}s, {rate:.1f} Hz) → {self.output_path}'
        )


def main():
    rclpy.init(args=sys.argv)
    node = JointStateLogger()

    def shutdown(signum, frame):
        node.close()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
