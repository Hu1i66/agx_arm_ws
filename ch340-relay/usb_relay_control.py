#!/usr/bin/env python3
"""
CH340 USB Relay Controller - 自锁模式

用法:
    python3 usb_relay_control.py on      # 打开继电器
    python3 usb_relay_control.py off     # 关闭继电器
    python3 usb_relay_control.py toggle  # 切换状态
    python3 usb_relay_control.py         # 默认切换

支持多种常见协议，通过 --proto 切换:
    ascii   发送 '1'/'0'  (最常见)
    jdy     发送 0x20 0x00 / 0x20 0x01
    binary  发送 0x11 / 0x12
"""

import argparse
import serial
import sys
import os

DEVICE = "/dev/ttyUSB0"
BAUD = 9600

PROTO_MAP = {
    "ascii":  {"on": b"1",       "off": b"0"},
    "jdy":    {"on": b"\x20\x00", "off": b"\x20\x01"},
    "binary": {"on": b"\x11",    "off": b"\x12"},
}


def send_cmd(proto_name: str, cmd: str):
    """打开串口，发送指令，打印结果。"""
    if not os.path.exists(DEVICE):
        print(f"错误: {DEVICE} 不存在")
        print("请先确认 CH340 设备已正确连接: lsusb | grep 1a86")
        sys.exit(1)

    proto = PROTO_MAP.get(proto_name)
    if not proto:
        print(f"未知协议: {proto_name}")
        sys.exit(1)

    data = proto.get(cmd)
    if data is None:
        print(f"协议 {proto_name} 不支持命令: {cmd}")
        sys.exit(1)

    print(f"发送 [{cmd}] -> {data.hex()}")

    ser = serial.Serial(DEVICE, BAUD, timeout=1)
    try:
        ser.reset_output_buffer()
        ser.reset_input_buffer()
        n = ser.write(data)
        ser.flush()
        print(f"已发送 {n} 字节")
    except serial.SerialException as e:
        print(f"发送失败: {e}")
        sys.exit(1)
    finally:
        ser.close()


def main():
    parser = argparse.ArgumentParser(description="CH340 USB 继电器控制")
    parser.add_argument(
        "action",
        nargs="?",
        default="toggle",
        choices=["on", "off", "toggle"],
        help="动作: on / off / toggle (默认: toggle)",
    )
    parser.add_argument(
        "--proto",
        choices=list(PROTO_MAP.keys()),
        default="ascii",
        help="通信协议 (默认: ascii)",
    )
    args = parser.parse_args()

    if args.action == "toggle":
        # 自锁模式下，toggle 就是发一个脉冲信号
        # 多数继电器板收到 on 就吸合，收到 off 就释放
        # 这里默认发 on，你也可以改成发 off
        print("切换模式: 发送 ON 脉冲")
        send_cmd(args.proto, "on")
    else:
        send_cmd(args.proto, args.action)


if __name__ == "__main__":
    main()
