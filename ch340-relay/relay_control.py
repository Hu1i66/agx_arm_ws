#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CH340 USB 继电器控制脚本（Modbus RTU 协议）

硬件: CH340 USB 转串口继电器模块
协议: Modbus RTU
    - 波特率: 9600, 8N1（8 位数据位, 无校验, 1 位停止位）
    - 从机地址: 1（可用 --slave 修改）
    - 功能码 0x05: 写单个线圈（开/关）
    - 功能码 0x01: 读线圈（查询当前状态）
    - 线圈地址: 0x0000
    - 写入值: 0xFF00=打开, 0x0000=关闭

上位机参考报文:
    打开: 01 05 00 00 FF 00 8C 3A
    关闭: 01 05 00 00 00 00 CD CA
正常应答: 原样回显 8 字节报文。

用法示例:
    python3 relay_control.py on                       # 打开继电器
    python3 relay_control.py off                      # 关闭继电器
    python3 relay_control.py toggle                   # 切换状态
    python3 relay_control.py status                   # 查询当前状态
    python3 relay_control.py pulse 2.0                # 打开 2 秒后自动关闭
    python3 relay_control.py --port /dev/ttyUSB0 on   # 指定串口
    python3 relay_control.py --slave 2 on             # 指定从机地址
    python3 relay_control.py --list                   # 列出可用串口
    python3 relay_control.py --dry-run                # 只打印报文，不连接串口

作为模块导入:
    from relay_control import RelayController
    rc = RelayController("/dev/usbrelay")
    rc.on()
    rc.off()
    state = rc.read_state()   # True=开, False=关, None=未知

依赖: pyserial  (Ubuntu: sudo apt install python3-serial)
"""

import argparse
import os
import struct
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("❌ 缺少依赖 pyserial，请安装：")
    print("    sudo apt install python3-serial")
    print("  或")
    print("    pip3 install pyserial")
    sys.exit(1)


# ────────────────────── 默认通信参数（与上位机一致）──────────────────────
# 优先使用 udev 规则生成的固定软链接 /dev/usbrelay（见 99-ch340-usbrelay.rules），
# 避免 USB 插拔后 ttyUSB 编号漂移；找不到时回退到 /dev/ttyUSB0。
DEFAULT_PORT = "/dev/usbrelay"
FALLBACK_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 9600
DEFAULT_SLAVE_ID = 1

# Modbus 功能码
FUNC_READ_COILS = 0x01           # 读线圈（查询状态）
FUNC_WRITE_SINGLE_COIL = 0x05    # 写单个线圈（开/关）

# 线圈地址与写入值
COIL_ADDRESS = 0x0000
COIL_ON_VALUE = 0xFF00           # 打开
COIL_OFF_VALUE = 0x0000          # 关闭

# CH340 USB VID:PID（用于自动识别串口）
CH340_VID = 0x1A86
CH340_PID = 0x7523


# ────────────────────── Modbus 工具函数 ──────────────────────

def crc16(data: bytes) -> int:
    """计算 Modbus RTU CRC16 校验（多项式 0xA001 反序）。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def _append_crc(payload: bytes) -> bytes:
    """给 payload 追加 CRC16（低字节在前）。"""
    crc = crc16(payload)
    return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def build_write_coil(slave_id: int, coil_address: int, turn_on: bool) -> bytes:
    """构造「写单个线圈」报文（8 字节，含 CRC）。

    打开: 01 05 00 00 FF 00 8C 3A
    关闭: 01 05 00 00 00 00 CD CA
    """
    value = COIL_ON_VALUE if turn_on else COIL_OFF_VALUE
    payload = struct.pack(">BBHH", slave_id, FUNC_WRITE_SINGLE_COIL, coil_address, value)
    return _append_crc(payload)


def build_read_coils(slave_id: int, coil_address: int, quantity: int = 1) -> bytes:
    """构造「读线圈」报文（8 字节，含 CRC），用于查询状态。"""
    payload = struct.pack(">BBHH", slave_id, FUNC_READ_COILS, coil_address, quantity)
    return _append_crc(payload)


def verify_crc(frame: bytes) -> bool:
    """校验整帧（含末尾 2 字节 CRC）是否正确。"""
    if len(frame) < 3:
        return False
    body, recv_crc = frame[:-2], frame[-2:]
    expected = crc16(body)
    return recv_crc == bytes([expected & 0xFF, (expected >> 8) & 0xFF])


# ────────────────────── 继电器控制器类 ──────────────────────

class RelayController:
    """CH340 USB 继电器控制器（Modbus RTU）。

    既可被 CLI 调用，也可作为模块导入集成到其他程序（如分拣系统）。
    """

    def __init__(
        self,
        port: str = DEFAULT_PORT,
        baudrate: int = DEFAULT_BAUDRATE,
        slave_id: int = DEFAULT_SLAVE_ID,
        coil_address: int = COIL_ADDRESS,
        timeout: float = 0.4,
        retries: int = 6,
    ):
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.coil_address = coil_address
        self.timeout = timeout
        self.retries = retries

    # ── 串口连接 ──
    @staticmethod
    def auto_detect_port() -> str:
        """自动查找 CH340 串口。优先返回 /dev/usbrelay 软链接，其次按 VID:PID 匹配。"""
        # 1) 优先使用 udev 软链接
        if os.path.exists("/dev/usbrelay"):
            return "/dev/usbrelay"
        # 2) 按 VID:PID 匹配 CH340
        for p in serial.tools.list_ports.comports():
            if p.vid == CH340_VID and p.pid == CH340_PID:
                return p.device
        # 3) 退而求其次：任意 ttyUSB*
        for p in serial.tools.list_ports.comports():
            if p.device.startswith("/dev/ttyUSB") or p.device.startswith("/dev/ttyACM"):
                return p.device
        return FALLBACK_PORT

    def _resolve_port(self) -> str:
        """(重新)解析串口路径，用于 USB 重枚举后重新定位设备。

        固定端口仍存在则沿用；否则按 VID:PID → ttyUSB* 回退。
        注意用 os.path.exists（跟随软链接）：/dev/usbrelay 变成悬空链接时
        会返回 False，从而触发按 VID:PID 重新查找。
        """
        if self.port and os.path.exists(self.port):
            return self.port
        for p in serial.tools.list_ports.comports():
            if p.vid == CH340_VID and p.pid == CH340_PID:
                return p.device
        for p in serial.tools.list_ports.comports():
            if p.device.startswith("/dev/ttyUSB") or p.device.startswith("/dev/ttyACM"):
                return p.device
        return self.port  # 兜底返回原值（打开时会报错并继续重试）

    def _open_serial(self, port: str) -> serial.Serial:
        return serial.Serial(
            port=port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
        )

    def _transact(self, request: bytes, expected_len: int) -> bytes:
        """发送一帧并读取响应，带重试。返回响应原始字节。

        针对 CH340 偶发 USB 重枚举（/dev/ttyUSB* 与 /dev/usbrelay 瞬时消失）做容错：
        每次重试前重新解析串口，并留出 ~0.3s 窗口等待设备回归。
        """
        last_err = None
        total = self.retries + 1
        for attempt in range(total):
            port = self._resolve_port()
            if not port or not os.path.exists(port):
                last_err = f"串口未就绪（等待 USB 重枚举 {attempt + 1}/{total}）"
                time.sleep(0.3)
                continue
            try:
                with self._open_serial(port) as ser:
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()
                    ser.write(request)
                    ser.flush()
                    time.sleep(0.05)  # 给设备留出响应时间
                    response = ser.read(expected_len)
                if response:
                    return response
                last_err = "设备无响应（超时）"
            except serial.SerialException as e:
                last_err = str(e)
            time.sleep(0.3)
        raise IOError(f"通信失败: {last_err}")

    # ── 公开操作 ──
    def set(self, turn_on: bool) -> bool:
        """打开(turn_on=True)或关闭(turn_on=False)继电器。返回是否成功。"""
        cmd = build_write_coil(self.slave_id, self.coil_address, turn_on)
        action = "打开" if turn_on else "关闭"
        try:
            resp = self._transact(cmd, expected_len=8)
        except IOError as e:
            print(f"❌ {action}失败: {e}")
            return False

        # 正常应答 = 原样回显
        if resp == cmd and verify_crc(resp):
            print(f"✅ 继电器已{action}")
            return True
        if not verify_crc(resp):
            print(f"❌ {action}失败: 响应 CRC 校验错误 (收到: {resp.hex(' ').upper()})")
        else:
            print(f"❌ {action}失败: 响应异常 (收到: {resp.hex(' ').upper()})")
        return False

    def on(self) -> bool:
        return self.set(True)

    def off(self) -> bool:
        return self.set(False)

    def toggle(self) -> bool:
        """读取当前状态后切换。查询失败时默认执行「打开」。"""
        state = self.read_state()
        if state is None:
            print("⚠️  无法读取当前状态，默认执行「打开」")
            return self.on()
        return self.set(not state)

    def pulse(self, duration: float = 1.0, turn_on: bool = True) -> bool:
        """脉冲模式：置位 duration 秒后自动复位。"""
        opp = not turn_on
        action = "打开" if turn_on else "关闭"
        print(f"⏱️  脉冲: {action} {duration:.2f}s 后复位")
        if not self.set(turn_on):
            return False
        time.sleep(duration)
        return self.set(opp)

    def read_state(self):
        """读取线圈状态。返回 True=开, False=关, None=查询失败。"""
        cmd = build_read_coils(self.slave_id, self.coil_address, quantity=1)
        try:
            resp = self._transact(cmd, expected_len=5)
        except IOError as e:
            print(f"⚠️  状态查询失败: {e}")
            return None

        if len(resp) < 5 or not verify_crc(resp):
            print(f"⚠️  状态查询响应异常 (收到: {resp.hex(' ').upper()})")
            return None
        # 响应格式: [slave][0x01][byte_count=1][coil_data][crc_lo][crc_hi]
        coil_data = resp[3]
        return bool(coil_data & 0x01)


# ────────────────────── CLI ──────────────────────

def list_serial_ports():
    """列出本机可用串口，标注 CH340 设备。"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未找到可用串口。")
        return
    print("可用串口列表：")
    for p in ports:
        vid = p.vid if p.vid is not None else 0
        pid = p.pid if p.pid is not None else 0
        tag = "  ← CH340 继电器" if (vid == CH340_VID and pid == CH340_PID) else ""
        print(f"  {p.device:<14} vid={vid:#06x} pid={pid:#06x}  {p.description}{tag}")


def resolve_port(port_arg: str) -> str:
    """解析最终使用的串口：显式指定 > 自动检测。"""
    if port_arg:
        return port_arg
    detected = RelayController.auto_detect_port()
    print(f"🔍 未指定串口，自动选择: {detected}")
    return detected


def main():
    parser = argparse.ArgumentParser(
        description="CH340 USB 继电器控制（Modbus RTU）"
    )
    parser.add_argument(
        "action",
        nargs="?",
        choices=["on", "off", "toggle", "status", "pulse"],
        help="动作: on=打开, off=关闭, toggle=切换, status=查询状态, pulse=脉冲",
    )
    parser.add_argument(
        "pulse_seconds",
        nargs="?",
        type=float,
        help="配合 pulse 使用: 脉冲持续秒数（如: pulse 2.0）",
    )
    parser.add_argument(
        "--port", "-p",
        help=f"串口号，默认自动检测（优先 {DEFAULT_PORT}）",
    )
    parser.add_argument(
        "--baudrate", "-b",
        type=int,
        default=DEFAULT_BAUDRATE,
        help=f"波特率，默认 {DEFAULT_BAUDRATE}",
    )
    parser.add_argument(
        "--slave", "-s",
        type=int,
        default=DEFAULT_SLAVE_ID,
        help=f"从机地址，默认 {DEFAULT_SLAVE_ID}",
    )
    parser.add_argument(
        "--list", "-l",
        action="store_true",
        help="列出可用串口",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只打印将要发送的报文，不连接串口",
    )

    args = parser.parse_args()

    # --list
    if args.list:
        list_serial_ports()
        return

    # --dry-run
    if args.dry_run:
        print("===  dry-run 模式，不连接串口 ===")
        on_cmd = build_write_coil(args.slave, COIL_ADDRESS, True)
        off_cmd = build_write_coil(args.slave, COIL_ADDRESS, False)
        read_cmd = build_read_coils(args.slave, COIL_ADDRESS, 1)
        print(f"打开命令:   {on_cmd.hex(' ').upper()}")
        print(f"关闭命令:   {off_cmd.hex(' ').upper()}")
        print(f"状态查询:   {read_cmd.hex(' ').upper()}")
        return

    if not args.action:
        parser.print_help()
        return

    port = resolve_port(args.port)
    rc = RelayController(port=port, baudrate=args.baudrate, slave_id=args.slave)

    if args.action == "on":
        rc.on()
    elif args.action == "off":
        rc.off()
    elif args.action == "toggle":
        rc.toggle()
    elif args.action == "status":
        state = rc.read_state()
        if state is None:
            print("❓ 未能获取继电器状态")
        elif state:
            print("💡 继电器当前状态: 打开 (ON)")
        else:
            print("⚫ 继电器当前状态: 关闭 (OFF)")
    elif args.action == "pulse":
        dur = args.pulse_seconds if args.pulse_seconds is not None else 1.0
        rc.pulse(duration=dur)


if __name__ == "__main__":
    main()
