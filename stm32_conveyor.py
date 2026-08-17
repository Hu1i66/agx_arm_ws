#!/usr/bin/env python3
"""
STM32 传送带控制模块（串口通信协议）

基于 STM32 下位机协议（USART1, 115200-8N1）：
- 命令: 'S'=启动, 'P'=停止, 'F'=正转, 'R'=反转（单字节 ASCII）
- 应答: OK\\r\\n / ERR\\r\\n / BUSY\\r\\n
- 状态帧: V:2.04,O:1,M1:1,M2:1\\r\\n（每 100ms 一帧）

硬件: 蓝丸 STM32F103C8T6 + USB-TTL (CH340)
协议文档: STM32传送带控制_通信协议说明.md
"""

import re
import threading
import time
import os
from collections import deque

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    raise ImportError("缺少依赖 pyserial，请安装：sudo apt install python3-serial")


# ────────────────────── 通信参数 ──────────────────────
DEFAULT_BAUDRATE = 115200
DEFAULT_TIMEOUT = 0.1          # 串口读取超时
CMD_RESPONSE_TIMEOUT = 0.8     # 命令等待应答超时 (F/R 换向需 500ms + 余量)

# ⚠️ 速度校正系数: STM32 固件测速换算系数与实物不符, GUI 显示约为实际 24 倍。
# 实测校准: GUI 1.13→实际 0.0468 (×0.0414), 2.67→0.11 (×0.0412),
#           1.85→0.0764 (×0.0413), 3.54→0.1489 (×0.0421)
# 取平均 0.0415。修改此值即可校正, 无需改固件。
SPEED_SCALE_FACTOR = 0.0415

# 状态帧正则: V:2.04,O:1,M1:1,M2:1
FRAME_PATTERN = re.compile(rb'V:([\d.]+),O:(\d),M1:(\d),M2:(\d)')

# USB VID:PID 匹配表
KNOWN_VID_PID = [
    (0x1A86, 0x7523),  # CH340
    (0x1A86, 0x55D4),  # CH340 (其他变体)
    (0x0483, 0x5740),  # STM32 Virtual ComPort (ST-Link)
    (0x10C4, 0xEA60),  # CP2102
    (0x067B, 0x2303),  # PL2303
]


# ────────────────────── 设备查找 ──────────────────────

def auto_detect_port() -> str:
    """自动查找 STM32 串口设备。优先固定软链接，其次按已知 VID:PID 匹配。"""
    # 1) 优先用户自定义的 udev 软链接
    for link in ("/dev/stm32_conveyor", "/dev/ttySTM32"):
        if os.path.exists(link):
            return link
    # 2) 按已知 VID:PID 匹配
    for p in serial.tools.list_ports.comports():
        if (p.vid, p.pid) in KNOWN_VID_PID:
            return p.device
    # 3) 回退: 任意 ttyUSB* / ttyACM*
    for p in serial.tools.list_ports.comports():
        dev = p.device
        if dev.startswith("/dev/ttyUSB") or dev.startswith("/dev/ttyACM"):
            return dev
    return "/dev/ttyUSB0"  # 兜底


# ────────────────────── STM32 传送带控制器 ──────────────────────

class STM32Conveyor:
    """STM32 传送带控制器。

    持续后台读取状态帧，提供线程安全的命令接口和状态查询。
    用法:
        conv = STM32Conveyor()
        conv.connect()
        conv.start()          # 发送 'S'
        conv.stop()           # 发送 'P'
        conv.forward()        # 发送 'F'
        conv.reverse()        # 发送 'R'
        status = conv.get_status()  # 返回当前状态字典
        conv.disconnect()
    """

    def __init__(self, port: str = None, baudrate: int = DEFAULT_BAUDRATE):
        self.port = port or auto_detect_port()
        self.baudrate = baudrate
        self._ser: serial.Serial = None
        self._lock = threading.Lock()      # 保护串口写
        self._read_lock = threading.Lock() # 保护串口读 (reader_loop 与命令应答共用)
        self._status_lock = threading.Lock()  # 保护状态变量
        self._running = False
        self._reader_thread: threading.Thread = None

        # ── 状态变量（由 _reader_loop 更新）──
        self._speed = 0.0                   # V: 线速度 (m/s), 滤波后
        self._speed_raw = 0.0               # V: 最新原始速度 (m/s), 未滤波
        self._speed_win = deque(maxlen=10)  # 速度滑动窗口 (最近 10 帧 ≈ 1s, 100ms/帧)
        self._object_detected = False       # O: 物体检测 (1=有物体 → 反转: 0=有物体)
        self._motor_running = False         # M1: 电机运行 (1=运行)
        self._motor_direction = True        # M2: 方向 (1=正转, 0=反转)
        self._last_status_time = 0.0        # 最后收到帧的时间戳

        # ── 帧缓冲 ──
        self._buf = b""

    # ── 连接管理 ──

    def connect(self):
        """打开串口并启动后台读取线程。

        ⚠️ exclusive=True: 串口被其他进程占用时直接抛 SerialException,
        避免"静默打开但抢不到数据" (孤儿 worker 抢占导致 GUI 显示 0)。
        """
        if self._ser and self._ser.is_open:
            return
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=DEFAULT_TIMEOUT,
            exclusive=True,
        )
        self._running = True
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def disconnect(self):
        """停止后台线程并关闭串口。"""
        self._running = False
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    # ── 后台读取循环 ──

    def _reader_loop(self):
        """持续读取串口，解析状态帧并更新状态变量。"""
        while self._running and self._ser and self._ser.is_open:
            try:
                # 与 _send_command 共享读锁, 避免命令应答与状态帧互相抢占
                with self._read_lock:
                    data = self._ser.read(256)
                if not data:
                    continue
                self._buf += data
                # 按 \r\n 分帧
                while b'\r\n' in self._buf:
                    line, self._buf = self._buf.split(b'\r\n', 1)
                    self._parse_frame(line)
            except serial.SerialException:
                # 串口可能断开，等待重连
                time.sleep(0.5)
            except Exception:
                # 其他异常不中断读取
                time.sleep(0.1)

    def _parse_frame(self, frame: bytes):
        """解析单行状态帧 V:..,O:..,M1:..,M2:..

        ⚠️ O 字段反转: STM32 实际发送 O:0=有物体, O:1=无物体 (与文档相反,
        经实测确认)。此处统一在解析层反转, GUI/上层无需关心硬件细节。
        """
        m = FRAME_PATTERN.match(frame)
        if not m:
            return
        with self._status_lock:
            # ⚠️ 速度校正: STM32 固件测速系数偏差, 乘 SPEED_SCALE_FACTOR 校准到实际 m/s
            self._speed_raw = float(m.group(1)) * SPEED_SCALE_FACTOR
            # 速度滤波: 滑动窗口平均 (最近 10 帧 ≈ 1s), 平滑脉冲计数抖动
            self._speed_win.append(self._speed_raw)
            self._speed = sum(self._speed_win) / len(self._speed_win)
            # O 反转: O:0=有物体, O:1=无物体
            self._object_detected = (int(m.group(2)) == 0)
            self._motor_running = bool(int(m.group(3)))
            self._motor_direction = bool(int(m.group(4)))
            self._last_status_time = time.time()

    # ── 命令发送 ──

    def _send_command(self, cmd: bytes) -> bool:
        """发送单字节命令并等待应答。返回 True=OK, False=失败/超时/BUSY。

        ⚠️ 注意:
        1. STM32 每 100ms 持续上报状态帧 (V:..,O:..,M1:..,M2:..), 读应答时
           必须忽略状态帧, 只匹配 OK/ERR/BUSY。
        2. reader_loop 后台线程也在读串口, 命令应答必须持有 _read_lock,
           否则 OK 应答会被 reader_loop 抢走, 导致命令误判超时/失败。
        """
        if not self.is_connected:
            return False
        with self._lock:
            try:
                # 暂停后台读取, 独占串口读命令应答
                with self._read_lock:
                    self._ser.reset_input_buffer()
                    self._ser.write(cmd)
                    self._ser.flush()
                    # 等待应答 (F/R 可能有 500ms 换向延迟; 状态帧不干扰)
                    deadline = time.time() + CMD_RESPONSE_TIMEOUT
                    while time.time() < deadline:
                        chunk = self._ser.read(64)
                        if chunk:
                            if b'OK' in chunk:
                                return True
                            if b'ERR' in chunk:
                                return False
                            if b'BUSY' in chunk:
                                # 下位机正在换向等待, 等待完成后再尝试
                                time.sleep(0.6)
                                return False
                        else:
                            # 无数据时短暂休眠，避免忙等
                            time.sleep(0.01)
                return False
            except serial.SerialException:
                return False

    def start(self) -> bool:
        """启动传送带 (S)."""
        return self._send_command(b'S')

    def stop(self) -> bool:
        """停止传送带 (P)."""
        return self._send_command(b'P')

    def forward(self) -> bool:
        """正转 (F): 自动停止→换向→重启."""
        return self._send_command(b'F')

    def reverse(self) -> bool:
        """反转 (R): 自动停止→换向→重启."""
        return self._send_command(b'R')

    # ── 状态查询 ──

    def get_status(self) -> dict:
        """返回当前传送带状态快照。

        Returns:
            dict: {
                'speed': float,          # 线速度 (m/s), 滤波后
                'speed_raw': float,      # 原始速度 (m/s), 未滤波
                'object_detected': bool, # 物体检测 (已反转, True=有物体)
                'motor_running': bool,   # 电机运行中
                'motor_direction': bool, # 方向 (True=正转)
                'last_status_time': float, # 最后帧时间戳
                'connected': bool,       # 串口连接状态
            }
        """
        with self._status_lock:
            return {
                'speed': self._speed,
                'speed_raw': self._speed_raw,
                'object_detected': self._object_detected,
                'motor_running': self._motor_running,
                'motor_direction': self._motor_direction,
                'last_status_time': self._last_status_time,
                'connected': self.is_connected,
            }


# ────────────────────── CLI ──────────────────────

def main():
    """命令行入口，兼容 relay_control.py 的接口风格。"""
    import argparse

    parser = argparse.ArgumentParser(description="STM32 传送带控制 (串口协议)")
    parser.add_argument("action", nargs="?", choices=["on", "off", "forward", "reverse",
                                                       "status", "start", "stop"],
                        help="动作: on/start=启动, off/stop=停止, forward=正转, reverse=反转, status=状态")
    parser.add_argument("--port", "-p", help="串口号，默认自动检测")
    parser.add_argument("--baudrate", "-b", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--list", "-l", action="store_true", help="列出可用串口")
    parser.add_argument("--monitor", "-m", action="store_true", help="持续监控状态 (Ctrl+C 退出)")

    args = parser.parse_args()

    if args.list:
        ports = serial.tools.list_ports.comports()
        if not ports:
            print("未找到可用串口。")
            return
        print("可用串口列表：")
        for p in ports:
            vid = p.vid if p.vid is not None else 0
            pid = p.pid if p.pid is not None else 0
            tag = "  ← STM32" if (vid, pid) in KNOWN_VID_PID else ""
            print(f"  {p.device:<14} vid={vid:#06x} pid={pid:#06x}  {p.description}{tag}")
        return

    if not args.action and not args.monitor:
        parser.print_help()
        return

    conv = STM32Conveyor(port=args.port, baudrate=args.baudrate)
    try:
        conv.connect()
        print(f"✅ 已连接 STM32: {conv.port} @ {conv.baudrate}")

        if args.monitor:
            print("⏳ 实时监控状态 (Ctrl+C 退出) [速度=滤波后, 括号内为原始值]:")
            print(f"{'时间':>8s}  {'速度(m/s)':>14s}  {'物体':>5s}  {'电机':>5s}  {'方向':>5s}")
            try:
                while True:
                    s = conv.get_status()
                    obj = "有" if s['object_detected'] else "无"
                    motor = "运行" if s['motor_running'] else "停止"
                    direc = "正转" if s['motor_direction'] else "反转"
                    print(f"{time.time():>8.0f}  {s['speed']:>9.2f}({s['speed_raw']:.2f})  {obj:>5s}  {motor:>5s}  {direc:>5s}")
                    time.sleep(0.2)
            except KeyboardInterrupt:
                print("\n监控结束")

        elif args.action in ("status",):
            s = conv.get_status()
            obj = "有物体" if s['object_detected'] else "无物体"
            motor = "运行中" if s['motor_running'] else "已停止"
            direc = "正转" if s['motor_direction'] else "反转"
            print(f"线速度: {s['speed']:.2f} m/s")
            print(f"物体检测: {obj}")
            print(f"电机状态: {motor}")
            print(f"运行方向: {direc}")
            print(f"串口连接: {'已连接' if s['connected'] else '断开'}")

        else:
            # 映射 on/start -> start, off/stop -> stop
            action_map = {
                "on": conv.start, "start": conv.start,
                "off": conv.stop, "stop": conv.stop,
                "forward": conv.forward, "reverse": conv.reverse,
            }
            func = action_map.get(args.action)
            if func:
                ok = func()
                action_names = {"on": "启动", "start": "启动", "off": "停止", "stop": "停止",
                                "forward": "正转", "reverse": "反转"}
                name = action_names.get(args.action, args.action)
                print(f"{'✅' if ok else '❌'} 传送带{name}")

    finally:
        if not args.monitor:
            conv.disconnect()


if __name__ == "__main__":
    main()