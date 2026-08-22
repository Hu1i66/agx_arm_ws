#!/usr/bin/env python3
"""STM32 传送带串口测试界面。

功能:
1. 实时显示 STM32 发送的原始数据 (十六进制 + ASCII + 按 \\r\\n 分帧解析)
2. 按钮发送所有命令: S=启动, P=停止, F=正转, R=反转
3. 显示解析后的 V(速度)/O(物体)/M1(电机)/M2(方向) 及帧统计

⚠️ 线程安全设计:
- 串口读取在后台 daemon 线程, 数据放入 queue.Queue
- Tkinter 主线程用 after() 轮询队列更新界面 (禁止跨线程操作 Tk)
- 串口读写共用一把锁, 避免 read/write 竞态
"""

import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

import serial
import serial.tools.list_ports


# ────────────────── 配置 ──────────────────
DEFAULT_BAUDRATE = 115200
KNOWN_VID_PID = [
    (0x1A86, 0x7523),  # CH340
    (0x1A86, 0x55D4),  # CH340 (其他变体)
    (0x0483, 0x5740),  # STM32 Virtual ComPort (ST-Link)
    (0x10C4, 0xEA60),  # CP2102
    (0x067B, 0x2303),  # PL2303
]

def auto_detect_port() -> str:
    """自动查找 STM32 串口设备。"""
    import os
    for link in ("/dev/stm32_conveyor", "/dev/ttySTM32"):
        if os.path.exists(link):
            return link
    for p in serial.tools.list_ports.comports():
        if (p.vid, p.pid) in KNOWN_VID_PID:
            return p.device
    for p in serial.tools.list_ports.comports():
        if p.device.startswith("/dev/ttyUSB") or p.device.startswith("/dev/ttyACM"):
            return p.device
    return "/dev/ttyUSB0"


# ────────────────── 串口线程 ──────────────────
class SerialBridge:
    """串口读写桥: 后台线程持续读原始数据, 主线程按钮发送命令。"""

    def __init__(self, port: str, baudrate: int = DEFAULT_BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self._ser = None
        self._io_lock = threading.Lock()   # 保护串口 read/write
        self._running = False
        self._reader_thread = None
        self.frame_q = queue.Queue()       # (解析后 dict) → 主线程
        self.last_error = None

    # ── 连接 ──
    def connect(self) -> bool:
        try:
            self._ser = serial.Serial(
                port=self.port, baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.1,
                exclusive=True,  # 串口被占用时直接报错
            )
            self._running = True
            self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._reader_thread.start()
            return True
        except Exception as e:
            self.last_error = f"{type(e).__name__}: {e}"
            return False

    def disconnect(self):
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
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    # ── 读取循环 ──
    def _reader_loop(self):
        buf = b""
        while self._running and self._ser and self._ser.is_open:
            try:
                with self._io_lock:
                    data = self._ser.read(256)
                if not data:
                    continue
                buf += data
                # 按 \r\n 分帧
                while b'\r\n' in buf:
                    line, buf = buf.split(b'\r\n', 1)
                    self._parse_line(line)
            except serial.SerialException as e:
                self.last_error = str(e)
                time.sleep(0.5)
            except Exception:
                time.sleep(0.1)

    def _parse_line(self, line: bytes):
        """解析单行, 不管是否合法都送 frame_q (带原始字节供诊断)。

        ⚠️ O 字段反转: STM32 实际发送 O:0=有物体, O:1=无物体 (与文档相反)。
        原始帧文本仍显示 STM32 发送的原始 O 值, 仅解析后的 O_ok 反转。
        """
        line = line.strip()
        if not line:
            return
        info = {"raw": line, "valid": False,
                "V": None, "O": None, "M1": None, "M2": None,
                "O_ok": False, "V_filt": None}
        try:
            # 容忍前导字符, 用 search 而非 match
            import re
            m = re.search(rb'V:([\d.]+),O:(\d),M1:(\d),M2:(\d)', line)
            if m:
                info.update({
                    "valid": True,
                    "V": float(m.group(1)),  # 串口直接为实际值
                    "O": int(m.group(2)),
                    "M1": int(m.group(3)),
                    "M2": int(m.group(4)),
                    # O 反转: O:0=有物体, O:1=无物体
                    "O_ok": (int(m.group(2)) == 0),
                })
        except Exception:
            pass
        self.frame_q.put(info)

    # ── 发送命令 ──
    def send_command(self, cmd: bytes) -> bool:
        """发送单字节命令并等待应答。返回是否收到 OK。
        注意: STM32 每 100ms 持续上报状态帧, 应答 OK 混在数据流中。
        """
        if not self.is_open:
            return False
        with self._io_lock:
            try:
                self._ser.reset_input_buffer()
                self._ser.write(cmd)
                self._ser.flush()
                deadline = time.time() + 0.8
                while time.time() < deadline:
                    chunk = self._ser.read(64)
                    if chunk:
                        if b'OK' in chunk:
                            return True
                        if b'ERR' in chunk:
                            return False
                        if b'BUSY' in chunk:
                            time.sleep(0.6)
                            return False
                    else:
                        time.sleep(0.01)
                return False
            except serial.SerialException:
                return False

    def send(self, cmd: bytes) -> bool:
        """仅写入, 不等待应答 (用于快速连发/诊断)。"""
        if not self.is_open:
            return False
        with self._io_lock:
            try:
                self._ser.reset_input_buffer()
                self._ser.write(cmd)
                self._ser.flush()
                return True
            except serial.SerialException:
                return False


# ────────────────── GUI ──────────────────
class TestApp(tk.Tk):
    def __init__(self, bridge: SerialBridge):
        super().__init__()
        self.title("STM32 传送带串口测试")
        self.geometry("900x680")
        self.bridge = bridge

        # 显示变量
        self.conn_var = tk.StringVar(value="连接中...")
        self.frame_count = 0
        self.valid_count = 0
        self.last_v = 0.0
        self.last_o = 0
        self.last_m1 = 0
        self.last_m2 = 0
        self.stat_var = tk.StringVar(value="帧: 0  |  有效: 0")
        self.speed_var = tk.StringVar(value="速度: -- m/s")
        self.obj_var = tk.StringVar(value="物体: --")
        self.m1_var = tk.StringVar(value="电机: --")
        self.m2_var = tk.StringVar(value="方向: --")

        self._setup_ui()

        # 连接串口
        if bridge.connect():
            self.conn_var.set(f"✅ 已连接: {bridge.port} @ {bridge.baudrate}")
            self.conn_label.config(fg="green")
        else:
            self.conn_var.set(f"❌ 连接失败: {bridge.last_error}")
            self.conn_label.config(fg="red")

        # 启动轮询
        self.after(50, self._poll_data)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── UI ──
    def _setup_ui(self):
        # 连接状态
        top = tk.Frame(self)
        top.pack(fill="x", padx=8, pady=6)
        self.conn_label = tk.Label(top, textvariable=self.conn_var, font=("Arial", 11, "bold"))
        self.conn_label.pack(side=tk.LEFT)

        # 解析状态 (最新帧)
        stat_frame = tk.LabelFrame(self, text="实时解析 (最新一帧)", padx=8, pady=6)
        stat_frame.pack(fill="x", padx=8, pady=4)
        row = tk.Frame(stat_frame)
        row.pack(fill="x")
        tk.Label(row, textvariable=self.speed_var, font=("Arial", 13, "bold"), fg="blue").pack(side=tk.LEFT, padx=10)
        tk.Label(row, textvariable=self.obj_var, font=("Arial", 13, "bold"), fg="green").pack(side=tk.LEFT, padx=10)
        tk.Label(row, textvariable=self.m1_var, font=("Arial", 13, "bold"), fg="orange").pack(side=tk.LEFT, padx=10)
        tk.Label(row, textvariable=self.m2_var, font=("Arial", 13, "bold"), fg="purple").pack(side=tk.LEFT, padx=10)
        tk.Label(row, textvariable=self.stat_var, font=("Arial", 10), fg="gray").pack(side=tk.RIGHT, padx=10)

        # 文本数据显示 (解析后的帧)
        raw_frame = tk.LabelFrame(self, text="STM32 数据 (分帧文本)", padx=6, pady=6)
        raw_frame.pack(fill="both", expand=True, padx=8, pady=4)
        self.raw_text = tk.Text(raw_frame, font=("Courier", 11), state="disabled",
                                bg="#111", fg="#0f0", wrap="none")
        self.raw_text.pack(fill="both", expand=True)
        sb = ttk.Scrollbar(raw_frame, command=self.raw_text.yview)
        sb.pack(side=tk.RIGHT, fill="y")
        self.raw_text.config(yscrollcommand=sb.set)
        self.raw_text.tag_configure("bad", foreground="#f44")
        self.raw_text.tag_configure("good", foreground="#0f0")

        # 命令按钮
        btn_frame = tk.LabelFrame(self, text="命令 (单字节 ASCII, 大写)", padx=8, pady=8)
        btn_frame.pack(fill="x", padx=8, pady=6)
        bf = tk.Frame(btn_frame)
        bf.pack()

        def mk_btn(text, cmd_byte, bg, fg="white", wait=True):
            def do():
                if not self.bridge.is_open:
                    self._append_text(f"<串口未连接, 无法发送 {text}>", "bad")
                    return
                if wait:
                    ok = self.bridge.send_command(cmd_byte)
                    r = "OK" if ok else "FAIL"
                    self._append_text(f">>> 发送 [{text}] (0x{cmd_byte[0]:02X}) → 应答 {r}",
                                     "good" if ok else "bad")
                else:
                    self.bridge.send(cmd_byte)
                    self._append_text(f">>> 发送 [{text}] (0x{cmd_byte[0]:02X})", "good")
            return tk.Button(bf, text=text, command=do, font=("Arial", 12, "bold"),
                             bg=bg, fg=fg, width=12, padx=6, pady=6)

        mk_btn("启动 S", b'S', "#4CAF50").pack(side=tk.LEFT, padx=5)
        mk_btn("停止 P", b'P', "#f44336").pack(side=tk.LEFT, padx=5)
        mk_btn("正转 F", b'F', "#2196F3").pack(side=tk.LEFT, padx=5)
        mk_btn("反转 R", b'R', "#FF9800").pack(side=tk.LEFT, padx=5)
        tk.Button(bf, text="清空显示", command=self._clear_raw,
                  font=("Arial", 11), bg="#555", fg="white", padx=6, pady=6).pack(side=tk.LEFT, padx=5)
        tk.Button(bf, text="断开", command=self._on_close,
                  font=("Arial", 11), bg="#333", fg="white", padx=6, pady=6).pack(side=tk.LEFT, padx=5)

        hint = tk.Label(btn_frame, text="协议: S/P/F/R 单字节命令 | 状态帧 V:..,O:..,M1:..,M2:.. (每100ms) | 波特率 115200-8N1",
                        font=("Arial", 9), fg="gray")
        hint.pack(pady=(6, 0))

    # ── 数据轮询 (主线程) ──
    def _poll_data(self):
        # 解析帧 → 状态变量 + 统计 + 文本显示
        try:
            while True:
                info = self.bridge.frame_q.get_nowait()
                self.frame_count += 1
                raw = info["raw"].decode('ascii', errors='replace').strip()
                if info["valid"]:
                    self.valid_count += 1
                    self.last_v = info["V"]
                    self.last_o = info["O"]
                    self.last_m1 = info["M1"]
                    self.last_m2 = info["M2"]
                    # 速度滤波: 滑动窗口平均 (最近 10 帧 ≈ 1s)
                    if not hasattr(self, '_speed_win'):
                        from collections import deque
                        self._speed_win = deque(maxlen=10)
                    self._speed_win.append(info["V"])
                    v_filt = sum(self._speed_win) / len(self._speed_win)
                    self.speed_var.set(f"速度: {v_filt:.4f} m/s (原始{info['V']:.4f})")
                    self.obj_var.set(f"物体: {'有' if info['O_ok'] else '无'}")
                    self.m1_var.set(f"电机: {'运行' if info['M1'] else '停止'}")
                    self.m2_var.set(f"方向: {'正转' if info['M2'] else '反转'}")
                    self._append_text(f"{time.strftime('%H:%M:%S.%f')[:-3]}  {raw}", "good")
                else:
                    self._append_text(f"{time.strftime('%H:%M:%S.%f')[:-3]}  ⚠ 无效帧: {raw}", "bad")
                self.stat_var.set(f"帧: {self.frame_count}  |  有效: {self.valid_count}")
        except queue.Empty:
            pass
        self.after(50, self._poll_data)

    # ── 显示文本帧 ──
    def _append_text(self, text: str, tag: str = "good"):
        self.raw_text.config(state="normal")
        self.raw_text.insert("end", text + "\n", tag)
        # 限制行数
        lines = int(self.raw_text.index('end-1c').split('.')[0])
        if lines > 800:
            self.raw_text.delete("1.0", f"{lines - 400}.0")
        self.raw_text.see("end")
        self.raw_text.config(state="disabled")

    def _clear_raw(self):
        self.raw_text.config(state="normal")
        self.raw_text.delete("1.0", "end")
        self.raw_text.config(state="disabled")

    def _on_close(self):
        try:
            self.bridge.disconnect()
        except Exception:
            pass
        self.destroy()


def main():
    port = auto_detect_port()
    bridge = SerialBridge(port=port)
    app = TestApp(bridge)
    try:
        app.mainloop()
    finally:
        bridge.disconnect()


if __name__ == "__main__":
    main()