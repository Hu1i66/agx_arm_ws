#!/usr/bin/env python3
"""机械臂分拣控制面板 GUI。

启动时自动检测并拉起相机 + YOLO 检测节点（后台非阻塞），
画面优先显示 YOLO 标注后的 JPEG，降级为原始相机画面。

⚠️ 关键：所有 Tkinter/PIL 导入均在 main() 内部，
避免 multiprocessing spawn 子进程在无显示器环境中加载 Tcl/Tk C 库导致 SIGSEGV。
"""
import json, os, math, multiprocessing, queue, subprocess, threading, time, io

_LOG = "/tmp/sorting_gui_startup.log"
def _log(msg):
    with open(_LOG, "a") as f:
        f.write(f"[{time.strftime('%H:%M:%S')}] {msg}\n")

# ═══════════════════════ 工具 ═══════════════════════

def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    r, p, y = math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
    cr, sr = math.cos(r*.5), math.sin(r*.5)
    cp, sp = math.cos(p*.5), math.sin(p*.5)
    cy, sy = math.cos(y*.5), math.sin(y*.5)
    return (sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy)

# ═══════════════════════ ROS 子进程 ═══════════════════════

def ros_process_worker(cmd_q, st_q, pose_q, img_q, det_q, conv_q, conv_status_q, mp_status_q):
    """ROS2 子进程 — camera/YOLO 回调即时推送 JPEG。 无 Tkinter 依赖。

    ⚠️ auto_start_pipeline (相机/YOLO 自动拉起) 也在此子进程中以守护线程运行。
    原因: 主进程加载了 Tcl/Tk C 库, 多线程中 fork() (subprocess.run/Popen)
    会破坏 Tcl 引用计数 → "Tcl_Release couldn't find reference" → 段错误。
    本子进程无 Tcl 依赖, fork() 安全。
    """
    # 在 ROS worker 子进程中启动自动启动管线 (此进程无 Tcl, fork 安全)
    def _mp_set_status(msg):
        try: mp_status_q.put_nowait(msg)
        except: pass
    threading.Thread(target=auto_start_pipeline, args=(_mp_set_status,), daemon=True).start()

    # ═════════ STM32 传送带初始化 (串口 115200-8N1) ═════════
    # 取代原 CH340 继电器控制 (Modbus RTU 9600)
    from stm32_conveyor import STM32Conveyor
    _conv = STM32Conveyor()
    _conv_ok = False
    try:
        _conv.connect()
        _conv_ok = True
        _log(f"STM32 传送带已连接: {_conv.port} @ {_conv.baudrate}")
    except Exception as e:
        _log(f"STM32 传送带连接失败: {e} (将使用模拟模式)")

    # ⚠️ 父进程监控: GUI 崩溃/退出后, worker 必须自动释放串口,
    # 否则变成孤儿进程持续抢占 /dev/ttyUSB0 → 下次 GUI 显示速度 0。
    _ppid0 = os.getppid()
    def _parent_watchdog():
        while True:
            try:
                if os.getppid() != _ppid0:
                    _log(f"父进程已退出 (pid {_ppid0}), worker 自动退出")
                    try: _conv.disconnect()
                    except Exception: pass
                    os._exit(0)
            except Exception:
                pass
            time.sleep(1.0)
    threading.Thread(target=_parent_watchdog, daemon=True).start()
    # 传送带状态监控线程: 每 250ms 推送一次状态到 conv_status_q
    def _conv_monitor():
        while True:
            try:
                if _conv_ok:
                    s = _conv.get_status()
                    try:
                        conv_status_q.put_nowait({'type': 'status', 'speed': s['speed'],
                                       'object_detected': s['object_detected'],
                                       'motor_running': s['motor_running']})
                    except Exception:
                        pass
                else:
                    try:
                        conv_status_q.put_nowait({'type': 'status', 'speed': 0.0,
                                       'object_detected': False, 'motor_running': False})
                    except Exception:
                        pass
            except Exception:
                pass
            time.sleep(0.25)
    threading.Thread(target=_conv_monitor, daemon=True).start()

    # ═════════ 模式切换 (水果 YOLO ↔ 蓝方块检测) ═════════
    # 在 ROS worker 子进程中执行 kill/launch (fork 安全), 通过 mp_status_q 回报状态。
    # 主进程 (Tkinter) 禁止 fork, 所有 subprocess 调用均委托至此。
    def _do_switch_mode(mode):
        """切换检测节点 (在独立线程中执行, 不阻塞 ROS spinner)。
        mode='blue_block': 停止 realsense_yolo_node.py, 启动 blue_block_detector.py
        mode='fruit':      停止 blue_block_detector.py, 启动 realsense_yolo_node.py
        """
        if mode == 'blue_block':
            _mp_set_status("正在切换到蓝方块模式: 停止水果 YOLO 节点...")
            _kill_by_pattern("realsense_yolo_node.py")
            time.sleep(1.5)  # 等待 ROS 节点清理 (订阅/发布注销)
            _mp_set_status("正在切换到蓝方块模式: 启动 blue_block_detector...")
            _launch("BLUE_BLOCK", _BLUE_BLOCK_CMD)
            # 等待 blue_block_detector 上线 (检测 /detection_info 话题有新发布者)
            for i in range(15):
                time.sleep(2); _mp_set_status(f"等待蓝方块节点上线... ({(i+1)*2}s)")
                if _topic_ok('/detection_info'):
                    _mp_set_status("已切换到蓝方块模式 (blue_block_detector 已上线)")
                    return
            _mp_set_status("蓝方块节点启动超时, 请检查 /tmp/sorting_gui_startup.log")
        elif mode == 'fruit':
            _mp_set_status("正在切换到水果模式: 停止 blue_block_detector...")
            _kill_by_pattern("blue_block_detector.py")
            time.sleep(1.5)
            _mp_set_status("正在切换到水果模式: 启动 realsense_yolo_node...")
            _launch("YOLO", _YOLO_CMD)
            for i in range(20):
                time.sleep(3); _mp_set_status(f"等待 YOLO 加载... ({(i+1)*3}s)")
                if _topic_ok('/detection_info'):
                    _mp_set_status("已切换到水果模式 (realsense_yolo_node 已上线)")
                    return
            _mp_set_status("YOLO 节点启动超时, 请检查 /tmp/sorting_gui_startup.log")

    import rclpy
    from rclpy.node import Node
    from rclpy.logging import LoggingSeverity, set_logger_level
    from std_msgs.msg import String
    from sensor_msgs.msg import CompressedImage, Image
    import tf2_ros, numpy as np, cv2

    class N(Node):
        def __init__(self):
            super().__init__('sorting_headless_client_node')
            for n in ('tf2','tf2_ros','tf2_buffer','tf2_ros_buffer'):
                try: set_logger_level(n, LoggingSeverity.ERROR)
                except: pass
            self._pub = self.create_publisher(String, '/sorting_cmds', 10)
            # 曝光控制命令发布器 (realsense_yolo_node.py 订阅 /camera/exposure_ctrl)
            self._exp_pub = self.create_publisher(String, '/camera/exposure_ctrl', 10)
            # 急停/返回待机位 命令发布器 (auto_sorting_action.py 订阅 /emergency_stop)
            self._emergency_pub = self.create_publisher(String, '/emergency_stop', 10)
            self.create_subscription(String, '/sorting_status', self._on_st, 10)
            self._tf_buf = tf2_ros.Buffer()
            self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)
            self.create_subscription(CompressedImage, '/yolo/annotated_image/compressed', self._on_yolo, 10)
            self.create_subscription(Image, '/camera/camera/color/image_raw', self._on_raw, 10)
            self.create_subscription(String, '/detection_info', self._on_det, 10)
            self._yolo_ts = 0.0
            self.create_timer(0.05, self._cmd_tick)
            self.create_timer(0.2, self._tf_tick)

        def _on_st(self, m): st_q.put(m.data)
        def _on_yolo(self, m):
            try:
                self._yolo_ts = time.time()
                _putq(img_q, m.data)
            except: pass
        def _on_raw(self, m):
            if time.time() - self._yolo_ts < 1.0: return
            try:
                d = m.data
                a = np.frombuffer(d, np.uint8) if isinstance(d,(bytes,bytearray)) else np.array(d, np.uint8, copy=False)
                e = m.encoding
                if e in ('bgr8','bgr'):     r = cv2.cvtColor(a.reshape((m.height,m.width,3)), cv2.COLOR_BGR2RGB)
                elif e in ('rgb8','rgb'):    r = a.reshape((m.height,m.width,3))
                elif e in ('rgba8','rgba'):  r = cv2.cvtColor(a.reshape((m.height,m.width,4)), cv2.COLOR_RGBA2RGB)
                elif e in ('bgra8','bgra'):  r = cv2.cvtColor(a.reshape((m.height,m.width,4)), cv2.COLOR_BGRA2RGB)
                elif e == 'mono8':           r = cv2.cvtColor(a.reshape((m.height,m.width)), cv2.COLOR_GRAY2RGB)
                else:                        r = a.reshape((m.height,m.width,3))
                ok, j = cv2.imencode('.jpg', cv2.cvtColor(r, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY,85])
                if ok: _putq(img_q, j.tobytes())
            except: pass
        def _on_det(self, m):
            try:
                while not det_q.empty():
                    try: det_q.get_nowait()
                    except: break
                det_q.put(m.data)
            except: pass
        def _tf_tick(self):
            try:
                t = self._tf_buf.lookup_transform('base_link','link6',rclpy.time.Time())
                _putq(pose_q, {'x':t.transform.translation.x,'y':t.transform.translation.y,'z':t.transform.translation.z})
            except: pass
        def _cmd_tick(self):
            while not cmd_q.empty():
                cmd = cmd_q.get()
                # 传送带控制: STM32 串口协议 (115200-8N1, 取代原 CH340 继电器)
                if cmd.get("cmd") == "conveyor":
                    relay_action = cmd.get("relay_action", "off")
                    try:
                        if _conv_ok:
                            if relay_action == "on":
                                ok = _conv.start()
                            else:
                                ok = _conv.stop()
                            err = ""
                        else:
                            ok = False
                            err = "STM32 未连接"
                    except Exception as e:
                        ok = False
                        err = str(e)
                    try:
                        conv_q.put_nowait({'ok': ok, 'err': err, 'type': 'cmd_result'})
                    except:
                        pass
                    continue
                # 模式切换 (水果 YOLO ↔ 蓝方块检测): 在 ROS worker 子进程中执行 kill/launch
                # 主进程有 Tcl, fork 会导致崩溃, 所有 subprocess 调用委托至此子进程
                if cmd.get("cmd") == "switch_mode":
                    mode = cmd.get("mode")  # "blue_block" 或 "fruit"
                    threading.Thread(target=_do_switch_mode, args=(mode,), daemon=True).start()
                    continue
                m = String(); m.data = json.dumps(cmd)
                # 曝光控制命令发到 /camera/exposure_ctrl, 其他命令发到 /sorting_cmds
                if cmd.get("cmd") == "exposure":
                    self._exp_pub.publish(m)
                elif cmd.get("cmd") in ("emergency_stop", "standby"):
                    # 急停/返回待机位: 发布到独立 /emergency_stop topic
                    # auto_sorting_action.py 在分拣序列检查点中检测此信号并执行安全停机
                    self._emergency_pub.publish(m)
                else:
                    self._pub.publish(m)

    rclpy.init(); node = N()
    try: rclpy.spin(node)
    except: pass
    finally:
        node.destroy_node()
        try: rclpy.shutdown()
        except: pass

def _putq(q, item):
    while not q.empty():
        try: q.get_nowait()
        except: break
    try: q.put_nowait(item)
    except: pass

# ═══════════════════════ 自动启动管线 ═══════════════════════

_ROS2_SRC = "source /opt/ros/humble/setup.bash"
_VENV_SRC = "source /home/lxf/orange_dataset/.venv/bin/activate"
_YOLO_SCRIPT = "/home/lxf/orange_dataset/realsense_yolo_node.py"
# 蓝方块检测节点 (Task 1.1-1.4): HSV 颜色分割 + minAreaRect + RANSAC 深度
_BLUE_BLOCK_SCRIPT = "/home/lxf/orange_dataset/blue_block_detector.py"
# 蓝方块节点 Python 环境: 直接用 venv python (与 shebang 一致, 避免 activate 开销)
_BLUE_BLOCK_VENV_PY = "/home/lxf/orange_dataset/.venv/bin/python3"
# 相机启动: 显式指定 640x480@30 与另一台电脑 (pyrealsense2 直接配置) 保持一致
# align_depth.enable:=true: 启用深度图与 RGB 对齐 (D455 双目深度 + PCA 短轴对齐必需)
# 深度后处理滤波链 (Phase 0 优化): rs_launch.py 仅支持 *.enable 参数
#   子参数 (filter_magnitude/filter_smooth_alpha/fill_mode 等) 未在 launch 声明, 用节点默认值
#   如需调子参数, 启动后用 ros2 param set /camera/camera <param> <value> 动态调整
_CAMERA_CMD = (f"{_ROS2_SRC} && ros2 launch realsense2_camera rs_launch.py "
               f"rgb_camera.color_profile:=640,480,30 "
               f"depth_module.profile:=640,480,30 "
               f"align_depth.enable:=true "
               f"spatial_filter.enable:=true "
               f"temporal_filter.enable:=true "
               f"hole_filling_filter.enable:=true")
_YOLO_CMD   = f"{_ROS2_SRC} && {_VENV_SRC} && python3 {_YOLO_SCRIPT} --ros-args -p show_gui_window:=false"
# 蓝方块检测节点启动命令 (venv python 直接调用, 不通过 activate)
_BLUE_BLOCK_CMD = f"{_ROS2_SRC} && {_BLUE_BLOCK_VENV_PY} {_BLUE_BLOCK_SCRIPT}"



def _topic_ok(topic, timeout=3.0):
    try:
        r = subprocess.run(
            f"bash -c '{_ROS2_SRC} && ros2 topic info {topic}'",
            shell=True, capture_output=True, text=True, timeout=timeout)
        return "Publisher count: 0" not in r.stdout and "Type:" in r.stdout
    except: return False

def _launch(label, cmd):
    _log(f"LAUNCH {label}")
    return subprocess.Popen(f"bash -c '{cmd}'", shell=True,
                            stdout=open(_LOG,"a"), stderr=open(_LOG,"a"),
                            preexec_fn=os.setsid)

def _kill_by_pattern(pattern):
    """通过 pkill -f 杀死匹配 pattern 的进程 (在 ROS worker 子进程中调用, fork 安全).
    用于模式切换时停止旧视觉节点 (realsense_yolo_node.py / blue_block_detector.py).
    Returns: True 如果命令执行成功 (即使没匹配到进程也算成功)."""
    try:
        subprocess.run(['pkill', '-f', pattern], capture_output=True, timeout=5)
        _log(f"KILL pattern='{pattern}' done")
        return True
    except Exception as e:
        _log(f"KILL pattern='{pattern}' 失败: {e}")
        return False

def auto_start_pipeline(set_status):
    _log("AUTO_START begin")
    if _topic_ok('/detection_info'):
        _log("YOLO already online")
        set_status("YOLO 已在线"); return
    if not _topic_ok('/camera/camera/color/image_raw'):
        _log("Camera offline, launching")
        set_status("正在启动相机...")
        _launch("CAMERA", _CAMERA_CMD)
        for i in range(15):
            time.sleep(2); set_status(f"等待相机上线... ({(i+1)*2}s)")
            if _topic_ok('/camera/camera/color/image_raw'):
                _log("Camera online"); break
        else:
            _log("Camera timeout"); set_status("相机启动失败"); return
    _log("Launching YOLO")
    set_status("正在启动 YOLO 检测...")
    _launch("YOLO", _YOLO_CMD)
    for i in range(20):
        time.sleep(3); set_status(f"等待 YOLO 加载... ({(i+1)*3}s)")
        if _topic_ok('/detection_info'):
            _log("YOLO online"); set_status("YOLO 已上线"); return
    _log("YOLO timeout"); set_status("YOLO 启动超时")


# ═══════════════════════ main ═══════════════════════
# 所有 Tkinter / PIL / GUI 类定义均在 main() 内部，
# 确保 spawn 子进程不会加载它们。

def main():
    import tkinter as tk
    from tkinter import ttk, messagebox

    _log("GUI MAIN START")

    cmd_q = multiprocessing.Queue(); st_q = multiprocessing.Queue()
    pose_q = multiprocessing.Queue(); img_q = multiprocessing.Queue(maxsize=1)
    det_q = multiprocessing.Queue(maxsize=1)
    conv_q = multiprocessing.Queue()  # 传送带命令结果队列 (ROS worker → GUI, 仅 cmd_result)
    conv_status_q = multiprocessing.Queue()  # 传送带状态队列 (ROS worker → GUI, 仅 status)
    mp_status_q = multiprocessing.Queue()  # 相机/YOLO 启动状态消息队列 (ROS worker → GUI)

    ros_proc = multiprocessing.Process(target=ros_process_worker,
                                       args=(cmd_q,st_q,pose_q,img_q,det_q,conv_q,conv_status_q,mp_status_q), name="ros")
    ros_proc.start()
    _log("ROS proc started")

    # =========== GUI 类定义 ===========

    class SortingApp(tk.Tk):
        def __init__(self):
            super().__init__()
            self.cmd_queue=cmd_q; self.status_queue=st_q; self.pose_queue=pose_q
            self.image_queue=img_q; self.detection_queue=det_q
            self.conveyor_queue=conv_q  # 传送带命令结果队列 (仅 cmd_result)
            self.conveyor_status_queue=conv_status_q  # 传送带状态队列 (仅 status)
            self.title("机械臂分拣控制面板")
            self.geometry("1350x680"); self.configure(padx=10, pady=10)
            self.poses = {}; self.load_poses()
            self.current_status='idle'; self.task_queue=[]
            self.last_dispatch_time=0; self.queue_running=False
            self.latest_detection=None; self._last_frame_time=0.0
            self._canvas_img_id=None; self._canvas_text_id=None
            self._photos=[]; self._canvas_w=400; self._canvas_h=300
            # ── 抓取模式: 单次检测直接抓取 / 两阶段精定位 (D455+PCA) / GraspNet 6DoF 主导 ──
            # 缓存最近一次 /detection_info 的 header_stamp (浮点秒数), 保留备用
            self.latest_detection_stamp=0.0
            self.default_grasp_roll=tk.DoubleVar(value=180.0)
            self.default_grasp_pitch=tk.DoubleVar(value=0.0)
            self.default_grasp_yaw=tk.DoubleVar(value=0.0)
            # 放置料框选择: 1=料框1, 2=料框2 (抓取前由用户选定, 决定放置关节点位)
            self.bin_var=tk.IntVar(value=1)
            # 传送带(继电器)状态: True=运行, False=停止, None=未知
            self._conveyor_busy=False
            self.conveyor_state=None
            self._conveyor_run=None         # 当前任务对应的传送带意图
            self._conveyor_start_time=0     # 继电器命令发送时间 (用于超时检测)
            # 曝光控制状态 (与 realsense_yolo_node.py 的 self.auto_exposure/current_exposure 对应)
            self._exposure_auto=True        # True=自动曝光, False=手动
            self._exposure_value=100        # 手动曝光值 (微秒, 范围 1-10000)
            # ── 检测模式: 'fruit' (水果 YOLO) 或 'blue_block' (蓝方块 HSV+RANSAC) ──
            # 模式切换通过 ROS worker 子进程 kill/launch 对应视觉节点 (互斥运行, 避免 D455 USB 带宽竞争)
            self.detection_mode='fruit'
            self._mode_switching=False      # True=正在切换中 (禁用按钮防重复点击)
            self._mode_switch_deadline=0    # 切换超时时间戳 (用于检测切换是否卡住)
            # ── 自动分拣状态机 ──
            self.auto_sort_running=False       # 是否运行中
            self.auto_sort_state='IDLE'        # 当前状态
            self._auto_sort_current_obj=None   # 当前分拣物体 (用于重试)
            self._auto_sort_retry_count=0      # 重试计数 (0=首次, 1=重试)
            self._auto_error_seen=False        # SORTING 中是否看到 error
            self._auto_sort_stop_requested=False  # 停止请求标志
            self._auto_sort_line_u=260         # 分界线 u 坐标 (向左偏移, 物体从左向右移动, 越线后进入抓取区)
            self._auto_sort_tick_id=None       # after 句柄
            self._conv_stop_retry=0            # 传送带停止重试计数
            self._conv_stopped_ts=0            # 传送带停止完成时间戳 (用于观察 1s 等物体静止)
            self.setup_ui()
            self._update_status_loop()
            self._update_camera_loop()
            self._update_detection_loop()
            self._draw_dividing_line()  # 初始化分界线 (默认隐藏)
            # 启动时自动发送传送带停止命令:
            # 1) 确保传送带处于停止状态 (安全默认)
            # 2) 继电器调用已委托给 ROS worker 进程, 不会触发 Tcl 崩溃
            self.after(800, lambda: self._conveyor_set(False))
            # 启动传送带状态轮询 (速度、物体检测显示)
            self.after(1000, self._conveyor_status_poll)

        def load_poses(self):
            f='/home/lxf/agx_arm_ws/saved_poses.json'
            if os.path.exists(f):
                try:
                    with open(f) as fh:
                        raw=json.load(fh)
                        self.poses={n:{'x':float(p['x']),'y':float(p['y']),'z':float(p['z'])}
                                    for n,p in raw.items() if isinstance(p,dict) and all(k in p for k in('x','y','z'))}
                except: self.poses={}
            else: self.poses={}

        def save_poses(self):
            with open('/home/lxf/agx_arm_ws/saved_poses.json','w') as f:
                json.dump(self.poses,f,indent=4)
            self.update_comboboxes()

        def setup_ui(self):
            self.left_frame=tk.Frame(self,width=420); self.left_frame.pack(side=tk.LEFT,fill=tk.Y,padx=(0,10)); self.left_frame.pack_propagate(False)
            self.mid_frame=tk.Frame(self,width=240); self.mid_frame.pack(side=tk.LEFT,fill=tk.Y,padx=(0,10)); self.mid_frame.pack_propagate(False)
            self.right_frame=tk.Frame(self); self.right_frame.pack(side=tk.LEFT,fill=tk.BOTH,expand=True)
            self._ui_camera(); self._ui_detection(); self._ui_queue(); self._ui_control()

        def _ui_camera(self):
            f=tk.LabelFrame(self.left_frame,text=" 相机画面",padx=5,pady=5); f.pack(fill=tk.X,pady=(0,5))
            self.camera_canvas=tk.Canvas(f,width=self._canvas_w,height=self._canvas_h,bg='#1a1a1a'); self.camera_canvas.pack()
            self._canvas_img_id=self.camera_canvas.create_image(self._canvas_w//2,self._canvas_h//2,anchor=tk.CENTER)
            self._canvas_text_id=self.camera_canvas.create_text(
                self._canvas_w//2,self._canvas_h//2,text="等待画面...",fill="#888",font=("Arial",14),anchor=tk.CENTER)
            self.camera_status_var=tk.StringVar(value="正在检测设备...")
            tk.Label(f,textvariable=self.camera_status_var,font=("Arial",9),fg="gray").pack(pady=(2,0))

            # ── 曝光控制 (通过 /camera/exposure_ctrl 话题发给 realsense_yolo_node.py) ──
            ef=tk.LabelFrame(f,text="曝光控制",padx=5,pady=5); ef.pack(fill=tk.X,pady=(5,0))
            self.exposure_var=tk.StringVar(value="当前: 自动")
            tk.Label(ef,textvariable=self.exposure_var,font=("Arial",10,"bold"),fg="green").pack(anchor=tk.W,pady=(0,3))
            ebf=tk.Frame(ef); ebf.pack(fill=tk.X)
            self.exp_toggle_btn=tk.Button(ebf,text="切换 自动/手动",command=self._exposure_toggle,
                                          font=("Arial",9),width=14); self.exp_toggle_btn.pack(side=tk.LEFT,padx=(0,4))
            self.exp_inc_btn=tk.Button(ebf,text="曝光 +",command=lambda:self._exposure_adjust(1),
                                       font=("Arial",9),width=8,state=tk.DISABLED); self.exp_inc_btn.pack(side=tk.LEFT,padx=(0,4))
            self.exp_dec_btn=tk.Button(ebf,text="曝光 -",command=lambda:self._exposure_adjust(-1),
                                       font=("Arial",9),width=8,state=tk.DISABLED); self.exp_dec_btn.pack(side=tk.LEFT)
            tk.Label(ef,text="手动模式 +/- 调节 (步长 100, 范围 1-10000)",font=("Arial",8),fg="gray").pack(anchor=tk.W,pady=(3,0))

            # ── 夹爪力矩控制 (通过 /sorting_cmds 发给 auto_sorting_action.py) ──
            gf=tk.LabelFrame(f,text="夹爪力矩控制",padx=5,pady=5); gf.pack(fill=tk.X,pady=(5,0))
            self.gripper_force_var=tk.DoubleVar(value=1.5)
            self.gripper_force_label=tk.StringVar(value="当前: 1.50N")
            tk.Label(gf,textvariable=self.gripper_force_label,font=("Arial",10,"bold"),fg="blue").pack(anchor=tk.W,pady=(0,3))
            # 力矩滑块 (0.5-3.0N) + 快速预设按钮
            sf=tk.Frame(gf); sf.pack(fill=tk.X)
            tk.Scale(sf,from_=0.5,to=3.0,resolution=0.1,orient=tk.HORIZONTAL,
                     variable=self.gripper_force_var,command=self._on_force_change,
                     length=160,font=("Arial",8)).pack(side=tk.LEFT,padx=(0,4))
            for label,val in [("软",0.8),("中",1.5),("硬",2.5)]:
                tk.Button(sf,text=label,command=lambda v=val:self._set_force(v),
                          font=("Arial",8),width=3).pack(side=tk.LEFT,padx=1)
            # 抓取状态显示
            ttk.Separator(gf,orient='horizontal').pack(fill=tk.X,pady=3)
            self.grasp_status_var=tk.StringVar(value="抓取状态: --")
            self.grasp_status_label=tk.Label(gf,textvariable=self.grasp_status_var,
                     font=("Arial",10),fg="gray")
            self.grasp_status_label.pack(anchor=tk.W)
            self.grasp_detail_var=tk.StringVar(value="")
            tk.Label(gf,textvariable=self.grasp_detail_var,font=("Arial",8),fg="gray").pack(anchor=tk.W)

        def _ui_detection(self):
            df=tk.LabelFrame(self.left_frame,text=" 检测信息",padx=5,pady=5); df.pack(fill=tk.BOTH,expand=True)
            self.det_status_var=tk.StringVar(value=" 等待检测数据...")
            tk.Label(df,textvariable=self.det_status_var,font=("Arial",12,"bold"),fg="gray").pack(anchor=tk.W,pady=(0,5))
            self.det_obj=tk.StringVar(value="物体: --"); self.det_conf=tk.StringVar(value="置信度: --")
            self.det_meth=tk.StringVar(value="方法: --")
            for v in(self.det_obj,self.det_conf,self.det_meth): tk.Label(df,textvariable=v,font=("Arial",10)).pack(anchor=tk.W)
            ttk.Separator(df,orient='horizontal').pack(fill=tk.X,pady=5)
            tk.Label(df,text="基座坐标 (base_link):",font=("Arial",10,"bold")).pack(anchor=tk.W)
            self.det_x=tk.StringVar(value="  X: --"); self.det_y=tk.StringVar(value="  Y: --"); self.det_z=tk.StringVar(value="  Z: --")
            for v in(self.det_x,self.det_y,self.det_z): tk.Label(df,textvariable=v,font=("Arial",10,"italic")).pack(anchor=tk.W)
            self.det_dia=tk.StringVar(value="直径: --"); self.det_dist=tk.StringVar(value="距离: --")
            tk.Label(df,textvariable=self.det_dia,font=("Arial",10)).pack(anchor=tk.W)
            tk.Label(df,textvariable=self.det_dist,font=("Arial",10)).pack(anchor=tk.W)
            ttk.Separator(df,orient='horizontal').pack(fill=tk.X,pady=5)
            # ── 所有检测物体列表 (点击切换夹取目标) ──
            tk.Label(df,text="所有检测物体 (点击设为夹取目标):",font=("Arial",10,"bold")).pack(anchor=tk.W)
            lf2=tk.Frame(df); lf2.pack(fill=tk.X,pady=2)
            self.det_listbox=tk.Listbox(lf2,height=6,font=("Arial",9),selectmode=tk.SINGLE,exportselection=False)
            sb2=tk.Scrollbar(lf2,orient="vertical",command=self.det_listbox.yview)
            self.det_listbox.pack(side=tk.LEFT,fill=tk.X,expand=True)
            sb2.pack(side=tk.RIGHT,fill=tk.Y)
            self.det_listbox.config(yscrollcommand=sb2.set)
            self.det_listbox.bind('<<ListboxSelect>>', self._on_det_listbox_select)
            self._det_objects=[]
            ttk.Separator(df,orient='horizontal').pack(fill=tk.X,pady=5)
            tk.Label(df,text="抓取姿态 (欧拉角 度):",font=("Arial",10,"bold")).pack(anchor=tk.W)
            rf=tk.Frame(df); rf.pack(fill=tk.X,pady=2)
            tk.Label(rf,text="Roll:",font=("Arial",9)).pack(side=tk.LEFT)
            tk.Entry(rf,textvariable=self.default_grasp_roll,width=6,font=("Arial",9)).pack(side=tk.LEFT,padx=(2,8))
            tk.Label(rf,text="Pitch:",font=("Arial",9)).pack(side=tk.LEFT)
            tk.Entry(rf,textvariable=self.default_grasp_pitch,width=6,font=("Arial",9)).pack(side=tk.LEFT,padx=(2,8))
            tk.Label(rf,text="Yaw:",font=("Arial",9)).pack(side=tk.LEFT)
            tk.Entry(rf,textvariable=self.default_grasp_yaw,width=6,font=("Arial",9)).pack(side=tk.LEFT,padx=(2,0))
            ttk.Separator(df,orient='horizontal').pack(fill=tk.X,pady=8)
            self.pick_btn=tk.Button(df,text=" 从检测坐标夹取 ->",command=self._pick_from_detection,
                                    font=("Arial",12,"bold"),bg="#4CAF50",fg="white",state=tk.DISABLED)
            self.pick_btn.pack(fill=tk.X,pady=(0,2))
            self.pick_two_btn=tk.Button(df,text=" 两阶段精定位夹取 ->",command=self._pick_from_detection_two_stage,
                                        font=("Arial",12,"bold"),bg="gray",fg="white",state=tk.DISABLED)
            self.pick_two_btn.pack(fill=tk.X,pady=(0,2))
            self.pick_graspnet_btn=tk.Button(df,text=" GraspNet抓取 ->",command=self._pick_from_detection_graspnet,
                                             font=("Arial",12,"bold"),bg="#FF9800",fg="white",state=tk.DISABLED)
            self.pick_graspnet_btn.pack(fill=tk.X,pady=(0,2))
            # ── 蓝方块抓取按钮 (仅蓝方块模式可用, 蓝色 #33CAE8 与按钮模式色一致) ──
            self.pick_blue_block_btn=tk.Button(df,text=" 抓取蓝方块 ->",command=self._pick_blue_block,
                                               font=("Arial",12,"bold"),bg="#33CAE8",fg="white",state=tk.DISABLED)
            self.pick_blue_block_btn.pack(fill=tk.X,pady=(0,2))
            tk.Label(df,text=" =单次检测 | =两阶段精定位(已禁用) | =GraspNet 6DoF主导 | =蓝方块(蓝方块模式专用)",
                     font=("Arial",8),fg="gray").pack()

        def _ui_queue(self):
            tk.Label(self.mid_frame,text="任务排队队列",font=("Arial",14,"bold")).pack(pady=(0,10))
            lf=tk.Frame(self.mid_frame); lf.pack(fill=tk.BOTH,expand=True)
            self.qlist=tk.Listbox(lf,font=("Arial",11),selectmode=tk.SINGLE); self.qlist.pack(side=tk.LEFT,fill=tk.BOTH,expand=True)
            sb=tk.Scrollbar(lf,orient="vertical",command=self.qlist.yview); sb.pack(side=tk.RIGHT,fill=tk.Y)
            self.qlist.config(yscrollcommand=sb.set)
            bf=tk.Frame(self.mid_frame); bf.pack(fill=tk.X,pady=10)
            tk.Button(bf,text="删除选定任务",command=self.remove_from_queue,width=12).pack(side=tk.LEFT,padx=(0,5))
            tk.Button(bf,text="清空队列",command=self.clear_queue,width=12,fg="red").pack(side=tk.RIGHT)
            self.qrun=tk.Button(self.mid_frame,text=" 开始执行排队任务",command=self.toggle_queue_execution,
                                bg="lightgreen",font=("Arial",12,"bold")); self.qrun.pack(fill=tk.X,pady=5)

        def _ui_control(self):
            self.status_var=tk.StringVar(value="当前状态: 未知")
            self.status_label=tk.Label(self.right_frame,textvariable=self.status_var,font=("Arial",16,"bold"),fg="blue")
            self.status_label.pack(pady=(0,10))
            # ── 检测模式切换 (水果 YOLO ↔ 蓝方块 HSV+RANSAC) ──
            # 两种视觉节点互斥运行 (避免 D455 USB 带宽竞争), 切换通过 ROS worker 子进程 kill/launch
            fms=tk.LabelFrame(self.right_frame,text=" 检测模式切换",padx=10,pady=10); fms.pack(fill="x",pady=5)
            mbf=tk.Frame(fms); mbf.pack(fill="x")
            self.fruit_mode_btn=tk.Button(mbf,text=" 水果分拣模式 ",
                                          command=self._switch_to_fruit_mode,
                                          font=("Arial",12,"bold"),bg="#4CAF50",fg="white",
                                          relief=tk.SUNKEN)  # 默认水果模式高亮
            self.fruit_mode_btn.pack(side=tk.LEFT,expand=True,fill="x",padx=(0,4))
            self.blue_block_mode_btn=tk.Button(mbf,text=" 蓝方块分拣模式 ",
                                              command=self._switch_to_blue_block_mode,
                                              font=("Arial",12,"bold"),bg="#33CAE8",fg="white",
                                              relief=tk.RAISED)
            self.blue_block_mode_btn.pack(side=tk.LEFT,expand=True,fill="x",padx=(4,0))
            self.mode_status_var=tk.StringVar(value="当前模式: 水果分拣 (realsense_yolo_node)")
            tk.Label(fms,textvariable=self.mode_status_var,font=("Arial",10),fg="gray").pack(pady=(5,0))
            tk.Label(fms,text=" 切换会停止当前视觉节点并启动新节点 (约 5-30s), 互斥运行避免 D455 USB 带宽竞争",
                     font=("Arial",8),fg="gray").pack()
            fn=tk.LabelFrame(self.right_frame,text="新建与保存坐标",padx=10,pady=10); fn.pack(fill="x",pady=5)
            tk.Label(fn,text="坐标名称:").grid(row=0,column=0,padx=5)
            self.name_entry=tk.Entry(fn,width=15); self.name_entry.grid(row=0,column=1,padx=5)
            tk.Button(fn,text="读取机械臂当前位置",command=self.fetch_current_pose).grid(row=0,column=2,padx=10)
            self.pose_text_var=tk.StringVar(value="请点击读取位置...")
            tk.Entry(fn,textvariable=self.pose_text_var,state='readonly',width=50).grid(row=1,column=0,columnspan=3,pady=10)
            tk.Button(fn,text="保存该坐标",command=self.save_current_pose,bg="lightblue").grid(row=1,column=3,padx=10)
            fm=tk.LabelFrame(self.right_frame,text="坐标管理",padx=10,pady=5); fm.pack(fill="x",pady=5)
            tk.Label(fm,text="选择已存坐标:").grid(row=0,column=0,padx=5)
            self.del_combo=ttk.Combobox(fm,state="readonly",width=15); self.del_combo.grid(row=0,column=1,padx=5)
            tk.Button(fm,text="  删除选中坐标",command=self.delete_pose,fg="red").grid(row=0,column=2,padx=10)
            ft=tk.LabelFrame(self.right_frame,text="分拣任务下发",padx=10,pady=10); ft.pack(fill="x",pady=5)
            tk.Label(ft,text="夹取点 (Pick):").grid(row=0,column=0,pady=10)
            self.pick_combo=ttk.Combobox(ft,state="readonly",width=15); self.pick_combo.grid(row=0,column=1,padx=10)
            tk.Label(ft,text="放置料框:").grid(row=0,column=2,pady=10)
            bf=tk.Frame(ft); bf.grid(row=0,column=3,padx=10)
            tk.Radiobutton(bf,text="料框1",variable=self.bin_var,value=1,font=("Arial",11,"bold")).pack(side=tk.LEFT)
            tk.Radiobutton(bf,text="料框2",variable=self.bin_var,value=2,font=("Arial",11,"bold")).pack(side=tk.LEFT,padx=(8,0))
            tk.Button(ft,text="添加至分拣队列",command=self.add_to_queue,font=("Arial",11,"bold"),bg="lightblue",width=18).grid(row=1,column=0,columnspan=2,pady=10)
            tk.Button(ft,text="立即发送分拣指令",command=self.send_sort,font=("Arial",11,"bold"),bg="lightgreen",width=18).grid(row=1,column=2,columnspan=2,pady=10)
            self.update_comboboxes()
            # ── 传送带控制（STM32 串口协议 115200-8N1）──
            fc=tk.LabelFrame(self.right_frame,text="传送带控制",padx=10,pady=10); fc.pack(fill="x",pady=5)
            self.conveyor_state_var=tk.StringVar(value="传送带状态: 未知")
            self.conveyor_state_label=tk.Label(fc,textvariable=self.conveyor_state_var,font=("Arial",11,"bold"),fg="gray")
            self.conveyor_state_label.pack(pady=(0,4))
            # 速度 & 物体检测显示
            info_f=tk.Frame(fc); info_f.pack(fill="x",pady=(0,6))
            self._conveyor_speed_var=tk.StringVar(value="速度: -- m/s")
            tk.Label(info_f,textvariable=self._conveyor_speed_var,font=("Arial",10),fg="blue").pack(side=tk.LEFT,padx=5)
            self._conveyor_obj_var=tk.StringVar(value="物体: --")
            self._conveyor_obj_label=tk.Label(info_f,textvariable=self._conveyor_obj_var,font=("Arial",10),fg="gray")
            self._conveyor_obj_label.pack(side=tk.LEFT,padx=5)
            cbf=tk.Frame(fc); cbf.pack(fill="x")
            self.conveyor_on_btn=tk.Button(cbf,text="传送带开",command=lambda:self._conveyor_set(True),
                                           font=("Arial",12,"bold"),bg="#4CAF50",fg="white")
            self.conveyor_on_btn.pack(side=tk.LEFT,expand=True,fill="x",padx=(0,4))
            self.conveyor_off_btn=tk.Button(cbf,text="传送带关",command=lambda:self._conveyor_set(False),
                                            font=("Arial",12,"bold"),bg="#f44336",fg="white")
            self.conveyor_off_btn.pack(side=tk.LEFT,expand=True,fill="x",padx=(4,0))
            tk.Label(fc,text=" STM32 串口协议 115200bps | 命令: S/P/F/R",font=("Arial",8),fg="gray").pack(pady=(6,0))
            # ── 自动分拣控制 ──
            fas=tk.LabelFrame(self.right_frame,text=" 自动分拣",padx=10,pady=10); fas.pack(fill="x",pady=5)
            self.auto_sort_state_var=tk.StringVar(value="[IDLE] 未运行")
            tk.Label(fas,textvariable=self.auto_sort_state_var,font=("Arial",10,"bold"),fg="purple").pack(anchor=tk.W,pady=(0,5))
            bf2=tk.Frame(fas); bf2.pack(fill="x")
            self.auto_sort_start_btn=tk.Button(bf2,text=" 开始自动分拣 ",command=self.start_auto_sort,
                                              font=("Arial",12,"bold"),bg="#9C27B0",fg="white")
            self.auto_sort_start_btn.pack(side=tk.LEFT,expand=True,fill="x",padx=(0,4))
            self.auto_sort_stop_btn=tk.Button(bf2,text=" 停止自动分拣 ",command=self.stop_auto_sort,
                                             font=("Arial",12,"bold"),bg="#FF5722",fg="white",state=tk.DISABLED)
            self.auto_sort_stop_btn.pack(side=tk.LEFT,expand=True,fill="x",padx=(4,0))
            tk.Label(fas,text=" 分界线 u=260 (物体左→右) | 苹果/草莓/橙→框1 | 柠檬/桃/梨→框2 | 失败重试1次",
                     font=("Arial",8),fg="gray").pack(pady=(6,0))
            fa=tk.Frame(self.right_frame); fa.pack(pady=10)
            tk.Button(fa,text="🛑 急停 (EMERGENCY STOP)",command=self._on_emergency_stop,
                      font=("Arial",14,"bold"),bg="#FF0000",fg="white",
                      activebackground="#CC0000",activeforeground="white",
                      height=2,width=40).pack(pady=5)
            tk.Button(fa,text="🔄 复位 (急停后回待机位)",command=self._on_reset,
                      font=("Arial",12,"bold"),bg="#4CAF50",fg="white",width=40).pack(pady=5)
            tk.Button(fa,text="返回待机位 (中断当前动作)",command=self.send_observe,
                      font=("Arial",12,"bold"),bg="#FF9800",fg="white",width=40).pack(pady=5)
            tk.Button(fa,text="一键回到待机位并关闭夹爪",command=self.send_reset,font=("Arial",12,"bold"),bg="orange",width=40).pack(pady=5)
            tk.Button(fa,text="退出服务端系统",command=self.send_quit,font=("Arial",12,"bold"),bg="tomato",fg="white",width=40).pack(pady=5)

        def update_comboboxes(self):
            names=list(self.poses.keys())
            for cb in(self.pick_combo,getattr(self,'del_combo',None)):
                if cb: cb['values']=names
            if names:
                if not self.pick_combo.get(): self.pick_combo.current(0)
                if hasattr(self,'del_combo') and not self.del_combo.get(): self.del_combo.current(0)
            else:
                for cb in(self.pick_combo,getattr(self,'del_combo',None)):
                    if cb: cb.set('')

        # ── 相机画面 ──
        def _update_camera_loop(self):
            try:
                latest=None
                while not self.image_queue.empty():
                    try: latest=self.image_queue.get_nowait()
                    except: break
                if latest is not None:
                    self._last_frame_time=time.time()

                    # 延迟导入 PIL，仅在 GUI 主线程中加载
                    from PIL import Image as PImg
                    from PIL import ImageTk

                    img=PImg.open(io.BytesIO(latest))
                    img.thumbnail((self._canvas_w,self._canvas_h),PImg.LANCZOS)

                    # ⚠️ Tcl_Release 防护: 固定池循环复用 PhotoImage, 永不创建/销毁 Tcl 对象。
                    # 背景: 每次 itemconfig(image=新PhotoImage) 替换后, 旧 image 在 Tcl 侧被释放,
                    # 若 Python 侧旧 PhotoImage 再被 GC (pop 或截断) → __del__ → Tcl delete 已释放
                    # 对象 → "Tcl_Release couldn't find reference" → SIGABRT 核心转储。
                    # 方案: 预创建固定数量同尺寸 PhotoImage, 用 paste() 更新内容循环复用,
                    # 对象集合恒定 → GC 永远不触发 __del__。
                    if not hasattr(self, '_photo_pool'):
                        self._photo_pool = []
                        self._photo_pool_idx = 0
                    try:
                        # 固定尺寸 (与创建池时一致), 保证 paste() 不会因尺寸不符报错
                        fixed = img.resize((self._canvas_w, self._canvas_h), PImg.LANCZOS)
                        if len(self._photo_pool) < 8:
                            self._photo_pool.append(ImageTk.PhotoImage(fixed))
                            photo = self._photo_pool[-1]
                        else:
                            photo = self._photo_pool[self._photo_pool_idx]
                            photo.paste(fixed)
                            self._photo_pool_idx = (self._photo_pool_idx + 1) % len(self._photo_pool)
                    except Exception:
                        # paste 异常回退: 保留引用不主动释放, 避免 __del__ 触发 Tcl_Release
                        photo = ImageTk.PhotoImage(img)
                        self._photos.append(photo)

                    self.camera_canvas.itemconfig(self._canvas_img_id,image=photo,state='normal')
                    self.camera_canvas.itemconfig(self._canvas_text_id,state='hidden')

                if time.time()-self._last_frame_time>3.0:
                    self.camera_canvas.itemconfig(self._canvas_img_id,state='hidden')
                    self.camera_canvas.itemconfig(self._canvas_text_id,text="无画面信号",state='normal')
            except: pass
            self.after(33,self._update_camera_loop)

        # ── 检测信息 ──
        def _update_detection_loop(self):
            try:
                raw=None
                while not self.detection_queue.empty():
                    try: raw=self.detection_queue.get_nowait()
                    except: break
                if raw:
                    d=json.loads(raw); self.latest_detection=d
                    # 缓存 header_stamp (浮点秒数), 保留备用 (随动抓取预留)
                    try:
                        hs = d.get('header_stamp')
                        if hs is not None:
                            self.latest_detection_stamp = float(hs)
                    except (TypeError, ValueError):
                        pass
                    # ═════════ 蓝方块模式: 解析 detections 数组, 显示蓝方块专用字段 ═════════
                    if self.detection_mode == 'blue_block':
                        detections = d.get('detections', []) or []
                        if detections:
                            self._update_blue_block_detection_panel(detections[0])
                        else:
                            self._update_blue_block_detection_panel(None)
                        # 蓝方块模式不使用物体列表 (单目标), 清空避免残留
                        self.det_listbox.delete(0, tk.END)
                        self._det_objects = []
                        self.after(200, self._update_detection_loop)
                        return
                    # ═════════ 水果模式: 解析 objects 数组 (原有逻辑) ═════════
                    objects = d.get('objects', []) or []
                    # 默认主物体显示 (顶层字段, 保持向后兼容)
                    if d.get('detected'):
                        self.det_status_var.set(f" 已检测到物体 (共 {len(objects)} 个)")
                        # 自动分拣运行时保持手动按钮禁用
                        if not self.auto_sort_running:
                            self.pick_btn.config(state=tk.NORMAL,bg="#4CAF50")
                            # 两阶段精定位在 eye-to-hand 下无效 (相机固定, 移动机械臂不改变视角), 永久禁用
                            self.pick_graspnet_btn.config(state=tk.NORMAL,bg="#FF9800")
                        self.det_obj.set(f"物体: {d.get('object_name','--')}")
                        self.det_conf.set(f"置信度: {d.get('confidence',0)*100:.2f}%")
                        self.det_meth.set(f"方法: {d.get('method','--')}")
                        bp=d.get('base_position_m',{})
                        if bp and all(k in bp for k in('x','y','z')):
                            self.det_x.set(f"  X: {bp['x']:.3f} m"); self.det_y.set(f"  Y: {bp['y']:.3f} m")
                            self.det_z.set(f"  Z: {bp['z']:.3f} m")
                        else: self.det_x.set("  X: --"); self.det_y.set("  Y: --"); self.det_z.set("  Z: --")
                        # 优先用 D455 估计直径, fallback 预设尺寸
                        dia = d.get('estimated_diameter_m') or d.get('size_m',{}).get('diameter')
                        _dia_src = '估计' if d.get('estimated_diameter_m') else '预设'
                        self.det_dia.set(f"直径: {dia:.3f} m ({_dia_src})" if dia else "直径: --")
                        dist=d.get('distance_to_robot_m')
                        self.det_dist.set(f"距离: {dist:.3f} m" if dist else "距离: --")
                    else:
                        self.det_status_var.set(" 未检测到物体")
                        for v in(self.det_obj,self.det_conf,self.det_meth,self.det_x,self.det_y,self.det_z,self.det_dia,self.det_dist):
                            v.set(v.get().split(":")[0]+": --")
                        self.pick_btn.config(state=tk.DISABLED,bg="gray")
                        self.pick_two_btn.config(state=tk.DISABLED,bg="gray")
                        self.pick_graspnet_btn.config(state=tk.DISABLED,bg="gray")
                        # 水果模式下蓝方块抓取按钮始终禁用
                        self.pick_blue_block_btn.config(state=tk.DISABLED,bg="gray")
                    # ── 更新所有物体列表 (realsense_yolo_node.py 发布的 objects 数组) ──
                    prev_sel = self.det_listbox.curselection()
                    prev_idx = prev_sel[0] if prev_sel else -1
                    self.det_listbox.delete(0,tk.END)
                    for obj in objects:
                        bp = obj.get('base_position_m', {})
                        conf_pct = obj.get('confidence',0)*100
                        if bp:
                            line = f"#{obj.get('index','?')} {obj.get('object_name','?')} {conf_pct:.0f}% B({bp['x']:.2f},{bp['y']:.2f},{bp['z']:.2f})"
                        else:
                            line = f"#{obj.get('index','?')} {obj.get('object_name','?')} {conf_pct:.0f}% (无基座坐标)"
                        self.det_listbox.insert(tk.END, line)
                    self._det_objects = objects
                    # 恢复选中: 优先保持用户选择, 否则选中主物体 (第 0 项)
                    if objects:
                        restore_idx = prev_idx if (0 <= prev_idx < len(objects)) else 0
                        self.det_listbox.selection_set(restore_idx)
                        self._apply_selected_object(restore_idx)
            except: pass
            self.after(200,self._update_detection_loop)

        def _apply_selected_object(self, idx):
            """把列表中第 idx 个物体的字段提升到 self.latest_detection 顶层,
            这样 _pick_from_detection / _pick_from_detection_two_stage 会用选中物体而不是默认主物体。"""
            if not hasattr(self, '_det_objects') or idx >= len(self._det_objects):
                return
            obj = self._det_objects[idx]
            # 提升选中物体字段到顶层 (覆盖主物体)
            self.latest_detection['object_name'] = obj.get('object_name','--')
            self.latest_detection['confidence'] = obj.get('confidence',0)
            self.latest_detection['bbox_pixel'] = obj.get('bbox_pixel',{})
            self.latest_detection['camera_position_m'] = obj.get('camera_position_m',{})
            self.latest_detection['end_effector_position_m'] = obj.get('end_effector_position_m',{})
            self.latest_detection['monocular_depth_m'] = obj.get('monocular_depth_m',0)
            self.latest_detection['size_m'] = obj.get('size_m',{})
            # 同步 D455 估计直径 (供 _pick_from_detection / two-stage 优先使用)
            self.latest_detection['estimated_diameter_m'] = obj.get('estimated_diameter_m')
            self.latest_detection['volume_m3'] = obj.get('volume_m3',0)
            # 同步 header_stamp (顶层主物体的 stamp 与 objects 数组中选中物体的 stamp 一致)
            if 'header_stamp' in obj:
                self.latest_detection['header_stamp'] = obj['header_stamp']
            if 'base_position_m' in obj:
                self.latest_detection['base_position_m'] = obj['base_position_m']
            if 'distance_to_robot_m' in obj:
                self.latest_detection['distance_to_robot_m'] = obj['distance_to_robot_m']
            # 同步刷新主物体显示面板
            self.det_obj.set(f"物体: {obj.get('object_name','--')}")
            self.det_conf.set(f"置信度: {obj.get('confidence',0)*100:.2f}%")
            bp = obj.get('base_position_m', {})
            if bp and all(k in bp for k in ('x','y','z')):
                self.det_x.set(f"  X: {bp['x']:.3f} m"); self.det_y.set(f"  Y: {bp['y']:.3f} m")
                self.det_z.set(f"  Z: {bp['z']:.3f} m")
            dia = obj.get('estimated_diameter_m') or obj.get('size_m',{}).get('diameter')
            _dia_src = '估计' if obj.get('estimated_diameter_m') else '预设'
            self.det_dia.set(f"直径: {dia:.3f} m ({_dia_src})" if dia else "直径: --")
            dist = obj.get('distance_to_robot_m')
            self.det_dist.set(f"距离: {dist:.3f} m" if dist else "距离: --")

        def _on_det_listbox_select(self, event):
            try:
                sel = self.det_listbox.curselection()
                if sel:
                    self._apply_selected_object(sel[0])
            except: pass

        # ── 单次检测直接夹取 ──
        def _pick_from_detection(self):
            d=self.latest_detection
            if not d: return messagebox.showwarning("无数据","尚未收到检测信息。")
            if not d.get('detected'): return messagebox.showwarning("未检测到","当前未发现物体。")
            bp=d.get('base_position_m',{})
            if not bp or not all(k in bp for k in('x','y','z')): return messagebox.showerror("坐标缺失","缺少 base_position_m。")
            bin_num=self.bin_var.get()
            try: r=float(self.default_grasp_roll.get()); p=float(self.default_grasp_pitch.get()); y=float(self.default_grasp_yaw.get())
            except ValueError: return messagebox.showerror("姿态错误","R/P/Y 必须是数字。")
            qx,qy,qz,qw=euler_to_quaternion(r,p,y)
            pp={'x':round(float(bp['x']),3),'y':round(float(bp['y']),3),'z':round(float(bp['z']),3),
                'qx':round(qx,6),'qy':round(qy,6),'qz':round(qz,6),'qw':round(qw,6)}
            # 优先用 D455 估计直径, fallback 预设尺寸
            dia=d.get('estimated_diameter_m') or d.get('size_m',{}).get('diameter')
            if dia: dia=round(float(dia),3)
            dia_str=f"物体直径: {dia} m\n\n" if dia else ""
            txt=(f"检测坐标夹取确认\n\nPick: {d.get('object_name','物体')} (detected)\n"
                 f"  坐标: ({pp['x']:.3f}, {pp['y']:.3f}, {pp['z']:.3f})\n  姿态: R={r}  P={p}  Y={y} \n\n"
                 f"Place: 料框{bin_num} (关节空间预设点位)\n\n"
                 f"{dia_str}确定发送？")
            if not messagebox.askyesno("确认",txt): return
            cmd={"cmd":"sort","pick":pp,"bin":bin_num,"pick_name":f"{d.get('object_name','物体')} (detected)","place_name":f"料框{bin_num}"}
            if dia: cmd["object_diameter_m"]=float(dia)
            if self.current_status=='busy':
                if messagebox.askyesno("忙碌","加入排队队列？"):
                    self.task_queue.append(cmd); self.refresh_queue_listbox()
                    messagebox.showinfo("已加入","任务已加入排队队列。")
            else: self.last_dispatch_time=time.time(); self.cmd_queue.put(cmd); messagebox.showinfo("已发送",f"夹取指令已发送！\n检测物体 -> 【料框{bin_num}】")

        # ── 两阶段精定位夹取 (D455 深度 + PCA 短轴对齐) ──
        def _pick_from_detection_two_stage(self):
            d=self.latest_detection
            if not d: return messagebox.showwarning("无数据","尚未收到检测信息。")
            if not d.get('detected'): return messagebox.showwarning("未检测到","当前未发现物体。")
            bp=d.get('base_position_m',{})
            if not bp or not all(k in bp for k in('x','y','z')): return messagebox.showerror("坐标缺失","缺少 base_position_m。")
            bin_num=self.bin_var.get()
            try: r=float(self.default_grasp_roll.get()); p=float(self.default_grasp_pitch.get()); y=float(self.default_grasp_yaw.get())
            except ValueError: return messagebox.showerror("姿态错误","R/P/Y 必须是数字。")
            qx,qy,qz,qw=euler_to_quaternion(r,p,y)
            pp={'x':round(float(bp['x']),3),'y':round(float(bp['y']),3),'z':round(float(bp['z']),3),
                'qx':round(qx,6),'qy':round(qy,6),'qz':round(qz,6),'qw':round(qw,6)}
            # 优先用 D455 估计直径, fallback 预设尺寸
            dia=d.get('estimated_diameter_m') or d.get('size_m',{}).get('diameter')
            if dia: dia=round(float(dia),3)
            dia_str=f"物体直径: {dia} m\n" if dia else ""
            txt=(f" 两阶段精定位夹取确认\n\n待机位检测坐标:\n  ({pp['x']:.3f}, {pp['y']:.3f}, {pp['z']:.3f})\n\n"
                 f"-> 机械臂移动到物体正上方\n-> YOLO 近距离二次检测 (D455 深度)\n-> PCA 短轴对齐生成夹爪朝向\n-> 以二次结果为准直接抓取\n\n"
                 f"Place: 料框{bin_num} (关节空间预设点位)\n"
                 f"{dia_str}\n确定发送？")
            if not messagebox.askyesno("确认两阶段精定位夹取",txt): return
            cmd={"cmd":"sort_verify","pick":pp,"bin":bin_num,"pick_name":f"{d.get('object_name','物体')} (two-stage)","place_name":f"料框{bin_num}"}
            if dia: cmd["object_diameter_m"]=float(dia)
            sent_msg=f"两阶段精定位指令已发送！\n检测物体 -> 【料框{bin_num}】"
            queue_msg="两阶段精定位任务已加入排队队列。"
            if self.current_status=='busy':
                if messagebox.askyesno("忙碌","加入排队队列？"):
                    self.task_queue.append(cmd); self.refresh_queue_listbox()
                    messagebox.showinfo("已加入",queue_msg)
            else: self.last_dispatch_time=time.time(); self.cmd_queue.put(cmd); messagebox.showinfo("已发送",sent_msg)

        # ── GraspNet 6DoF 主导抓取 (待机位深度图 → GraspNet 位姿) ──
        def _pick_from_detection_graspnet(self):
            d=self.latest_detection
            if not d: return messagebox.showwarning("无数据","尚未收到检测信息。")
            if not d.get('detected'): return messagebox.showwarning("未检测到","当前未发现物体。")
            bp=d.get('base_position_m',{})
            if not bp or not all(k in bp for k in('x','y','z')): return messagebox.showerror("坐标缺失","缺少 base_position_m。")
            bin_num=self.bin_var.get()
            # pick 坐标仅作为 GraspNet 邻近过滤提示 (锁定目标物体), 最终位姿由 GraspNet 生成
            pp={'x':round(float(bp['x']),3),'y':round(float(bp['y']),3),'z':round(float(bp['z']),3)}
            # 直径优先用 D455 估计 (GraspNet 也会返回宽度, action 端以 GraspNet 宽度为准)
            dia=d.get('estimated_diameter_m') or d.get('size_m',{}).get('diameter')
            if dia: dia=round(float(dia),3)
            dia_str=f"物体直径: {dia} m\n" if dia else ""
            txt=(f" GraspNet 6DoF 抓取确认\n\n待机位检测坐标 (邻近提示):\n  ({pp['x']:.3f}, {pp['y']:.3f}, {pp['z']:.3f})\n\n"
                 f"-> 机械臂回到待机位\n-> GraspNet 对 D455 深度图推理 (6DoF 位姿)\n"
                 f"-> 短边对齐 + 中心抓取 + approach 朝下安全检查\n-> 直接下降抓取\n\n"
                 f"Place: 料框{bin_num} (关节空间预设点位)\n"
                 f"{dia_str}\n⚠️ 需已启动 graspnet_service_node.py\n\n确定发送？")
            if not messagebox.askyesno("确认 GraspNet 抓取",txt): return
            cmd={"cmd":"sort_graspnet","pick":pp,"bin":bin_num,
                 "pick_name":f"{d.get('object_name','物体')} (graspnet)","place_name":f"料框{bin_num}"}
            if dia: cmd["object_diameter_m"]=float(dia)
            sent_msg=f"GraspNet 抓取指令已发送！\n检测物体 -> 【料框{bin_num}】"
            queue_msg="GraspNet 抓取任务已加入排队队列。"
            if self.current_status=='busy':
                if messagebox.askyesno("忙碌","加入排队队列？"):
                    self.task_queue.append(cmd); self.refresh_queue_listbox()
                    messagebox.showinfo("已加入",queue_msg)
            else: self.last_dispatch_time=time.time(); self.cmd_queue.put(cmd); messagebox.showinfo("已发送",sent_msg)

        # ═══════════════════════ 蓝方块模式 (Task 2.2) ═══════════════════════

        def _switch_to_blue_block_mode(self):
            """切换到蓝方块分拣模式: 停止水果 YOLO 节点, 启动 blue_block_detector.py。
            通过 ROS worker 子进程执行 kill/launch (主进程禁止 fork)。
            """
            if self.detection_mode == 'blue_block' or self._mode_switching:
                return
            if self.auto_sort_running:
                return messagebox.showwarning("自动分拣运行中", "请先停止自动分拣再切换模式。")
            if not messagebox.askyesno("确认切换模式",
                "切换到蓝方块分拣模式？\n\n"
                "将停止水果 YOLO 节点 (realsense_yolo_node.py),\n"
                "启动蓝方块检测节点 (blue_block_detector.py)。\n"
                "切换约需 5-30s, 期间检测信息会暂时清空。"):
                return
            self._mode_switching = True
            self._mode_switch_deadline = time.time() + 35.0  # 超时上限 35s
            self.detection_mode = 'blue_block'
            # 禁用两个模式按钮防重复点击
            self.fruit_mode_btn.config(state=tk.DISABLED, relief=tk.RAISED)
            self.blue_block_mode_btn.config(state=tk.DISABLED, relief=tk.RAISED)
            self.mode_status_var.set("当前模式: 切换中 (停止 YOLO → 启动蓝方块)...")
            # 禁用所有水果模式抓取按钮, 蓝方块抓取按钮保持禁用直到检测到物块
            self.pick_btn.config(state=tk.DISABLED, bg="gray")
            self.pick_two_btn.config(state=tk.DISABLED, bg="gray")
            self.pick_graspnet_btn.config(state=tk.DISABLED, bg="gray")
            self.pick_blue_block_btn.config(state=tk.DISABLED, bg="gray")
            # 清空检测信息缓存 (旧格式数据不属于蓝方块模式)
            self.latest_detection = None
            self._det_objects = []
            self.det_listbox.delete(0, tk.END)
            # 重置检测面板为等待状态
            self.det_status_var.set(" 等待蓝方块检测节点上线...")
            for v in (self.det_obj, self.det_conf, self.det_meth, self.det_x, self.det_y, self.det_z, self.det_dia, self.det_dist):
                v.set(v.get().split(":")[0] + ": --")
            # 发送切换命令到 ROS worker 子进程 (在其中执行 kill/launch, fork 安全)
            self.cmd_queue.put({"cmd": "switch_mode", "mode": "blue_block"})
            self.after(500, self._poll_mode_switch_done)

        def _switch_to_fruit_mode(self):
            """切换回水果分拣模式: 停止 blue_block_detector.py, 启动 realsense_yolo_node.py。
            通过 ROS worker 子进程执行 kill/launch (主进程禁止 fork)。
            """
            if self.detection_mode == 'fruit' or self._mode_switching:
                return
            if self.auto_sort_running:
                return messagebox.showwarning("自动分拣运行中", "请先停止自动分拣再切换模式。")
            if not messagebox.askyesno("确认切换模式",
                "切换回水果分拣模式？\n\n"
                "将停止蓝方块检测节点 (blue_block_detector.py),\n"
                "启动水果 YOLO 节点 (realsense_yolo_node.py)。\n"
                "切换约需 5-30s, 期间检测信息会暂时清空。"):
                return
            self._mode_switching = True
            self._mode_switch_deadline = time.time() + 65.0  # YOLO 加载较慢, 超时 65s
            self.detection_mode = 'fruit'
            self.fruit_mode_btn.config(state=tk.DISABLED, relief=tk.RAISED)
            self.blue_block_mode_btn.config(state=tk.DISABLED, relief=tk.RAISED)
            self.mode_status_var.set("当前模式: 切换中 (停止蓝方块 → 启动 YOLO)...")
            # 禁用所有抓取按钮
            self.pick_btn.config(state=tk.DISABLED, bg="gray")
            self.pick_two_btn.config(state=tk.DISABLED, bg="gray")
            self.pick_graspnet_btn.config(state=tk.DISABLED, bg="gray")
            self.pick_blue_block_btn.config(state=tk.DISABLED, bg="gray")
            # 清空检测信息缓存
            self.latest_detection = None
            self._det_objects = []
            self.det_listbox.delete(0, tk.END)
            self.det_status_var.set(" 等待水果 YOLO 节点上线...")
            for v in (self.det_obj, self.det_conf, self.det_meth, self.det_x, self.det_y, self.det_z, self.det_dia, self.det_dist):
                v.set(v.get().split(":")[0] + ": --")
            # 发送切换命令到 ROS worker 子进程
            self.cmd_queue.put({"cmd": "switch_mode", "mode": "fruit"})
            self.after(500, self._poll_mode_switch_done)

        def _poll_mode_switch_done(self):
            """轮询模式切换是否完成 (通过检测 /detection_info 是否有新数据)。
            切换完成 = 收到新模式的 /detection_info 消息; 超时 = 强制恢复按钮。
            """
            if not self._mode_switching:
                return
            # 检测是否有新数据到达 (latest_detection 被清空后重新填充 = 新节点上线)
            if self.latest_detection is not None:
                self._finish_mode_switch()
                return
            # 超时检查
            if time.time() > self._mode_switch_deadline:
                _log(f"模式切换超时 (mode={self.detection_mode}), 强制恢复按钮")
                self._finish_mode_switch()
                return
            self.after(500, self._poll_mode_switch_done)

        def _finish_mode_switch(self):
            """模式切换完成 (或超时), 恢复按钮高亮状态。"""
            self._mode_switching = False
            self._mode_switch_deadline = 0
            if self.detection_mode == 'blue_block':
                # 蓝方块模式: 蓝方块按钮高亮, 水果按钮灰显
                self.fruit_mode_btn.config(state=tk.NORMAL, relief=tk.RAISED)
                self.blue_block_mode_btn.config(state=tk.NORMAL, relief=tk.SUNKEN)
                self.mode_status_var.set("当前模式: 蓝方块分拣 (blue_block_detector)")
                # 水果模式抓取按钮保持禁用 (不在水果模式)
                self.pick_btn.config(state=tk.DISABLED, bg="gray")
                self.pick_two_btn.config(state=tk.DISABLED, bg="gray")
                self.pick_graspnet_btn.config(state=tk.DISABLED, bg="gray")
                # 蓝方块抓取按钮保持禁用, 等检测到物块后由 _update_detection_loop 启用
                self.pick_blue_block_btn.config(state=tk.DISABLED, bg="#33CAE8")
            else:
                # 水果模式: 水果按钮高亮, 蓝方块按钮灰显
                self.fruit_mode_btn.config(state=tk.NORMAL, relief=tk.SUNKEN)
                self.blue_block_mode_btn.config(state=tk.NORMAL, relief=tk.RAISED)
                self.mode_status_var.set("当前模式: 水果分拣 (realsense_yolo_node)")
                # 蓝方块抓取按钮禁用
                self.pick_blue_block_btn.config(state=tk.DISABLED, bg="gray")
                # 水果模式抓取按钮保持禁用, 等检测到物体后由 _update_detection_loop 启用
                self.pick_btn.config(state=tk.DISABLED, bg="#4CAF50")
                self.pick_two_btn.config(state=tk.DISABLED, bg="gray")
                self.pick_graspnet_btn.config(state=tk.DISABLED, bg="#FF9800")

        def _pick_blue_block(self):
            """蓝方块抓取按钮回调: 从 latest_detection 提取蓝方块字段, 发送 sort_blue_block 命令。"""
            d = self.latest_detection
            if not d:
                return messagebox.showwarning("无数据", "尚未收到蓝方块检测信息。")
            # 蓝方块检测节点发布格式: {"detections": [{...}]}
            detections = d.get('detections', [])
            if not detections:
                return messagebox.showwarning("未检测到", "当前未发现蓝方块。")
            det = detections[0]
            bc = det.get('base_coords', {})
            if not bc or not all(k in bc for k in ('x', 'y', 'z')):
                return messagebox.showerror("坐标缺失", "缺少 base_coords 字段, 无法发送抓取命令。")
            bin_num = self.bin_var.get()
            pp = {'x': round(float(bc['x']), 3),
                  'y': round(float(bc['y']), 3),
                  'z': round(float(bc['z']), 3)}
            # 蓝方块专用字段 (从 /detection_info 缓存提取)
            surface_z = det.get('surface_z_m')
            block_w = det.get('block_width_m')
            block_l = det.get('block_length_m')
            block_h = det.get('block_height_m')
            rot = det.get('block_rotation_deg')
            depth_method = det.get('depth_method', '?')
            # 确认对话框
            size_str = ""
            if all(v is not None for v in (block_l, block_w, block_h)):
                size_str = (f"  尺寸(长×宽×高): {block_l*1000:.0f}×{block_w*1000:.0f}×{block_h*1000:.0f} mm\n")
            surf_str = f"  顶面高度: {surface_z:.3f} m [{depth_method}]\n" if surface_z is not None else ""
            rot_str = f"  旋转角: {rot:.1f}°\n" if rot is not None else ""
            txt = (f"蓝方块抓取确认\n\nPick: blue_block (detected)\n"
                   f"  坐标: ({pp['x']:.3f}, {pp['y']:.3f}, {pp['z']:.3f})\n"
                   f"{surf_str}{size_str}{rot_str}\n"
                   f"Place: 料框{bin_num} (关节空间预设点位)\n\n确定发送？")
            if not messagebox.askyesno("确认蓝方块抓取", txt):
                return
            cmd = {
                "cmd": "sort_blue_block",
                "pick": pp,
                "surface_z_m": surface_z,
                "block_width_m": block_w,
                "block_length_m": block_l,
                "block_height_m": block_h,
                "block_rotation_deg": rot,
                "bin": bin_num,
                "pick_name": "blue_block (detected)",
                "place_name": f"料框{bin_num}",
            }
            sent_msg = f"蓝方块抓取指令已发送！\nblue_block -> 【料框{bin_num}】"
            queue_msg = "蓝方块抓取任务已加入排队队列。"
            if self.current_status == 'busy':
                if messagebox.askyesno("忙碌", "加入排队队列？"):
                    self.task_queue.append(cmd); self.refresh_queue_listbox()
                    messagebox.showinfo("已加入", queue_msg)
            else:
                self.last_dispatch_time = time.time()
                self.cmd_queue.put(cmd)
                messagebox.showinfo("已发送", sent_msg)

        def _dispatch_blue_block_sort_cmd(self, det, retry=False):
            """构造蓝方块分拣命令并发送 (供自动分拣/外部调用使用)。
            det: /detection_info 中 detections[0] 的 dict。
            """
            bc = det.get('base_coords', {})
            if not bc or not all(k in bc for k in ('x', 'y', 'z')):
                self._auto_sort_log("蓝方块 base_coords 缺失, 跳过")
                return
            bin_num = self.bin_var.get()
            pp = {'x': round(float(bc['x']), 3),
                  'y': round(float(bc['y']), 3),
                  'z': round(float(bc['z']), 3)}
            cmd = {
                "cmd": "sort_blue_block",
                "pick": pp,
                "surface_z_m": det.get('surface_z_m'),
                "block_width_m": det.get('block_width_m'),
                "block_length_m": det.get('block_length_m'),
                "block_height_m": det.get('block_height_m'),
                "block_rotation_deg": det.get('block_rotation_deg'),
                "bin": bin_num,
                "pick_name": f"blue_block (auto-sort{'-retry' if retry else ''})",
                "place_name": f"料框{bin_num}",
            }
            self.last_dispatch_time = time.time()
            self.cmd_queue.put(cmd)
            self._auto_sort_log(f"蓝方块分拣: →料框{bin_num} {'[重试]' if retry else ''}")

        def _update_blue_block_detection_panel(self, det):
            """更新蓝方块检测信息面板 (蓝方块模式专用字段)。
            det: /detection_info 中 detections[0] 的 dict, 或 None (无检测)。
            显示字段: 物块中心 XYZ (base) / 顶面高度 (标注 RANSAC/分位数) /
                     尺寸 (长×宽×高) / 旋转角。
            """
            if det is None:
                self.det_status_var.set(" 未检测到蓝方块")
                for v in (self.det_obj, self.det_conf, self.det_meth,
                          self.det_x, self.det_y, self.det_z,
                          self.det_dia, self.det_dist):
                    v.set(v.get().split(":")[0] + ": --")
                # 禁用蓝方块抓取按钮
                if not self._mode_switching and self.detection_mode == 'blue_block':
                    self.pick_blue_block_btn.config(state=tk.DISABLED, bg="gray")
                return
            bc = det.get('base_coords', {})
            surface_z = det.get('surface_z_m')
            depth_method = det.get('depth_method', '?')
            block_w = det.get('block_width_m')
            block_l = det.get('block_length_m')
            block_h = det.get('block_height_m')
            rot = det.get('block_rotation_deg')
            self.det_status_var.set(" 已检测到蓝方块")
            self.det_obj.set(f"物体: {det.get('name', 'blue_block')}")
            self.det_conf.set(f"置信度: {det.get('confidence', 0)*100:.2f}%")
            # 方法字段: 标注 RANSAC / 分位数方法
            if depth_method == 'ransac':
                self.det_meth.set("方法: RANSAC 平面拟合")
            elif depth_method == 'percentile_25':
                self.det_meth.set("方法: 25% 分位数 (RANSAC 失败回退)")
            else:
                self.det_meth.set(f"方法: {depth_method}")
            # 物块中心 XYZ (base 坐标系)
            if bc and all(k in bc for k in ('x', 'y', 'z')):
                self.det_x.set(f"  X: {bc['x']:.3f} m")
                self.det_y.set(f"  Y: {bc['y']:.3f} m")
                self.det_z.set(f"  Z: {bc['z']:.3f} m")
            else:
                self.det_x.set("  X: --"); self.det_y.set("  Y: --"); self.det_z.set("  Z: --")
            # 直径行 → 改为显示顶面高度 (标注 RANSAC/分位数方法)
            if surface_z is not None:
                self.det_dia.set(f"顶面高度 (surface_z): {surface_z:.3f} m [{depth_method}]")
            else:
                self.det_dia.set("顶面高度: --")
            # 距离行 → 改为显示尺寸 (长×宽×高) + 旋转角
            if all(v is not None for v in (block_l, block_w, block_h)):
                rot_str = f"  旋转: {rot:.1f}°" if rot is not None else ""
                self.det_dist.set(f"尺寸(长×宽×高): {block_l*1000:.0f}×{block_w*1000:.0f}×{block_h*1000:.0f} mm{rot_str}")
            elif rot is not None:
                self.det_dist.set(f"旋转角: {rot:.1f}°")
            else:
                self.det_dist.set("尺寸: --")
            # 启用蓝方块抓取按钮 (仅在非切换状态且 base_coords 有效时)
            if not self._mode_switching and bc and all(k in bc for k in ('x', 'y', 'z')):
                if not self.auto_sort_running:
                    self.pick_blue_block_btn.config(state=tk.NORMAL, bg="#33CAE8")

        # ── 原有功能 ──
        def fetch_current_pose(self):
            pose=None
            while not self.pose_queue.empty():
                try: pose=self.pose_queue.get_nowait()
                except: break
            if pose:
                self.current_fetched_pose=pose
                self.pose_text_var.set(f"已读取: x:{pose['x']:.2f}, y:{pose['y']:.2f}, z:{pose['z']:.2f}")
                messagebox.showinfo("成功","成功读取了机械臂当前法兰盘坐标！")
            else: messagebox.showwarning("警告","TF 位姿尚未准备好。")

        def save_current_pose(self):
            n=self.name_entry.get().strip()
            if not n: return messagebox.showerror("错误","请输入坐标名称！")
            if not hasattr(self,'current_fetched_pose'): return messagebox.showerror("错误","请先读取机械臂位置！")
            self.poses[n]={'x':float(self.current_fetched_pose['x']),'y':float(self.current_fetched_pose['y']),'z':float(self.current_fetched_pose['z'])}
            self.save_poses(); messagebox.showinfo("成功",f"坐标 '{n}' 已保存！")

        def delete_pose(self):
            n=self.del_combo.get()
            if not n: return messagebox.showerror("错误","未选择坐标！")
            if messagebox.askyesno("确认",f"永久删除 '{n}'？"): del self.poses[n]; self.save_poses(); messagebox.showinfo("成功","已删除。")

        def _on_emergency_stop(self):
            """急停按钮回调: 无条件中断机械臂所有运动, 停在原地."""
            self.cmd_queue.put({"cmd": "emergency_stop", "req": "estop"})
            self.clear_queue()

        def _on_reset(self):
            """复位按钮回调: 急停后清除锁定, 回待机位."""
            self.cmd_queue.put({"cmd": "emergency_stop", "req": "reset"})

        def send_observe(self):
            # 使用急停机制的返回待机位: 无条件中断并回待机位
            self.cmd_queue.put({"cmd": "standby", "req": "standby"})
            self.clear_queue()

        # ── 曝光控制 (通过 cmd_queue → ROS 子进程 → /camera/exposure_ctrl 话题) ──
        def _exposure_toggle(self):
            """切换自动/手动曝光。命令经 /camera/exposure_ctrl 话题由 realsense_yolo_node.py 处理。"""
            self._exposure_auto = not self._exposure_auto
            if self._exposure_auto:
                self.cmd_queue.put({"cmd":"exposure","auto":True})
                self.exposure_var.set("当前: 自动")
                self.exp_inc_btn.config(state=tk.DISABLED)
                self.exp_dec_btn.config(state=tk.DISABLED)
            else:
                self.cmd_queue.put({"cmd":"exposure","auto":False,"value":int(self._exposure_value)})
                self.exposure_var.set(f"当前: {int(self._exposure_value)}")
                self.exp_inc_btn.config(state=tk.NORMAL)
                self.exp_dec_btn.config(state=tk.NORMAL)

        def _exposure_adjust(self, direction):
            """手动模式下 +/- 调节曝光值。direction: +1 增加, -1 减少。"""
            if self._exposure_auto:
                return
            step = 100 if self._exposure_value >= 100 else 10
            self._exposure_value = max(1, min(10000, self._exposure_value + direction * step))
            self.cmd_queue.put({"cmd":"exposure","auto":False,"value":int(self._exposure_value)})
            self.exposure_var.set(f"当前: {int(self._exposure_value)}")

        def _on_force_change(self, val):
            """力矩滑块变化回调: 发送 set_gripper_force 命令到 action 节点."""
            force = round(float(val), 2)
            self.gripper_force_label.set(f"当前: {force:.2f}N")
            self.cmd_queue.put({"cmd": "set_gripper_force", "force": force})

        def _set_force(self, val):
            """快速预设按钮: 跳到指定力矩值."""
            self.gripper_force_var.set(val)
            self._on_force_change(val)

        def _on_grasp_result(self, d):
            """处理 action 节点发来的抓取结果 JSON."""
            success = d.get("success", False)
            width = d.get("width", 0)
            force = d.get("force", 0)
            reason = d.get("reason", "")
            if success:
                self.grasp_status_var.set("抓取状态: ✅ 成功")
                self.grasp_status_label.configure(fg="green")
            else:
                self.grasp_status_var.set(f"抓取状态: ❌ 失败 ({reason})")
                self.grasp_status_label.configure(fg="red")
            self.grasp_detail_var.set(f"宽度={width*1000:.1f}mm  力矩={force:.2f}N")

        def send_reset(self):
            if self.current_status=='busy': return messagebox.showwarning("忙碌","机械臂正在执行动作。")
            if messagebox.askyesno("确认","机械臂将回到待机位并关闭夹爪。确定？"):
                self.cmd_queue.put({"cmd":"reset"}); self.clear_queue(); messagebox.showinfo("已发送","复位指令发送成功！")

        def add_to_queue(self):
            pn=self.pick_combo.get()
            if not pn: return messagebox.showerror("错误","请选择夹取点！")
            bin_num=self.bin_var.get()
            self.task_queue.append({"cmd":"sort","pick":self.poses[pn],"bin":bin_num,"pick_name":pn,"place_name":f"料框{bin_num}"})
            self.refresh_queue_listbox()

        def remove_from_queue(self):
            sel=self.qlist.curselection()
            if not sel: return messagebox.showwarning("提示","请先选中任务！")
            del self.task_queue[sel[0]]; self.refresh_queue_listbox()

        def clear_queue(self):
            self.task_queue.clear(); self.refresh_queue_listbox()
            self.queue_running=False; self.qrun.config(text=" 开始执行排队任务",bg="lightgreen")

        def toggle_queue_execution(self):
            if not self.queue_running:
                if not self.task_queue: return messagebox.showwarning("提示","排队队列为空！")
                self.queue_running=True; self.qrun.config(text="  暂停排队执行",bg="yellow")
                self.last_dispatch_time=time.time()-2.0
            else: self.queue_running=False; self.qrun.config(text=" 开始执行排队任务",bg="lightgreen")

        def refresh_queue_listbox(self):
            self.qlist.delete(0,tk.END)
            for i,t in enumerate(self.task_queue): self.qlist.insert(tk.END,f"{i+1}. {t['pick_name']} -> {t['place_name']}")

        def send_sort(self):
            if self.current_status=='busy': return messagebox.showwarning("忙碌","机械臂正在执行动作。")
            pn=self.pick_combo.get()
            if not pn: return messagebox.showerror("错误","请选择夹取点！")
            bin_num=self.bin_var.get()
            self.last_dispatch_time=time.time()
            self.cmd_queue.put({"cmd":"sort","pick":self.poses[pn],"bin":bin_num,"pick_name":pn,"place_name":f"料框{bin_num}"})
            messagebox.showinfo("已发送",f"分拣指令已发送！\n【{pn}】->【料框{bin_num}】")

        def send_quit(self):
            if messagebox.askyesno("确认","让服务端回到待机位、关闭夹爪并结束程序。确定？"):
                self.cmd_queue.put({"cmd":"quit"}); self.after(500,self.destroy)

        # ── 传送带控制 (STM32 串口协议 115200-8N1) ──
        # 命令: 'S'=启动, 'P'=停止, 'F'=正转, 'R'=反转
        # 状态帧: V:速度,O:物体检测,M1:电机,M2:方向 (每 100ms)
        # 命令委托给 ROS worker 进程执行 (避免主进程 fork 与 Tcl 冲突)
        def _conveyor_set(self, run):
            """设置传送带运行/停止。"""
            if self._conveyor_busy:
                return
            self._conveyor_busy=True
            self.conveyor_on_btn.config(state=tk.DISABLED)
            self.conveyor_off_btn.config(state=tk.DISABLED)
            self.conveyor_state_var.set("传送带状态: 切换中...")
            self.conveyor_state_label.config(fg="orange")
            # STM32: run=True=启动(S), run=False=停止(P)
            relay_action='on' if run else 'off'
            try:
                self._conveyor_run=run
                self._conveyor_start_time=time.time()
                # 委托给 ROS worker 进程执行 (避免主进程 fork 与 Tcl 冲突)
                self.cmd_queue.put({"cmd":"conveyor","relay_action":relay_action})
                self.after(50, self._conveyor_poll)
            except Exception as e:
                self._conveyor_done(run, False, str(e))

        def _conveyor_poll(self):
            """主线程轮询 conveyor_queue 是否有命令结果 (cmd_result)。不阻塞 GUI。"""
            try:
                while True:
                    try:
                        result = self.conveyor_queue.get_nowait()
                    except queue.Empty:
                        result = None
                        break
                    except Exception:
                        # worker 进程崩溃时队列可能抛 BrokenPipe 等, 视为无结果
                        result = None
                        break
                    # 只处理 cmd_result 类型; 其他类型跳过 (理论不应出现, status 走独立队列)
                    if result.get('type') == 'cmd_result':
                        break
                if result and result.get('type') == 'cmd_result':
                    run = self._conveyor_run
                    self._conveyor_run = None
                    self._conveyor_done(run, result.get('ok', False), result.get('err', ''))
                    return
            except Exception:
                pass
            # 超时检查 (8秒, STM32 应答通常 <1s)
            if self._conveyor_run is not None and time.time() - self._conveyor_start_time > 8.0:
                run = self._conveyor_run
                self._conveyor_run = None
                self._conveyor_done(run, False, "STM32 通信超时 (ROS worker 无响应)")
                return
            self.after(50, self._conveyor_poll)   # 还没结果, 继续等

        def _conveyor_done(self, run, ok, err):
            self._conveyor_busy=False
            # 自动分拣运行时保持手动按钮禁用
            if not self.auto_sort_running:
                self.conveyor_on_btn.config(state=tk.NORMAL)
                self.conveyor_off_btn.config(state=tk.NORMAL)
            if ok:
                self.conveyor_state=run
                if run:
                    self.conveyor_state_var.set("传送带状态: 运行中 🟢")
                    self.conveyor_state_label.config(fg="green")
                else:
                    self.conveyor_state_var.set("传送带状态: 已停止 🔴")
                    self.conveyor_state_label.config(fg="red")
            else:
                self.conveyor_state=None
                self.conveyor_state_var.set("传送带状态: 切换失败")
                self.conveyor_state_label.config(fg="gray")
                # ⚠️ 不弹模态 messagebox: 模态框嵌套事件循环会让 _update_camera_loop
                # 等 after 回调继续跑, 与 Tcl 对象操作竞争 (曾导致 Tcl_Release 崩溃)。
                # 改用非阻塞状态栏 + 日志提示。
                msg = f"传送带切换失败: {err}"
                try:
                    self._auto_sort_log(msg)
                except Exception:
                    pass
                try:
                    self.status_var.set("当前服务器状态: " + msg)
                except Exception:
                    pass

        # ── 传送带状态轮询 (速度、物体检测) ──
        # 由 ROS worker 后台线程每 250ms 推送一次状态到 conveyor_status_queue
        def _conveyor_status_poll(self):
            """轮询 conveyor_status_queue 中的 status 消息，更新速度/物体检测显示。"""
            updated = False
            while True:
                try:
                    result = self.conveyor_status_queue.get_nowait()
                except queue.Empty:
                    break
                except Exception:
                    # worker 进程崩溃时队列可能抛 BrokenPipe 等, 停止本轮轮询
                    break
                try:
                    if result.get('type') == 'status':
                        speed = result.get('speed', 0.0)
                        obj = result.get('object_detected', False)
                        motor = result.get('motor_running', False)
                        self._conveyor_speed_var.set(f"速度: {speed:.4f} m/s")
                        obj_text = "有物体" if obj else "无物体"
                        self._conveyor_obj_var.set(f"物体: {obj_text}")
                        self._conveyor_obj_label.config(fg="green" if obj else "gray")
                        updated = True
                    elif result.get('type') == 'cmd_result':
                        # cmd_result 由 _conveyor_poll 处理，这里不会出现
                        pass
                except Exception:
                    pass
            if updated:
                self.after(200, self._conveyor_status_poll)
            else:
                self.after(300, self._conveyor_status_poll)

        def _update_status_loop(self):
            while not self.status_queue.empty():
                try: s=self.status_queue.get_nowait()
                except: break
                # JSON 状态消息 (如 grasp_result) — 立即处理, 不覆盖 idle/busy/error
                if s and isinstance(s, str) and s.startswith('{'):
                    try:
                        d = json.loads(s)
                        if d.get("type") == "grasp_result":
                            self._on_grasp_result(d)
                    except: pass
                    continue
                self.current_status = s
            s=self.current_status
            if s=='idle':
                self.status_var.set("当前服务器状态: 空闲 (Idle)"); self.status_label.configure(fg="green")
                self.error_notified=False
                # ⚠️ 自动分拣运行时禁止弹出排队任务: 状态机自己通过 _dispatch_sort_cmd 直接
                # 下发 sort_verify, 排队队列的 cmd_queue.put 会与服务端正在处理的指令冲突。
                if not self.auto_sort_running and getattr(self,'queue_running',False):
                    if self.task_queue and (time.time()-self.last_dispatch_time>1.5):
                        self.cmd_queue.put(self.task_queue.pop(0)); self.refresh_queue_listbox()
                        self.last_dispatch_time=time.time()
                    elif not self.task_queue and (time.time()-self.last_dispatch_time>1.5):
                        self.queue_running=False; self.qrun.config(text=" 开始执行排队任务",bg="lightgreen")
                        messagebox.showinfo("完成","排队队列所有任务已执行完毕！")
            elif s=='busy': self.status_var.set("当前服务器状态: 忙碌 (Busy)"); self.status_label.configure(fg="red")
            elif s=='error':
                self.status_var.set("当前服务器状态: 异常 (Error)"); self.status_label.configure(fg="orange")
                if self.auto_sort_running:
                    pass  # 自动分拣运行时, 状态机自行处理 error, 不弹窗
                elif not getattr(self,'error_notified',False):
                    self.error_notified=True; self.clear_queue()
                    messagebox.showwarning("规划失败","机械臂执行失败。任务队列已清空。")
            self.after(500,self._update_status_loop)

        # ═══════════════════════ 自动分拣 ═══════════════════════

        def _auto_sort_log(self, msg):
            """自动分拣日志: 写文件 + 更新状态标签。"""
            _log(f"[AUTO_SORT] [{self.auto_sort_state}] {msg}")
            try:
                self.auto_sort_state_var.set(f"[{self.auto_sort_state}] {msg}")
            except: pass

        def _bin_for_object(self, name):
            """分拣规则: apple/green apple/strawberry/orange→料框1, lemon/honey peach/pear→料框2。"""
            n = (name or '').lower().strip()
            for k in ('apple', 'green apple', 'strawberry', 'orange'):
                if k in n: return 1
            for k in ('lemon', 'honey peach', 'pear'):
                if k in n: return 2
            return 1  # 未知默认料框1

        def _pick_object_right_of_line(self):
            """筛选 x2 ≥ 分界线 且有 base_position_m 的物体, 返回 center_u 最大的 (最右侧, 最远离分界线)。

            物体从左向右移动, 越过左侧分界线后进入抓取区 (分界线右侧).
            选 center_u 最大的 = 最右侧 = 最远离分界线的物体 (先抓最远的, 避免机械臂遮挡后续物体).
            """
            d = self.latest_detection
            if not d or not d.get('detected'): return None
            objects = d.get('objects', []) or []
            candidates = []
            for obj in objects:
                bp = obj.get('base_position_m')
                if not bp or not all(k in bp for k in ('x','y','z')): continue
                bbox = obj.get('bbox_pixel', {})
                x1 = bbox.get('x1'); x2 = bbox.get('x2')
                if x1 is None or x2 is None: continue
                if x2 >= self._auto_sort_line_u:
                    center_u = (x1 + x2) / 2.0
                    candidates.append((center_u, obj))
            if not candidates: return None
            candidates.sort(key=lambda t: t[0], reverse=True)  # center_u 降序 (最大=最右侧, 最远离分界线)
            return candidates[0][1]

        def _dispatch_sort_cmd(self, obj, retry=False):
            """构造分拣命令并发送: 使用 sort (单次检测, eye-to-hand 下无需两阶段精定位)。"""
            bp = obj['base_position_m']
            try:
                r = float(self.default_grasp_roll.get())
                p = float(self.default_grasp_pitch.get())
                y = float(self.default_grasp_yaw.get())
            except ValueError:
                r, p, y = 180.0, 0.0, 0.0
            qx, qy, qz, qw = euler_to_quaternion(r, p, y)
            pick = {'x': round(float(bp['x']), 3), 'y': round(float(bp['y']), 3), 'z': round(float(bp['z']), 3),
                    'qx': round(qx, 6), 'qy': round(qy, 6), 'qz': round(qz, 6), 'qw': round(qw, 6)}
            name = obj.get('object_name', '物体')
            bin_num = self._bin_for_object(name)
            # 优先用 D455 估计直径, fallback 预设尺寸
            dia = obj.get('estimated_diameter_m') or obj.get('size_m', {}).get('diameter')
            cmd = {"cmd": "sort", "pick": pick, "bin": bin_num,
                   "pick_name": f"{name} (auto-sort{'-retry' if retry else ''})",
                   "place_name": f"料框{bin_num}"}
            method_tag = "single"
            if dia: cmd["object_diameter_m"] = round(float(dia), 3)
            self.last_dispatch_time = time.time()
            self.cmd_queue.put(cmd)
            self._auto_sort_log(f"分拣[{method_tag}]: {name}→料框{bin_num} {'[重试]' if retry else ''}")

        def _set_manual_buttons_state(self, state):
            """批量禁用/启用手动按钮。state: tk.NORMAL 或 tk.DISABLED。"""
            for btn in (getattr(self, 'conveyor_on_btn', None), getattr(self, 'conveyor_off_btn', None),
                        getattr(self, 'pick_btn', None), getattr(self, 'pick_two_btn', None),
                        getattr(self, 'pick_graspnet_btn', None),
                        getattr(self, 'pick_blue_block_btn', None),
                        getattr(self, 'qrun', None)):
                if btn is not None:
                    try: btn.config(state=state)
                    except: pass

        def _draw_dividing_line(self):
            """在相机画布上画黄色虚线。位置由 self._auto_sort_line_u 决定 (原图 640 宽 → canvas 400 宽等比映射)。"""
            try:
                # 原图 u 坐标 → canvas x 坐标 (等比缩放)
                x_line = int(self._canvas_w * self._auto_sort_line_u / 640)
                self._div_line_id = self.camera_canvas.create_line(
                    x_line, 0, x_line, self._canvas_h, fill='yellow', dash=(6, 4), width=2)
                self._div_label_id = self.camera_canvas.create_text(
                    x_line + 5, 12, text=f"分界线 u={self._auto_sort_line_u}", fill='yellow', anchor=tk.NW, font=("Arial", 9, "bold"))
                self.camera_canvas.itemconfig(self._div_line_id, state='hidden')
                self.camera_canvas.itemconfig(self._div_label_id, state='hidden')
            except Exception as e:
                _log(f"画分界线失败: {e}")

        def start_auto_sort(self):
            """启动自动分拣。"""
            if self.auto_sort_running: return
            self.auto_sort_running = True
            self._auto_sort_current_obj = None
            self._auto_sort_retry_count = 0
            self._auto_error_seen = False
            self._auto_sort_stop_requested = False
            self._conv_stop_retry = 0
            self._conv_stopped_ts = 0
            self._set_manual_buttons_state(tk.DISABLED)
            self.auto_sort_start_btn.config(state=tk.DISABLED, text=" 自动分拣运行中 ")
            self.auto_sort_stop_btn.config(state=tk.NORMAL)
            # 显示分界线
            if hasattr(self, '_div_line_id'):
                self.camera_canvas.itemconfig(self._div_line_id, state='normal')
                self.camera_canvas.itemconfig(self._div_label_id, state='normal')
            self._auto_sort_log("自动分拣启动")
            self.auto_sort_state = 'CHECK_RIGHT'
            self._auto_sort_tick()

        def stop_auto_sort(self):
            """停止自动分拣 (等当前分拣完成后完全停止)。"""
            if not self.auto_sort_running: return
            self._auto_sort_stop_requested = True
            self.auto_sort_stop_btn.config(state=tk.DISABLED)
            # 立即停传送带
            if self.conveyor_state == True and not self._conveyor_busy:
                self._conveyor_set(False)
            self.auto_sort_state = 'STOPPING'
            self._auto_sort_log("停止请求已发出, 等待当前任务结束")

        def _finish_auto_sort(self):
            """完全停止, 恢复按钮。"""
            self.auto_sort_running = False
            self.auto_sort_state = 'IDLE'
            self._auto_sort_current_obj = None
            self._auto_sort_retry_count = 0
            self._auto_error_seen = False
            self._auto_sort_stop_requested = False
            self._conv_stop_retry = 0
            self._conv_stopped_ts = 0
            self._set_manual_buttons_state(tk.NORMAL)
            self.auto_sort_start_btn.config(state=tk.NORMAL, text=" 开始自动分拣 ")
            self.auto_sort_stop_btn.config(state=tk.DISABLED)
            # 隐藏分界线
            if hasattr(self, '_div_line_id'):
                self.camera_canvas.itemconfig(self._div_line_id, state='hidden')
                self.camera_canvas.itemconfig(self._div_label_id, state='hidden')
            self._auto_sort_log("自动分拣已停止")
            if self._auto_sort_tick_id is not None:
                try: self.after_cancel(self._auto_sort_tick_id)
                except: pass
                self._auto_sort_tick_id = None

        def _auto_sort_tick(self):
            """状态机主循环, 每 300ms 调用一次。"""
            if not self.auto_sort_running: return
            try:
                st = self.auto_sort_state
                if st == 'CHECK_RIGHT':       self._tick_check_right()
                elif st == 'CONVEYOR_WAIT':  self._tick_conveyor_wait()
                elif st == 'CONVEYOR_STOPPING': self._tick_conveyor_stopping()
                elif st == 'SORTING':        self._tick_sorting()
                elif st == 'STOPPING':       self._tick_stopping()
            except Exception as e:
                self._auto_sort_log(f"tick 异常: {e}")
            self._auto_sort_tick_id = self.after(300, self._auto_sort_tick)

        def _tick_check_right(self):
            """检查分界线右侧有无物体。"""
            # 等服务端空闲
            if self.current_status == 'busy': return
            obj = self._pick_object_right_of_line()
            if obj is not None:
                self._auto_sort_current_obj = obj
                self._auto_sort_retry_count = 0
                self._auto_error_seen = False
                self._dispatch_sort_cmd(obj, retry=False)
                self.auto_sort_state = 'SORTING'
            else:
                # 无物体, 开传送带
                if not self._conveyor_busy and self.conveyor_state != True:
                    self._conveyor_set(True)
                self.auto_sort_state = 'CONVEYOR_WAIT'
                self._auto_sort_log("右侧无物体, 开启传送带")

        def _tick_conveyor_wait(self):
            """传送带运行中, 等待物体跨越分界线。"""
            if self._conveyor_busy: return  # 等传送带切换完成
            if self.conveyor_state != True:
                # 传送带未运行 (可能切换失败), 重新开
                self._conveyor_set(True)
                return
            obj = self._pick_object_right_of_line()
            if obj is not None:
                self._auto_sort_current_obj = obj
                self._auto_sort_retry_count = 0
                self._auto_error_seen = False
                self._conveyor_set(False)
                self._conv_stop_retry = 0
                self._conv_stopped_ts = 0  # 重置停止观察时间戳
                self.auto_sort_state = 'CONVEYOR_STOPPING'
                name = obj.get('object_name', '?')
                self._auto_sort_log(f"检测到物体越线: {name}, 停止传送带")

        def _tick_conveyor_stopping(self):
            """等传送带停止后, 再观察 1s 确保物体完全静止, 然后重新选物体并分拣。

            ⚠️ 必须等 1s: 传送带断电后物体可能因惯性继续滑动一小段距离,
            若立即用当前检测坐标分拣, 会抓到物体已经离开的位置 → 空夹。
            EMA 滤波也需要几帧才能收敛到静止后的真实位置。
            """
            if self._conveyor_busy: return  # 等传送带停止完成
            if self.conveyor_state == False:
                # 第一次检测到已停止, 记录时间戳, 进入观察期
                if self._conv_stopped_ts == 0:
                    self._conv_stopped_ts = time.time()
                    self._auto_sort_log("传送带已停, 观察 1s 等物体静止")
                    return
                # 观察 1s 等物体完全静止 + EMA 滤波收敛
                if time.time() - self._conv_stopped_ts < 1.0:
                    return
                # 观察完成, 重新选物体 (位置可能在停止/惯性滑动后变化)
                self._conv_stopped_ts = 0
                obj = self._pick_object_right_of_line()
                if obj is None:
                    self._auto_sort_log("物体已离开视野, 回到检测")
                    self.auto_sort_state = 'CHECK_RIGHT'
                    return
                self._auto_sort_current_obj = obj
                self._dispatch_sort_cmd(obj, retry=False)
                self.auto_sort_state = 'SORTING'
            else:
                # 传送带未停止, 重试
                self._conv_stop_retry += 1
                if self._conv_stop_retry <= 3:
                    self._conveyor_set(False)
                else:
                    self._auto_sort_log("传送带停止失败, 进入停止流程")
                    self.auto_sort_state = 'STOPPING'

        def _tick_sorting(self):
            """等分拣完成, 处理 error 重试/跳过。"""
            st = self.current_status
            if st == 'busy':
                return  # 正在执行
            if st == 'error':
                self._auto_error_seen = True
                return  # 等服务端自动恢复 → idle
            if st == 'idle':
                if self._auto_error_seen:
                    # error 后恢复完成, 决定重试或跳过
                    self._auto_error_seen = False
                    if self._auto_sort_retry_count == 0:
                        self._auto_sort_retry_count = 1
                        obj = self._auto_sort_current_obj
                        if obj:
                            self._dispatch_sort_cmd(obj, retry=True)
                            self._auto_sort_log("分拣失败, 重试一次")
                            return  # 留在 SORTING 等待结果
                        else:
                            self._auto_sort_log("无重试目标, 跳过")
                    else:
                        self._auto_sort_log("重试仍失败, 跳过该物体")
                # 成功或跳过 → 回 CHECK_RIGHT
                self._auto_sort_current_obj = None
                self._auto_sort_retry_count = 0
                self.auto_sort_state = 'CHECK_RIGHT'
                self._auto_sort_log("分拣完成, 继续检测")

        def _tick_stopping(self):
            """停止流程: 等当前任务完成, 回待机位, 结束。"""
            # 确保传送带已停
            if self.conveyor_state != False and not self._conveyor_busy:
                self._conveyor_set(False)
                return
            if self._conveyor_busy: return
            # 等当前分拣/恢复完成
            if self.current_status in ('busy', 'error'): return
            # idle → 安全结束
            self._finish_auto_sort()

    # =========== 启动 GUI ===========
    app = SortingApp()
    app.protocol("WM_DELETE_WINDOW", lambda: app.destroy())

    # 相机/YOLO 启动状态轮询：auto_start_pipeline 现在在 ROS worker 子进程中运行，
    # 状态消息通过 mp_status_q (multiprocessing.Queue) 跨进程传回主线程。
    # 主线程用 after() 轮询取消息再更新 Tkinter，避免跨线程/跨进程访问 Tk。
    # ⚠️ 主进程中不再有任何 fork() (subprocess.run/Popen) 调用，Tcl 引用计数不会被破坏。
    def poll_status_msg():
        while True:
            try: msg=mp_status_q.get_nowait()
            except queue.Empty: break
            try: app.camera_status_var.set(msg)
            except: pass
        app.after(200, poll_status_msg)
    app.after(200, poll_status_msg)
    _log("GUI mainloop start")

    try: app.mainloop()
    finally:
        _log("GUI exit")
        ros_proc.terminate(); ros_proc.join(timeout=3)


if __name__ == "__main__":
    multiprocessing.set_start_method('spawn')
    main()
