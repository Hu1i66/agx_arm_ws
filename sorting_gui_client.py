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

def ros_process_worker(cmd_q, st_q, pose_q, img_q, det_q):
    """ROS2 子进程 — camera/YOLO 回调即时推送 JPEG。 无 Tkinter 依赖。"""
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
                m = String(); m.data = json.dumps(cmd_q.get()); self._pub.publish(m)

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
_CAMERA_CMD = f"{_ROS2_SRC} && ros2 launch realsense2_camera rs_launch.py"
_YOLO_CMD   = f"{_ROS2_SRC} && {_VENV_SRC} && python3 {_YOLO_SCRIPT} --ros-args -p show_gui_window:=false"

# CH340 USB 继电器控制脚本（Modbus RTU）。继电器接在传送带电路的常闭(NC)接口上。
_RELAY_SCRIPT = "/home/lxf/agx_arm_ws/ch340-relay/relay_control.py"

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

    ros_proc = multiprocessing.Process(target=ros_process_worker,
                                       args=(cmd_q,st_q,pose_q,img_q,det_q), name="ros")
    ros_proc.start()
    _log("ROS proc started")

    # =========== GUI 类定义 ===========

    class SortingApp(tk.Tk):
        def __init__(self):
            super().__init__()
            self.cmd_queue=cmd_q; self.status_queue=st_q; self.pose_queue=pose_q
            self.image_queue=img_q; self.detection_queue=det_q
            self.title("机械臂分拣控制面板")
            self.geometry("1350x680"); self.configure(padx=10, pady=10)
            self.poses = {}; self.load_poses()
            self.current_status='idle'; self.task_queue=[]
            self.last_dispatch_time=0; self.queue_running=False
            self.latest_detection=None; self._last_frame_time=0.0
            self._canvas_img_id=None; self._canvas_text_id=None
            self._photos=[]; self._canvas_w=400; self._canvas_h=300
            self.default_grasp_roll=tk.DoubleVar(value=180.0)
            self.default_grasp_pitch=tk.DoubleVar(value=0.0)
            self.default_grasp_yaw=tk.DoubleVar(value=0.0)
            # 传送带(继电器)状态: True=运行, False=停止, None=未知
            self._conveyor_busy=False
            self.conveyor_state=None
            self._conveyor_proc=None        # 当前继电器子进程(Popen)，None=无任务
            self._conveyor_run=None         # 当前任务对应的传送带意图
            self.setup_ui()
            self._update_status_loop()
            self._update_camera_loop()
            self._update_detection_loop()

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
                                        font=("Arial",12,"bold"),bg="#2196F3",fg="white",state=tk.DISABLED)
            self.pick_two_btn.pack(fill=tk.X,pady=(0,2))
            tk.Label(df,text=" =单次检测直接抓取 |  =两阶段精定位: 近距离二次识别后抓取",font=("Arial",8),fg="gray").pack()

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
            tk.Label(ft,text="放置点 (Place):").grid(row=0,column=2,pady=10)
            self.place_combo=ttk.Combobox(ft,state="readonly",width=15); self.place_combo.grid(row=0,column=3,padx=10)
            tk.Button(ft,text="添加至分拣队列",command=self.add_to_queue,font=("Arial",11,"bold"),bg="lightblue",width=18).grid(row=1,column=0,columnspan=2,pady=10)
            tk.Button(ft,text="立即发送分拣指令",command=self.send_sort,font=("Arial",11,"bold"),bg="lightgreen",width=18).grid(row=1,column=2,columnspan=2,pady=10)
            self.update_comboboxes()
            # ── 传送带控制（继电器接常闭 NC，存在反转，详见 _conveyor_set 注释）──
            fc=tk.LabelFrame(self.right_frame,text="传送带控制",padx=10,pady=10); fc.pack(fill="x",pady=5)
            self.conveyor_state_var=tk.StringVar(value="传送带状态: 未知")
            self.conveyor_state_label=tk.Label(fc,textvariable=self.conveyor_state_var,font=("Arial",11,"bold"),fg="gray")
            self.conveyor_state_label.pack(pady=(0,8))
            cbf=tk.Frame(fc); cbf.pack(fill="x")
            self.conveyor_on_btn=tk.Button(cbf,text="传送带开",command=lambda:self._conveyor_set(True),
                                           font=("Arial",12,"bold"),bg="#4CAF50",fg="white")
            self.conveyor_on_btn.pack(side=tk.LEFT,expand=True,fill="x",padx=(0,4))
            self.conveyor_off_btn=tk.Button(cbf,text="传送带关",command=lambda:self._conveyor_set(False),
                                            font=("Arial",12,"bold"),bg="#f44336",fg="white")
            self.conveyor_off_btn.pack(side=tk.LEFT,expand=True,fill="x",padx=(4,0))
            tk.Label(fc,text=" 继电器接常闭: 传送带开=继电器off / 传送带关=继电器on",font=("Arial",8),fg="gray").pack(pady=(6,0))
            fa=tk.Frame(self.right_frame); fa.pack(pady=10)
            tk.Button(fa,text="回到观察位",command=self.send_observe,font=("Arial",12,"bold"),bg="#2196F3",fg="white",width=40).pack(pady=5)
            tk.Button(fa,text="一键回到待机点并关闭夹爪",command=self.send_reset,font=("Arial",12,"bold"),bg="orange",width=40).pack(pady=5)
            tk.Button(fa,text="退出服务端系统",command=self.send_quit,font=("Arial",12,"bold"),bg="tomato",fg="white",width=40).pack(pady=5)

        def update_comboboxes(self):
            names=list(self.poses.keys())
            for cb in(self.pick_combo,self.place_combo,getattr(self,'del_combo',None)):
                if cb: cb['values']=names
            if names:
                if not self.pick_combo.get(): self.pick_combo.current(0)
                if not self.place_combo.get(): self.place_combo.current(0)
                if hasattr(self,'del_combo') and not self.del_combo.get(): self.del_combo.current(0)
            else:
                for cb in(self.pick_combo,self.place_combo,getattr(self,'del_combo',None)):
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
                    photo=ImageTk.PhotoImage(img)

                    # 永久保留旧 PhotoImage 阻止 GC →
                    # Python 永远不会调 __del__ → 永远不会有 Tcl_Release 双重释放
                    self._photos.append(photo)
                    while len(self._photos)>60:
                        self._photos.pop(0)

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
                    if d.get('detected'):
                        self.det_status_var.set(" 已检测到物体")
                        self.pick_btn.config(state=tk.NORMAL,bg="#4CAF50")
                        self.pick_two_btn.config(state=tk.NORMAL,bg="#2196F3")
                        self.det_obj.set(f"物体: {d.get('object_name','--')}")
                        self.det_conf.set(f"置信度: {d.get('confidence',0)*100:.2f}%")
                        self.det_meth.set(f"方法: {d.get('method','--')}")
                        bp=d.get('base_position_m',{})
                        if bp and all(k in bp for k in('x','y','z')):
                            self.det_x.set(f"  X: {bp['x']:.3f} m"); self.det_y.set(f"  Y: {bp['y']:.3f} m")
                            self.det_z.set(f"  Z: {bp['z']:.3f} m")
                        else: self.det_x.set("  X: --"); self.det_y.set("  Y: --"); self.det_z.set("  Z: --")
                        sm=d.get('size_m',{}); dia=sm.get('diameter')
                        self.det_dia.set(f"直径: {dia:.3f} m" if dia else "直径: --")
                        dist=d.get('distance_to_robot_m')
                        self.det_dist.set(f"距离: {dist:.3f} m" if dist else "距离: --")
                    else:
                        self.det_status_var.set(" 未检测到物体")
                        for v in(self.det_obj,self.det_conf,self.det_meth,self.det_x,self.det_y,self.det_z,self.det_dia,self.det_dist):
                            v.set(v.get().split(":")[0]+": --")
                        self.pick_btn.config(state=tk.DISABLED,bg="gray")
                        self.pick_two_btn.config(state=tk.DISABLED,bg="gray")
            except: pass
            self.after(200,self._update_detection_loop)

        # ── 单次检测直接夹取 ──
        def _pick_from_detection(self):
            d=self.latest_detection
            if not d: return messagebox.showwarning("无数据","尚未收到检测信息。")
            if not d.get('detected'): return messagebox.showwarning("未检测到","当前未发现物体。")
            bp=d.get('base_position_m',{})
            if not bp or not all(k in bp for k in('x','y','z')): return messagebox.showerror("坐标缺失","缺少 base_position_m。")
            pn=self.place_combo.get()
            if not pn or pn not in self.poses: return messagebox.showerror("错误","请选择有效的放置点。")
            try: r=float(self.default_grasp_roll.get()); p=float(self.default_grasp_pitch.get()); y=float(self.default_grasp_yaw.get())
            except ValueError: return messagebox.showerror("姿态错误","R/P/Y 必须是数字。")
            qx,qy,qz,qw=euler_to_quaternion(r,p,y)
            pp={'x':round(float(bp['x']),3),'y':round(float(bp['y']),3),'z':round(float(bp['z']),3),
                'qx':round(qx,6),'qy':round(qy,6),'qz':round(qz,6),'qw':round(qw,6)}
            dia=d.get('size_m',{}).get('diameter')
            if dia: dia=round(float(dia),3)
            dia_str=f"物体直径: {dia} m\n\n" if dia else ""
            txt=(f"检测坐标夹取确认\n\nPick: {d.get('object_name','物体')} (detected)\n"
                 f"  坐标: ({pp['x']:.3f}, {pp['y']:.3f}, {pp['z']:.3f})\n  姿态: R={r}  P={p}  Y={y} \n\n"
                 f"Place: {pn}\n  坐标: ({self.poses[pn]['x']:.3f}, {self.poses[pn]['y']:.3f}, {self.poses[pn]['z']:.3f})\n\n"
                 f"{dia_str}确定发送？")
            if not messagebox.askyesno("确认",txt): return
            pqx,pqy,pqz,pqw=euler_to_quaternion(180,0,0)
            pp2={'x':round(float(self.poses[pn]['x']),3),'y':round(float(self.poses[pn]['y']),3),'z':round(float(self.poses[pn]['z']),3),
                 'qx':round(pqx,6),'qy':round(pqy,6),'qz':round(pqz,6),'qw':round(pqw,6)}
            cmd={"cmd":"sort","pick":pp,"place":pp2,"pick_name":f"{d.get('object_name','物体')} (detected)","place_name":pn}
            if dia: cmd["object_diameter_m"]=float(dia)
            if self.current_status=='busy':
                if messagebox.askyesno("忙碌","加入排队队列？"):
                    self.task_queue.append(cmd); self.refresh_queue_listbox()
                    messagebox.showinfo("已加入","任务已加入排队队列。")
            else: self.last_dispatch_time=time.time(); self.cmd_queue.put(cmd); messagebox.showinfo("已发送",f"夹取指令已发送！\n检测物体 -> 【{pn}】")

        # ── 两阶段精定位夹取 ──
        def _pick_from_detection_two_stage(self):
            d=self.latest_detection
            if not d: return messagebox.showwarning("无数据","尚未收到检测信息。")
            if not d.get('detected'): return messagebox.showwarning("未检测到","当前未发现物体。")
            bp=d.get('base_position_m',{})
            if not bp or not all(k in bp for k in('x','y','z')): return messagebox.showerror("坐标缺失","缺少 base_position_m。")
            pn=self.place_combo.get()
            if not pn or pn not in self.poses: return messagebox.showerror("错误","请选择有效的放置点。")
            try: r=float(self.default_grasp_roll.get()); p=float(self.default_grasp_pitch.get()); y=float(self.default_grasp_yaw.get())
            except ValueError: return messagebox.showerror("姿态错误","R/P/Y 必须是数字。")
            qx,qy,qz,qw=euler_to_quaternion(r,p,y)
            pp={'x':round(float(bp['x']),3),'y':round(float(bp['y']),3),'z':round(float(bp['z']),3),
                'qx':round(qx,6),'qy':round(qy,6),'qz':round(qz,6),'qw':round(qw,6)}
            dia=d.get('size_m',{}).get('diameter')
            if dia: dia=round(float(dia),3)
            dia_str=f"物体直径: {dia} m\n" if dia else ""
            txt=(f" 两阶段精定位夹取确认\n\n观察位检测坐标:\n  ({pp['x']:.3f}, {pp['y']:.3f}, {pp['z']:.3f})\n\n"
                 f"-> 机械臂移动到物体正上方\n-> YOLO 近距离二次检测\n-> 以二次结果为准直接抓取\n\n"
                 f"Place: {pn}\n  坐标: ({self.poses[pn]['x']:.3f}, {self.poses[pn]['y']:.3f}, {self.poses[pn]['z']:.3f})\n"
                 f"{dia_str}\n确定发送？")
            if not messagebox.askyesno("确认两阶段精定位夹取",txt): return
            pqx,pqy,pqz,pqw=euler_to_quaternion(180,0,0)
            pp2={'x':round(float(self.poses[pn]['x']),3),'y':round(float(self.poses[pn]['y']),3),'z':round(float(self.poses[pn]['z']),3),
                 'qx':round(pqx,6),'qy':round(pqy,6),'qz':round(pqz,6),'qw':round(pqw,6)}
            cmd={"cmd":"sort_verify","pick":pp,"place":pp2,"pick_name":f"{d.get('object_name','物体')} (two-stage)","place_name":pn}
            if dia: cmd["object_diameter_m"]=float(dia)
            if self.current_status=='busy':
                if messagebox.askyesno("忙碌","加入排队队列？"):
                    self.task_queue.append(cmd); self.refresh_queue_listbox()
                    messagebox.showinfo("已加入","两阶段精定位任务已加入排队队列。")
            else: self.last_dispatch_time=time.time(); self.cmd_queue.put(cmd); messagebox.showinfo("已发送",f"两阶段精定位指令已发送！\n检测物体 -> 【{pn}】")

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

        def send_observe(self):
            if self.current_status=='busy': return messagebox.showwarning("忙碌","机械臂正在执行动作。")
            if messagebox.askyesno("确认","机械臂将以关节空间运动到观察位。确定？"):
                self.cmd_queue.put({"cmd":"observe"}); messagebox.showinfo("已发送","观察位指令发送成功！")

        def send_reset(self):
            if self.current_status=='busy': return messagebox.showwarning("忙碌","机械臂正在执行动作。")
            if messagebox.askyesno("确认","机械臂将回到待机位并关闭夹爪。确定？"):
                self.cmd_queue.put({"cmd":"reset"}); self.clear_queue(); messagebox.showinfo("已发送","复位指令发送成功！")

        def add_to_queue(self):
            pn,pp=self.pick_combo.get(),self.place_combo.get()
            if not pn or not pp: return messagebox.showerror("错误","请选择完整的夹取点和放置点！")
            self.task_queue.append({"cmd":"sort","pick":self.poses[pn],"place":self.poses[pp],"pick_name":pn,"place_name":pp})
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
            pn,pp=self.pick_combo.get(),self.place_combo.get()
            if not pn or not pp: return messagebox.showerror("错误","请选择完整的夹取点和放置点！")
            self.last_dispatch_time=time.time()
            self.cmd_queue.put({"cmd":"sort","pick":self.poses[pn],"place":self.poses[pp],"pick_name":pn,"place_name":pp})
            messagebox.showinfo("已发送",f"分拣指令已发送！\n【{pn}】->【{pp}】")

        def send_quit(self):
            if messagebox.askyesno("确认","让服务端回到待机位、关闭夹爪并结束程序。确定？"):
                self.cmd_queue.put({"cmd":"quit"}); self.after(500,self.destroy)

        # ── 传送带控制 ──
        # 实现说明：不使用子线程。Tkinter 非线程安全，从子线程调用
        # self.after()/任何 Tk 方法会破坏 Tcl 引用计数，触发
        # "Tcl_Release couldn't find reference for 0x..." 并核心转储。
        # 改用 subprocess.Popen 非阻塞启动继电器脚本，主线程用 after() 轮询
        # 进程是否结束，保证所有 Tk 访问都在主线程。
        def _conveyor_set(self, run):
            """设置传送带运行/停止。

            ⚠️ 关键反转逻辑（继电器接常闭 NC 接口）：
                传送带开(run=True)  → 传送带需通电 → 继电器须断电(NC闭合) → relay off
                传送带关(run=False) → 传送带需断电 → 继电器须通电(NC断开) → relay on
            即：传送带意图与继电器命令是反相的。
            """
            if self._conveyor_busy:
                return
            if not os.path.exists(_RELAY_SCRIPT):
                messagebox.showerror("错误", f"继电器控制脚本不存在:\n{_RELAY_SCRIPT}")
                return
            self._conveyor_busy=True
            self.conveyor_on_btn.config(state=tk.DISABLED)
            self.conveyor_off_btn.config(state=tk.DISABLED)
            self.conveyor_state_var.set("传送带状态: 切换中...")
            self.conveyor_state_label.config(fg="orange")
            relay_action='off' if run else 'on'   # ← 关键反转点：传送带开=relay off, 传送带关=relay on
            try:
                self._conveyor_run=run
                self._conveyor_proc=subprocess.Popen(
                    ['python3',_RELAY_SCRIPT,relay_action],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                self.after(50, self._conveyor_poll)
            except Exception as e:
                self._conveyor_done(run, False, str(e))

        def _conveyor_poll(self):
            """主线程轮询继电器子进程是否结束。不阻塞 GUI，不跨线程访问 Tkinter。"""
            proc=self._conveyor_proc
            if proc is None:
                return
            if proc.poll() is None:
                self.after(50, self._conveyor_poll)   # 仍在运行，继续等
                return
            # 进程已退出，读取输出（已退出故 communicate 立即返回）
            try:
                out, err=proc.communicate()
            except Exception:
                out, err='', ''
            ok=(proc.returncode==0 and '✅' in (out or ''))
            errstr=(err or out or '').strip()[-300:]
            run=self._conveyor_run
            self._conveyor_proc=None; self._conveyor_run=None
            self._conveyor_done(run, ok, errstr)

        def _conveyor_done(self, run, ok, err):
            self._conveyor_busy=False
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
                messagebox.showerror("传送带控制失败", f"继电器通信失败:\n{err}")

        def _update_status_loop(self):
            while not self.status_queue.empty():
                try: self.current_status=self.status_queue.get_nowait()
                except: break
            s=self.current_status
            if s=='idle':
                self.status_var.set("当前服务器状态: 空闲 (Idle)"); self.status_label.configure(fg="green")
                self.error_notified=False
                if getattr(self,'queue_running',False):
                    if self.task_queue and (time.time()-self.last_dispatch_time>1.5):
                        self.cmd_queue.put(self.task_queue.pop(0)); self.refresh_queue_listbox()
                        self.last_dispatch_time=time.time()
                    elif not self.task_queue and (time.time()-self.last_dispatch_time>1.5):
                        self.queue_running=False; self.qrun.config(text=" 开始执行排队任务",bg="lightgreen")
                        messagebox.showinfo("完成","排队队列所有任务已执行完毕！")
            elif s=='busy': self.status_var.set("当前服务器状态: 忙碌 (Busy)"); self.status_label.configure(fg="red")
            elif s=='error':
                self.status_var.set("当前服务器状态: 异常 (Error)"); self.status_label.configure(fg="orange")
                if not getattr(self,'error_notified',False):
                    self.error_notified=True; self.clear_queue()
                    messagebox.showwarning("规划失败","机械臂执行失败。任务队列已清空。")
            self.after(500,self._update_status_loop)

    # =========== 启动 GUI ===========
    app = SortingApp()
    app.protocol("WM_DELETE_WINDOW", lambda: app.destroy())

    # 线程安全的相机/YOLO 状态上报：子线程只往 queue 放消息，
    # 主线程用 after() 轮询取消息再更新 Tkinter，避免跨线程访问 Tk
    # 触发 "Tcl_Release couldn't find reference" 崩溃。
    status_msg_q = queue.Queue()
    def set_status(msg):
        try: status_msg_q.put_nowait(msg)
        except: pass

    threading.Thread(target=auto_start_pipeline, args=(set_status,), daemon=True).start()

    def poll_status_msg():
        while True:
            try: msg=status_msg_q.get_nowait()
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
