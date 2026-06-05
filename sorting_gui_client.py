#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, messagebox
import json
import os
import math
import multiprocessing
import time
import io
import struct

def euler_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """将欧拉角(度)转为四元数 (qx, qy, qz, qw)，使用 ZYX 内旋（RPY）约定。"""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def decode_sensor_image(msg) -> bytes:
    """手动解码 sensor_msgs/Image 为 PNG 字节流，不依赖 cv_bridge。"""
    import numpy as np
    from PIL import Image as PILImage

    encoding = msg.get('encoding', 'rgb8')
    height = msg['height']
    width = msg['width']
    step = msg.get('step', width * 3)
    data = msg['data']

    # data 是 list[int] / bytes，统一转为 np.uint8
    if isinstance(data, list):
        arr = np.array(data, dtype=np.uint8)
    elif isinstance(data, bytes):
        arr = np.frombuffer(data, dtype=np.uint8)
    else:
        return b''

    total_pixels = height * width

    if encoding in ('rgb8', 'rgb'):
        channels = 3
        flat = arr[:total_pixels * channels]
        img_arr = flat.reshape((height, width, channels))
        mode = 'RGB'
    elif encoding in ('bgr8', 'bgr'):
        channels = 3
        flat = arr[:total_pixels * channels]
        img_arr = flat.reshape((height, width, channels))[:, :, ::-1]  # BGR → RGB
        mode = 'RGB'
    elif encoding in ('rgba8', 'rgba'):
        channels = 4
        flat = arr[:total_pixels * channels]
        img_arr = flat.reshape((height, width, channels))
        mode = 'RGBA'
    elif encoding in ('bgra8', 'bgra'):
        channels = 4
        flat = arr[:total_pixels * channels]
        # BGRA → RGBA: swap R and B
        img_arr = flat.reshape((height, width, channels))
        b, g, r, a = img_arr[:, :, 0].copy(), img_arr[:, :, 1].copy(), img_arr[:, :, 2].copy(), img_arr[:, :, 3].copy()
        img_arr = np.stack([r, g, b, a], axis=2)
        mode = 'RGBA'
    elif encoding == 'mono8':
        flat = arr[:total_pixels]
        img_arr = flat.reshape((height, width))
        mode = 'L'
    elif encoding == '16UC1':
        flat = arr[:total_pixels * 2]
        img_arr16 = np.frombuffer(flat.tobytes(), dtype=np.uint16).reshape((height, width))
        # 归一化到 0-255
        img_arr = (img_arr16 / 256).astype(np.uint8)
        mode = 'L'
    else:
        # 不认识的编码，尝试按 rgb8 解析
        channels = 3
        flat = arr[:total_pixels * channels]
        img_arr = flat.reshape((height, width, channels))
        mode = 'RGB'

    pil_img = PILImage.fromarray(img_arr, mode=mode)
    buf = io.BytesIO()
    pil_img.save(buf, format='PNG')
    return buf.getvalue()


def ros_process_worker(cmd_queue, status_queue, pose_queue, image_queue, detection_queue):
    import rclpy
    from rclpy.node import Node
    from rclpy.logging import LoggingSeverity, set_logger_level
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    import tf2_ros

    class HeadlessROSNode(Node):
        def __init__(self):
            super().__init__('sorting_headless_client_node')
            for logger_name in ('tf2', 'tf2_ros', 'tf2_buffer', 'tf2_ros_buffer'):
                try:
                    set_logger_level(logger_name, LoggingSeverity.ERROR)
                except Exception:
                    pass

            self.cmd_pub = self.create_publisher(String, '/sorting_cmds', 10)
            self.status_sub = self.create_subscription(String, '/sorting_status', self.status_cb, 10)
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

            # ── 新增：订阅相机画面 ──
            self.image_sub = self.create_subscription(
                Image,
                '/camera/camera/color/image_raw',
                self.image_cb,
                10,
            )
            # ── 新增：订阅检测结果 ──
            self.detection_sub = self.create_subscription(
                String,
                '/detection_info',
                self.detection_cb,
                10,
            )

            # 定时器：处理GUI发来的指令（从队列读），发布ROS话题
            self.cmd_timer = self.create_timer(0.05, self.process_commands)
            # 定时器：获取TF丢入队列
            self.tf_timer = self.create_timer(0.2, self.tf_timer_cb)

        def status_cb(self, msg):
            status_queue.put(msg.data)

        def image_cb(self, msg):
            """收到相机帧，解码后丢入 image_queue（只保留最新帧）。"""
            try:
                image_dict = {
                    'height': msg.height,
                    'width': msg.width,
                    'encoding': msg.encoding,
                    'step': msg.step,
                    'data': list(msg.data),  # 转为 list 以便跨进程序列化
                }
                # 清空队列，只留最新一帧
                while not image_queue.empty():
                    try:
                        image_queue.get_nowait()
                    except Exception:
                        break
                image_queue.put(image_dict)
            except Exception:
                pass

        def detection_cb(self, msg):
            """收到检测信息，丢入 detection_queue（只保留最新）。"""
            try:
                # 清空队列，只留最新
                while not detection_queue.empty():
                    try:
                        detection_queue.get_nowait()
                    except Exception:
                        break
                detection_queue.put(msg.data)
            except Exception:
                pass

        def tf_timer_cb(self):
            try:
                trans = self.tf_buffer.lookup_transform('base_link', 'link6', rclpy.time.Time())
                pose_data = {
                    'x': trans.transform.translation.x,
                    'y': trans.transform.translation.y,
                    'z': trans.transform.translation.z,
                }
                while not pose_queue.empty():
                    pose_queue.get_nowait()
                pose_queue.put(pose_data)
            except Exception:
                pass

        def process_commands(self):
            while not cmd_queue.empty():
                cmd_data = cmd_queue.get()
                msg = String()
                msg.data = json.dumps(cmd_data)
                self.cmd_pub.publish(msg)

    rclpy.init()
    node = HeadlessROSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


class SortingApp(tk.Tk):
    def __init__(self, cmd_queue, status_queue, pose_queue, image_queue, detection_queue):
        super().__init__()
        self.cmd_queue = cmd_queue
        self.status_queue = status_queue
        self.pose_queue = pose_queue
        self.image_queue = image_queue
        self.detection_queue = detection_queue

        self.title("机械臂分拣控制面板")
        self.geometry("1350x680")
        self.configure(padx=10, pady=10)

        self.saved_poses_file = '/home/lxf/agx_arm_ws/saved_poses.json'
        self.poses = {}
        self.load_poses()
        self.current_status = 'idle'

        self.task_queue = []
        self.last_dispatch_time = 0
        self.queue_running = False

        # ── 检测相关状态 ──
        self.latest_detection = None      # 最新的 /detection_info 解析结果
        self.camera_photo = None           # 当前相机帧 PhotoImage 引用（必须持有引用防止GC）
        self.default_grasp_roll = tk.DoubleVar(value=180.0)
        self.default_grasp_pitch = tk.DoubleVar(value=0.0)
        self.default_grasp_yaw = tk.DoubleVar(value=0.0)

        self.setup_ui()
        self._update_status_loop()
        self._update_camera_loop()
        self._update_detection_loop()

    # ═══════════════════════════════════════════════
    # 坐标持久化
    # ═══════════════════════════════════════════════
    def load_poses(self):
        if os.path.exists(self.saved_poses_file):
            try:
                with open(self.saved_poses_file, 'r') as f:
                    raw = json.load(f)
                    self.poses = {
                        name: {
                            'x': float(pose['x']),
                            'y': float(pose['y']),
                            'z': float(pose['z']),
                        }
                        for name, pose in raw.items()
                        if isinstance(pose, dict) and all(k in pose for k in ('x', 'y', 'z'))
                    }
            except:
                self.poses = {}
        else:
            self.poses = {}

    def save_poses(self):
        with open(self.saved_poses_file, 'w') as f:
            json.dump(self.poses, f, indent=4)
        self.update_comboboxes()

    # ═══════════════════════════════════════════════
    # UI 布局
    # ═══════════════════════════════════════════════
    def setup_ui(self):
        # ── 三栏布局 ──
        # 左栏：相机画面 + 检测信息
        self.left_frame = tk.Frame(self, width=420)
        self.left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        self.left_frame.pack_propagate(False)  # 固定宽度不随内容伸缩

        # 中栏：任务队列
        self.mid_frame = tk.Frame(self, width=240)
        self.mid_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        self.mid_frame.pack_propagate(False)

        # 右栏：控制区
        self.right_frame = tk.Frame(self)
        self.right_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # ──── 左栏：相机 + 检测 ────
        self._build_camera_panel()
        self._build_detection_panel()

        # ──── 中栏：任务队列 ────
        self._build_queue_panel()

        # ──── 右栏：控制区 ────
        self._build_control_panel()

    def _build_camera_panel(self):
        """左栏上部：实时相机画面。"""
        cam_frame = tk.LabelFrame(self.left_frame, text="📷 相机实时画面", padx=5, pady=5)
        cam_frame.pack(fill=tk.X, pady=(0, 5))

        self.camera_canvas = tk.Canvas(cam_frame, width=400, height=300, bg='black')
        self.camera_canvas.pack()

        self.camera_info_label = tk.Label(
            cam_frame, text="等待画面...", font=("Arial", 9), fg="gray"
        )
        self.camera_info_label.pack(pady=(2, 0))

    def _build_detection_panel(self):
        """左栏下部：检测信息 + 抓取姿态 + 夹取按钮。"""
        det_frame = tk.LabelFrame(self.left_frame, text="📦 检测信息", padx=5, pady=5)
        det_frame.pack(fill=tk.BOTH, expand=True)

        # 检测状态指示
        self.det_status_var = tk.StringVar(value="⚪ 等待检测数据...")
        tk.Label(
            det_frame, textvariable=self.det_status_var,
            font=("Arial", 12, "bold"), fg="gray"
        ).pack(anchor=tk.W, pady=(0, 5))

        # 物体信息
        self.det_object_var = tk.StringVar(value="物体: --")
        self.det_confidence_var = tk.StringVar(value="置信度: --")
        self.det_method_var = tk.StringVar(value="方法: --")
        tk.Label(det_frame, textvariable=self.det_object_var, font=("Arial", 10)).pack(anchor=tk.W)
        tk.Label(det_frame, textvariable=self.det_confidence_var, font=("Arial", 10)).pack(anchor=tk.W)
        tk.Label(det_frame, textvariable=self.det_method_var, font=("Arial", 10)).pack(anchor=tk.W)

        # 分隔
        ttk.Separator(det_frame, orient='horizontal').pack(fill=tk.X, pady=5)

        # base_link 坐标系下的坐标
        tk.Label(det_frame, text="基座坐标 (base_link):", font=("Arial", 10, "bold")).pack(anchor=tk.W)
        self.det_x_var = tk.StringVar(value="  X: --")
        self.det_y_var = tk.StringVar(value="  Y: --")
        self.det_z_var = tk.StringVar(value="  Z: --")
        tk.Label(det_frame, textvariable=self.det_x_var, font=("Arial", 10, "italic")).pack(anchor=tk.W)
        tk.Label(det_frame, textvariable=self.det_y_var, font=("Arial", 10, "italic")).pack(anchor=tk.W)
        tk.Label(det_frame, textvariable=self.det_z_var, font=("Arial", 10, "italic")).pack(anchor=tk.W)

        # 附加信息
        self.det_diameter_var = tk.StringVar(value="直径: --")
        self.det_distance_var = tk.StringVar(value="距离: --")
        tk.Label(det_frame, textvariable=self.det_diameter_var, font=("Arial", 10)).pack(anchor=tk.W)
        tk.Label(det_frame, textvariable=self.det_distance_var, font=("Arial", 10)).pack(anchor=tk.W)

        # 分隔
        ttk.Separator(det_frame, orient='horizontal').pack(fill=tk.X, pady=5)

        # 抓取姿态（欧拉角，可调）—— 当检测坐标只有 x/y/z 时用于补齐四元数
        tk.Label(det_frame, text="抓取姿态 (欧拉角 度):", font=("Arial", 10, "bold")).pack(anchor=tk.W)

        rpy_frame = tk.Frame(det_frame)
        rpy_frame.pack(fill=tk.X, pady=2)
        tk.Label(rpy_frame, text="Roll:", font=("Arial", 9)).pack(side=tk.LEFT)
        tk.Entry(rpy_frame, textvariable=self.default_grasp_roll, width=6,
                 font=("Arial", 9)).pack(side=tk.LEFT, padx=(2, 8))
        tk.Label(rpy_frame, text="Pitch:", font=("Arial", 9)).pack(side=tk.LEFT)
        tk.Entry(rpy_frame, textvariable=self.default_grasp_pitch, width=6,
                 font=("Arial", 9)).pack(side=tk.LEFT, padx=(2, 8))
        tk.Label(rpy_frame, text="Yaw:", font=("Arial", 9)).pack(side=tk.LEFT)
        tk.Entry(rpy_frame, textvariable=self.default_grasp_yaw, width=6,
                 font=("Arial", 9)).pack(side=tk.LEFT, padx=(2, 0))

        # ── 夹取按钮 ──
        ttk.Separator(det_frame, orient='horizontal').pack(fill=tk.X, pady=8)

        self.pick_from_det_btn = tk.Button(
            det_frame,
            text="📌 从检测坐标夹取 →",
            command=self._pick_from_detection,
            font=("Arial", 12, "bold"),
            bg="#4CAF50", fg="white",
            state=tk.DISABLED,
        )
        self.pick_from_det_btn.pack(fill=tk.X, pady=(0, 2))

        tk.Label(
            det_frame,
            text="使用上方检测坐标作为 Pick 点\n配合右侧选择的 Place 点执行分拣",
            font=("Arial", 8), fg="gray"
        ).pack()

    def _build_queue_panel(self):
        """中栏：任务排队队列。"""
        tk.Label(self.mid_frame, text="任务排队队列", font=("Arial", 14, "bold")).pack(pady=(0, 10))

        listbox_frame = tk.Frame(self.mid_frame)
        listbox_frame.pack(fill=tk.BOTH, expand=True)

        self.queue_listbox = tk.Listbox(listbox_frame, font=("Arial", 11), selectmode=tk.SINGLE)
        self.queue_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        scrollbar = tk.Scrollbar(listbox_frame, orient="vertical", command=self.queue_listbox.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.queue_listbox.config(yscrollcommand=scrollbar.set)

        btn_frame = tk.Frame(self.mid_frame)
        btn_frame.pack(fill=tk.X, pady=10)
        tk.Button(btn_frame, text="删除选定任务", command=self.remove_from_queue, width=12).pack(side=tk.LEFT, padx=(0, 5))
        tk.Button(btn_frame, text="清空队列", command=self.clear_queue, width=12, fg="red").pack(side=tk.RIGHT)

        # 队列执行控制
        exec_frame = tk.Frame(self.mid_frame)
        exec_frame.pack(fill=tk.X, pady=5)
        self.queue_run_btn = tk.Button(exec_frame, text="▶ 开始执行排队任务", command=self.toggle_queue_execution,
                                       bg="lightgreen", font=("Arial", 12, "bold"))
        self.queue_run_btn.pack(fill=tk.X)

    def _build_control_panel(self):
        """右栏：控制区（原样保留）。"""
        # 1. 状态显示
        self.status_var = tk.StringVar(value="当前状态: 未知")
        self.status_label = tk.Label(self.right_frame, textvariable=self.status_var,
                                     font=("Arial", 16, "bold"), fg="blue")
        self.status_label.pack(pady=(0, 10))

        # 2. 坐标录入区
        frame_new = tk.LabelFrame(self.right_frame, text="新建与保存坐标 (请先将机械臂拖动到位)", padx=10, pady=10)
        frame_new.pack(fill="x", pady=5)

        tk.Label(frame_new, text="坐标名称:").grid(row=0, column=0, padx=5)
        self.name_entry = tk.Entry(frame_new, width=15)
        self.name_entry.grid(row=0, column=1, padx=5)

        tk.Button(frame_new, text="读取机械臂当前位置", command=self.fetch_current_pose).grid(row=0, column=2, padx=10)

        self.pose_text_var = tk.StringVar(value="请点击读取位置...")
        tk.Entry(frame_new, textvariable=self.pose_text_var, state='readonly', width=50).grid(row=1, column=0, columnspan=3, pady=10)
        tk.Button(frame_new, text="保存该坐标", command=self.save_current_pose, bg="lightblue").grid(row=1, column=3, padx=10)

        # 3. 坐标管理区
        frame_manage = tk.LabelFrame(self.right_frame, text="坐标管理", padx=10, pady=5)
        frame_manage.pack(fill="x", pady=5)
        tk.Label(frame_manage, text="选择已存坐标:").grid(row=0, column=0, padx=5)
        self.del_combo = ttk.Combobox(frame_manage, state="readonly", width=15)
        self.del_combo.grid(row=0, column=1, padx=5)
        tk.Button(frame_manage, text="❌ 删除选中坐标", command=self.delete_pose, fg="red").grid(row=0, column=2, padx=10)

        # 4. 分拣任务下发区
        frame_task = tk.LabelFrame(self.right_frame, text="分拣任务下发", padx=10, pady=10)
        frame_task.pack(fill="x", pady=5)

        tk.Label(frame_task, text="夹取点 (Pick):").grid(row=0, column=0, pady=10)
        self.pick_combo = ttk.Combobox(frame_task, state="readonly", width=15)
        self.pick_combo.grid(row=0, column=1, padx=10)

        tk.Label(frame_task, text="放置点 (Place):").grid(row=0, column=2, pady=10)
        self.place_combo = ttk.Combobox(frame_task, state="readonly", width=15)
        self.place_combo.grid(row=0, column=3, padx=10)

        tk.Button(frame_task, text="添加至分拣队列", command=self.add_to_queue,
                  font=("Arial", 11, "bold"), bg="lightblue", width=18).grid(row=1, column=0, columnspan=2, pady=10)
        tk.Button(frame_task, text="立即发送分拣指令", command=self.send_sort,
                  font=("Arial", 11, "bold"), bg="lightgreen", width=18).grid(row=1, column=2, columnspan=2, pady=10)

        self.update_comboboxes()

        # 5. 操作区
        frame_action = tk.Frame(self.right_frame)
        frame_action.pack(pady=10)
        tk.Button(frame_action, text="一键回到待机点并关闭夹爪", command=self.send_reset,
                  font=("Arial", 12, "bold"), bg="orange", width=40).pack(pady=5)
        tk.Button(frame_action, text="退出服务端系统 (复位并关闭夹爪)", command=self.send_quit,
                  font=("Arial", 12, "bold"), bg="tomato", fg="white", width=40).pack(pady=5)

    # ═══════════════════════════════════════════════
    # 逗号框管理
    # ═══════════════════════════════════════════════
    def update_comboboxes(self):
        names = list(self.poses.keys())
        self.pick_combo['values'] = names
        self.place_combo['values'] = names
        if hasattr(self, 'del_combo'):
            self.del_combo['values'] = names

        if names:
            if not self.pick_combo.get() and names:
                self.pick_combo.current(0)
            if not self.place_combo.get() and names:
                self.place_combo.current(0)
            if hasattr(self, 'del_combo') and not self.del_combo.get():
                self.del_combo.current(0)
        else:
            self.pick_combo.set('')
            self.place_combo.set('')
            if hasattr(self, 'del_combo'):
                self.del_combo.set('')

    # ═══════════════════════════════════════════════
    # 相机画面更新循环
    # ═══════════════════════════════════════════════
    def _update_camera_loop(self):
        """每 ~66ms 拉取最新相机帧并渲染到 Canvas。"""
        try:
            latest_image = None
            while not self.image_queue.empty():
                latest_image = self.image_queue.get_nowait()

            if latest_image is not None:
                from PIL import Image as PILImage
                from PIL import ImageTk

                png_bytes = decode_sensor_image(latest_image)
                pil_img = PILImage.open(io.BytesIO(png_bytes))

                # 缩放到 Canvas 尺寸 (400x300)
                pil_img.thumbnail((400, 300), PILImage.LANCZOS)

                self.camera_photo = ImageTk.PhotoImage(pil_img)
                self.camera_canvas.delete("all")
                self.camera_canvas.create_image(
                    200, 150, image=self.camera_photo, anchor=tk.CENTER
                )

                w, h = pil_img.size
                orig_w, orig_h = latest_image['width'], latest_image['height']
                self.camera_info_label.config(
                    text=f"画面: {orig_w}x{orig_h} → 显示 {w}x{h} | 编码: {latest_image['encoding']}"
                )
        except Exception:
            pass

        self.after(66, self._update_camera_loop)

    # ═══════════════════════════════════════════════
    # 检测信息更新循环
    # ═══════════════════════════════════════════════
    def _update_detection_loop(self):
        """每 ~200ms 拉取最新检测信息并更新面板。"""
        try:
            latest_raw = None
            while not self.detection_queue.empty():
                latest_raw = self.detection_queue.get_nowait()

            if latest_raw is not None:
                data = json.loads(latest_raw)
                self.latest_detection = data

                detected = data.get('detected', False)
                if detected:
                    self.det_status_var.set("🟢 已检测到物体")
                    # 更新按钮状态
                    if hasattr(self, 'pick_from_det_btn'):
                        self.pick_from_det_btn.config(state=tk.NORMAL, bg="#4CAF50")

                    obj_name = data.get('object_name', '--')
                    confidence = data.get('confidence', 0.0)
                    method = data.get('method', '--')

                    self.det_object_var.set(f"物体: {obj_name}")
                    self.det_confidence_var.set(f"置信度: {confidence * 100:.2f}%")
                    self.det_method_var.set(f"方法: {method}")

                    # 基座坐标 (base_position_m)
                    base_pos = data.get('base_position_m', {})
                    if base_pos and all(k in base_pos for k in ('x', 'y', 'z')):
                        self.det_x_var.set(f"  X: {base_pos['x']:.3f} m")
                        self.det_y_var.set(f"  Y: {base_pos['y']:.3f} m")
                        self.det_z_var.set(f"  Z: {base_pos['z']:.3f} m")
                    else:
                        self.det_x_var.set("  X: -- (无 base_position_m)")
                        self.det_y_var.set("  Y: --")
                        self.det_z_var.set("  Z: --")

                    # 尺寸
                    size_m = data.get('size_m', {})
                    diameter = size_m.get('diameter', None)
                    if diameter is not None:
                        self.det_diameter_var.set(f"直径: {diameter:.3f} m")
                    else:
                        self.det_diameter_var.set("直径: --")

                    # 距离
                    distance = data.get('distance_to_robot_m', None)
                    if distance is not None:
                        self.det_distance_var.set(f"距离: {distance:.3f} m")
                    else:
                        self.det_distance_var.set("距离: --")
                else:
                    self.det_status_var.set("🔴 未检测到物体")
                    self.det_object_var.set("物体: --")
                    self.det_confidence_var.set("置信度: --")
                    self.det_method_var.set("方法: --")
                    self.det_x_var.set("  X: --")
                    self.det_y_var.set("  Y: --")
                    self.det_z_var.set("  Z: --")
                    self.det_diameter_var.set("直径: --")
                    self.det_distance_var.set("距离: --")
                    if hasattr(self, 'pick_from_det_btn'):
                        self.pick_from_det_btn.config(state=tk.DISABLED, bg="gray")
        except Exception:
            pass

        self.after(200, self._update_detection_loop)

    # ═══════════════════════════════════════════════
    # 从检测坐标生成分拣指令
    # ═══════════════════════════════════════════════
    def _pick_from_detection(self):
        """使用 /detection_info 中的 base_position_m 作为 Pick 点，执行分拣。"""
        # 1. 检查是否有有效检测
        if self.latest_detection is None:
            messagebox.showwarning("无检测数据", "尚未收到任何检测信息，请等待检测节点发布数据。")
            return

        data = self.latest_detection
        if not data.get('detected', False):
            messagebox.showwarning("未检测到物体", "当前检测信息显示未发现物体，无法执行夹取。")
            return

        base_pos = data.get('base_position_m', {})
        if not base_pos or not all(k in base_pos for k in ('x', 'y', 'z')):
            messagebox.showerror("坐标缺失", "检测数据中缺少 base_position_m，无法获取目标坐标。")
            return

        # 2. 检查 Place 点
        place_name = self.place_combo.get()
        if not place_name or place_name not in self.poses:
            messagebox.showerror("错误", "请在右侧选择有效的放置点 (Place)！")
            return

        # 3. 组装 Pick 坐标（补齐四元数）
        try:
            roll = float(self.default_grasp_roll.get())
            pitch = float(self.default_grasp_pitch.get())
            yaw = float(self.default_grasp_yaw.get())
        except ValueError:
            messagebox.showerror("姿态错误", "抓取姿态 (R/P/Y) 必须是有效数字。")
            return

        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

        pick_pose = {
            'x': round(float(base_pos['x']), 3),
            'y': round(float(base_pos['y']), 3),
            'z': round(float(base_pos['z']), 3),
            'qx': round(qx, 6),
            'qy': round(qy, 6),
            'qz': round(qz, 6),
            'qw': round(qw, 6),
        }

        # 4. 物体直径
        size_m = data.get('size_m', {})
        object_diameter_m = size_m.get('diameter', None)
        if object_diameter_m is not None:
            object_diameter_m = round(float(object_diameter_m), 3)

        # 5. 确认
        obj_name = data.get('object_name', '物体')
        pick_desc = f"{obj_name} (detected)"
        place_desc = place_name

        confirm_msg = (
            f"检测坐标夹取确认\n\n"
            f"Pick: {pick_desc}\n"
            f"  坐标: ({pick_pose['x']:.3f}, {pick_pose['y']:.3f}, {pick_pose['z']:.3f})\n"
            f"  姿态: R={roll}° P={pitch}° Y={yaw}°\n\n"
            f"Place: {place_desc}\n"
            f"  坐标: ({self.poses[place_name]['x']:.3f}, "
            f"{self.poses[place_name]['y']:.3f}, "
            f"{self.poses[place_name]['z']:.3f})\n\n"
            f"{'物体直径: ' + str(object_diameter_m) + ' m' if object_diameter_m else '无物体直径信息'}\n\n"
            f"确定发送此分拣指令？"
        )

        if not messagebox.askyesno("确认检测坐标夹取", confirm_msg):
            return

        # 6. 组装 sort 指令
        place_qx, place_qy, place_qz, place_qw = euler_to_quaternion(180.0, 0.0, 0.0)
        place_pose = {
            'x': round(float(self.poses[place_name]['x']), 3),
            'y': round(float(self.poses[place_name]['y']), 3),
            'z': round(float(self.poses[place_name]['z']), 3),
            'qx': round(place_qx, 6),
            'qy': round(place_qy, 6),
            'qz': round(place_qz, 6),
            'qw': round(place_qw, 6),
        }

        sort_cmd = {
            "cmd": "sort",
            "pick": pick_pose,
            "place": place_pose,
            "pick_name": pick_desc,
            "place_name": place_desc,
        }
        if object_diameter_m is not None:
            sort_cmd["object_diameter_m"] = float(object_diameter_m)

        if self.current_status == 'busy':
            answer = messagebox.askyesno(
                "服务端忙碌",
                "机械臂正在执行其他动作。\n\n选「是」将指令加入排队队列\n选「否」取消操作"
            )
            if not answer:
                return
            self.task_queue.append(sort_cmd)
            self.refresh_queue_listbox()
            messagebox.showinfo("已加入队列", f"已将该检测夹取任务加入排队队列。")
        else:
            self.last_dispatch_time = time.time()
            self.cmd_queue.put(sort_cmd)
            messagebox.showinfo("指令已发送", f"检测坐标夹取指令发送成功！\n\n从检测物体 → 到【{place_name}】")

    # ═══════════════════════════════════════════════
    # 原有功能 (不变)
    # ═══════════════════════════════════════════════
    def fetch_current_pose(self):
        pose = None
        while not self.pose_queue.empty():
            pose = self.pose_queue.get_nowait()

        if pose:
            self.current_fetched_pose = pose
            pos_str = f"x:{pose['x']:.2f}, y:{pose['y']:.2f}, z:{pose['z']:.2f}"
            self.pose_text_var.set(f"已读取: {pos_str}")
            messagebox.showinfo("读取成功", "成功读取了机械臂当前的法兰盘坐标！")
        else:
            messagebox.showwarning("警告", "最新TF位姿尚未准备好或无法获取（确保后台在跑，并且机械臂状态正常）。")

    def save_current_pose(self):
        name = self.name_entry.get().strip()
        if not name:
            messagebox.showerror("错误", "请输入坐标名称！")
            return
        if not hasattr(self, 'current_fetched_pose'):
            messagebox.showerror("错误", "请先点击【读取机械臂当前位置】！")
            return

        self.poses[name] = {
            'x': float(self.current_fetched_pose['x']),
            'y': float(self.current_fetched_pose['y']),
            'z': float(self.current_fetched_pose['z']),
        }
        self.save_poses()
        messagebox.showinfo("成功", f"坐标 '{name}' 已经保存！")

    def delete_pose(self):
        name = self.del_combo.get()
        if not name:
            messagebox.showerror("错误", "当前没有选择或可删除的坐标！")
            return
        answer = messagebox.askyesno("确认删除", f"您确定要永久删除坐标 '{name}' 吗？")
        if answer:
            del self.poses[name]
            self.save_poses()
            messagebox.showinfo("成功", f"坐标 '{name}' 已经成功删除！")

    def send_reset(self):
        if self.current_status == 'busy':
            messagebox.showwarning("服务端忙碌", "机械臂正在执行其他动作，请稍后再试！")
            return
        answer = messagebox.askyesno("确认一键复位", "这将让机械臂立刻回到待机位并关闭夹爪，等待下一次指令。\n确定要执行吗？")
        if answer:
            data = {"cmd": "reset"}
            self.cmd_queue.put(data)
            self.clear_queue()
            messagebox.showinfo("指令已发送", "一键回待机指令发送成功！")

    def add_to_queue(self):
        pick_name = self.pick_combo.get()
        place_name = self.place_combo.get()
        if not pick_name or not place_name:
            messagebox.showerror("错误", "请选择完整的夹取点和放置点！")
            return
        task = {
            "cmd": "sort",
            "pick": self.poses[pick_name],
            "place": self.poses[place_name],
            "pick_name": pick_name,
            "place_name": place_name
        }
        self.task_queue.append(task)
        self.refresh_queue_listbox()

    def remove_from_queue(self):
        selection = self.queue_listbox.curselection()
        if not selection:
            messagebox.showwarning("提示", "请先在左侧列表中点击选中要删除的任务！")
            return
        index = selection[0]
        del self.task_queue[index]
        self.refresh_queue_listbox()

    def clear_queue(self):
        self.task_queue.clear()
        self.refresh_queue_listbox()
        self.queue_running = False
        if hasattr(self, 'queue_run_btn'):
            self.queue_run_btn.config(text="▶ 开始执行排队任务", bg="lightgreen")

    def toggle_queue_execution(self):
        if not self.queue_running:
            if not self.task_queue:
                messagebox.showwarning("提示", "排队队列为空，请先在右侧【分拣任务下发】区域添加任务后再执行！")
                return
            self.queue_running = True
            self.queue_run_btn.config(text="⏸ 暂停排队执行", bg="yellow")
            self.last_dispatch_time = time.time() - 2.0
        else:
            self.queue_running = False
            self.queue_run_btn.config(text="▶ 开始执行排队任务", bg="lightgreen")

    def refresh_queue_listbox(self):
        self.queue_listbox.delete(0, tk.END)
        for i, task in enumerate(self.task_queue):
            text = f"{i+1}. {task['pick_name']} -> {task['place_name']}"
            self.queue_listbox.insert(tk.END, text)

    def send_sort(self):
        if self.current_status == 'busy':
            messagebox.showwarning("服务端忙碌", "机械臂正在执行其他动作，请稍后再试！")
            return

        pick_name = self.pick_combo.get()
        place_name = self.place_combo.get()
        if not pick_name or not place_name:
            messagebox.showerror("错误", "请选择完整的夹取点和放置点！")
            return

        data = {
            "cmd": "sort",
            "pick": self.poses[pick_name],
            "place": self.poses[place_name],
            "pick_name": pick_name,
            "place_name": place_name
        }
        self.last_dispatch_time = time.time()
        self.cmd_queue.put(data)
        messagebox.showinfo("指令已发送", f"直接执行分拣指令发送成功！\n\n从【{pick_name}】 -> 到【{place_name}】\n\n注意：如果队列中有任务，将在该指令完成后继续执行。")

    def send_quit(self):
        answer = messagebox.askyesno("确认退出", "这将让服务端回到待机位、关闭夹爪并结束程序。\n确定要退出吗？")
        if answer:
            data = {"cmd": "quit"}
            self.cmd_queue.put(data)
            self.after(500, self.destroy)

    def _update_status_loop(self):
        while not self.status_queue.empty():
            self.current_status = self.status_queue.get_nowait()

        status = self.current_status
        if status == 'idle':
            self.status_var.set("当前服务器状态: 空闲 (Idle)")
            self.status_label.configure(fg="green")
            self.error_notified = False

            if getattr(self, 'queue_running', False):
                if self.task_queue and (time.time() - self.last_dispatch_time > 1.5):
                    next_task = self.task_queue.pop(0)
                    self.refresh_queue_listbox()
                    self.cmd_queue.put(next_task)
                    self.last_dispatch_time = time.time()
                elif not self.task_queue and (time.time() - self.last_dispatch_time > 1.5):
                    self.queue_running = False
                    if hasattr(self, 'queue_run_btn'):
                        self.queue_run_btn.config(text="▶ 开始执行排队任务", bg="lightgreen")
                    messagebox.showinfo("队列执行结束", "排队队列当中的所有分拣任务已全部执行完毕！")

        elif status == 'busy':
            self.status_var.set("当前服务器状态: 忙碌 (Busy)")
            self.status_label.configure(fg="red")
            self.error_notified = False
        elif status == 'error':
            self.status_var.set("当前服务器状态: 发生异常 (Error)")
            self.status_label.configure(fg="orange")
            if not getattr(self, 'error_notified', False):
                self.error_notified = True
                self.clear_queue()
                messagebox.showwarning("规划失败或干涉", "机械臂执行失败（可能是达到限位触发了防碰撞，错误码: 99999）\n服务端已自动张开夹爪、回到待机点并完成复位！任务队列已全部清空。")

        self.after(500, self._update_status_loop)


def main():
    # 使用多进程，完全分离 ROS2 和 Tkinter
    cmd_queue = multiprocessing.Queue()
    status_queue = multiprocessing.Queue()
    pose_queue = multiprocessing.Queue()
    image_queue = multiprocessing.Queue(maxsize=1)
    detection_queue = multiprocessing.Queue(maxsize=1)

    ros_process = multiprocessing.Process(
        target=ros_process_worker,
        args=(cmd_queue, status_queue, pose_queue, image_queue, detection_queue),
    )
    ros_process.start()

    app = SortingApp(cmd_queue, status_queue, pose_queue, image_queue, detection_queue)
    app.protocol("WM_DELETE_WINDOW", lambda: app.destroy())

    try:
        app.mainloop()
    finally:
        ros_process.terminate()
        ros_process.join(timeout=2.0)


if __name__ == "__main__":
    multiprocessing.set_start_method('spawn')
    main()
