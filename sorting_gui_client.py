#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, messagebox
import json
import os
import multiprocessing
import time

def ros_process_worker(cmd_queue, status_queue, pose_queue):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    import tf2_ros
    
    class HeadlessROSNode(Node):
        def __init__(self):
            super().__init__('sorting_headless_client_node')
            self.cmd_pub = self.create_publisher(String, '/sorting_cmds', 10)
            self.status_sub = self.create_subscription(String, '/sorting_status', self.status_cb, 10)
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            
            # 定时器：处理GUI发来的指令（从队列读），发布ROS话题
            self.cmd_timer = self.create_timer(0.05, self.process_commands)
            # 定时器：获取TF丢入队列
            self.tf_timer = self.create_timer(0.2, self.tf_timer_cb)

        def status_cb(self, msg):
            status_queue.put(msg.data)

        def tf_timer_cb(self):
            try:
                trans = self.tf_buffer.lookup_transform('base_link', 'link6', rclpy.time.Time())
                pose_data = {
                    'x': trans.transform.translation.x,
                    'y': trans.transform.translation.y,
                    'z': trans.transform.translation.z,
                    'qx': trans.transform.rotation.x,
                    'qy': trans.transform.rotation.y,
                    'qz': trans.transform.rotation.z,
                    'qw': trans.transform.rotation.w
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
    def __init__(self, cmd_queue, status_queue, pose_queue):
        super().__init__()
        self.cmd_queue = cmd_queue
        self.status_queue = status_queue
        self.pose_queue = pose_queue
        
        self.title("机械臂分拣控制面板")
        self.geometry("950x600")
        self.configure(padx=20, pady=20)
        
        self.saved_poses_file = '/home/lxf/agx_arm_ws/saved_poses.json'
        self.poses = {}
        self.load_poses()
        self.current_status = 'idle'
        
        self.task_queue = []
        self.last_dispatch_time = 0
        self.queue_running = False
        
        self.setup_ui()
        self._update_status_loop()

    def load_poses(self):
        if os.path.exists(self.saved_poses_file):
            try:
                with open(self.saved_poses_file, 'r') as f:
                    self.poses = json.load(f)
            except:
                self.poses = {}
        else:
            self.poses = {}

    def save_poses(self):
        with open(self.saved_poses_file, 'w') as f:
            json.dump(self.poses, f, indent=4)
        self.update_comboboxes()

    def setup_ui(self):
        # 创建左右布局
        self.left_frame = tk.Frame(self, width=200)
        self.left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 20))
        
        self.right_frame = tk.Frame(self)
        self.right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # ========= 左侧：任务队列区 =========
        tk.Label(self.left_frame, text="任务排队队列", font=("Arial", 14, "bold")).pack(pady=(0, 10))
        
        listbox_frame = tk.Frame(self.left_frame)
        listbox_frame.pack(fill=tk.BOTH, expand=True)
        
        self.queue_listbox = tk.Listbox(listbox_frame, font=("Arial", 11), selectmode=tk.SINGLE)
        self.queue_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        scrollbar = tk.Scrollbar(listbox_frame, orient="vertical", command=self.queue_listbox.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.queue_listbox.config(yscrollcommand=scrollbar.set)
        
        btn_frame = tk.Frame(self.left_frame)
        btn_frame.pack(fill=tk.X, pady=10)
        tk.Button(btn_frame, text="删除选定任务", command=self.remove_from_queue, width=12).pack(side=tk.LEFT, padx=(0, 5))
        tk.Button(btn_frame, text="清空队列", command=self.clear_queue, width=12, fg="red").pack(side=tk.RIGHT)

        # ========= 新增控制队列执行区域 =========
        exec_frame = tk.Frame(self.left_frame)
        exec_frame.pack(fill=tk.X, pady=5)
        self.queue_run_btn = tk.Button(exec_frame, text="▶ 开始执行排队任务", command=self.toggle_queue_execution, bg="lightgreen", font=("Arial", 12, "bold"))
        self.queue_run_btn.pack(fill=tk.X)

        # ========= 右侧：控制区 =========
        # 1. 状态显示
        self.status_var = tk.StringVar(value="当前状态: 未知")
        self.status_label = tk.Label(self.right_frame, textvariable=self.status_var, font=("Arial", 16, "bold"), fg="blue")
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
        
        tk.Button(frame_task, text="添加至分拣队列", command=self.add_to_queue, font=("Arial", 11, "bold"), bg="lightblue", width=18).grid(row=1, column=0, columnspan=2, pady=10)
        tk.Button(frame_task, text="立即发送分拣指令", command=self.send_sort, font=("Arial", 11, "bold"), bg="lightgreen", width=18).grid(row=1, column=2, columnspan=2, pady=10)

        self.update_comboboxes()

        # 5. 操作区
        frame_action = tk.Frame(self.right_frame)
        frame_action.pack(pady=10)
        tk.Button(frame_action, text="一键回到待机点并关闭夹爪", command=self.send_reset, font=("Arial", 12, "bold"), bg="orange", width=40).pack(pady=5)
        tk.Button(frame_action, text="退出服务端系统 (复位并关闭夹爪)", command=self.send_quit, font=("Arial", 12, "bold"), bg="tomato", fg="white", width=40).pack(pady=5)

    def update_comboboxes(self):
        names = list(self.poses.keys())
        self.pick_combo['values'] = names
        self.place_combo['values'] = names
        if hasattr(self, 'del_combo'):
            self.del_combo['values'] = names

        if names:
            if not self.pick_combo.get() and names: self.pick_combo.current(0)
            if not self.place_combo.get() and names: self.place_combo.current(0)
            if hasattr(self, 'del_combo') and not self.del_combo.get(): self.del_combo.current(0)
        else:
            self.pick_combo.set('')
            self.place_combo.set('')
            if hasattr(self, 'del_combo'):
                self.del_combo.set('')

    def fetch_current_pose(self):
        pose = None
        while not self.pose_queue.empty():
            pose = self.pose_queue.get_nowait()
            
        if pose:
            self.current_fetched_pose = pose
            pos_str = f"x:{pose['x']:.2f}, y:{pose['y']:.2f}, z:{pose['z']:.2f}"
            self.pose_text_var.set(f"已读取: {pos_str} (姿态已保存)")
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
            
        self.poses[name] = self.current_fetched_pose
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
            self.clear_queue() # 复位时自动清空排队队列
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
            self.last_dispatch_time = time.time() - 2.0  # 使其立刻满足防抖进入发送逻辑
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
            # 等待命令发出
            self.after(500, self.destroy)

    def _update_status_loop(self):
        while not self.status_queue.empty():
            self.current_status = self.status_queue.get_nowait()
            
        status = self.current_status
        if status == 'idle':
            self.status_var.set("当前服务器状态: 空闲 (Idle)")
            self.status_label.configure(fg="green")
            self.error_notified = False
            
            # 自动处理任务队列逻辑 (防抖1.5秒防止服务端状态没切过去)
            if getattr(self, 'queue_running', False):
                if self.task_queue and (time.time() - self.last_dispatch_time > 1.5):
                    next_task = self.task_queue.pop(0)
                    self.refresh_queue_listbox()
                    self.cmd_queue.put(next_task)
                    self.last_dispatch_time = time.time()
                elif not self.task_queue and (time.time() - self.last_dispatch_time > 1.5):
                    # 队列空了且上个任务已经执行结束
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
                self.clear_queue() # 发生异常时清空任务队列
                messagebox.showwarning("规划失败或干涉", "机械臂执行失败（可能是达到限位触发了防碰撞，错误码: 99999）\n服务端已自动张开夹爪、回到待机点并完成复位！任务队列已全部清空。")
        
        self.after(500, self._update_status_loop)

def main():
    # 使用多进程，完全分离 ROS2 和 Tkinter
    cmd_queue = multiprocessing.Queue()
    status_queue = multiprocessing.Queue()
    pose_queue = multiprocessing.Queue()
    
    ros_process = multiprocessing.Process(target=ros_process_worker, args=(cmd_queue, status_queue, pose_queue))
    ros_process.start()
    
    app = SortingApp(cmd_queue, status_queue, pose_queue)
    app.protocol("WM_DELETE_WINDOW", lambda: app.destroy())
    
    try:
        app.mainloop()
    finally:
        ros_process.terminate()
        ros_process.join(timeout=2.0)

if __name__ == "__main__":
    multiprocessing.set_start_method('spawn')
    main()
