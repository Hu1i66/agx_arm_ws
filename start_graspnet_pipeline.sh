#!/bin/bash
# ============================================================
# start_graspnet_pipeline.sh
# 一键启动 D455 + GraspNet + 机械臂分拣全链路 (眼在手外 eye-to-hand)
# ============================================================
# 前置条件:
#   1. CAN 已激活 (bash /home/lxf/piper_ws/src/piper_ros/can_activate.sh can0 1000000)
#   2. 机械臂已使能 (python3 /home/lxf/agx_arm_ws/activate_robot.py)
#   3. D455 相机已连接 (固定安装在传送带上方)
#   4. CH340 继电器已连接
# ============================================================
# 启动顺序 (在独立终端中执行):
#   T1: ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py
#   T2: ros2 launch realsense2_camera rs_launch.py ... publish_tf:=false (眼在手外: 禁用相机自带 TF)
#   T2b: ros2 launch eye_to_hand_camera_tf.launch.py (发布 base_link → camera_color_optical_frame 静态 TF)
#   T3: cd /home/lxf/orange_dataset && source .venv/bin/activate && python3 realsense_yolo_node.py --ros-args -p show_gui_window:=false
#   T4: cd /home/lxf/agx_arm_ws && source /home/lxf/orange_dataset/.venv/bin/activate && python3 graspnet_service_node.py
#   T5: cd /home/lxf/agx_arm_ws && python3 auto_sorting_action.py
#   T6: cd /home/lxf/agx_arm_ws && python3 sorting_gui_client.py
# ============================================================

set -u
# ⚠️ source ROS2 setup.bash 时临时关闭 -u: setup.bash 内部引用 AMENT_TRACE_SETUP_FILES
# 等未绑定变量, set -u 会误判为错误终止脚本
set +u
source /opt/ros/humble/setup.bash
source /home/lxf/agx_arm_ws/install/setup.bash
set -u

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }

# ---------- 前置检查 ----------
echo "============================================================"
echo "  GraspNet 全链路启动前置检查"
echo "============================================================"

# 1. CAN
if ! ip -details link show can0 2>/dev/null | grep -q "can"; then
    warn "can0 未配置, 尝试自动激活..."
    bash /home/lxf/piper_ws/src/piper_ros/can_activate.sh can0 1000000 || {
        echo "❌ CAN 激活失败, 请手动执行: bash /home/lxf/piper_ws/src/piper_ros/can_activate.sh can0 1000000"
        exit 1
    }
fi
info "can0 OK"

# 2. D455
if ! lsusb | grep -qi "realtech\|intel.*realsense\|8086:0b5"; then
    echo "❌ D455 相机未连接"
    exit 1
fi
info "D455 已连接"

# 3. 机械臂使能 (通过检查 /feedback/tcp_pose 是否有发布者)
set +u
source /opt/ros/humble/setup.bash
set -u
if ! timeout 2 ros2 topic info /feedback/tcp_pose 2>&1 | grep -q "Publisher count: [1-9]"; then
    warn "/feedback/tcp_pose 无发布者, 机械臂可能未使能"
    warn "请在新终端执行: python3 /home/lxf/agx_arm_ws/activate_robot.py"
fi

# 4. GraspNet checkpoint
CKPT="/home/lxf/graspnet/checkpoints/checkpoint-rs.tar"
if [ ! -f "${CKPT}" ]; then
    echo "❌ GraspNet checkpoint 不存在: ${CKPT}"
    exit 1
fi
info "GraspNet checkpoint OK ($(du -h ${CKPT} | cut -f1))"

# 5. 手眼标定数据 (eye-to-hand: 相机固定在传送带上方)
HANDEYE="/home/lxf/handeye/result/2026-07-26_16-17-46_calibration.json"
if [ ! -f "${HANDEYE}" ]; then
    echo "❌ 手眼标定数据不存在: ${HANDEYE}"
    exit 1
fi
info "手眼标定数据 OK (eye-to-hand)"

# 6. agx_arm_msgs 已编译
if [ ! -d "/home/lxf/agx_arm_ws/install/agx_arm_msgs/local/lib/python3.10/dist-packages/agx_arm_msgs" ]; then
    echo "❌ agx_arm_msgs 未编译"
    echo "   请执行: cd /home/lxf/agx_arm_ws && colcon build --packages-select agx_arm_msgs"
    exit 1
fi
info "agx_arm_msgs 已编译"

echo ""
echo "============================================================"
echo "  ✅ 前置检查全部通过, 请在 7 个独立终端中依次执行以下命令:"
echo "============================================================"
echo ""
echo "┌─ T1: MoveIt2 + 机械臂控制 ────────────────────────────────┐"
echo "│  source /opt/ros/humble/setup.bash && \\                  │"
echo "│  source /home/lxf/agx_arm_ws/install/setup.bash && \\     │"
echo "│  ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \\"
echo "│    can_port:=can0 arm_type:=piper effector_type:=agx_gripper │"
echo "└──────────────────────────────────────────────────────────┘"
echo ""
echo "┌─ T2: D455 相机 (align_depth=true, 禁用自带TF) ──────────┐"
echo "│  source /opt/ros/humble/setup.bash && \\                  │"
echo "│  ros2 launch realsense2_camera rs_launch.py \\            │"
echo "│    rgb_camera.color_profile:=640,480,30 \\                │"
echo "│    align_depth.enable:=true \\                            │"
echo "│    depth_module.profile:=640,480,30 \\                    │"
echo "│    publish_tf:=false                                     │"
echo "└──────────────────────────────────────────────────────────┘"
echo ""
echo "┌─ T2b: 相机 TF 发布器 (眼在手外, base_link→camera) ───────┐"
echo "│  source /opt/ros/humble/setup.bash && \\                  │"
echo "│  ros2 launch /home/lxf/agx_arm_ws/launch/eye_to_hand_camera_tf.launch.py │"
echo "└──────────────────────────────────────────────────────────┘"
echo ""
echo "┌─ T3: YOLO 检测节点 (D455 深度) ──────────────────────────┐"
echo "│  cd /home/lxf/orange_dataset && \\                        │"
echo "│  source .venv/bin/activate && \\                          │"
echo "│  source /opt/ros/humble/setup.bash && \\                  │"
echo "│  python3 realsense_yolo_node.py \\                        │"
echo "│    --ros-args -p show_gui_window:=false \\                │"
echo "│    -p use_depth_camera:=true                             │"
echo "└──────────────────────────────────────────────────────────┘"
echo ""
echo "┌─ T4: GraspNet 抓取位姿服务节点 ─────────────────────────┐"
echo "│  cd /home/lxf/agx_arm_ws && \\                            │"
echo "│  source /home/lxf/orange_dataset/.venv/bin/activate && \\ │"
echo "│  source /opt/ros/humble/setup.bash && \\                  │"
echo "│  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \\       │"
echo "│  export CYCLONEDDS_URI=file:///home/lxf/ROS2-Gazebo-GO2/src/docker/cyclonedds.xml && \\"
echo "│  python3 graspnet_service_node.py                        │"
echo "└──────────────────────────────────────────────────────────┘"
echo ""
echo "┌─ T5: 分拣动作执行节点 ──────────────────────────────────┐"
echo "│  cd /home/lxf/agx_arm_ws && \\                            │"
echo "│  source /opt/ros/humble/setup.bash && \\                  │"
echo "│  source install/setup.bash && \\                          │"
echo "│  python3 auto_sorting_action.py                          │"
echo "└──────────────────────────────────────────────────────────┘"
echo ""
echo "┌─ T6: GUI 控制面板 ──────────────────────────────────────┐"
echo "│  cd /home/lxf/agx_arm_ws && \\                            │"
echo "│  source /opt/ros/humble/setup.bash && \\                  │"
echo "│  source install/setup.bash && \\                          │"
echo "│  python3 sorting_gui_client.py                           │"
echo "└──────────────────────────────────────────────────────────┘"
echo ""
echo "============================================================"
echo "  阶段 7 验证流程"
echo "============================================================"
echo "1. 启动以上 7 个终端 (T1→T2→T2b→T3→T4→T5→T6 顺序)"
echo "2. 等 T3 输出 'YOLO 已上线', T4 输出 'graspnet_service_node 就绪, 等待请求...'"
echo "3. 在传送带放置物体 (苹果/橘子/香蕉等)"
echo "4. 在 GUI 选择放置料框 (料框1/料框2)"
echo "5. 点击橙色 'GraspNet抓取' 按钮触发 sort_graspnet 命令"
echo "6. 观察终端 T4/T5 日志:"
echo "   - T4: '📥 收到抓取请求', '✅ 推理完成: N 个候选', '✅ 返回位姿: t=(...)'"
echo "   - T5: '📡 已请求 GraspNet 服务', '🎯 GraspNet 位姿: t=(...) score=... width=...'"
echo "7. 验证机械臂用 GraspNet 6DoF 姿态成功抓取并放置到正确料框"
echo "8. 失败路径: 遮挡相机 → T5 应打印 '❌ GraspNet 失败 (...), 跳过本轮' 不崩溃"
echo ""
echo "============================================================"
echo "  测试辅助: 运行 test_graspnet_hardware.sh 检查就绪状态"
echo "============================================================"
