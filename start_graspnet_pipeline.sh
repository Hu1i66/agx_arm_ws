#!/bin/bash
# ============================================================
# start_graspnet_pipeline.sh — D455 + GraspNet + 机械臂分拣 前置检查与命令提示
#
# 本脚本只做**只读检查**并打印统一入口命令，不启动任何节点、
# **不自动激活 CAN**（CAN 激活涉及设备动作，由操作者显式执行）。
# ============================================================
# 前置条件（操作者确认）:
#   1. CAN 已激活: bash ~/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh can0 1000000 && cansend can0 123#1122334455667788 && ip -s link show can0
#   2. T1 前运行机械臂激活脚本: /usr/bin/python3 /home/lxf/agx_arm_ws/activate_robot.py
#   3. D455 相机已连接（固定安装在传送带上方）
#   4. 传送带串口已连接（/dev/ttyUSB*）
# ============================================================

set -u
set +u
source /opt/ros/humble/setup.bash
set -u
WS="/home/lxf/agx_arm_ws"
[ -f "${WS}/install/setup.bash" ] && { set +u; source "${WS}/install/setup.bash"; set -u; }

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
fail()  { echo -e "${RED}[FAIL]${NC} $1"; }

echo "============================================================"
echo "  GraspNet 全链路只读检查"
echo "============================================================"

# 1. CAN（只读：不自动激活）
if ip -details link show can0 2>/dev/null | grep -q "can"; then
    info "can0 接口存在；请核对 UP / bitrate 1000000，并确认已在 T1 前运行 activate_robot.py"
else
    warn "can0 未配置。请手动执行（本脚本不自动激活 CAN）:
      bash ~/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh can0 1000000 && cansend can0 123#1122334455667788 && ip -s link show can0"
fi

# 2. D455
if lsusb | grep -qi "realtech\|intel.*realsense\|8086:0b5"; then
    info "D455 已连接"
else
    warn "D455 相机未连接"
fi

# 3. 机械臂反馈（只读话题检查）
if timeout 2 ros2 topic info /feedback/tcp_pose 2>&1 | grep -q "Publisher count: [1-9]"; then
    info "/feedback/tcp_pose 有发布者"
else
    warn "/feedback/tcp_pose 无发布者。先完成 T0a CAN 激活、T0b activate_robot.py，再启动 T1；此话题检查不能确认激活指令是否执行。"
fi

# 4. 部署资源（统一入口 preflight 覆盖；此处快速检查关键项）
for f in "${WS}/models/yolo/fruits12.pt" "${WS}/models/graspnet/checkpoint-rs.tar" \
         "${WS}/config/calibration/d455_eye_to_hand.json" "${WS}/config/dds/cyclonedds.xml"; do
    if [ -f "$f" ]; then info "资源 OK: $f"; else fail "资源缺失: $f"; fi
done

echo ""
echo "============================================================"
echo "  先完成 T0 硬件激活，再在 7 个独立终端启动 ROS2"
echo "============================================================"
cat <<'EOF'
T0：连接机械臂并上电后，先在一个终端依次执行；每步成功后再继续：
  cd /home/lxf/agx_arm_ws
  bash ~/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh can0 1000000 && cansend can0 123#1122334455667788 && ip -s link show can0
  ip -details link show can0
  /usr/bin/python3 /home/lxf/agx_arm_ws/activate_robot.py
  等待激活脚本结束，再启动下面的 T1。

┌─ T1: MoveIt2 + 机械臂控制 ───────────────────────────────┐
│  cd /home/lxf/agx_arm_ws                                  │
│  ./scripts/run_component.sh arm                           │
└──────────────────────────────────────────────────────────┘
┌─ T2: D455 相机 (640x480@30, 滤波链, publish_tf=false) ───┐
│  ./scripts/run_component.sh camera --camera d455          │
└──────────────────────────────────────────────────────────┘
┌─ T2b: 相机 TF (眼在手外, base_link→camera, 读活动外参) ──┐
│  ./scripts/run_component.sh camera_tf                     │
└──────────────────────────────────────────────────────────┘
┌─ T3: 水果 YOLO 检测 ─────────────────────────────────────┐
│  ./scripts/run_component.sh detector --mode fruit         │
└──────────────────────────────────────────────────────────┘
┌─ T4: GraspNet 抓取位姿服务 (等日志出现"就绪, 等待请求") ─┐
│  ./scripts/run_component.sh graspnet                      │
└──────────────────────────────────────────────────────────┘
┌─ T5: 分拣动作执行节点 ───────────────────────────────────┐
│  ./scripts/run_component.sh action                        │
└──────────────────────────────────────────────────────────┘
┌─ T6: GUI 控制面板 ───────────────────────────────────────┐
│  ./scripts/run_component.sh gui                           │
└──────────────────────────────────────────────────────────┘
EOF

echo ""
echo "只读检查: ./scripts/run_component.sh preflight [--live]"
echo "TF 复核:  source scripts/runtime_env.sh && ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame"
