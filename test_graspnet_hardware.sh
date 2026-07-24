#!/bin/bash
# ============================================================
# test_graspnet_hardware.sh
# 阶段 7: D455 + GraspNet + 机械臂 端到端硬件测试辅助脚本
# ============================================================
# 用途:
#   1. 检查硬件就绪状态 (D455/CAN/继电器)
#   2. 检查 ROS topic/service 可用性
#   3. 提供 service 手动测试命令
#   4. 不启动任何节点, 仅做就绪性检查
# ============================================================

set -u
source /opt/ros/humble/setup.bash

GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; NC='\033[0m'
ok()   { echo -e "${GREEN}[✓]${NC} $1"; }
fail() { echo -e "${RED}[✗]${NC} $1"; }
warn() { echo -e "${YELLOW}[!]${NC} $1"; }

echo "============================================================"
echo "  阶段 7 硬件测试 - 就绪性检查"
echo "============================================================"

# ---------- 1. D455 相机 ----------
echo ""
echo "── 1. D455 相机 ──"
if lsusb | grep -qi "realtech\|intel.*realsense\|8086:0b5"; then
    ok "D455 相机已连接 (USB)"
else
    fail "D455 相机未检测到 (lsusb 无 RealSense 设备)"
fi
if ls /dev/video* 2>/dev/null | grep -q video; then
    warn "存在 /dev/video* 设备: $(ls /dev/video* 2>/dev/null | tr '\n' ' ')"
fi

# ---------- 2. CAN 接口 ----------
echo ""
echo "── 2. CAN 接口 (机械臂) ──"
if ip -details link show can0 2>/dev/null | grep -q "can"; then
    BITRATE=$(ip -details link show can0 2>/dev/null | grep -oP 'bitrate \K\d+' || echo "?")
    STATE=$(ip link show can0 2>/dev/null | grep -q "UP" && echo "UP" || echo "DOWN")
    ok "can0 已配置 (bitrate=${BITRATE}, state=${STATE})"
    if [ "$BITRATE" != "1000000" ]; then
        fail "bitrate 不是 1000000 (硬约束违反)"
    fi
else
    fail "can0 未配置 → 请运行: bash /home/lxf/piper_ws/src/piper_ros/can_activate.sh can0 1000000"
fi

# ---------- 3. CH340 继电器 ----------
echo ""
echo "── 3. CH340 继电器 (传送带) ──"
if ls /dev/ttyUSB* 2>/dev/null | grep -q ttyUSB; then
    ok "CH340 已连接: $(ls /dev/ttyUSB* 2>/dev/null | tr '\n' ' ')"
else
    fail "CH340 继电器未连接 (/dev/ttyUSB* 不存在)"
fi

# ---------- 4. ROS topic 检查 ----------
echo ""
echo "── 4. ROS Topic 检查 (需要节点已启动) ──"
check_topic() {
    local topic="$1" expected_type="$2"
    local info
    info=$(timeout 2 ros2 topic info "${topic}" 2>&1)
    if echo "${info}" | grep -q "Publisher count: 0"; then
        fail "${topic} - 无发布者"
    elif echo "${info}" | grep -q "Type:"; then
        local actual_type
        actual_type=$(echo "${info}" | grep "Type:" | awk '{print $2}')
        ok "${topic} (type=${actual_type}, publishers=$(echo "${info}" | grep -oP 'Publisher count: \K\d+'))"
    else
        fail "${topic} - 检查失败"
    fi
}

check_topic "/camera/camera/aligned_depth_to_color/image_raw" "sensor_msgs/msg/Image"
check_topic "/camera/camera/color/image_raw" "sensor_msgs/msg/Image"
check_topic "/camera/camera/color/camera_info" "sensor_msgs/msg/CameraInfo"
check_topic "/feedback/tcp_pose" "geometry_msgs/msg/PoseStamped"
check_topic "/detection_info" "std_msgs/msg/String"
check_topic "/sorting_cmds" "std_msgs/msg/String"

# ---------- 5. Service 检查 ----------
echo ""
echo "── 5. GraspNet Service 检查 ──"
SVC_INFO=$(timeout 2 ros2 service info /generate_grasp_pose 2>&1)
if echo "${SVC_INFO}" | grep -q "Type:"; then
    ok "service /generate_grasp_pose 已注册"
    echo "${SVC_INFO}"
else
    fail "service /generate_grasp_pose 未注册 (grasp_pose_node 未启动?)"
fi

# ---------- 6. MoveIt2 / move_group 检查 ----------
echo ""
echo "── 6. MoveIt2 检查 ──"
MG_NODES=$(ros2 node list 2>/dev/null | grep -E "move_group|move_action_server" || true)
if [ -n "${MG_NODES}" ]; then
    ok "MoveIt2 节点已启动:"
    echo "${MG_NODES}"
else
    fail "MoveIt2 move_group 未启动"
fi

# ---------- 7. 节点列表 ----------
echo ""
echo "── 7. 当前 ROS 节点列表 ──"
ros2 node list 2>/dev/null

# ---------- 8. 手动测试命令提示 ----------
echo ""
echo "============================================================"
echo "  手动测试命令"
echo "============================================================"
echo ""
echo "[A] 启动 grasp_pose_node (单独终端):"
echo "    cd /home/lxf/orange_dataset"
echo "    source .venv/bin/activate"
echo "    source /opt/ros/humble/setup.bash"
echo "    source /home/lxf/agx_arm_ws/install/setup.bash"
echo "    python3 grasp_pose_node.py"
echo ""
echo "[B] 测试 GraspNet service (空请求, 验证 service 响应):"
echo "    ros2 service call /generate_grasp_pose agx_arm_msgs/srv/GenerateGraspPose \\"
echo "      '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"base_link\"}, bbox: {x_offset: 100, y_offset: 100, width: 200, height: 200, do_rectify: false}, class_name: \"apple\", max_candidates: 5}'"
echo ""
echo "[C] 查看 GraspNet 可视化 marker:"
echo "    ros2 topic echo /grasp_pose_marker --once"
echo ""
echo "[D] 验证 detection_info 包含 header_stamp:"
echo "    ros2 topic echo /detection_info --once | python3 -c 'import sys,json; d=json.loads(sys.stdin.read().split(\"data: \")[1].strip()); print(\"header_stamp=\", d.get(\"header_stamp\"), \"method=\", d.get(\"method\"))'"
echo ""
echo "============================================================"
