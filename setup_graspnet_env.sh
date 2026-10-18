#!/bin/bash
# ============================================================
# setup_graspnet_env.sh — GraspNet 全链路环境（兼容共享 env）
#
# 每个终端 source 一次。优先复用统一入口环境 scripts/runtime_env.sh
# （ROS + 主空间 overlay + 项目内 DDS），否则回退到本脚本内置设置。
#
# 用法: source /home/lxf/agx_arm_ws/setup_graspnet_env.sh [d455|dabai]
# ============================================================
set +u   # source ROS setup.bash 内部引用未绑定变量
_WS="/home/lxf/agx_arm_ws"

if [ -f "${_WS}/scripts/runtime_env.sh" ]; then
    # 共享环境：ROS + (Dabai 时 Orbbec) overlay + 主空间 overlay
    #            + RMW_IMPLEMENTATION=rmw_cyclonedds_cpp + 项目内 CYCLONEDDS_URI
    source "${_WS}/scripts/runtime_env.sh" "${1:-d455}"
else
    source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file://"${_WS}/config/dds/cyclonedds.xml"
    if [ -f "${_WS}/install/setup.bash" ]; then
        source "${_WS}/install/setup.bash"
    fi
fi
export ROS_LOCALHOST_ONLY=0

echo "✅ GraspNet 环境已加载:"
echo "   ROS_DISTRO=${ROS_DISTRO}"
echo "   RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "   CYCLONEDDS_URI=${CYCLONEDDS_URI}"
echo "   ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
