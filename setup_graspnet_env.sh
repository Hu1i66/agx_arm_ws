#!/bin/bash
# ============================================================
# setup_graspnet_env.sh
# GraspNet 全链路环境变量配置 (每个终端 source 一次)
# ============================================================
# 用途: 统一 DDS 实现 (cyclonedds) + ROS2 工作空间
#       确保所有节点能相互发现 (与用户现有相机节点配置一致)
# ============================================================
# 用法: source /home/lxf/agx_arm_ws/setup_graspnet_env.sh
# ============================================================

# 1. ROS2 Humble 基础环境
source /opt/ros/humble/setup.bash

# 2. DDS 实现: 与用户现有相机节点保持一致 (cyclonedds + localhost 多播)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/lxf/ROS2-Gazebo-GO2/src/docker/cyclonedds.xml
export ROS_LOCALHOST_ONLY=0

# 3. agx_arm 工作空间 (提供 agx_arm_msgs, MoveIt2 配置等)
if [ -f /home/lxf/agx_arm_ws/install/setup.bash ]; then
    source /home/lxf/agx_arm_ws/install/setup.bash
fi

# 4. 验证
echo "✅ GraspNet 环境已加载:"
echo "   ROS_DISTRO=${ROS_DISTRO}"
echo "   RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "   CYCLONEDDS_URI=${CYCLONEDDS_URI}"
echo "   ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
