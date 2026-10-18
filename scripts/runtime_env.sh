#!/bin/bash
# ============================================================
# runtime_env.sh — 共享环境加载（被 run_component.sh 与各入口 source）
#
# 规则（计划 3.2）：
#   - D455 生产依次加载 /opt/ros/humble/setup.bash、主空间 install/setup.bash
#   - Dabai 工具额外在主空间 overlay 前加载 Orbbec overlay
#   - setup 后显式设置 RMW_IMPLEMENTATION 与项目内 CYCLONEDDS_URI
#   - ROS_DOMAIN_ID 保留调用者值；未设置按 ROS 默认 0
#   - 不 source ~/.bashrc；不全局 export PYTHONNOUSERSITE 或 IK 包路径
#   - 兼容调用者 set -u：临时关闭 -u，处理后恢复原 shell 选项
#
# 用法:
#   source scripts/runtime_env.sh [d455|dabai]
#   WITH_ORBBEC=1 source scripts/runtime_env.sh
# ============================================================

# ---- 兼容 set -u：记录并临时关闭 -u ----
_RUNTIME_ENV_HAD_U=0
case "$-" in *u*) _RUNTIME_ENV_HAD_U=1; set +u ;; esac

_runtime_env_restore_u() {
    if [ "$_RUNTIME_ENV_HAD_U" = "1" ]; then set -u; fi
    unset _RUNTIME_ENV_HAD_U
}

# ---- 根目录（按脚本位置，不依赖 cwd）----
_RUNTIME_ENV_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
AGX_ARM_WS="${AGX_ARM_WS:-$(cd "$_RUNTIME_ENV_SCRIPT_DIR/.." && pwd)}"
export AGX_ARM_WS

_RUNTIME_CAMERA_PROFILE="${1:-d455}"
if [ -n "${WITH_ORBBEC:-}" ] && [ "${WITH_ORBBEC}" != "0" ]; then
    _RUNTIME_CAMERA_PROFILE="dabai"
fi

# ---- ROS 与 overlay ----
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "[runtime_env][ERROR] /opt/ros/humble/setup.bash 不存在" >&2
    _runtime_env_restore_u
    return 1 2>/dev/null || exit 1
fi

# Dabai 工具：Orbbec overlay 在主空间 overlay 之前
if [ "$_RUNTIME_CAMERA_PROFILE" = "dabai" ] || [ "$_RUNTIME_CAMERA_PROFILE" = "dual" ]; then
    if [ -f /home/lxf/orbbec_ros2_ws/install/setup.bash ]; then
        source /home/lxf/orbbec_ros2_ws/install/setup.bash
    else
        echo "[runtime_env][WARN] Orbbec overlay 不存在: /home/lxf/orbbec_ros2_ws/install/setup.bash" >&2
    fi
fi

if [ -f "$AGX_ARM_WS/install/setup.bash" ]; then
    source "$AGX_ARM_WS/install/setup.bash"
else
    echo "[runtime_env][WARN] 主空间 overlay 不存在: $AGX_ARM_WS/install/setup.bash（标定包构建后生成）" >&2
fi

# ---- DDS：显式设置，与全项目一致 ----
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$AGX_ARM_WS/config/dds/cyclonedds.xml"
unset PYTHONNOUSERSITE  # Build-only flag must not suppress runtime user libraries.

# ROS_DOMAIN_ID 保留调用者值；未设置不主动覆盖（ROS 默认 0）
# 即：不 export ROS_DOMAIN_ID

# ---- 仅将主空间根加入 PYTHONPATH（供 launch/脚本定位 runtime_config 等）----
# 不注入 IK site-packages，不设置 PYTHONNOUSERSITE=1
case ":$PYTHONPATH:" in
    *":$AGX_ARM_WS:"*) ;;
    *) export PYTHONPATH="$AGX_ARM_WS${PYTHONPATH:+:$PYTHONPATH}" ;;
esac

unset _RUNTIME_CAMERA_PROFILE
unset _RUNTIME_ENV_SCRIPT_DIR
_runtime_env_restore_u
unset -f _runtime_env_restore_u
