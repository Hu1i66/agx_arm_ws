#!/bin/bash
# ============================================================
# run_component.sh — 用户统一入口（计划 3.3）
#
# 用法（每条在独立终端先 cd /home/lxf/agx_arm_ws）:
#   arm 前先执行（详见 README §2）:
#     bash ~/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh can0 1000000 && cansend can0 123#1122334455667788 && ip -s link show can0
#     /usr/bin/python3 /home/lxf/agx_arm_ws/activate_robot.py
#   ./scripts/run_component.sh preflight [--live]
#   ./scripts/run_component.sh arm
#   ./scripts/run_component.sh camera --camera d455
#   ./scripts/run_component.sh camera_tf
#   ./scripts/run_component.sh detector --mode fruit
#   ./scripts/run_component.sh detector --mode blue_block
#   ./scripts/run_component.sh graspnet
#   ./scripts/run_component.sh action
#   ./scripts/run_component.sh gui
#   ./scripts/run_component.sh camera --camera dabai
#   ./scripts/run_component.sh calibrate-intrinsics --camera d455
#   ./scripts/run_component.sh calibrate-intrinsics --camera dabai
#   ./scripts/run_component.sh verify-tcp --dry-run
#   ./scripts/run_component.sh verify-tcp -- --mode math
#   ./scripts/run_component.sh calibrate-tcp
#   ./scripts/run_component.sh calibrate-handeye --camera d455
#   ./scripts/run_component.sh calibrate-handeye --camera dabai
#   ./scripts/run_component.sh stop <目标组件> [--initiator gui] [--allow-cross-initiator]
#   ./scripts/run_component.sh status
#
# 所有启动入口支持 --dry-run（只输出解释器/argv/配置/关键环境）。
# 手动命令默认前台监督：转发 SIGINT/TERM、wait 回收、返回真实退出码。
# 校准命令保留终端 stdin/stdout。
# ============================================================

set -u
# ⚠️ source ROS setup.bash 时临时关闭 -u（setup.bash 内部引用未绑定变量）
set +u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
AGX_ARM_WS="${AGX_ARM_WS:-$(cd "$SCRIPT_DIR/.." && pwd)}"
export AGX_ARM_WS

# ---- 解析相机配置（决定 Orbbec overlay 是否加载）----
CAMERA_PROFILE="d455"
PREV=""
for arg in "$@"; do
    if [ "$PREV" = "--camera" ]; then
        CAMERA_PROFILE="$arg"
    fi
    PREV="$arg"
done

# ---- 加载共享环境（ROS + overlay + DDS）----
# shellcheck source=/dev/null
source "$SCRIPT_DIR/runtime_env.sh" "$CAMERA_PROFILE" || exit $?

set -u

# ---- 交给统一启动器（系统 Python，仅 stdlib + 子进程 ros2）----
exec /usr/bin/python3 "$AGX_ARM_WS/runtime_launcher.py" "$@"
