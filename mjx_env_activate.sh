#!/usr/bin/env bash
set -e
cd /home/lxf/agx_arm_ws
source .venv_mjx/bin/activate
export PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple
export PIP_EXTRA_INDEX_URL=https://mirrors.aliyun.com/pypi/simple
echo "[mjx-env] activated: $VIRTUAL_ENV"
python -V
