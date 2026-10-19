#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
export AGX_ARM_WS="$ROOT"
source "$SCRIPT_DIR/runtime_env.sh" dual
cd "$ROOT"

/usr/bin/python3 - <<'PY'
import json
import runtime_launcher
for component in ("workbench-service", "tablet-gateway", "tablet-media"):
    result = runtime_launcher.ensure_background(component, camera="dual", initiator="workbench", wait_ready=False)
    print(f"{component}: {result.get('result')}")
PY

IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
printf '平板入口: https://%s:8443/\n' "${IP:-piper.local}"
printf '首次使用请在电脑管理页下载并信任本地 CA 证书。\n'
