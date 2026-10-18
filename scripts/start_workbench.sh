#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export AGX_ARM_WS="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/runtime_env.sh" dabai
cd "$AGX_ARM_WS"
/usr/bin/python3 - <<'PY'
import runtime_launcher
result=runtime_launcher.ensure_background('workbench-service',camera='dual',initiator='workbench',wait_ready=False)
print('工作台后台:',result.get('result'))
PY
exec "$SCRIPT_DIR/run_component.sh" workbench --camera dual "$@"
