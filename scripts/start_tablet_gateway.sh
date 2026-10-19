#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
export AGX_ARM_WS="$ROOT"
source "$SCRIPT_DIR/runtime_env.sh" dual
cd "$ROOT"
exec "$ROOT/.venv_tablet/bin/python" -m workbench.gateway "$@"
