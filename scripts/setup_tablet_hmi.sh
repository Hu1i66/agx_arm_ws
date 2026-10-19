#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT"

if [ ! -x .venv_tablet/bin/python ]; then
  python3 -m venv --system-site-packages .venv_tablet
fi
.venv_tablet/bin/python -m pip install --disable-pip-version-check --no-input -r requirements-tablet.txt
npm --prefix tablet_hmi install --no-audit --no-fund
/usr/bin/python3 scripts/build_tablet_robot_model.py
npm --prefix tablet_hmi run build

printf '平板上位机依赖与前端构建完成\n'
printf '网关: https://<本机局域网IP>:8443/\n'
printf 'CA:   %s/.runtime/tablet_hmi/certs/ca.crt\n' "$ROOT"
