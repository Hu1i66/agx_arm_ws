#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WORKBENCH_INSTALL_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
/usr/bin/python3 - <<'PY'
import os,pathlib,subprocess
root=pathlib.Path(os.environ['WORKBENCH_INSTALL_ROOT'])
text=(root/'assets/desktop/piper-workbench.desktop.in').read_text().replace('@ROOT@',str(root))
apps=pathlib.Path.home()/'.local/share/applications';apps.mkdir(parents=True,exist_ok=True)
target=apps/'piper-workbench.desktop';target.write_text(text);target.chmod(0o755)
result=subprocess.run(['xdg-user-dir','DESKTOP'],capture_output=True,text=True)
if result.returncode==0:
    desktop=pathlib.Path(result.stdout.strip())
    if desktop.is_dir():
        shortcut=desktop/'piper-workbench.desktop';shortcut.write_text(text);shortcut.chmod(0o755)
        subprocess.run(['gio','set',str(shortcut),'metadata::trusted','true'],capture_output=True)
        print('桌面快捷方式:',shortcut)
print('应用菜单入口:',target)
PY
