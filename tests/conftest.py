"""pytest 配置：把主空间根加入 sys.path，便于导入 runtime_* 模块。"""
import pathlib
import sys

_WS_ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(_WS_ROOT) not in sys.path:
    sys.path.insert(0, str(_WS_ROOT))
