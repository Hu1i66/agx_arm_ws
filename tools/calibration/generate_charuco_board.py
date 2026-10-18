#!/usr/bin/env python3
"""生成 ChArUco 标定板 PNG 与尺寸说明（计划 5.3）。

复用 charuco_detect 的唯一板常量（7×5, square=0.032m, marker=0.024m, DICT_5X5_1000），
避免与标定节点出现两套板参数。

用法:
  ./scripts/run_component.sh generate-board            # 默认写到 config/calibration/board/
  python3 tools/calibration/generate_charuco_board.py --out-dir /tmp/board \
      --dpi 300 --margin-mm 20

输出:
  charuco_<NxM>_<square>mm.png   可直接打印的板图（含空白边缘, 100% 比例打印）
  charuco_<NxM>_<square>mm.md    尺寸/数量说明与打印要求
"""
from __future__ import annotations

import argparse
import math
import importlib.util
import pathlib
import sys

import cv2
from PIL import Image

WS = pathlib.Path(__file__).resolve().parents[2]
_CHARUCO_PATH = (WS / "src" / "handeye_calibration_ros"
                 / "handeye_calibration_ros" / "charuco_detect.py")


def _load_charuco():
    # charuco_detect 在模块级 `import axxb`（同包模块）:
    # 先把包目录加入 sys.path, 否则按文件路径加载时找不到 axxb。
    _pkg_dir = _CHARUCO_PATH.parent
    if str(_pkg_dir) not in sys.path:
        sys.path.insert(0, str(_pkg_dir))
    spec = importlib.util.spec_from_file_location("charuco_detect", _CHARUCO_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)   # 仅定义函数/常量, 不访问硬件
    return mod


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="生成 ChArUco 标定板")
    parser.add_argument("--out-dir", default=str(WS / "config" / "calibration" / "board"))
    parser.add_argument("--dpi", type=int, default=300)
    parser.add_argument("--margin-mm", type=float, default=20.0,
                        help="板面四周空白边缘 (mm), 便于打印固定")
    args = parser.parse_args(argv)
    if args.dpi <= 0 or not math.isfinite(args.margin_mm) or args.margin_mm < 0:
        parser.error("dpi须为正整数，margin-mm须为有限非负数")

    cd = _load_charuco()
    squares_x = cd.DEFAULT_SQUARES_X
    squares_y = cd.DEFAULT_SQUARES_Y
    square_m = cd.DEFAULT_SQUARE_LENGTH
    marker_m = cd.DEFAULT_MARKER_LENGTH
    dict_name = cd.DEFAULT_DICT
    full_markers = getattr(cd, "FULL_MARKERS", 17)
    # 角点数 = (squares_x-1) * (squares_y-1)
    full_corners = (squares_x - 1) * (squares_y - 1)

    board = cd.make_board(square_m, marker_m, dict_name, squares_x, squares_y)

    px_per_m = args.dpi / 0.0254
    margin_px = int(round(args.margin_mm / 1000.0 * px_per_m))
    board_px_w = int(round(squares_x * square_m * px_per_m))
    board_px_h = int(round(squares_y * square_m * px_per_m))

    img = board.generateImage((board_px_w, board_px_h), marginSize=0, borderBits=1)
    canvas = cv2.copyMakeBorder(img, margin_px, margin_px, margin_px, margin_px,
                                cv2.BORDER_CONSTANT, value=255)

    out_dir = pathlib.Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f"charuco_{squares_x}x{squares_y}_{int(round(square_m * 1000))}mm"
    png = out_dir / f"{stem}.png"
    Image.fromarray(canvas).save(png, dpi=(args.dpi, args.dpi))

    board_w = squares_x * square_m * 1000.0
    board_h = squares_y * square_m * 1000.0
    md = out_dir / f"{stem}.md"
    md.write_text(
        f"""# ChArUco 标定板 {squares_x}×{squares_y}（square {square_m*1000:.0f}mm）

- 板常量唯一来源: `src/handeye_calibration_ros/handeye_calibration_ros/charuco_detect.py`
- 格数: {squares_x}×{squares_y}；方格边长: {square_m*1000:.0f}mm；marker 边长: {marker_m*1000:.0f}mm
- ArUco 字典: {dict_name}
- 板面尺寸: {board_w:.0f}×{board_h:.0f} mm（{squares_x}×{square_m*1000:.0f} = {board_w:.0f}, {squares_y}×{square_m*1000:.0f} = {board_h:.0f}）
- marker 数: {full_markers}；Charuco 内角点数: {full_corners}
- 生成参数: dpi={args.dpi}（PNG内已写入物理分辨率）, 空白边缘={args.margin_mm:.0f}mm
- 含边缘整图尺寸: {board_w+2*args.margin_mm:.0f}×{board_h+2*args.margin_mm:.0f} mm；默认可用A4横向或更大纸张，确保可打印区域足够
- 图片: `{png.name}`（像素 {canvas.shape[1]}×{canvas.shape[0]}，含边缘）

## 打印要求
1. 打印/PDF 导出时**关闭"适合页面/缩放"**，按 **100% 实际尺寸**输出。
2. 用卡尺实测方格边长应为 **{square_m*1000:.0f}mm**（误差 <0.5mm），否则重新打印。
3. 固定在**平整刚性**载体上（亚克力/铝板/泡沫板），板面不得弯曲。
4. 旧 `marker_582_*.pdf` 是历史单 ArUco 标记，**不能**当当前 ChArUco 板使用。
""", encoding="utf-8")

    print(f"已生成: {png}")
    print(f"尺寸说明: {md}")
    print(f"板面 {board_w:.0f}x{board_h:.0f}mm, {full_markers} markers, {full_corners} corners, "
          f"square={square_m*1000:.0f}mm marker={marker_m*1000:.0f}mm {dict_name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
