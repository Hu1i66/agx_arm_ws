# ChArUco 标定板 7×5（square 32mm）

- 板常量唯一来源: `src/handeye_calibration_ros/handeye_calibration_ros/charuco_detect.py`
- 格数: 7×5；方格边长: 32mm；marker 边长: 24mm
- ArUco 字典: DICT_5X5_1000
- 板面尺寸: 224×160 mm（7×32 = 224, 5×32 = 160）
- marker 数: 17；Charuco 内角点数: 24
- 生成参数: dpi=300（PNG内已写入物理分辨率）, 空白边缘=20mm
- 含边缘整图尺寸: 264×200 mm；默认可用A4横向或更大纸张，确保可打印区域足够
- 图片: `charuco_7x5_32mm.png`（像素 3118×2362，含边缘）

## 打印要求
1. 打印/PDF 导出时**关闭"适合页面/缩放"**，按 **100% 实际尺寸**输出。
2. 用卡尺实测方格边长应为 **32mm**（误差 <0.5mm），否则重新打印。
3. 固定在**平整刚性**载体上（亚克力/铝板/泡沫板），板面不得弯曲。
4. 旧 `marker_582_*.pdf` 是历史单 ArUco 标记，**不能**当当前 ChArUco 板使用。
