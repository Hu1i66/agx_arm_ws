# 眼在手上 (Eye-in-Hand) 手眼标定完整指南

## 硬件配置

| 项目 | 型号/参数 |
|------|----------|
| 机械臂 | Piper |
| 相机 | Intel RealSense D455（固定在机械臂手腕，距法兰约 7cm） |
| 标定板 | **ChArUco 7×5**，方格 32.00 mm，标记 24.00 mm，`DICT_5X5_1000`，17 标记 / 24 内角点 |

### 标定板参数（唯一真相源）

| 参数 | 值 |
|------|-----|
| 方格阵列 | 7 × 5（列 × 行） |
| `square_length` | **0.032 m**（可用卡尺实测后回填） |
| `marker_length` | **0.024 m** |
| 字典 | `cv2.aruco.DICT_5X5_1000` |
| 标记 / 内角点 | 17 个（id 0–16）/ 24 个（6×4） |
| 板面尺寸 | 224 × 160 mm；原点在左上角，x 向右、y 向下 |

### 打印与验收

1. 打印必须选“**实际大小 / 100%**”，禁用“适应页面 / 缩放”。
2. 量尺验证：页脚 100 mm 标尺实测应为 **100.0 mm**（±0.5 mm）。超出说明被缩放。
3. 纸张平整度是隐藏项：**整面喷胶贴在铝板/亚克力板上**（不能只贴四角），纸面卷曲 1–3 mm 会直接污染外参。
4. 若量得方格不是 32.0 mm，标定时用 `-p square_length:=<实测米值>` 覆盖。

---

## 前置准备

- ROS2 Humble 已安装。
- `~/.local` 中需有支持 ChArUco 新 API 的 OpenCV（**≥ 4.7**，本机为 4.13.0）。
  验证：`python3 -c "import cv2; print(cv2.__version__, hasattr(cv2.aruco,'CharucoBoard'))"` 应输出 `4.13.0 True`。
  若输出 `4.5.4 False`，说明被系统 OpenCV 遮蔽了，请先安装/启用新版本。
- 编译（注意 `PYTHONNOUSERSITE=1`，见下方“常见问题”）：

```bash
source /opt/ros/humble/setup.bash
cd /home/lxf/handeye
PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select handeye_calibration_ros
```

> **运行节点时不要加 `PYTHONNOUSERSITE=1`**，否则会回退到系统 OpenCV 4.5.4 而没有 ChArUco API。

---

## 第一步：标定相机内参（不要用厂商出厂内参）

> 内参必须与**采集时的分辨率**一致；出厂内参与你的实际模组常有偏差且缺畸变系数。

**终端 1：启动 RealSense 相机**

```bash
conda activate orange_dataset
ros2 launch realsense2_camera rs_launch.py
```

**终端 2：启动内参标定**

```bash
source /opt/ros/humble/setup.bash
source /home/lxf/handeye/install/setup.bash
ros2 run handeye_calibration_ros camera_intrinsic_calibration
```

采集要求：**15–20 帧，务必覆盖画面中心与四角**（畸变系数 k1/k2/k3 全靠边缘样本约束）。
每帧都能看到程序打印的画面覆盖格数；缺少的区域会明确提示。

| 按键 | 功能 |
|------|------|
| **Enter** | 采集一帧 |
| **d** | 回退上一帧 |
| **q** | 计算内参并写入 YAML（需 ≥15 帧） |
| **c** | 退出 |

内参保存在 `./result/camera_intrinsics.yaml`（含 `image_width/height`、`camera_matrix`、
`distortion_coefficients`、`reprojection_error`）。
**重投影误差应 < 0.3 px**；≥0.5 px 时程序会告警，建议检查模糊/曝光/板面平整度后重采。

---

## 第二步：固定标定板

> **⚠️ 最关键的一步！标定板必须在整个标定过程中绝对固定不动！**

将标定板用胶带/夹具固定在桌面上，标定过程中只移动机械臂，**绝不碰标定板**。

---

## 第三步：启动机械臂（示教模式）

**终端 3：**

```bash
source ~/agx_arm_ws/install/setup.bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py can_port:=can0 arm_type:=piper effector_type:=agx_gripper
```

> 机械臂切换到**示教模式**，以便手动拖动机器人。

**终端 4（可选）：查看检测画面**

```bash
ros2 run rqt_image_view rqt_image_view
```

选择话题 **`/handeye/debug_image`**（不再是原始图）。该话题直接叠加显示检出的角点、
板面坐标轴，以及 `m17/17 c24/24 rms0.21px sq100px` 一行质量指标；不通过质量门时数字变红。

> 旧版的 `aruco_ros single` 节点已不再需要（ChArUco 检测已内联进标定程序）。

---

## 第四步：启动标定程序

**重要：不要用 `ros2 launch` 启动标定程序**（stdin 被重定向会导致 termios 异常 crash）。请用 `ros2 run`：

```bash
source /opt/ros/humble/setup.bash
source /home/lxf/handeye/install/setup.bash
ros2 run handeye_calibration_ros handeye_calibration --ros-args -p mode:=eye_in_hand
```

---

## 第五步：采集数据

| 按键 | 功能 |
|------|------|
| **Enter** | 采集一组（走质量门；不合格**不计数**并当场提示原因） |
| **d** | 删除上一组（同时删除该帧原始图像） |
| **q** | 计算标定结果并保存（需超过 `min_num`） |
| **c** | 退出 |

### 每帧质量门（不通过立即重采）

| 门 | 阈值 |
|----|------|
| 检出标记数 | = 17（至少 ≥14） |
| 检出内角点数 | ≥ 20（满分 24） |
| 单帧重投影 RMS | < 0.5 px |
| 方格像素边长 | ≥ 40 px（推荐 ≥60 px） |
| 图像与位姿同步 | 时间戳差 < 50 ms（时钟域不同时自动回退到接收时间差并提示） |

### 位姿配方（15–25 组，**方位跨度是精度的唯一大杠杆**）

1. 中心正视 1 个（基准帧）。
2. 绕两个轴各 ±25–35° 倾斜，每轴 6 个方向（上下左右 + 两个对角）——**跨度来源**。
3. 绕相机光轴（板面法线）自转 4 个角度（0/90/180/270°）——只转图像不换视野，性价比最高。
4. 距离 ±20% 变化 3 个（改善深度方向约束）。
5. 至少 4 个位姿把板推到画面**四角**（畸变约束全靠这些帧）。

采集过程中程序会持续打印 `[跨度提示]` 当前相对转角中位；**低于 25° 时会提示你补跨度**。
固定板距 0.30–0.45 m，让板宽占画面宽度的 25%–50%。

---

## 第六步：读取标定结果与判据

标定结果保存在 `./result/`：

```bash
ls result/
# 2026-10-07_15-30-00_calibration.json   # 标定结果 (position/orientation/rpy + criteria)
# 2026-10-07_15-30-00_raw_data.json      # 原始数据（离线分析用）
# 2026-10-07_15-30-00_session/           # 会话目录: frames.json + f000.png ... 原始图像
```

标定结果 JSON 格式：

```json
{
    "position": [-0.061, -0.008, 0.029],            // 相机在 TCP 坐标系下的位置 (m)
    "orientation": [-0.129, 0.131, -0.713, 0.677],  // 相机在 TCP 坐标系下的四元数
    "rpy": [[-0.369, -0.007, -1.620]],              // RPY 欧拉角 (rad)
    "criteria": { "...": "判据实际值，供下游做容差设计" }
}
```

### 判据（按可信度排序，按 q 后直接打印）

| 判据 | 阈值 |
|------|------|
| **留出位姿验证**（最硬，留 4 组不参与求解） | 平移误差 **< 1 mm** |
| 双算法一致性（病态报警器） | **< 0.02°**，超过说明方位跨度不足，回去补跨度 |
| 相对转角中位 | **≥ 25°**（理想 30–40°） |
| 可用运动对数（≥25°） | **≥ 10** |
| 单帧重投影 RMS | < 0.5 px（数据质量，不是标定精度） |
| 方程残差 ‖A·X−X·B‖ | **仅参考，不作为验收**（实测对病态不敏感） |

`cv2.calibrateHandEye` 的 5 种方法（TSAI/PARK/HORAUD/ANDREFF/DANIILIDIS）会作为
**交叉参考表**打印（含与主解的旋转差），其残差同样仅作参考。

---

## 第七步：验证

### 7.1 更新验证文件

将标定结果写入 [verify_handeye.launch.py](src/handeye_calibration_ros/launch/verify_handeye.launch.py)：

```python
# position (x, y, z) — 直接使用标定结果的 position
x = "标定结果 position[0]"
y = "标定结果 position[1]"
z = "标定结果 position[2]"

# orientation (qx, qy, qz, qw) — 使用标定结果 orientation 的共轭（xyz 取反、w 不变）
qx = "标定结果 orientation[0] 取反"
qy = "标定结果 orientation[1] 取反"
qz = "标定结果 orientation[2] 取反"
qw = "标定结果 orientation[3] 不变"
```

> **为什么取共轭？** 标定结果 `T_cam2gripper` 是相机→TCP，TF 需要 `T_gripper2cam`（TCP→相机），两者互为逆变换。

> 同时确认 `parent_frame = "tcp_link"`，`child_frame = "camera_link"`。

### 7.2 启动验证

```bash
source /opt/ros/humble/setup.bash
source /home/lxf/handeye/install/setup.bash
ros2 launch handeye_calibration_ros verify_handeye.launch.py
```

### 7.3 在 RViz 中检查

1. **Add → TF** → 展开 `tcp_link` → 看 `camera_link` 是否挂在正确位置（距 TCP 约 6–7cm）
2. **Add → PointCloud2** → 话题 `/camera/camera/depth/points`（可选）
3. 移动机械臂，观察深度点云与物理环境是否一致，无漂移即验证通过

---

## 离线复算

会话目录里的 `frames.json` 与原始图像可以在不重做实验的情况下换阈值/换算法/剔除坏帧重解：

```bash
source /opt/ros/humble/setup.bash
source /home/lxf/handeye/install/setup.bash
python3 - <<'PY'
import json, sys, numpy as np
sys.path.insert(0, '/home/lxf/handeye/src/handeye_calibration_ros/handeye_calibration_ros')
import charuco_detect as cd, axxb
session = 'result/<时间戳>_session'
frames = json.load(open(f'{session}/frames.json'))
K = np.array(frames['camera']['K']); dist = np.array(frames['camera']['dist'])
T_g2b = [np.array(f['T_g2b']) for f in frames['frames']]
T_t2c = [np.array(f['T_t2c']) for f in frames['frames']]   # 或按需用 f*.png 重新 board_pose
r = axxb.solve_axxb(np.array(T_g2b), np.array(T_t2c))
print(axxb.format_criteria_report(r, axxb.holdout_validate(np.array(T_g2b), np.array(T_t2c))))
PY
```

---

## 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| 标定结果每次都不一样 / 差几十 cm | 标定板中途被移动，或板面不平整 | 标定板固定死；整面喷胶贴硬板 |
| `AttributeError: module 'cv2.aruco' has no attribute 'CharucoBoard'` | 用了系统 OpenCV 4.5.4（无 ChArUco 新 API） | 确认 `python3 -c "import cv2;print(cv2.__version__)"` 为 4.7+；**运行时不要设 `PYTHONNOUSERSITE=1`** |
| 编译报 `option --editable not recognized` / `--uninstall not recognized` | `~/.local` 的 setuptools 过新（83.x 已移除 `develop`/`install`），与 Humble 构建不兼容 | 编译时加 `PYTHONNOUSERSITE=1` 使用系统 setuptools 59.6.0 |
| 平移误差 10–30 mm、旋转误差 1–2° | **方位跨度不足**（几乎没换视角） | 补 ±25–35° 倾斜与绕光轴自转（第五步） |
| 采集时一直提示“质量门不通过” | 太远/太偏/模糊/曝光漂移 | 靠近到 15–25cm、让板完整入画、锁定曝光 |
| 图像与位姿不同步告警 | RealSense 与机械臂 SDK 时钟源不同 | 程序已自动回退到接收时间差；若两路都超限，检查是否有节点阻塞 |
| `ros2 launch` 启动标定 exit code 255 | stdin 非 TTY 导致 termios 异常 | 改用 `ros2 run` |
| ANDREFF 方法崩溃 | 旋转矩阵接近奇异 | 正常现象，属交叉参考表，不影响主解 |

---

## 文件结构速查

```
/home/lxf/handeye/
├── src/handeye_calibration_ros/
│   ├── handeye_calibration_ros/
│   │   ├── handeye_calibration.py     # 标定主程序 (内联 ChArUco 检测 + 双算法求解)
│   │   ├── charuco_detect.py          # 板构造 / 检测 / PnP / 质量指标 / 内参 YAML
│   │   ├── axxb.py                    # AX=XB 双算法 + 留出位姿验证 (纯 numpy)
│   │   └── intrinsic_calibration.py   # 单目内参标定 (camera_intrinsic_calibration)
│   └── launch/
│       ├── handeye_calibration_launch.py  # 标定 launch
│       └── verify_handeye.launch.py       # 验证 launch
├── result/                                # 标定结果 / 内参 YAML / 会话目录
└── HANDEYE_CALIBRATION_GUIDE.md           # 本文档
```

> `aruco_ros` 包仍保留在工作空间中（独立 git 仓库），但标定流程已不再依赖它。