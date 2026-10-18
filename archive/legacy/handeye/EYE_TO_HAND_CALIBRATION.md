# 眼在手外 (Eye-to-Hand) 手眼标定完整指南

## 硬件配置

| 项目 | 型号/参数 |
|------|----------|
| 机械臂 | Piper |
| 相机 | Intel RealSense D455（固定在外部支架，整个标定过程不动） |
| 标定板 | **ChArUco 7×5**，方格 32.00 mm，标记 24.00 mm，`DICT_5X5_1000`，17 标记 / 24 内角点 |

### 标定板参数（唯一真相源）

| 参数 | 值 |
|------|-----|
| 方格阵列 | 7 × 5（列 × 行） |
| `square_length` | **0.032 m**（可用卡尺实测后回填） |
| `marker_length` | **0.024 m** |
| 字典 | `cv2.aruco.DICT_5X5_1000` |
| 标记 / 内角点 | 17 个（id 0–16）/ 24 个（6×4） |
| 板面尺寸 | 224 × 160 mm |

### 打印与验收

1. 打印必须选“**实际大小 / 100%**”，禁用“适应页面 / 缩放”。
2. 量尺验证：页脚 100 mm 标尺实测应为 **100.0 mm**（±0.5 mm）。
3. 标定板贴在硬板（铝板/亚克力）上并刚性固定在机械臂末端，**不能只贴四角**。

---

## 原理简述：眼在手外 vs 眼在手上

| | 眼在手上 eye_in_hand | 眼在手外 eye_to_hand |
|---------------------|---------------------|---------------------|
| 相机位置 | 固定在机械臂手腕 | **固定在外部环境** |
| 标定板位置 | 固定在桌面 | **固定在机械臂末端/夹爪** |
| 机械臂运动时 | 相机动，标定板不动 | **标定板动，相机不动** |
| 标定结果物理含义 | T_cam2gripper（相机相对 TCP） | **T_base2cam（相机相对基座）** |
| 验证 TF 发布 | tcp_link → camera_link（需取共轭） | **base_link → camera_link（直接使用）** |

> 眼在手外模式下，程序内部会对 TCP 位姿取逆后再求解（`PoseMartix(check_mode=True, mode='eye_to_hand')`），
> 因此**输出的仍然是 T_base2cam**，下游与验证流程按“直接用、不取共轭”处理。

---

## 前置准备

- ROS2 Humble 已安装。
- `~/.local` 中需有支持 ChArUco 新 API 的 OpenCV（**≥ 4.7**，本机 4.13.0）。
  验证：`python3 -c "import cv2; print(cv2.__version__, hasattr(cv2.aruco,'CharucoBoard'))"` → `4.13.0 True`。
- 编译（注意 `PYTHONNOUSERSITE=1`）：

```bash
source /opt/ros/humble/setup.bash
cd /home/lxf/handeye
PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select handeye_calibration_ros
```

> **运行节点时不要加 `PYTHONNOUSERSITE=1`**，否则会回退到系统 OpenCV 4.5.4 而没有 ChArUco API。

---

## 第一步：物理布置（最关键，决定成败）

### 1.1 相机固定

将 D455 用支架固定在工作台旁，**整个标定过程绝不移动相机**。

### 1.2 标定板固定到机械臂末端

> **最关键的一步！标定板相对机械臂 TCP 必须绝对刚性固定，不能有任何松动！**

1. **刚性固定**：绝不能有松动/位移/旋转，1mm 的晃动会导致标定结果漂几十厘米
2. **朝向合理**：板面朝向相机，使机械臂在可达空间内运动时能持续正对相机
3. **运动可达**：安装位置不能限制机械臂做出绕三轴各 **25–35°** 的旋转姿态
4. **不遮挡**：胶带只能粘在四角白边或背面，不覆盖黑色边框和内部图案
5. **板面平整**：打印纸整面喷胶贴在硬板上，不能弯曲

> **不需要**精确知道标定板相对 TCP 的位置和角度——手眼标定会把这个未知量自动消去。

### 1.3 确认相机能拍到标定板

调整相机位置，使标定板在工作空间内运动时始终完整在视野内（固定板距 0.30–0.45 m 最佳，
让板宽占画面宽度的 25%–50%）。

---

## 第二步：标定相机内参（不要用厂商出厂内参）

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

采集 **15–20 帧，覆盖画面中心与四角**，程序会打印覆盖进度并提示缺失区域。
按 `q` 求解并写入 `./result/camera_intrinsics.yaml`，**重投影误差应 < 0.3 px**。

| 按键 | 功能 |
|------|------|
| **Enter** | 采集一帧 |
| **d** | 回退上一帧 |
| **q** | 计算内参并写入 YAML（需 ≥15 帧） |
| **c** | 退出 |

---

## 第三步：启动各节点

**终端 3：启动机械臂（示教模式）**

```bash
source ~/agx_arm_ws/install/setup.bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py can_port:=can0 arm_type:=piper effector_type:=agx_gripper
```

> 机械臂切换到**示教模式**，以便手动拖动。小心标定板不要撞到相机。

**终端 4（可选）：查看检测画面**

```bash
ros2 run rqt_image_view rqt_image_view
```

选择话题 **`/handeye/debug_image`**：叠加显示检出角点、板面坐标轴与质量指标
（`m17/17 c24/24 rms0.21px sq100px`）；不通过质量门时数字变红。

> 旧版的 `aruco_ros single` 节点已不再需要（ChArUco 检测已内联进标定程序）。

---

## 第四步：启动标定程序

**重要：不要用 `ros2 launch` 启动标定程序**（stdin 会被重定向导致 TTY 异常 crash）。请用 `ros2 run`：

```bash
source /opt/ros/humble/setup.bash
source /home/lxf/handeye/install/setup.bash
ros2 run handeye_calibration_ros handeye_calibration --ros-args -p mode:=eye_to_hand
```

> `mode:=eye_to_hand` 是**必须参数**，默认值是 `eye_in_hand`。
> 可选参数见 [readme_cn.md](src/handeye_calibration_ros/readme_cn.md) 参数表。

---

## 第五步：采集数据

| 按键 | 功能 |
|------|------|
| **Enter** | 采集一组（走质量门；不合格**不计数**并当场提示原因） |
| **d** | 删除上一组（同时删除该帧原始图像） |
| **q** | 计算标定结果并保存 |
| **c** | 退出 |

### 每帧质量门

| 门 | 阈值 |
|----|------|
| 检出标记数 | = 17（至少 ≥14） |
| 检出内角点数 | ≥ 20（满分 24） |
| 单帧重投影 RMS | < 0.5 px |
| 方格像素边长 | ≥ 40 px（推荐 ≥60 px） |
| 图像与位姿同步 | 时间戳差 < 50 ms（时钟域不同时自动回退到接收时间差并提示） |

### 位姿配方（15–25 组）

1. 中心正视 1 个（基准帧）
2. **绕两个轴各 ±25–35° 倾斜**，每轴 6 个方向（上下左右 + 对角）——**跨度来源**
3. 绕相机光轴自转 4 个角度（0/90/180/270°）
4. 距离 ±20% 变化 3 个
5. 至少 4 个位姿把板推到画面**四角**（畸变约束全靠这些帧）

中途**绝不碰相机和标定板**。程序会持续打印 `[跨度提示]`，低于 25° 时提示补跨度。

---

## 第六步：读取标定结果与判据

结果保存在 `./result/`：

```bash
ls result/
# 2026-10-07_15-30-00_calibration.json   # 标定结果
# 2026-10-07_15-30-00_raw_data.json      # 原始数据（离线分析用）
# 2026-10-07_15-30-00_session/           # frames.json + f000.png ... 原始图像
```

标定结果 JSON 格式（眼在手外）：

```json
{
    "position": [0.673, 0.011, 0.597],             // 相机在 base_link 坐标系下的位置 (m) = T_base2cam
    "orientation": [-0.693, -0.712, 0.088, 0.067],  // 相机在 base_link 坐标系下的四元数
    "rpy": [[-2.920, 0.027, 1.600]],                // RPY 欧拉角 (rad)
    "criteria": { "...": "判据实际值" }
}
```

### 判据（按 q 后直接打印，按可信度排序）

| 判据 | 阈值 |
|------|------|
| **留出位姿验证**（最硬，留 4 组不参与求解） | 平移误差 **< 1 mm** |
| 双算法一致性（病态报警器） | **< 0.02°**，超过说明方位跨度不足 |
| 相对转角中位 | **≥ 25°**（理想 30–40°） |
| 可用运动对数（≥25°） | **≥ 10** |
| 单帧重投影 RMS | < 0.5 px（数据质量，非标定精度） |
| 方程残差 ‖A·X−X·B‖ | **仅参考，不作为验收** |

---

## 第七步：更新验证文件并验证

将标定结果写入 [verify_handeye.launch.py](src/handeye_calibration_ros/launch/verify_handeye.launch.py)：

```python
# =============== 眼在手外: 标定结果 ===============
# position (x, y, z) — 相机在 base_link 下的位置，直接使用
x = "标定结果 position[0]"
y = "标定结果 position[1]"
z = "标定结果 position[2]"

# orientation (qx, qy, qz, qw) — 相机在 base_link 下的姿态，直接使用，不取共轭
qx = "标定结果 orientation[0]"
qy = "标定结果 orientation[1]"
qz = "标定结果 orientation[2]"
qw = "标定结果 orientation[3]"

# parent_frame: 机械臂基座；child_frame: 相机根坐标系
parent_frame = "base_link"
child_frame = "camera_link"
```

> **与眼在手上的关键差异：**
> 1. **不取共轭**：眼在手外结果本身就是 T_base2cam，TF 也是 base→cam，方向一致直接用
> 2. **parent_frame 改成 `base_link`**：不再是 `tcp_link`

启动验证：

```bash
source /opt/ros/humble/setup.bash
source /home/lxf/handeye/install/setup.bash
ros2 launch handeye_calibration_ros verify_handeye.launch.py
```

在 RViz 中检查：

1. **Add → TF** → `base_link` → `camera_link` 是否位于相机实际物理位置附近
2. **Add → PointCloud2** → 话题 `/camera/camera/depth/points`（可选）
3. 移动机械臂，深度点云与机械臂模型在空间中一致且**无漂移**，即验证通过
4. 若找不到 `base_link → camera_link`，用 `ros2 run tf2_tools view_frames` 确认实际基坐标系名称

---

## 离线复算

会话目录的 `frames.json` + 原始图像可离线换阈值/换算法/剔除坏帧重解，不必重做实验：

```bash
source /opt/ros/humble/setup.bash
source /home/lxf/handeye/install/setup.bash
python3 - <<'PY'
import json, sys, numpy as np
sys.path.insert(0, '/home/lxf/handeye/src/handeye_calibration_ros/handeye_calibration_ros')
import axxb
frames = json.load(open('result/<时间戳>_session/frames.json'))
T_g2b = np.array([f['T_g2b'] for f in frames['frames']])
T_t2c = np.array([f['T_t2c'] for f in frames['frames']])
r = axxb.solve_axxb(T_g2b, T_t2c)
print(axxb.format_criteria_report(r, axxb.holdout_validate(T_g2b, T_t2c)))
PY
```

---

## 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| 标定结果每次都差很多 / 位置差几十 cm | **标定板没固定牢**，或相机中途被碰动 | 板必须刚性固定在机械臂末端；相机绝对固定 |
| `AttributeError: 'cv2.aruco' has no attribute 'CharucoBoard'` | 用了系统 OpenCV 4.5.4 | 确认 `cv2.__version__` ≥ 4.7；**运行时不要设 `PYTHONNOUSERSITE=1`** |
| 编译报 `option --editable not recognized` | `~/.local` 的 setuptools 83.x 与 Humble 构建不兼容 | 编译时加 `PYTHONNOUSERSITE=1` |
| 平移误差 10–30 mm、旋转误差 1–2° | 方位跨度不足 | 补 ±25–35° 倾斜与绕光轴自转 |
| 残差很小但实际偏差大 | 只看残差验收 | 必须做留出位姿验证（第六步） |
| `ros2 launch` 启动标定 exit code 255 | stdin 非 TTY 导致 termios 异常 | 改用 `ros2 run` |
| marker/板方向不一致告警 | 标定板翻转过相机光轴 | 调整运动范围，避免板相对相机翻转 |

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
│       ├── handeye_calibration_launch.py   # 标定 launch（交互式，请改用 ros2 run）
│       └── verify_handeye.launch.py        # 验证 launch
├── result/                                 # 标定结果 / 内参 YAML / 会话目录
├── HANDEYE_CALIBRATION_GUIDE.md            # 眼在手上指南
└── EYE_TO_HAND_CALIBRATION.md              # 本文档
```

> `aruco_ros` 包仍保留在工作空间中（独立 git 仓库），但标定流程已不再依赖它。