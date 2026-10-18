# 基于ROS2的手眼标定程序包 (ChArUco 版)

|UBUNTU|ROS|PYTHON|OPENCV|STATE|
|---|---|---|---|---|
|![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![python](https://img.shields.io/badge/python-3.10-blue.svg)|![opencv](https://img.shields.io/badge/opencv-%E2%89%A54.7-blue.svg)|![Pass](https://img.shields.io/badge/Pass-green.svg)|

## 手眼标定
> 通过采集多组机械臂末端位姿和相机识别的**标定板位姿**作为输入，获取两种标定结果：
- 眼在手上：机械臂末端和相机之间的坐标变换矩阵。
- 眼在手外：机械臂底部和相机之间的坐标变换矩阵。

> 本版本已从“单个 ArUco 标记 + `aruco_ros`”改为 **ChArUco 标定板**，检测逻辑内联在标定节点中，
> 不再依赖 `aruco_ros`；同时按验收要点增加了逐帧质量门、双算法互校与留出位姿验证。

## 0、标定板参数（唯一真相源）

| 参数 | 值 |
|------|-----|
| 方格阵列 | 7 × 5（列 × 行） |
| `square_length` | **0.032 m**（可用卡尺实测后回填覆盖） |
| `marker_length` | **0.024 m** |
| 字典 | `cv2.aruco.DICT_5X5_1000` |
| 标记 / 内角点 | 17 个（id 0–16）/ 24 个（6×4） |
| 板面尺寸 | 224 × 160 mm；原点在左上角，x 向右、y 向下 |

## 1、 安装方法

### 1. 相关依赖

```bash
sudo apt-get install ros-$ROS_DISTRO-tf-transformations \
                     ros-$ROS_DISTRO-cv-bridge \
                     ros-$ROS_DISTRO-sensor-msgs \
                     python3-numpy python3-opencv python3-yaml
```

> **OpenCV 版本要求 ≥ 4.7**（`CharucoBoard` / `CharucoDetector` 新 API）。
> 本机使用 `~/.local` 中的 OpenCV 4.13.0；系统自带的 4.5.4 **没有** ChArUco 新 API。
> 验证：
> ```bash
> python3 -c "import cv2; print(cv2.__version__, hasattr(cv2.aruco,'CharucoBoard'))"
> # 期望: 4.13.0 True
> ```

### 2. 安装编译

```bash
source /opt/ros/humble/setup.bash
cd /home/lxf/handeye
PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select handeye_calibration_ros
```

> **为什么要 `PYTHONNOUSERSITE=1`？** `~/.local` 中的 setuptools 83.x 已移除 `develop`/`install`，
> 会导致 `option --editable/--uninstall not recognized` 而编译失败。加上该变量即使用系统 setuptools 59.6.0。
> **运行节点时则不要加**，否则会回退到系统 OpenCV 4.5.4 而缺少 ChArUco API。

## 2、 使用

### 1. 启动相机节点

```bash
conda activate orange_dataset
ros2 launch realsense2_camera rs_launch.py
```

### 2. 启动机械臂

```bash
source ~/agx_arm_ws/install/setup.bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py can_port:=can0 arm_type:=piper effector_type:=agx_gripper
```
> 注意机械臂需要进入**示教模式**

### 3. 标定相机内参（推荐，不要用厂商出厂内参）

```bash
source /opt/ros/humble/setup.bash
source /home/lxf/handeye/install/setup.bash
ros2 run handeye_calibration_ros camera_intrinsic_calibration
```

采集 15–20 帧、覆盖画面中心与四角，按 `q` 求解并写入 `./result/camera_intrinsics.yaml`。
按键：`Enter` 采集 / `d` 回退 / `q` 计算并存盘 / `c` 退出。

### 4. 启动手眼标定程序

> **重要：不要用 `ros2 launch` 启动标定程序**（stdin 被重定向会导致 termios 异常 crash），请用 `ros2 run`。
> 使用方法：`enter` 采集一组数据，`d` 删除一组数据，`q` 计算标定结果并打印退出，`c` 直接退出。
> 每次采集都要通过**逐帧质量门**（标记数 / 角点数 / 重投影 RMS / 方格像素边长 / 图像与位姿同步），
> 不通过会当场提示原因且不计入数量。

- 眼在手上
```bash
ros2 run handeye_calibration_ros handeye_calibration --ros-args -p mode:=eye_in_hand
```
> 采集说明：摄像头固定在机械臂末端，标定板平放在桌上。操作机械臂让摄像头能拍到桌上的标定板。

- 眼在手外
```bash
ros2 run handeye_calibration_ros handeye_calibration --ros-args -p mode:=eye_to_hand
```
> 采集说明：摄像头固定在外部，标定板刚性固定在机械臂末端。操作机械臂让相机能拍到板。

### 5. 查看检测画面（可选）

```bash
ros2 run rqt_image_view rqt_image_view
```
选择话题 **`/handeye/debug_image`**：叠加显示检出角点、板面坐标轴与质量指标。

### 6. 输出

```
result/
├── <时间戳>_calibration.json    # position / orientation / rpy + criteria(判据实际值)
├── <时间戳>_raw_data.json       # 原始位姿数据
└── <时间戳>_session/            # frames.json + f000.png ... 原始图像, 支持离线复算
```

## 3、 参数

### 标定节点 `handeye_calibration`

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| mode | string | `eye_in_hand` | 标定模式（`eye_in_hand` / `eye_to_hand`） |
| min_num | int | `15` | 最少采集位姿数（建议 15–25） |
| piper_topic | string | `/feedback/tcp_pose` | 机械臂 TCP 位姿话题（`geometry_msgs/PoseStamped`） |
| image_topic | string | `/camera/camera/color/image_raw` | 彩色图像话题 |
| camera_info_topic | string | `/camera/camera/color/camera_info` | 相机内参话题 |
| intrinsics_yaml | string | `./result/camera_intrinsics.yaml` | 自标内参 YAML；**存在则覆盖 camera_info** |
| result_save_path | string | `./result` | 结果与会话目录保存路径 |
| squares_x / squares_y | int | `7` / `5` | ChArUco 方格阵列 |
| square_length | double | `0.032` | 方格边长 (m)，可回填实测值 |
| marker_length | double | `0.024` | 标记边长 (m) |
| aruco_dict | string | `DICT_5X5_1000` | ArUco 字典名 |
| min_markers | int | `14` | 每帧最少检出标记数（满分 17） |
| min_corners | int | `20` | 每帧最少检出内角点数（满分 24） |
| max_reproj_rms | double | `0.5` | 单帧重投影 RMS 上限 (px) |
| min_square_px | double | `25.0` | 方格像素边长下限 (px) |
| settle_delay | double | `0.8` | 静止后延时再采样 (s) |
| max_time_diff | double | `0.05` | 图像与位姿时间戳差上限 (s) |
| holdout | int | `4` | 留出位姿验证的留出组数 |
| publish_debug_image | bool | `true` | 是否发布调试图 |
| debug_image_topic | string | `/handeye/debug_image` | 调试图话题 |

> 旧版的 `marker_topic` 参数已移除（不再使用 `aruco_ros`）。

### 内参标定节点 `camera_intrinsic_calibration`

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| image_topic | string | `/camera/camera/color/image_raw` | 彩色图像话题 |
| camera_info_topic | string | `/camera/camera/color/camera_info` | 用于确定采集分辨率 |
| out_yaml | string | `./result/camera_intrinsics.yaml` | 内参输出路径 |
| min_frames | int | `15` | 最少帧数 |
| min_corners | int | `6` | 单帧最少角点数 |
| settle_delay | double | `0.3` | 静止后延时再采样 (s) |
| squares_x / squares_y / square_length / marker_length / aruco_dict | — | 同标定节点 | 标定板参数 |

## 4、 验收判据

按 `q` 后程序直接打印（按可信度排序）：

| 判据 | 阈值 |
|------|------|
| **留出位姿验证**（最硬） | 留至少4组，平移中位误差 **< 15 mm**（用户粗标定标准） |
| 双算法一致性（病态报警器） | **< 0.02°** |
| 相对转角中位 | **≥ 20°**（理想 30–40°） |
| 可用运动对数（≥25°） | **≥ 10** |
| 单帧重投影 RMS | **< 0.5 px** |
| 方程残差 ‖A·X−X·B‖ | **仅参考，不作为验收** |

## 5、 代码结构

| 文件 | 作用 |
|------|------|
| `handeye_calibration_ros/handeye_calibration.py` | 标定主程序：内联 ChArUco 检测、逐帧质量门、落盘、双算法求解 |
| `handeye_calibration_ros/charuco_detect.py` | 板构造 / 单帧检测 / PnP 位姿 / 质量指标 / 内参 YAML 读写（不依赖 ROS） |
| `handeye_calibration_ros/axxb.py` | AX=XB 双算法互校 + 留出位姿验证（纯 numpy，可 `python3 axxb.py` 自测） |
| `handeye_calibration_ros/intrinsic_calibration.py` | 单目内参标定节点 |