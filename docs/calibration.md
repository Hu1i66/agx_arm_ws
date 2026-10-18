# 双相机标定手册（D455 eye-to-hand / Dabai DC1 eye-in-hand）

> 适用：`handeye_calibration_ros` 包（主空间构建）。总入口见根 [`README.md`](../README.md)。
> 标定使用 `/usr/bin/python3` + 用户 OpenCV(≥4.7；本机 4.13)/NumPy/SciPy；**不进入**旧 handeye 的两个 venv。
> TCP 固定点位置标定见 [tcp_calibration.md](tcp_calibration.md)；若要改变工具TCP，先定好TCP，再采集Dabai手眼。
> 两种相机的流程中，启动 `arm` 前都须先完成根 README §2 的 **CAN 激活 → `activate_robot.py`**。
> 若已有正常运行的 T1，停止抓取后进入标定流程；硬件激活在 T1 启动前完成。

---

## 0. 数据目录与候选/批准隔离

```text
config/calibration/
  d455_eye_to_hand.json     # 活动D455外参，来源见manifest
  d455/intrinsics.yaml      # 存在时=已批准本相机内参
  dabai/intrinsics.yaml     # 不存在则不伪造
  dabai_eye_in_hand.json    # 本轮启用22:53:28结果，相对link6
  manifest.json             # 外参语义/来源/legacy 状态/推广审计
  board/                    # 生成的板图与尺寸说明
  approved_backup/          # 批准前的旧文件与 manifest 备份
data/calibration/
  d455/intrinsics/<会话>/   # 原始帧、候选YAML、质量报告
  d455/handeye/             # 候选JSON、raw_data、各采集会话
  dabai/intrinsics/<会话>/
  dabai/handeye/
  history/handeye/          # 全部历史会话（含未完成/空会话）
  history/d455_intrinsics_legacy/   # 旧分辨率历史资料（禁止自动启用）
```

- 内参 `out_yaml` 指向**本相机会话候选**，不覆盖批准内参；外参 `result_save_path` 指向该相机 handeye 数据目录，不覆盖生产 JSON。
- 每个候选带 `camera_id / mode / 分辨率 / board / 有效帧数 / 来源 / 质量状态`。
- **候选 ≠ 达标**：失败候选与原始数据允许保存，状态明确 `failed/unverified`。
- 生产检测仍使用**原驱动 CameraInfo**；迁入的新内参不自动改变当前抓取 K/D。

---

## 1. 标定板

```bash
cd /home/lxf/agx_arm_ws
./scripts/run_component.sh generate-board
```

- 板常量唯一来源：`src/handeye_calibration_ros/handeye_calibration_ros/charuco_detect.py`
- **7×5，square=0.032m，marker=0.024m，DICT_5X5_1000**；板面 **224×160mm**，17 markers，24 Charuco 角点
- PNG 已写入 **300 DPI**；整图含20mm边缘为 **264×200mm**，使用可打印区域足够的 **A4横向或更大纸张**。
- 打印：**关闭缩放，按 100% 实际尺寸**；量尺确认方格 **32mm**、板面 **224×160mm**；固定在平整刚性载体。
  若打印软件忽略DPI，手动指定整图264×200mm；不能仅凭“100%”确认尺寸。
- 旧 `marker_582_*.pdf` 是历史**单 ArUco** 标记，不能当当前板使用

---

## 2. D455 eye-to-hand 完整步骤（外置固定）

1. 操作者停止分拣、GUI、GraspNet 与检测；**相机固定不动**。先完成内参采集，此阶段可手持板改变姿态与位置；手眼阶段再将板刚性装在末端。
2. 先按根 README §2 激活 CAN 并运行 `activate_robot.py`，再启动 `arm` 与 `camera --camera d455`；确认 RGB/CameraInfo 为 **640×480**，记录 TCP offset：
   ```bash
   ros2 param get /agx_arm_ctrl_single_node tcp_offset
   ```
   （零 offset 时机器人反馈对应 `link6`，非零时反馈描述实际 TCP。D455 外参始终相对 `base_link`；Dabai 外参相对采集时的 TCP。元数据记录 offset 与端点。）
3. 采集内参：
   ```bash
   ./scripts/run_component.sh calibrate-intrinsics --camera d455
   ```
   至少 15 帧（推荐 20～25 帧），**多个倾角/距离/画面区域**，务必覆盖**四角与中心**。
4. 交互：`Enter` 采样、`d` 回退、`q` 计算并存盘；检查打印的候选路径与质量报告。
5. 批准内参：
   ```bash
   ./scripts/run_component.sh approve-intrinsics --camera d455 \
     --candidate /home/lxf/agx_arm_ws/data/calibration/d455/intrinsics/<会话>/candidate.yaml
   ```
6. 内参批准后，将板**刚性装在末端**，确认板与末端无相对运动。由操作者扶稳机械臂后执行现有重力补偿工具：
   ```bash
   source /home/lxf/agx_arm_ws/scripts/runtime_env.sh
   /usr/bin/python3 /home/lxf/agx_arm_ws/enable_gravity_comp.py
   ```
7. 前台运行手眼标定：
   ```bash
   cd /home/lxf/agx_arm_ws
   ./scripts/run_component.sh calibrate-handeye --camera d455
   ```
   默认展开为：
   ```bash
   ros2 run handeye_calibration_ros handeye_calibration --ros-args \
     -p camera_id:=d455 -p mode:=eye_to_hand -p piper_topic:=/feedback/tcp_pose \
     -p image_topic:=/camera/camera/color/image_raw \
     -p camera_info_topic:=/camera/camera/color/camera_info \
     -p intrinsics_yaml:=/home/lxf/agx_arm_ws/config/calibration/d455/intrinsics.yaml \
     -p result_save_path:=/home/lxf/agx_arm_ws/data/calibration/d455/handeye
   ```
8. 多轴改变姿态与位置，停稳后 `Enter`，错误 `d`；满足数量后 `q`；检查候选判据。
9. 结果为 `T_base_camera`（`p_base = T_base_camera @ p_camera`）。
   **原生产外参保持不变**；只有质量与**现场坐标核查**通过且操作者明确选用新结果，才：
   ```bash
   ./scripts/run_component.sh approve-handeye --camera d455 \
     --candidate /home/lxf/agx_arm_ws/data/calibration/d455/handeye/<时间>_calibration.json \
     --scene-verified
   ```
10. 退出示教、移除板、恢复生产：
    ```bash
    ros2 service call /exit_teach_mode std_srvs/srv/Empty
    ```

---

## 3. Dabai DC1 eye-in-hand 完整步骤（腕载）

1. Dabai **刚性安装于手腕**；停止生产分拣/检测。内参阶段可手持板移动；进入手眼阶段前将板固定桌面，随后采样全程板不动。
2. 先按根 README §2 激活 CAN 并运行 `activate_robot.py`，再启动 `arm` 与 `camera --camera dabai`，核对彩色/CameraInfo 为 **640×480@15** 与 TCP offset：
   ```bash
   ./scripts/run_component.sh camera --camera dabai
   ```
   （Dabai 工具会额外先加载 Orbbec overlay；不指定深度 profile，不用 `dabai_dcl.py` 的 frame_sync。）
3. 采集并批准本相机内参：
   ```bash
   ./scripts/run_component.sh calibrate-intrinsics --camera dabai
   ./scripts/run_component.sh approve-intrinsics  --camera dabai --candidate <候选绝对路径>
   ```
4. 将板固定桌面。扶稳机械臂后进入示教（与 D455 使用同一工具），然后改变腕载相机位置与姿态，前台执行：
   ```bash
   source /home/lxf/agx_arm_ws/scripts/runtime_env.sh
   /usr/bin/python3 /home/lxf/agx_arm_ws/enable_gravity_comp.py
   ```
   ```bash
   ./scripts/run_component.sh calibrate-handeye --camera dabai
   ```
   展开为 `mode:=eye_in_hand`、Dabai 的 image/info 话题、相同 TCP 话题，
   `intrinsics_yaml=config/calibration/dabai/intrinsics.yaml`，
   `result_save_path=data/calibration/dabai/handeye`。
5. 采集 20～25 组，`q` 求解，保存 `T_tcp_camera` 与 offset/质量报告。
   本轮已有22:53:28的passed结果，现已按用户要求启用；元数据必须保留采集时的offset与parent。
6. 操作者退出示教：
   ```bash
   ros2 service call /exit_teach_mode std_srvs/srv/Empty
   ```
   通过并且现场核查的候选可：
   ```bash
   ./scripts/run_component.sh approve-handeye --camera dabai --candidate <候选> --scene-verified
   ```
   重新推广后须重启Dabai检测/TF/GraspNet，并检查MoveIt相机碰撞体。不能只改parent字符串来兼容新TCP。

### 本轮腕载坐标接入

本次Dabai标定采集时offset=0，所以结果是`T_link6_camera`：

```text
T_base_camera(t_image) = T_base_link6(t_image) @ T_link6_camera
                      = T_base_tcp_feedback(t_image) @ inverse(S) @ T_link6_camera
```

D455保持`base_link→camera_color_optical_frame`；Dabai发布`link6→dabai_color_optical_frame`，两相机驱动均禁用自带TF。
当前配置和完整抓取/切换教程见[dual_camera_grasping.md](dual_camera_grasping.md)。
如果将来在非零TCP下重新做Dabai标定，新结果的parent可能为tcp_link，应依据元数据严格转换；不得复用旧的parent解释。

---

### 驱动 CameraInfo 显式模式

推荐先采集并批准本相机内参。若尚无批准文件且操作者决定使用驱动内参，须显式指定：

```bash
./scripts/run_component.sh calibrate-handeye --camera d455 --ros-args -p use_camera_info:=true
# Dabai 同理：把 camera 改为 dabai
```

这会记录来源 `camera_info`。已有文件若非法、未批准或相机不匹配，会明确失败，不能用该选项静默绕过。
手眼节点启动时会读取 T1 的 `/agx_arm_ctrl_single_node/get_parameters`：`tcp_offset` 必须是六个有限 double，
服务未就绪会失败。零 offset 的端点是 `link6`，非零为 `tcp_link`；采样与审批记录实际 offset。
采集之前确认 TCP 配置，采集过程中保持该值不变。T1 只在启动时向 SDK 应用 offset，
不要用 `ros2 param set tcp_offset` 在线修改后继续采集；改变TCP必须重启T1并重新核对反馈语义。

## 4. 双相机参数对照

| 项目 | D455 | Dabai DC1 |
|---|---|---|
| 安装 | 工作区外部固定 | 手腕固定 |
| 板 | 末端刚性固定 | 桌面固定不动 |
| mode | `eye_to_hand` | `eye_in_hand` |
| image_topic | `/camera/camera/color/image_raw` | `/dabai/color/image_raw` |
| camera_info_topic | `/camera/camera/color/camera_info` | `/dabai/color/camera_info` |
| optical frame | `camera_color_optical_frame` | `dabai_color_optical_frame` |
| piper_topic | `/feedback/tcp_pose` | `/feedback/tcp_pose` |
| 彩色配置 | 640×480@30 | 640×480@15, MJPG |
| 求解结果 | `T_base_camera` | `T_tcp_camera` |

---

## 5. 质量门（批准判据）

### 内参批准门

- ≥15 有效帧（推荐 20～25 帧，多个倾角、距离和画面区域）
- 整体重投影 **RMS < 0.5px**
- **3×3 覆盖网格的中心和四角都被保留帧覆盖**（不能用"覆盖数量≥5"代替）
- 每帧记录覆盖 cells；`d` 回退后**按保留帧重新计算**（唯一覆盖帧撤回则该格重新判为未覆盖）
- 图像/CameraInfo/输出**尺寸一致**、数据有限、相机与板信息完整

### 手眼保留标准

- ≥15 有效姿态（推荐 20～25）
- 每帧 `markers≥14`、`corners≥20`、`RMS<0.5px`、`square≥25px`
- 停稳 0.8s；图像/位姿时间差 <0.05s；`holdout=4`
- `consistency_deg<0.02`、`median_rel_angle_deg≥20`、`n_pairs_ge25deg≥10`
- 留出验证确实执行，`median_mm<15.0`（按用户粗标定需求调整；`p95/max`仍如实报告）

### 新鲜度与同步（采集时强制）

- `max_message_age_s=1.0`：图像/CameraInfo/机器人位姿按**接收 monotonic 年龄**校验
- 接收序号识别新图像，**拒绝重复采样**；一次采样取**锁保护的输入快照**
- `sync_time_source` 默认 **`receive`**：按接收差 <50ms，元数据明确标注"**非硬件同步**"
- 显式 `header` 模式：要求有效**同域**时间戳；header 超限**直接拒绝**（不再用接收近掩盖）
- 位置单位 m、RPY 来源 rad、四元数顺序 **xyzw**（不再次除 1000）

---

## 6. TF 方向与数学约定

- 用 `^A T_B` 表示把 B 系点变换到 A 系。
  D455 得到 `^base T_camera`；Dabai 得到 `^tcp T_camera`。
  对应 parent→optical 的 TF **直接使用结果 R,t**。
- **完整逆**为 `[Rᵀ, -Rᵀt]`。旧教程"平移原样、仅四元数取共轭"是**错误**的，已删除。
- 旧 `verify_handeye.launch.py` 曾把 optical 外参直接发布给 `camera_link`（坐标系错误），已修正为
  发布到正确 optical child frame，并支持完整逆：
  ```bash
  # 直接使用活动外参（D455 复用统一外参入口）
  ros2 launch handeye_calibration_ros verify_handeye.launch.py
  # 离线/候选检查：显式覆盖 + 反向（完整逆）+ 不开 RViz
  ros2 launch handeye_calibration_ros verify_handeye.launch.py \
    calibration_file:=/abs/path/candidate.json invert:=true rviz:=false
  ```
- 验证候选 TF 时，先停原 `camera_tf` 和任何已有验证 TF，避免同一 child 多个发布者。
  Dabai 候选验证可使用 `camera:=dabai calibration_file:=<候选绝对路径>`；此时相机驱动需以
  `./scripts/run_component.sh camera --camera dabai -- publish_tf:=false` 重启，避免驱动自带 TF 冲突。
  若使用 `invert:=true`，确认反向 child 在当前 TF 树中没有其它父节点；该选项主要用于独立数学核查。
- 默认生产 TF 入口仍为 `launch/eye_to_hand_camera_tf.launch.py`（内部统一读配置，
  允许 `calibration_file` 显式覆盖）。
- 合成真值测试（固定种子、非零平移、非平凡旋转、≥2 非平行轴、容差 1e-4、q/-q 等价）见
  `tests/test_handeye_math.py`；其中也验证了"共轭而不改平移"必须失败。

---

## 7. 批准（推广）规则

- `approve-intrinsics`：仅接受满足内参批准门的候选（≥15 帧、RMS<0.5px、3×3 覆盖）。
- `approve-handeye`：仅接受满足手眼保留标准的候选，且必须由操作者显式加 `--scene-verified`。
- 推广要求相机、模式、帧端点、分辨率、质量和来源**完整**；任缺一项拒绝。
- `criteria.capture_frames` 保存每个保留帧的 markers/corners/RMS/square 像素原值，生成和审批分别按固定14/20/<0.5/≥25核验。
  调低采样门只用于诊断，不会降低推广门；缺少逐帧质量的旧候选需重新采集或用完整记录重新求解，不直接推广。
- 审批重新核验帧数、真实数值、覆盖、留出结果与坐标端点，不能只凭 `passed` 标签推广。
- 推广锁覆盖读取与写入；先把旧文件及 manifest 保存到 `config/calibration/approved_backup/<时间_UUID>/`，
  再原子替换并追加审计。manifest 更新失败会恢复旧活动文件；同秒两次批准不会覆盖备份。
- **已有运行节点不会自动重载**：先结束任务，再重启受影响的检测/GraspNet/TF 并重新核对完整矩阵。
- 历史文件不具备新质量元数据，标为 `legacy_production_baseline`，不伪造其已通过新判据。
- 默认生产 preflight **不把**尚未生成的 Dabai 结果或新标定内参当作 D455 抓取的必需依赖。

---

## 8. 故障排查（标定相关）

| 现象 | 原因 | 处理 |
|---|---|---|
| 手眼启动即失败，提示"尚未批准内参" | `config/calibration/<cam>/intrinsics.yaml` 不存在 | 先 `calibrate-intrinsics` 并 `approve-intrinsics` |
| 采集提示"图像过期/同一帧图像" | 相机停更，或原Best Effort图像订阅在本机标定节点中停止更新 | 先看图像是否持续发布；使用第9节已修复程序，D455应显示`QoS: reliable`，不要放宽时效门限 |
| 频繁"质量门不通过" | 运动模糊、过暗/过曝、板太小或不全 | 停稳、调曝光、靠近或换大板；保证完整入镜 |
| `consistency_deg` 超 0.02 | 姿态跨度不足/共轴 | 补绕两个轴的 ±25~35° 倾斜与绕光轴自转 |
| 内参 RMS≥0.5px 或覆盖不足 | 角点样本少、四角未覆盖 | 按 `[NG]` 项补采四角与中心、多倾角 |
| 批准被拒"质量状态非 passed" | 候选未达标 | 重新采集到达标；不要绕过门禁 |
| 手眼批准被拒"必须 --scene-verified" | 缺少现场坐标核查确认 | 完成现场核查后由操作者显式添加 |
| `dabai.enabled=true` 启动被拒 | 生产 detector 禁止占位 base 坐标 | 保持 `dabai.enabled=false`；用标定入口验证 Dabai |


## 9. 2026-10-17修复：D455图像停在第一帧

### 9.1 问题与修复

本次日志中图像接收年龄达到74～82秒。现场只读复现时，位姿约200Hz、CameraInfo约30Hz正常更新，
但图像序号停在1；后台线程持续运行，因此不是等待回车造成线程阻塞。
独立图像订阅能正常收图，换专用executor或关闭调试图未解决标定节点的停更。
仅将图像订阅从Best Effort改成Reliable后，原节点10秒连续收到292帧，图像接收年龄约37～48ms。
该对比确认了此节点配置下的图像传输问题，未断言所有Best Effort订阅都会失败。

- **QoS**是ROS消息的通信规则；Best Effort丢包后不补发，Reliable允许补发。
- 新共享`camera_qos.py`供手眼和内参标定使用，默认`image_reliability:=auto`。
  若相机图像发布端提供Reliable则使用Reliable；存在Best Effort发布端时用Best Effort兼容它。
  两种模式都只保留最近1帧，避免图像排队。
- 启动时最多等待发布端发现3秒；仍未发现时D455使用Reliable默认，其他相机使用Best Effort默认，并提示先启动相机。
- 可显式指定`-p image_reliability:=reliable`或`best_effort`；以实际驱动发布配置为准。
  Dabai的Best Effort发布配置已做兼容逻辑检查，本次未启动Dabai硬件。
- 没有改动1秒图像新鲜度、同步差或标定板质量门；没有重启相机/T1、CAN或发送机械臂运动。
- 手眼标定Ctrl+C退出时仅在ROS仍运行的情况下关闭上下文，修复重复shutdown报错。

### 9.2 运行本会话工作树中的修复版

修改位于`/home/lxf/.codex/worktrees/parallel-dev/agx_arm_ws`，分支`codex/parallel-dev`。
主目录仍保留部署版本，尚未将本轮改动整合回去；直接在主目录运行旧入口不会加载此修复。
T1和D455相机按原有流程保持运行，结束旧手眼标定终端，然后执行：

```bash
cd /home/lxf/.codex/worktrees/parallel-dev/agx_arm_ws
export AGX_ARM_WS="$PWD"
./scripts/run_component.sh calibrate-handeye --camera d455 --ros-args \
  -p use_camera_info:=true
```

应看到日志：`图像订阅QoS: reliable, depth=1`。
`use_camera_info:=true`仅在没有批准内参文件时使用驱动内参；若已存在批准内参文件，仍按既有规则优先加载文件。
Dabai现场标定改用`--camera dabai`，图像主题、Orbbec环境和模式仍由原统一入口选择。

本次已在本树构建标定包。换机器或需重建时（不操作硬件）：

```bash
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select handeye_calibration_ros
```

本树无agx_arm_msgs时先在本树构建该消息包；不要将主目录build/install软链接过来。

### 9.3 采集与保存

1. 等待图像更新，将标定板完整放入视野，机械臂/标定板停稳。
2. Enter采样；如果不再提示过期而提示标定板未检出或质量项不足，表示已进入正常图像检测阶段，按提示调整板的位置与姿态。
3. 采集和求解仍按第3节步骤；更换姿态时保持相机和标定板的安装关系符合眼在手外约定。
4. 本轮数据写入工作树`data/calibration/d455/handeye/<会话>`，不会覆盖主目录的实验数据或生产外参。
5. 完成现场核验后再按原批准流程发布结果；本修复没有自动应用外参。

修改前备份：`.local_backups/handeye-image-20261017-210841/before.tar`（90KiB），
包含两种标定节点和修改前操作文档。
针对性软件检查：`tests/test_handeye_receiver.py`、`test_calibration_logic.py`、
`test_board_detection.py`、`test_calibration_review_regressions.py`合计62项通过；本树标定包构建成功。
通信实测记录见`data/calibration/d455/receiver_check_20261017.json`；不等同于已完成整套手眼标定。


现场修复版30秒只读检查：连续收到897帧，逐秒检查的图像接收年龄最大0.0644秒；
位姿及CameraInfo也持续更新。统一入口真实启动后，在等待回车时按Ctrl+C，退出码为0，
没有再次出现`rcl_shutdown already called`。检查未发送运动、未保存标定样本。

清理：已确认本轮探测目录只含空目录，并用`gio trash`移入Ubuntu回收站：
`data/calibration/d455/handeye_receiver_probe`（20KiB）、`.runtime/handeye_receiver_check`（8KiB）、
`data/calibration/d455/handeye/2026-10-17_21-10-34_session`（4KiB），共32KiB。
通信检查JSON、日志、备份和必要构建产物保留；主目录原实验会话未改动。


## 10. 方格像素门限调整为25px（2026-10-17）

按用户要求，手眼采集默认值、求解时逐帧复核和approve-handeye批准统一采用**方格边长≥25px**。
三个环节共享`validation.MIN_HANDEYE_SQUARE_PX=25.0`，避免采集通过后被旧40px标准拦截。
重投影RMS、标记/角点数、同步和最终手眼误差判据保持原值。

退出正在运行的旧标定进程，再在本会话工作树重新启动：

```bash
cd /home/lxf/.codex/worktrees/parallel-dev/agx_arm_ws
export AGX_ARM_WS="$PWD"
./scripts/run_component.sh calibrate-handeye --camera d455 --ros-args \
  -p use_camera_info:=true -p min_square_px:=25.0
```

程序默认已是25.0；显式参数用于清楚记录本次设置。先前21.6px仍低于25px，需让板稍靠近，
在板倾角大致相同的情况下，距离约缩至原来的86%即可接近25px。相机保持固定。
正在运行的节点不会自动采用源码修改，须退出并重新启动标定程序；T1和相机保持原有运行状态。

修改前备份：`.local_backups/handeye-square25-20261017-212301/before.tar`（100KiB）。

本轮针对性验证60项通过，包含25/30/40px接受和24.9px拒绝；本树标定包重建成功。
本次两轮测试产生的`/tmp/pytest-of-lxf/pytest-3`和`pytest-4`已确认无运行锁并移入Ubuntu回收站，临时文件内容合计185592字节（约181.2KiB）；真实实验数据、备份和必要构建产物保留。


## 11. 启用2026-10-17_21-32-01结果及后续误差标准

### 11.1 本次已启用的外参

用户已现场验证并明确授权用于YOLO与抓取。本次已通过原批准工具更新以下两套活动配置及manifest：

- 主部署：`/home/lxf/agx_arm_ws/config/calibration/d455_eye_to_hand.json`。
- 本工作树：`/home/lxf/.codex/worktrees/parallel-dev/agx_arm_ws/config/calibration/d455_eye_to_hand.json`。

```text
position (m): [0.6163013695268662, -0.09175253586745957, 0.6072941375905626]
orientation (xyzw): [-0.6936988665644966, -0.7054387399658206, 0.12251711991558956, 0.07827912882855059]
parent: base_link
child: camera_color_optical_frame
```

原始`2026-10-17_21-32-01_calibration.json`保持原样，包含旧标准下的failed及真实误差；
`2026-10-17_21-32-01_calibration_accepted.json`保存按当前标准重新检查的结果、原始检查记录、用户现场接受原因。
没有更改拟合出来的平移、旋转、29组采样或实际误差。

本次留出验证中位11.7811mm、P95 15.0657mm、最大15.3025mm；姿态跨度中位23.9113°。
留出验证是用未参加拟合的姿态检查结果，不等同于用户对某个抓取点的现场观察。
这次接受属于用户要求的粗标定精度，不表示原1mm标准已通过。

### 11.2 后续统一标准

| 项目 | 原标准 | 新标准 |
|---|---:|---:|
| 留出平移中位误差 | <1mm | <15mm |
| 相对姿态转角中位 | ≥25° | ≥20° |
| 单格图像边长 | ≥40px（随后已调为25） | ≥25px |

双算法一致性<0.02°、至少15组、至少10个≥25°运动对、逐帧重投影误差<0.5px、
标记/角点数及输入新鲜度等检查保持原值。`n_pairs_ge25deg`仍确切计数≥25°运动对，没有随20°中位门悄悄改变含义。
求解报告、候选检查和批准工具引用axxb.py的同一组误差/姿态常量，新候选记录validation_thresholds。

后续标定和批准使用本工作树入口；主目录本次仅更新活动外参及审计配置，未整合标定程序源码。

### 11.3 抓取与YOLO实际加载路径

- 主目录YOLO的D455默认平移和四元数来自`runtime_config.load_handeye("d455")`。
- GraspNet抓取服务从同一个活动JSON构建相机→基座矩阵。
- 相机TF启动文件从同一JSON构建base_link→camera_color_optical_frame。
- 旧`orange_dataset/realsense_yolo_node.py`是包装入口，会转入主目录vision程序。
- 动作控制节点接收已转换的目标或GraspNet结果，不另存一份相机外参。

实际默认值/矩阵加载器的离线核对记录：`data/calibration/d455/production_activation_20261017.json`。
原YOLO额外偏移仍为X=-35mm、Y=-20mm、Z=+30mm、depth=0，它们会叠加在坐标变换结果上，
属于另行调节的补偿/高度参数。本次仅更换手眼外参，未擅自清零这些生产参数。

T2b、T3、T4在启动时加载外参；已经运行的实例需要停止后重新启动对应入口，T1无需因为相机外参更新而重启。
本次检查时ROS图只有相机和手眼验证TF，没有运行YOLO或抓取执行节点；没有自动启动抓取。
在原七终端流程中启动T2b/T3/T4即会读取此结果。验证TF与生产TF不可重复发布同一个父子变换。

### 11.4 备份与回退

修改前源码/配置备份：`.local_backups/handeye-enable-20261017-213834/before.tar`。
主部署旧外参和manifest自动备份在：
`/home/lxf/agx_arm_ws/config/calibration/approved_backup/2026-10-17_21-39-48_a6f172f0b6a54c36b08bc3e57b487931/`。
工作树对应备份在：
`config/calibration/approved_backup/2026-10-17_21-39-41_0f263186f52643049fca9637a8d9d9d2/`。

需要回退时，先结束相关视觉/TF节点，将对应备份目录内的外参JSON和manifest一起恢复到原config/calibration目录，
再重新启动相关节点；不要只恢复其中一个文件。

软件验证67项通过，工作树标定包构建成功；主部署YOLO实际默认值、GraspNet矩阵加载器、TF参数构建器
均在未加载模型/未启动节点的情况下核对，与此次候选完整平移和旋转一致。

本轮临时测试目录`.runtime/pytest-of-lxf/pytest-0`、`pytest-1`、`pytest-2`（文件内容共484802字节，约473.4KiB）已确认无运行锁并随父目录移入Ubuntu回收站；备份、标定会话、诊断报告及必要构建产物保留。


## 2026-10-18 D455抓取偏差：内参与手眼一致性复核

此次只读抓取记录中，SDK反馈TCP相对目标XYZ约为(+0.67,+0.38,+0.71)mm。内部跟踪正常不代表真实指尖/TCP标定一定准确，但不支持用控制命令增加固定XY偏置。D455板面RGB距离与原始深度在远、近处存在约1.9%差异；旧29帧重算得到多组不同内参，小像素误差并不意味着度量距离正确。详细数据见 `docs/d455_depth_vs_handeye_20261018.md`。

### 先补采D455彩色内参

内参是“图像像素如何对应空间射线”的参数；手眼外参是“相机坐标如何转换到机械臂基座”的参数。顺序应是内参确定后再算手眼，避免外参吸收内参误差。

1. 相机保持当前640×480，不改变D455支架。停止当前抓取任务；本步骤只需相机持续出图，不需要机械臂运动，也不需要重新激活CAN或回零。
2. 使用现有224×160mm、32mm方格/24mm标记的7×5 ChArUco板，尽量贴平在硬底板上。板不平会使图像测距和手眼结果受影响。
3. 从主运行目录启动现有内参工具：

   ```bash
   cd /home/lxf/agx_arm_ws
   ./scripts/run_component.sh calibrate-intrinsics --camera d455
   ```

4. 手持板采20～30张：分布在中心和画面四角，尤其补足下半部；同时改变距离和绕横/竖轴的倾角（例如约±20～35°）。不要只把同一正对姿态平移，也不要只有绕画面旋转180°。板不必每次填满图像，但角点需清晰，移到位置停稳再按Enter；每张间改变板姿态。`d`撤回误采，至少15张后`q`求解保存。
5. 保留终端打印的candidate.yaml路径及同目录原始图片。**本次排查先提供候选路径供复核**：用未参与求解的板图、RGB-D快照检查度量一致性，再选用。不要把当前离线试算的390或399直接填入运行参数。
6. 选用通过复核的内参时，现有命令为：

   ```bash
   ./scripts/run_component.sh approve-intrinsics --camera d455 --candidate /实际会话路径/candidate.yaml
   ```

   工具会备份旧文件。新文件为 `config/calibration/d455/intrinsics.yaml`。需要重启水果视觉节点；日志应显示该文件为“内参来源”。没有该文件时仍显示camera_info并使用驱动参数。

### 再处理D455手眼

先用新内参和原29张图片/原机械臂位姿离线重算。如果仍保留约1cm的留出误差，需要重新采集手眼；单纯换求解器已无法消除原数据的不一致。重新采集时将板刚性固定到末端，多轴改变姿态、位置并停稳采样；保持T1同一TCP配置，板相对工具不能移动。相机支架全程不动。

批准内参已存在后，手眼命令使用：

```bash
./scripts/run_component.sh calibrate-handeye --camera d455
```

启动日志必须显示 `approved_intrinsics_file:...` 对应的文件来源（实际启动提示为“已加载本相机批准内参”）；不要用缺失文件下的 `use_camera_info:=true` 回退来代替此次内参校核。按本文件前述手眼步骤求解、核查后再启用相应外参。Dabai、TCP及CAN启动命令无需因本项排查变更。

### 本轮代码修正范围

D455水果视觉节点优先读取上述已批准内参，与手眼工具使用同一文件；无文件时仍使用驱动CameraInfo。三维中心反投影使用相应K/D（焦距、主点和畸变），中心定义仍为原来的支撑平面/物体高度估计，没有添加人工XYZ补偿。已在主目录部署源码，生效需重启水果视觉节点。蓝方块及GraspNet自己的RGB-D内参链尚未在本轮切换；本节现场核查针对GUI“从检测坐标夹取”的水果流程，切换其他抓取方式前还需统一其内参来源。


## 2026-10-18 13:57：39帧D455内参复核及批准

已采用候选 `data/calibration/d455/intrinsics/2026-10-18_13-51-08_session/candidate.yaml` 为后续手眼标定的内参输入，主目录与parallel-dev树均已通过现有approve-intrinsics工具保存到 `config/calibration/d455/intrinsics.yaml`。39帧、覆盖9/9、640×480、RMS=0.298533px。K参数fx=392.393048、fy=392.110026、cx=326.448413、cy=232.238532；D=[-0.04066605,0.06412054,-0.00389668,0.00066413,-0.01451921]。

四折留出图片重投影RMS中位为0.239～0.312px；三组独立旧RGB-D板快照中，“深度减RGB板面Z”分别约−2.707、−2.254、−3.670mm（原驱动内参约+11.93、+9.30、+7.62mm）。不是零误差，但此前尺度差异已明显减小。未用这些深度值拟合或手调内参，也未改32mm方格参数。

新内参重算旧29帧手眼，留出中位10.937mm、最大13.407mm，仍有厘米量级不一致；只保存在诊断报告中，未替换活动手眼。下一步需要重新采集D455手眼：

1. 停止抓取任务，关闭动作节点T5；保持D455相机及T1正常运行。无须为了内参变更重启T1、激活CAN或改变TCP。现有在线视觉不会自动重载文件；在新手眼复核前先不进行新的精度抓取实验。
2. 把同一块平整标定板**刚性固定到末端**。手眼采样时不再手持板独立移动，板相对工具必须固定，D455支架不动。
3. 运行 `./scripts/run_component.sh calibrate-handeye --camera d455`。日志应显示加载 `/home/lxf/agx_arm_ws/config/calibration/d455/intrinsics.yaml`，fx约392.393、fy约392.110；不要删除批准文件改回驱动内参。
4. 使用用户原有示教/手动移动方式改变机械臂姿态，停稳后Enter采样。保持T1当前TCP配置，不必改成零。建议25～30组，多轴倾斜并改变XYZ位置，避免只旋转一个轴或总在同一位置；板面清晰、尽量完整入画。现有方格25px要求在大致50cm以内（倾斜后可能需更近），以界面实测像素为准。
5. 按q求解保存，把新 `_calibration.json` 路径提供给agent复核，先不覆盖旧手眼。旧内参、旧手眼及所有原始采样均保留，备份目录见报告。
6. 新手眼选用后再重启D455水果视觉/相机TF，重新只读对照检测目标与TCP反馈，并由用户现场验证真实指尖位置。不能只凭内部反馈宣布物理抓取误差解决。

本步骤不修改Dabai、TCP、CAN命令及控制源码。校核记录：parallel-dev树 `data/diagnostics/d455_new_intrinsics_review_20261018.json`。当前39帧原始图仍保留于主目录采集会话。


### 2026-10-18 主运行目录25像素修正

主目录之前仍有采集默认40px与候选校验40px，现已同步为25px。统一入口显式从config/runtime.json的handeye_gate.min_square_px传入25.0；采集默认与逐帧结果校验均使用MIN_HANDEYE_SQUARE_PX=25.0。已运行的旧采集进程需退出并重启；相机和T1无须重启。

```bash
cd /home/lxf/agx_arm_ws
./scripts/run_component.sh calibrate-handeye --camera d455 --ros-args -p min_square_px:=25.0
```

启动日志应显示“方格>=25.0px”。25px指图像中的单格边长，不是整板宽度；实物仍为32mm方格。已批准内参文件继续沿用，不要加use_camera_info回退。


## 2026-10-18 14:09：新33帧手眼复核

会话：主目录data/calibration/d455/handeye/2026-10-18_14-03-31_session，候选2026-10-18_14-07-01_calibration.json。确认使用39帧新内参；TCP端点为tcp_link，offset与当前活动参数一致。33帧方格像素25.348～37.581px，实际25px采集与逐帧校验已生效。原日志中的square>=40px是未同步的固定提示文字，已修正。

按原始33帧复算：留出索引0/11/21/32，中位12.673mm，p95=18.594mm，最大19.612mm。姿态跨度28.9°、324个>=25°运动对，不是单纯帧数不足或转角太小。作为额外离线诊断，固定全部样本、不删帧，以板原点位置拟合或使用不同旋转/平移权重拟合一对固定刚性变换，留出中位仍约11.73～12.32mm，最大约17.99～18.33mm，未发现仅换拟合权重就能消除误差的证据。

用户反馈板“固定了，但可能有松动或弯曲”。板与末端相对运动/板面挠曲是重点嫌疑；机器人模型、位姿精度、同步与图像位姿误差仍可能贡献，不能仅凭这组数据唯一归因。下一步先查看固定方式照片，改成轻质硬底板、平整贴合并牢固连接到末端，再决定是否重采。暂不要求用户盲目再采33帧。

另外查实主目录的手眼求解报告/候选校验仍是历史1mm标准，而parallel-dev和先前用户选用的配置为留出中位15mm、相对转角20°。已按既有worktree策略同步主目录axxb、候选校验与报告，并保持可用运动对计数固定按25°计算；没有为本次12.67mm结果新定阈值。日志、实际校验和配置现统一。按既有15mm策略这项中位指标满足，但最大仍19.61mm，不能将门限同步理解为精度改善。

原始候选failed状态及全部样本保持不变，没有将本次外参推广到活动文件，没有发送运动。TCP位置范围打印的是eye-to-hand求解中取逆后的坐标，不是物理TCP突然处于台面以下；原始base位姿在T_g2b_raw中。详细复算记录在parallel-dev/data/diagnostics/d455_handeye_33frame_rigidity_review_20261018.json。修改前备份位于parallel-dev/.local_backups/handeye-policy-sync-20261018-140942，约196.6KB。实际ROS加载模块已确认门限为25px/15mm/20°。


## 2026-10-18 14:28：选用42帧手眼结果进行抓取验证

用户明确选用2026-10-18_14-25-07_calibration.json。本次已备份旧外参和manifest，并将新position/orientation/rpy写入本树config/calibration/d455_eye_to_hand.json。相机位置[0.610466572986308,-0.08928292449370928,0.6187731911036556]m；四元数xyzw=[-0.6824158519514725,-0.715467816455973,0.11943770050796453,0.09027205724053125]。继续配套使用39帧批准内参，TCP不变。manifest标记selected_for_verification、scene_verified=false，准确记录现场物理抓取验证尚未完成。

已有进程不会自动重载：先结束当前任务并关闭GUI，退出旧T2b相机TF、T3检测、T5动作节点（若T4 GraspNet在运行也退出重启）；随后在各自终端依次运行：

```bash
cd /home/lxf/agx_arm_ws
./scripts/run_component.sh camera_tf --camera d455
# 新终端
./scripts/run_component.sh detector --camera d455 --mode fruit
# 新终端；按原流程会回待机并张开夹爪
./scripts/run_component.sh action
# 新终端
./scripts/run_component.sh gui
```

T1、相机驱动T2无需因外参替换重启，CAN/TCP无需修改。若运行目录为worktree，所有终端均使用同一worktree路径。水果检测日志核对内参来源为config/calibration/d455/intrinsics.yaml。此次先用静止的单个水果和GUI“从检测坐标夹取”复测之前的偏差，现场验证由用户触发；agent未自动启停节点或发送运动。GraspNet自己的内参来源仍需独立统一，本次选用外参不构成其精度验收。
