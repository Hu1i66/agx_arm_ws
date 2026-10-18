> **动态抓取循环提速（2026-10-19）**：放置移动用 0.8、待命移动用 0.7 的关节限值比例；上一轮已张开时跳过重复开爪（省约0.65s）；一次 `sort_dynamic` 没有下一个目标时先回到上一轮准备位，下个物体来了不用从放置点起飞；下游阈值 +0.18 m。需重启 T5。

> **动态抓取提速与下游判定放宽（2026-10-19）**：带宽 0.048 m/s 下两次失败只是“超阈值 1~17 mm”，阈值放到 +0.15 m；新增轨迹时长上限 4 s（超过说明物体已被拖出可达区，直接放弃而不是硬算 6 秒）；`velocity_scaling`/`acceleration_scaling` 提到 0.55，待命与放置移动同步变快。需重启 T5。

> **动态抓取 IK 无解修复（2026-10-19）**：离线扫描证实物体本身够得到，是 18 cm 预抓取高度把腕关节顶到极限（该高度只有≥15°倾角有解，12 cm 时 5°起就有解）。`pregrasp_m` 改为 0.12、缓存姿态加径向偏航校验、`velocity_scaling` 提到 0.4。需重启 T5。

> **动态抓取失败后重新就位再解（2026-10-19）**：原地重算会让物体越走越远、要补的距离越来越大（报告实测对齐 0.30→0.54 s、关节5加速度 24.9 rad/s²）。现在一轮失败后按物体新位置重新待命并换一套倾角再解，最多两轮；对齐时长改按 0.3 m/s² 的 TCP 加速度预算反推。需重启 T5。

> **动态抓取“够得到却判超限”修复（2026-10-19）**：报告显示速度只到限值0.93倍，是3/5关节加速度超了“舒适阈值(硬件×0.45)”。现改为：超舒适但未超硬件限值的轨迹照常执行（日志给警告并记入报告 `hard_limit_accepts`）；对齐时长按 `sqrt(6d/1.2)` 反推；采样加密到0.045s。需重启 T5。

> **动态抓取停顿与越界判定修复（2026-10-19）**：停顿来自“待命点按移动结束时刻算、轨迹启动却晚约1秒”，对齐段硬补6cm触发三次重算。现按“轨迹启动时”的位置待命、等待下沿收到3cm、去掉重复预解，停顿降到约0.3~0.4s；`intercept_s_max_m` 放到 +0.05 m，抓取点 `s=+0.01` 不再被误判越界。需重启 T5。

> **动态抓取动作连贯化（2026-10-19）**：等待从“到位后干等”改为“移动前等待”，物体进入窗口时机械臂一次走到位、随即下降；对齐段按残差自适应（≤3 cm 用 0.30 s）。报告新增 `pre_wait_s`。需重启 T5。

> **动态抓取“部分物体没抓/规划慢”修复（2026-10-18 23:50）**：下游判定从 `待命点±0.07 m` 改为物理上界 `-0.06 m`（物体只要没越过这条线就继续抓）；复用上一轮成功的倾角与深入量，命中只要 2 次 IK；深度退回改到倾角内层，避免重扫 7 个倾角；停带兜底由 GraspNet 改回质心静态抓取，去掉 15 s 超时。需重启 T5 与 T6。

> **动态抓取“总是求解失败”修复（2026-10-18 深夜）**：报告显示对齐段峰值加速度已达 1.46/1.50，且失败重算时物体继续前进，越算越远；另有待命点未算观测滞后导致“已越过可抓区”。调整 `align_s`=0.55、`acceleration_scaling`=0.45、规划提前量改为 `上次耗时×1.5+0.4`，待命点补上观测滞后，失败信息改为直接给出原因。需重启 T5。

> **动态抓取改为拦截区待命（2026-10-18 晚）**：机械臂先到固定拦截区上方 180 mm 待命，等水果进入 ±0.07 m 窗口再同步下降，不再按“几秒后的位置”追目标；这解决了夹爪挡住 D455 视线、以及预测点被推到 `y=+0.30 m` 工作区外的问题。速度用 STM32 读数加视觉沿带残差校准（实测 GUI/真实带速比为 86%~97%，无固定比例）。需停止任务后重启 T5/T6，详见[动态抓取说明](docs/dynamic_grasping.md)。

> **动态抓取单轮失败不再关闭模式（2026-10-18）**：一次IK无解/规划未收敛只放弃本轮目标，GUI保持RUNNING、传送带不停，等下一物体自动再抓；连续失败5次才交回idle。对齐阶段时长按起步距离自适应，避免远距离起步被限速放大导致拦截点跑到可达区外。需停止任务后重启T5与T6，详见[动态抓取说明](docs/dynamic_grasping.md)。

> **动态抓取持续等待与预测微调（2026-10-18）**：空场景继续由GUI等待下一物体，不报失败；预定位后用正常视觉帧估计沿带速度残差，减少预测超前。STM32原读数和GUI显示保持。停止任务后重启T5即可，启动仍会回待机并张爪。详见[动态抓取说明](docs/dynamic_grasping.md)。

> **动态抓取位置校正更新（2026-10-18）**：派发前用正常视觉观测校正XY，忽略遮挡深度跳变；完整移动路径碰台面时按原5mm步长退让深入量并重新检查。需停止任务后重启T5，启动仍会回待机并张爪。详见[动态抓取说明](docs/dynamic_grasping.md)。

> **动态抓取计时更新（2026-10-18）**：固定同轮规划的启动时刻，分阶段调整超限时间；接近前充分张爪，并记录实际开口。短暂遮挡时沿用已锁定模型。需停止任务后重启T5；T5启动仍会回待机并张爪。操作与回放结果见[动态抓取说明](docs/dynamic_grasping.md)。

> **动态抓取遮挡修复（2026-10-18）**：预定位前锁定水果三维点，夹爪进入视野后按STM32速度预测，避免遮挡深度改写抓取中心。需停止任务后重启T5，启动仍会回待机并张爪；其余节点无需为此补丁重启。详见[动态抓取说明](docs/dynamic_grasping.md)。

> **2026-10-18 动态抓取方向修复**：已修正旧传送带方向文件缺失M2参考位时被错误反向的问题。
> 当前现场正常输送为画面左→右、基座+Y，M2=0。STM32速度原值不变。
> 动态预解先检查碰撞，并在同一小倾角内退让深入量；张爪后更新预定位坐标。
> 本补丁需要停止任务后重启T5/T6，T5启动仍会回待机并张爪。详见[动态抓取说明](docs/dynamic_grasping.md)。

# 智能仓储分拣系统 — 主工作空间运行指南

## USB/STM32掉线恢复

若传送带出现`Input/output error`，同时机械臂报`-4`或TF断开，先检查USB与CAN。
本次已修复STM32失效句柄清理和GUI后台重连，重连不自动启动传送带。
CAN重新枚举后仍需按原T0→T1流程恢复，见[USB恢复说明](docs/usb_recovery.md)。

## 当前稳定版：双相机静止抓取（2026-10-18）

D455与Dabai的普通水果抓取已由用户现场确认可用。支持圆形和长条物体：自动选择竖直或小倾角，长条物体沿短轴夹持，沿工具+Z接近，无需手调GUI的RPY。优先额外深入2cm，受限时按20→15→10→5→0mm尝试；接近每航点0.05s，重试同步更新XYZ。最终深入量以动作日志为准。

日常运行目录为`/home/lxf/agx_arm_ws`。CAN激活和`activate_robot.py`仍按下方T0顺序执行；T1启动会应用当前TCP，不能用在线参数修改代替重启。D455采用2026-10-18 14:25:07手眼结果与本次39帧内参；Dabai采用2026-10-17 22:53:28手眼结果。当前TCP为15组固定点标定的两指尖中点。

完整启动、相机切换、观察位及抓取步骤见[双相机抓取教程](docs/dual_camera_grasping.md)；版本范围、外部依赖和Git恢复方式见[稳定版说明](docs/releases/2026-10-18-stable.md)。后面的日期小节保留历史记录，参数冲突以当前稳定版及教程末节为准。

本 README 是**日常入口**。之前的文档已恢复为 [`README_previous.md（旧版）`](README_previous.md)。详细标定步骤见 [`docs/calibration.md`](docs/calibration.md)，
本次整合的迁移证据见 [`docs/migration_report.md`](docs/migration_report.md)，
后续审查与修正记录见 [`docs/review_2026-10-14.md`](docs/review_2026-10-14.md)，
2026-10-15 的标定专项审查及 TCP 工具说明见 [`docs/calibration_review_2026-10-15.md`](docs/calibration_review_2026-10-15.md)。

## D455 动态抓取改进（待接带现场验收）

动态抓取采用当前帧质心和同一目标编号，保持 STM32 速度原值，不应用旧 `speed_scale`。
预定位改为“固定拦截区上方待命”，待命点纵向固定（`intercept_s_pref_m ± intercept_window_m`），
等水果进入窗口后再同步下降；已越过窗口的物体本轮直接放弃，不追到工作区外。
待命高度 180 mm，夹爪不再贴着物体挡住 D455。速度 = STM32 读数 + 视觉沿带残差（限 ±20%）。
复用静止抓取的短轴与倾角选择，预定位前预解完整移动轨迹；接近阶段固定姿态，沿物体随动坐标系的工具+Z接近，
夹爪闭合期间继续跟随，随后抬起、放置。接近/随动/抬起共用一条保留时间戳和连续关节速度的轨迹。
普通静止抓取、GraspNet、CAN激活、TCP及手眼配置保持原有用法。

配置见 `config/dynamic_grasp.json`，完整步骤与离线预览见 [动态抓取教程](docs/dynamic_grasping.md)。
本轮未连接传送带、未执行实机运动，不能把离线结果当作现场成功率。
更新后需正常关闭并重新启动 T3检测、T5动作、T6界面；T5原有启动行为会回待机并张开夹爪，重启时留出空间。

---

## 1. 环境解释与依赖路径

本工作空间**不新建、不合并虚拟环境**：控制、视觉、标定各自沿用现有解释器，由统一入口显式选择。

| 用途 | 解释器 / 环境 | 说明 |
|---|---|---|
| 控制、GUI、动作、GraspNet 依赖的 ROS 侧 | `/usr/bin/python3`（3.10.12） | 系统 Python + 用户库（`~/.local/lib/python3.10/site-packages`） |
| 视觉（YOLO / 蓝方块）与 GraspNet 运行时 | `/home/lxf/orange_dataset/.venv/bin/python3` | 复用共享虚拟环境，**不迁移、不重建、不升级** |
| 标定（内参/手眼） | `/usr/bin/python3` + 用户 OpenCV(≥4.7；本机 4.13)/NumPy/SciPy | 不进入旧 handeye 的两个 venv |
| IK / Pinocchio | `~/.local/lib/python3.10/site-packages`（cmeel pinocchio 3.9.0） | 控制代码保持 NumPy → IK 路径注入 → 延迟 Pinocchio 导入顺序 |

**DDS 统一**：所有组件显式使用 `rmw_cyclonedds_cpp` 与项目内 `config/dds/cyclonedds.xml`；
`ROS_DOMAIN_ID` 保留调用者值（未设置则 ROS 默认 0）；**不 source `~/.bashrc`**。

**关键路径**（全部记录在受 Git 管理的 `config/runtime.json`，键名固定）：

| 键 | 值 |
|---|---|
| `vision_python` | `/home/lxf/orange_dataset/.venv/bin/python3` |
| `system_python` | `/usr/bin/python3` |
| `ros_setup` | `/opt/ros/humble/setup.bash` |
| `workspace_setup` | `install/setup.bash` |
| `orbbec_setup` | `/home/lxf/orbbec_ros2_ws/install/setup.bash` |
| `dds_xml` | `config/dds/cyclonedds.xml` |
| `yolo_weights` | `models/yolo/fruits12.pt` |
| `graspnet_checkpoint` | `models/graspnet/checkpoint-rs.tar` |
| `graspnet_baseline` | `/home/lxf/graspnet/graspnet-baseline` |
| `d455_handeye` | `config/calibration/d455_eye_to_hand.json` |
| `ik_site_packages` / `cmeel_site_packages` | `~/.local/lib/python3.10/site-packages` / `.../cmeel.prefix/.../site-packages` |
| `ik_urdf` | `src/agx_arm_ros/src/agx_arm_moveit/config/test2.urdf` |
| `saved_poses` / `conveyor_calib` | `saved_poses.json` / `conveyor_calib.json` |

部署来源/哈希/外部实现路径另见 `config/deployment_manifest.json`。
大权重与环境不进 Git 大文件历史；**fresh clone 不含环境与权重**，需按下文"回退/部署"恢复部署资产。

### 只读预检（推荐在开工前执行）

```bash
cd /home/lxf/agx_arm_ws
./scripts/run_component.sh preflight            # 分组件报告缺失（只读）
./scripts/run_component.sh preflight --live     # 附加 live 接口查询
```

`--live` 会区分**"接口存在"**与**"模型就绪"**：GraspNet 的 req/result 话题存在**不等于**模型已加载；
托管进程以日志出现"就绪, 等待请求..."才算就绪，外部手动进程需人工核查其终端日志。

---

## 2. 硬件激活与六终端启动

**完整顺序：连接并上电 → 激活 CAN 接口 → 运行机械臂激活脚本 → T1 → T2 → T2b → T3 → T5 → T6。**

> GraspNet（T4）已退出主抓取流程，日常不需要启动。当前抓取走“D455/Dabai 质心 + 短轴 + 工具轴接近”，
> 需要 6DoF 位姿时才在 GUI 勾选“启用 GraspNet 6DoF”并自行启动 T4。

### T0：启动 ROS2 前先激活 CAN 和机械臂

连接机械臂并上电后，在终端按顺序执行；前一步成功后再执行下一步：

```bash
cd /home/lxf/agx_arm_ws

# T0a：激活 CAN 接口，名称 can0，波特率 1 Mbps；脚本内 sudo 可能要求密码
bash ~/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh can0 1000000 && cansend can0 123#1122334455667788 && ip -s link show can0

# 可选的详细确认：标志包含 UP、bitrate 1000000，且 CAN 状态不是 BUS-OFF
ip -details link show can0

# T0b：唤醒机械臂、切换 CAN 控制模式、发送电机使能指令
/usr/bin/python3 /home/lxf/agx_arm_ws/activate_robot.py
```

`activate_robot.py` 固定使用 `can0`，依次发送待机、CAN 控制和 `EnableArm(7)` 指令。
等待脚本执行完毕，再启动下面的 T1。脚本的完成提示表示激活指令已发送，实际反馈仍需在 T1 后核对。
仅激活 Linux CAN 接口还没有完成机械臂使能；`./scripts/run_component.sh arm` 只负责启动 ROS2，
不能省略前面的激活脚本。重新上电或重新连接后，按上述顺序重新准备。

多 CAN 设备的接口选择见 [`CAN 配置指南`](src/agx_arm_ros/docs/CAN_USER.md)。

### T1～T6：六终端启动

每条命令在**独立终端**执行，先 `cd /home/lxf/agx_arm_ws`。

| 终端 | 命令 | 就绪判据 |
|---|---|---|
| T1 | `./scripts/run_component.sh arm` | MoveIt、控制及机器人反馈就绪 |
| T2 | `./scripts/run_component.sh camera --camera d455` | RGB、对齐深度、CameraInfo 更新，`publish_tf=false` |
| T2b | `./scripts/run_component.sh camera_tf` | base→optical 完整矩阵与活动外参一致 |
| T3 | `./scripts/run_component.sh detector --mode fruit` | 检测消息更新（空场景 `detected=false` 合法） |
| T5 | `./scripts/run_component.sh action` | 分拣执行节点就绪 |
| T6 | `./scripts/run_component.sh gui` | 图像/状态/控制正常 |

可选：`./scripts/run_component.sh graspnet`（T4，GraspNet 6DoF 位姿服务，日志出现"就绪, 等待请求..."）。
不启动 T4 时，GUI 的 GraspNet 按钮保持灰置，其它抓取流程不受影响。

T1 底层仍是：

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=can0 arm_type:=piper effector_type:=agx_gripper \
  'tcp_offset:=[0.0015631133283368326,0.00007546221381904417,0.1484896328004612,0.0,0.0,0.0]'
```

TF 只读复核：

```bash
source /home/lxf/agx_arm_ws/scripts/runtime_env.sh
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
# 平移应接近 [0.610467, -0.089283, 0.618773]，同时核验完整旋转
```

**手动各入口默认前台监督**：由启动器创建组件进程组、转发 `SIGINT/SIGTERM`、`wait` 回收并返回真实退出码。
每个入口都支持 `--dry-run`（只输出解释器/argv/配置/关键环境，不加载模型、不建目录、不启动组件、不访问 CAN/串口）。

---

## 3. GUI 使用

```bash
cd /home/lxf/agx_arm_ws && ./scripts/run_component.sh gui
```

- **单实例**：GUI 启动即取得 `.runtime/gui.lock`；已有实例运行时第二个窗口会提示并退出，
  避免两个 worker 抢占相机/串口并发布错误状态。
- **自动启动范围**：GUI 只确保 **相机 + 相机 TF + 检测** 三类组件（共用统一入口的命令构造、锁与就绪规则）。
  机械臂（T1）、动作（T5）仍按明确终端入口启动；GraspNet（T4）只在需要 6DoF 时手动启动。
- **Tk 主进程 / ROS worker 架构保留**：自动管线在 ROS worker 内、`rclpy.init` 与 Node 创建**之后**启动；
  等待就绪期间 spinner 保持运转。
- **正常退出**：关闭窗口 → 队列请求 worker 清理本会话子进程、释放串口 → 收到 ACK 再 join；
  ACK 等待与正常 join 均有超时；未收到 ACK 或进程仍未退出时才兜底 `terminate`。
  图像/状态输出队列允许丢弃最后的遥测消息，ACK 队列正常刷新，避免关闭时等待未消费的图像。
  父进程异常退出时 worker 的 watchdog 也走同一清理路径。

- 启动时复用已在线的水果或蓝方块检测，不因 GUI 打开而替换模式。检测离线时可再次点击当前模式恢复。
- 自动分拣与动态抓取当前要求**水果模式**；蓝方块使用其单次抓取按钮。

### 模式切换（水果 ↔ 蓝方块）

- `requested_mode`（请求）与 `detection_mode`（已确认/active）**分开保存**；
  只有目标节点上线、`/detection_info` 恰有一个目标发布者、且收到切换后新消息，才提交 active 模式。
- 切换失败**保持旧模式**；若旧节点已不在线，界面显示 `offline`。
- 排队执行、单次抓取等待 busy 确认、自动分拣、动态抓取或控制 busy 时**拒绝**视觉切换。
  确认框返回后重新检查占用；切换期间暂停队列派发和新抓取，迟到的旧请求结果不能提交新模式。
- **显式模式切换允许停止同 workspace 登记的 manual detector**：若 T3 是手工终端启动且已登记，
  GUI 会停止它（`--allow-cross-initiator`），**旧 T3 监督终端随之结束**，新 detector 归当前 GUI。
- **外部未登记节点**：若检测节点是旧式手工命令启动、未登记，GUI **无法证明所有权故不停止**，
  会明确报冲突并提示关闭该终端；此时不会叠加第二个检测节点。

### 依赖差异（重要）

- 经 `./scripts/run_component.sh` 启动的组件**属于受管登记**（`.runtime/registry.json`），
  可被 `stop`/GUI 按身份与归属停止。
- 直接用旧命令（如 `python3 vision/realsense_yolo_node.py ...`）启动的节点属 **external**：
  仍可运行，但**不取得 GUI 的停止/切换权限**。

---

## 4. 参数：临时（在线热更新）与永久

### 人工位置偏置已移除

当前视觉XYZ/深度偏置为0；坐标只使用相机手眼、TCP和RGB-D几何中心估计，不再使用旧XYZ/类别Z/工具长度补偿。
历史非零offset命令不再适用，启动与在线回调会明确拒绝非零人工偏置。
相机来源在GUI选择；外参由`config/calibration/manifest.json`登记，TCP由`config/calibration/tcp.json`提供。
更换TCP或相机碰撞模型后需重启T1，不能使用在线param set代替SDK启动应用。

---

## 5. 操作流程

0. 连接并上电后，先完成 §2 的 **T0a CAN 激活 → T0b 机械臂激活**，再按七终端顺序启动。
1. 等相机、检测与 GraspNet 加载就绪，再放物体。
2. 选择料框 1/2，点"单次按钮"执行 GraspNet 抓取。
3. 自动分拣：物体**左→右越 u=260 停带** → 选**最右**目标 → 抓取放置 → 继续循环。
4. 停止按钮结束自动任务；在途动作按既有状态机处理（不新增未定义的回零动作）。
5. 模式切换按登记进程替换（见 §3）。
6. 正常停止顺序：GUI 停止自动/动态任务并暂停排队 → 等待机械臂完成、确认传送带停止和状态稳定 → 正常退出 GUI →
   `Ctrl+C` 停止 T5（若启动了 T4 一并停止）→ 停止检测、TF、相机 → **最后**停止 T1。现场急停按既有设备流程。

---

## 6. 停止、状态与日志排查

```bash
./scripts/run_component.sh status camera --camera d455   # 查看登记与身份校验
./scripts/run_component.sh stop detector --mode fruit    # 按身份受控停止
./scripts/run_component.sh stop detector --initiator gui --allow-cross-initiator  # 显式模式切换
```

- 停止前逐项校验身份（PID + `/proc` starttime + UID + boot_id + PGID 归属），
  `SIGINT` 等 5s → `SIGTERM` 等 5s → 仍存活则报告失败；**不按文件名 pkill**。
- leader 退出但 launch 子孙仍占相机/话题 → 报告"未完全退出"，保留登记，**不删状态后重启第二实例**。
- 日志：`log/runtime/launcher.log`（启动/停止/就绪事件）与 `log/runtime/<组件>.log`（组件输出）。

---

## 7. 双相机标定与候选批准

TCP 工具中心位置标定见 [`docs/tcp_calibration.md`](docs/tcp_calibration.md)：

```bash
./scripts/run_component.sh calibrate-tcp
```

固定点法标定 XYZ，保留当前 RPY；仅接收位姿和保存候选。核验后用候选给出的 `tcp_offset` 启动 T1，
生效需要重启 T1；不能通过 `ros2 param set tcp_offset` 在线应用。Dabai 手眼标定须记录同一 TCP 配置。

TCP 定点姿态验证（默认离线零运动）：

```bash
./scripts/run_component.sh verify-tcp --dry-run
```

现场只规划、明确确认执行、粗标定的**仅验证质量豁免**及人工漂移记录，见
[`docs/tcp_calibration.md §7`](docs/tcp_calibration.md#7-tcp-定点姿态运动验证)。
验证不启动 T1/CAN/使能/回零；必须先结束抓取任务并关闭 T5/T6。当前粗标定的原质量仍为 failed，
未自动应用到生产。


完整步骤（含板生成、采样、质量门、批准、恢复生产）见 **[`docs/calibration.md`](docs/calibration.md)**。速览：

```bash
./scripts/run_component.sh generate-board                 # 生成 7x5/32mm ChArUco 板
./scripts/run_component.sh calibrate-intrinsics --camera d455
./scripts/run_component.sh approve-intrinsics  --camera d455 --candidate <候选绝对路径>
./scripts/run_component.sh calibrate-handeye    --camera d455
./scripts/run_component.sh approve-handeye      --camera d455 --candidate <候选绝对路径> --scene-verified
```

- 内参输出到 `data/calibration/<cam>/intrinsics/<会话>/candidate.yaml`；手眼输出到 `data/calibration/<cam>/handeye/`。
- **候选不自动成为生产**：须通过质量门并由操作者 `approve-*` 批准（手眼另需 `--scene-verified`）。
- 批准会先备份旧批准文件与 manifest，再原子替换并追加审计记录（`config/calibration/approved_backup/`）。
- 生产 D455 抓取仍使用**原驱动 CameraInfo**；迁入的新内参不会自动改写当前 K/D。
- 批准后**运行中的节点不会自动重载**：先结束任务，再重启受影响的检测/GraspNet/TF 并重新核对完整矩阵。

### 恢复生产

```bash
ros2 service call /exit_teach_mode std_srvs/srv/Empty   # 退出示教
# 移除标定板 → 按 §2 重启七终端 → 复验单次/自动分拣
```

Dabai已接入图像时刻的腕载坐标变换，TF、检测、GraspNet和GUI均可使用，见双相机教程。首轮按停带静态单次流程操作。

---

## 8. 常见错误对照

| 现象 | 可能原因 | 处理 |
|---|---|---|
| T1 启动后机械臂无反馈或无法控制 | CAN 未激活、波特率错误，或遗漏机械臂激活脚本 | 回到 §2：先激活 `can0 1000000`，再运行 `activate_robot.py`，完成后启动 T1 |
| preflight 报资源缺失 | 权重/checkpoint/外参/配置缺失或被移动 | 按输出路径恢复（见 `config/deployment_manifest.json`）；必要时从备份取回 |
| 节点互相看不到 | DDS 不一致（RMW/CYCLONEDDS_URI） | 统一用 `./scripts/run_component.sh ...` 或 `source scripts/runtime_env.sh` |
| 相机启动失败/被占用 | 已有相机进程，或另一终端已启动 | `status`/`preflight --live` 查占用；按登记停止或关闭旧终端 |
| 没收到 CameraInfo | 相机未就绪/话题名不符 | 等相机组件就绪；核验 `camera_info_topic` |
| 双 TF / 位姿错乱 | 相机自带 TF 与项目 TF 同时发布 | 相机固定 `publish_tf=false`；TF 只由 `camera_tf` 组件发布 |
| 模式切换冲突 | 旧检测节点为外部未登记 | 关闭旧终端，或用统一入口启动以便受控切换 |
| 检测信息过期 | 检测节点停止/卡住 | 查看 `log/runtime/detector_d455.log`，重启 T3 |
| GraspNet 一直未就绪 | 模型仍在加载或 checkpoint 缺失 | 查日志"就绪, 等待请求..."；核验 checkpoint |
| 标定分辨率错误 | input 图像与内参分辨率不一致 | 用同一分辨率采集（640×480）；候选会被判 failed |
| 候选质量失败 | 帧数/RMS/覆盖/同步不达标 | 按候选打印的 `[NG]` 项补采（覆盖四角+中心、多倾角） |
| 串口占用（传送带） | 旧 GUI/worker 未释放 | 关闭重复 GUI；等待 watchdog 清理；确认单实例锁 |
| 第二个 GUI 打不开 | 单实例锁被占用 | 关闭已运行窗口 |

---

## 9. 回退

1. 正常停止任务与节点（§5/§6）。
2. 先在外部目录保全**迁移后新数据**（新会话/结果/配置/saved_poses/conveyor_calib/运行日志）并生成清单哈希。
3. 优先按清单恢复原入口/配置与索引；若全量回退：先将当前空间重命名保全，
   再从已验证归档恢复到**原绝对路径**（避免直接覆盖丢新数据）。
4. 原始归档重新验哈希时，清单实际位于备份根目录，须在 `archives/` 下运行：
   ```bash
   cd /home/lxf/agx_arm_ws_backups/20261013-214530/archives
   sha256sum -c ../SHA256SUMS
   ```
   备份位置与恢复要求见 `/home/lxf/agx_arm_ws_backups/20261013-214530/BACKUP_VERIFIED.md`。
5. 旧 `/home/lxf/orange_dataset` 与 `/home/lxf/handeye` 目录保留作回退来源（其中
   `realsense_yolo_node.py` / `blue_block_detector.py` 已改为薄包装，原实现保留为同名 `.orig`）。

---

## 10. 目录速览

```
config/            runtime.json、vision/*.yaml、calibration/、dds/、deployment_manifest.json
scripts/           runtime_env.sh（环境）、run_component.sh（统一入口）
vision/            realsense_yolo_node.py、blue_block_detector.py（canonical 实现）
models/            yolo/fruits12.pt、graspnet/checkpoint-rs.tar（部署资产，不入 Git）
data/calibration/  候选与历史（小 JSON 入 Git；大图像另归档）
archive/legacy/    旧代码/资料归档（见 archive/README.md）
docs/              guides/ reports/ requirements/ worklogs/ calibration.md migration_report.md
runtime_*.py       共享配置、进程管理、启动器（仅 stdlib + 子进程 ros2）
.runtime/          本机组件登记与锁（不提交）
log/runtime/       运行日志
```


### D455接近微调（2026-10-18）

普通D455水果抓取接近速度已提高，沿工具+Z额外深入1cm；如深入路径受碰撞/可达性限制，自动保留原深度并提示。Dabai仍为原抓取流程，不应用本次速度和深度调整。重启T5生效，详细说明见docs/dual_camera_grasping.md末节。


### 双相机长条水果自适应夹持（2026-10-18）

D455与Dabai普通水果抓取现支持从物体深度点云估计长短轴：长宽比≥1.5时，两指沿短轴闭合，开度参考短边宽度；近圆形物体仍使用原径向姿态。保持自动倾角、工具+Z接近和原速度/深入量，无需手调GUI的RPY。J6会在需要时转到能夹住短边的方向。

本次修改涉及T3检测、T5动作和T6 GUI，三者需要退出重启；T1/相机驱动/TF无需因本次修改重启。完整命令、日志及现场检查步骤见[双相机抓取说明](docs/dual_camera_grasping.md)末节。此前“Dabai仍为旧流程”的历史说明已被双相机统一流程替代。


### 矮物体抓取微调（2026-10-18，当前值）

D455/Dabai普通水果抓取优先沿工具+Z额外深入2cm；路径受限时按20→15→10→5→0mm逐级尝试，保留碰撞检查。接近每航点0.05s（原0.08s），重试同步刷新XYZ。当前动作结束后重启T5生效，完整说明见[双相机抓取说明](docs/dual_camera_grasping.md)末节；实际采用深度以动作日志为准。
