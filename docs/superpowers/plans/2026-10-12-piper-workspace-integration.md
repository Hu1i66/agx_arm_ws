# 松灵 Piper 工作空间整合与双相机标定迁移设计

> **执行 agent：** 按本文件逐项实施并记录复选框。若具备 superpowers 技能，使用 superpowers:executing-plans 或 superpowers:subagent-driven-development。现场目录和当前工作树是迁移来源；每阶段完成后记录检查结果。

**日期：** 2026-10-12，Asia/Shanghai  
**版本：** 1.1，完成源码复核后的交付设计  
**Goal：** 整合在线视觉和手眼标定代码、整理主工作空间、统一启动入口，并通过回归验收确认现有 D455 抓取工作流继续可用。  
**Architecture：** 控制、GUI、GraspNet 保留现有主体；在线视觉移入主空间，环境仍复用原视觉虚拟环境。增加共享资源配置、启动入口和受控进程管理；标定迁入一个独立 ROS 包，两相机分别保存内参、会话和外参候选。  
**Tech Stack：** Ubuntu、ROS 2 Humble、MoveIt2、Python 3.10、Tkinter、YOLO、GraspNet、OpenCV ChArUco、AX=XB、D455、Orbbec Dabai DC1。  
**Spec：** 本文件同时为设计规格和实施计划，执行者无需另找口头需求。  
**交付状态：** 这是设计文件。业务代码、配置、环境和硬件尚未按本计划修改；本文件中的实施项均待执行。

## 0. 用户要求、约束与复审结论

### 0.1 必须实现

- 开工前完成完整备份及校验，通过后才能修改业务文件、配置、Git 索引或构建标定包。
- 迁入运行所需的在线视觉程序，以及当前使用的手眼标定代码和历史标定资料。
- 整理主空间的旧文件和文档，删除范围限定为明确的可再生缓存。
- 保留 T1 → T2 → T2b → T3 → T4 → T5 → T6、单次 GraspNet、越线停带自动分拣的行为。
- 根 README 写出新的启动、操作、停止、故障排查和双相机标定步骤。
- 以软件回归和操作者在场的实机验收共同证明工作流正常；缺实机条件时明确标为待验收。

### 0.2 已经确认的选择

| 项目 | 固定决定 |
|---|---|
| 清理策略 | 清缓存，旧代码和资料归档 |
| 视觉环境 | 复用 /home/lxf/orange_dataset/.venv，不迁移、不重建、不升级 |
| D455 深度滤波 | spatial、temporal、hole filling 保留开启，手动与 GUI 参数一致 |
| 参数热更新 | 修复坐标/深度补偿实际不生效的问题，默认值和算法保留 |
| 生产相机 | D455 默认开启，Dabai 默认关闭 |
| Dabai 本次范围 | 标定工具和操作流程；动态腕载视觉到基座的接入后续单独开发 |
| 控制结构 | 保留根目录主要 Python 文件，避免同时做大规模拆分 |
| 活动外参 | 保留当前 2026-09-15 的 D455 生产结果，不按文件时间挑最新结果 |

虚拟环境是独立 Python 解释器和第三方包目录。此次整合源码不要求物理合并环境；统一入口选定解释器，日常使用不需要手动 activate。

### 0.3 全局约束

- 迁移读取当前工作树，包含 dirty、untracked、ignored 资产；不得用上游克隆替代现场版本。
- 控制 /usr/bin/python3、视觉原 .venv、标定系统 Python + 用户库分别保留。
- 保留 SDK、TRAC-IK、Pinocchio/CasADi、URDF、场景、料框位姿、传送带标定和仿真资源。
- 稳定保留话题、节点名称、JSON 结构、单位、四元数顺序和抓取状态机。
- 本次不升级依赖，不修改 ~/.bashrc，不重写 Git 历史，不修复历史子模块指针。
- 自动化软件检查不启动 T1，不发送 CAN/串口运动命令，不自动进入示教。
- 大权重和环境仍作为部署资产；Git 交付与部署资产交付分别验收。

### 0.4 复审重点与修正

| 易出错条件 | 设计要求 | 对应检查 |
|---|---|---|
| 备份里存在绝对软链接或原有断链 | 比较 lstat 类型和 readlink 文本，不通过解包链接读原工程做伪校验 | 任务 1 |
| 空场景、蓝方块消息无水果时间戳 | 按模式解析，接收 monotonic 计时；空目标也就绪 | 任务 3、8 |
| GraspNet 话题先于模型加载出现 | 接口存在与模型 ready 分开，ready 依据加载完成日志 | 任务 3、8 |
| 内参失败结果仍写 YAML、回退后覆盖虚假达标 | 内参也做候选/批准隔离，覆盖按保留帧重算 | 任务 5、8 |
| GUI 被直接 terminate、双 GUI 抢串口 | worker 收到关闭请求后清理并 ACK，启动前做 GUI 单实例锁 | 任务 4、8 |

以下相对目标路径均相对 /home/lxf/agx_arm_ws；真实来源使用绝对路径。

## 1. 任务 1：开工前备份与基线冻结

### 1.1 备份范围

备份根目录：

~~~text
/home/lxf/agx_arm_ws_backups/<北京时间 YYYYMMDD-HHMMSS>/
~~~

| 范围 | 必须保存的内容 |
|---|---|
| /home/lxf/agx_arm_ws | 完整目录、隐藏文件、.git、环境、ignored 资产、build/install/log、未跟踪文档 |
| /home/lxf/orange_dataset | 完整目录、最新双相机代码、训练数据、权重、.venv、本地 Git 修改 |
| /home/lxf/handeye | 完整目录、三个嵌套仓库、新增未跟踪模块、历史结果、会话、旧环境与安装产物 |
| /home/lxf/graspnet | 完整约 206 MB 目录，包含 baseline、checkpoint、verify 脚本、saved_frames 和编译扩展 |
| /home/lxf/.local/lib/python3.10/site-packages | 整个用户包目录及 native 库；标定和控制当前实际依赖 |
| /home/lxf/ROS2-Gazebo-GO2/src/docker/cyclonedds.xml | 当前通信配置 |
| Orbbec 驱动 | Git 状态/版本及实际 launch 和配置文件；驱动工作空间留原位 |

GraspNet 中 pointnet2/setup.py 当前有本地修改，knn/pointnet2 的 native 扩展被 Git 忽略，必须包含在全量文件归档。Git bundle 只能补充保存提交历史。

### 1.2 步骤

- [ ] 只读检查现场是否仍有抓取任务、GUI 或相机进程。现场操作者按现有流程结束运行后再冻结基线；不自动杀臂控制进程。
- [ ] 重新统计常规文件字节数及可用空间。至少预留 2.2 × 全量数据字节 + 计划部署新增资产预算，不依赖压缩率。
- [ ] 当前五个主要备份目录约 18.6 GiB，/home 约 106.8 GiB 可用；/tmp 约 6.46 GiB，不放全量备份。
- [ ] 备份目录放在上述源目录之外，防止递归归档。
- [ ] 记录系统/ROS/Python/CUDA/动态库版本、解释器、关键依赖版本和来源路径。
- [ ] 为每个独立仓库记录 HEAD、remote、status、binary diff；全量归档另保存未跟踪和 ignored 内容。
- [ ] 归档前后生成 manifest：相对路径、文件类型、常规文件字节和 SHA256、软链接原始文本、权限。
- [ ] tar 使用相对成员路径，保留权限与软链接；不使用 dereference、exclude-vcs 或 ignored 排除。
- [ ] 为归档生成 SHA256，验证可列出成员，再完整解包到外部 restore-check 目录。
- [ ] 常规文件对比解包自身内容；链接用 lstat/readlink 对比，不跟随绝对链接读取原工程。
- [ ] 检查归档前后 manifest；出现“读取时改变”则重备变化内容，不忽略警告。
- [ ] 生成 BACKUP_VERIFIED.md，记录范围、大小、哈希、解包校验、外部依赖及恢复原路径要求。

**阶段完成条件：** 全量文件和链接清单一致、归档可读、关键源码/资产存在、备份前后未发生未处理变化；此后才开始业务修改。

### 1.3 特别检查

Git 记录覆盖主空间、Orange、src/trac_ik、GraspNet baseline，以及 handeye 的 handeye_calibration_ros、aruco_ros、piper_ros。

重点保存：

- Orange 已修改的 realsense_yolo_node.py 和 启动说明.md。
- 标定包未跟踪的 axxb.py、charuco_detect.py、intrinsic_calibration.py、verify_handeye.launch.py。
- src/trac_ik 的三处本地源码修改。
- saved_poses.json、conveyor_calib.json、活动手眼外参和全部历史会话。
- .venv_ik、.venv_mjx、视觉 .venv 的解释器链接、pyvenv.cfg 和依赖内容。

当前 install 中有两个历史断链，应记录而不在本次修复：

~~~text
install/agx_arm_description/share/agx_arm_description/agx_arm_urdf/piper/urdf/agx_arm_no_gripper_description_gazebo.xacro
install/agx_arm_description/share/agx_arm_description/agx_arm_urdf/piper/urdf/agx_arm_description_gazebo.xacro
~~~

旧 handeye 安装目录含大量绝对链接。恢复旧环境要恢复原绝对路径；新主空间只重新构建迁入包。历史 agx_arm_urdf 的失效 .git 指针原样保存并记录，不以 Git 操作失败为由覆盖模型资源。

## 2. 任务 2：源码、模型与标定资源迁移

### 2.1 迁移表

| 来源 | 主空间目标 | 处理 |
|---|---|---|
| /home/lxf/orange_dataset/realsense_yolo_node.py | vision/realsense_yolo_node.py | 当前完整工作树；保留双相机结构和最新修改 |
| /home/lxf/orange_dataset/blue_block_detector.py | vision/blue_block_detector.py | 当前工作树 |
| /home/lxf/orange_dataset/runs/detect/runs/detect/train/weights/best.pt | models/yolo/fruits12.pt | 原样复制已部署权重 |
| /home/lxf/graspnet/checkpoints/checkpoint-rs.tar | models/graspnet/checkpoint-rs.tar | 原样复制 |
| /home/lxf/handeye/result/2026-09-15_21-25-46_calibration.json | config/calibration/d455_eye_to_hand.json | 原样复制活动基线，保持字节和哈希 |
| 当前 CycloneDDS XML | config/dds/cyclonedds.xml | 原样复制 |
| /home/lxf/handeye/src/handeye_calibration_ros | src/handeye_calibration_ros | 当前源码、manifest、入口、launch；不复制 .git、缓存、.bak |
| /home/lxf/handeye/result 全部 | data/calibration/history/handeye | 完整复制，包括未完成和空会话 |
| /home/lxf/orange_dataset/标定/calib_results | data/calibration/history/d455_intrinsics_legacy | 历史资料，注明旧分辨率，禁止自动启用 |
| 旧 handeye 教程、ArUco 工具/板样、TF 图和 .bak | archive/legacy/handeye | 保存来源，当前教程重写 |

- [ ] 复制前后逐文件核验，记录原路径、目标和哈希。
- [ ] 新标定包迁入四个主要模块及 setup/package/resource，不漏未跟踪源码。
- [ ] 不整体复制旧 handeye/src，避免 piper_description、piper_gazebo、piper_mujoco 包重名。
- [ ] 旧 aruco_ros 不进入默认构建，当前 ChArUco 已内联检测。
- [ ] 保留旧 Orange/handeye 目录作为回退来源，验收前不删除。

### 2.2 资产基线

| 资产 | 字节数 | SHA256 |
|---|---:|---|
| YOLO best.pt | 40,549,932 | 9162f7af73c86fa9e18b0a2a50a08d5ed8e683e7bcb3f9cda9014f152a78ce3c |
| GraspNet checkpoint-rs.tar | 12,468,415 | 60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868 |
| 活动 D455 JSON | 391 | 2d0907d8333e5c4c91fe176754a4e45345b0c31ee41d4698cc720a610b7f5c4f |

开工时如原文件发生变化，先调查并记录新基线，不能用本表哈希对应的旧文件覆盖现场新文件。

建立受 Git 管理的 config/deployment_manifest.json，记录来源、目标、大小、SHA256、解释器和外部实现路径。权重与环境不进 Git 大文件历史；部署目录和完整备份另交付。活动小外参和小型 JSON 历史纳入管理，大图像会话以明确资产归档交付。新增 ignore 不使用一条 data/** 静默排除全部标定成果。

### 2.3 保留的外部依赖

- /home/lxf/orange_dataset/.venv：视觉和 GraspNet。
- /home/lxf/graspnet/graspnet-baseline：GraspNet 实现与 native 扩展。
- /home/lxf/orbbec_ros2_ws：Dabai 驱动。
- Orange 训练数据、训练代码、训练输出和相机辅助工具。
- 旧 handeye 的环境、build/install/log：回退保留，新入口不依赖它们。

**阶段完成条件：** 复制资产一致，包无重名，外部依赖记录完整，生产外参未改变。

## 3. 任务 3：共享配置与启动入口

### 3.1 文件职责与最小接口

| 文件 | 职责 |
|---|---|
| runtime_config.py | 根目录定位、JSON 读取、资源路径解析、外参合法性校验 |
| runtime_launcher.py | 命令构造、前台监督、背景组件确保、进程登记和只读检查 |
| config/runtime.json | 外部依赖路径、ROS/DDS、相机参数、就绪超时 |
| config/vision/fruit.yaml | 水果 ROS 参数，不重复存外参数值 |
| config/vision/blue_block.yaml | 蓝方块 ROS 参数 |
| config/calibration/manifest.json | 生产外参语义、来源、legacy 状态和推广记录 |
| scripts/runtime_env.sh | 加载 ROS/overlay 并设置固定 DDS |
| scripts/run_component.sh | 用户统一入口 |

公共内部接口固定：

~~~python
workspace_root() -> pathlib.Path
load_runtime_config() -> dict
resolve_resource(key: str) -> pathlib.Path
load_handeye(camera: str) -> dict  # position、orientation、mode、parent、child
build_command(component: str, *, camera="d455", mode="fruit",
              ros_args=()) -> list[str]
~~~

配置路径字段使用具名键：vision_python、system_python、ros_setup、workspace_setup、orbbec_setup、dds_xml、yolo_weights、graspnet_checkpoint、graspnet_baseline、d455_handeye、ik_site_packages、cmeel_site_packages、ik_urdf、saved_poses、conveyor_calib。

项目内路径相对 workspace_root；外部路径用绝对路径并支持 expanduser。root 默认按脚本位置确定，兼容 AGX_ARM_WS；不能按当前 cwd 解析。视觉包从自身路径定位主空间模块，或由启动入口只将主空间根加入 PYTHONPATH，不能全局注入 IK 的 site-packages。

### 3.2 环境规则

- [ ] D455 生产依次加载 /opt/ros/humble/setup.bash、主空间 install/setup.bash。
- [ ] Dabai 工具额外在主空间 overlay 前加载 Orbbec overlay。
- [ ] setup 后显式设置 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 和项目内 CYCLONEDDS_URI。
- [ ] ROS_DOMAIN_ID 保留调用者值；未设置按 ROS 默认 0；不 source ~/.bashrc。
- [ ] shared source 脚本兼容调用者 set -u，临时处理后恢复原 shell 选项。
- [ ] 所有真实程序显式选择解释器；已 activate 的环境不能改变控制/GUI解释器。
- [ ] 不把 PYTHONNOUSERSITE=1 或 IK 包路径全局 export。
- [ ] 控制保留当前 NumPy → IK 路径注入 → 延迟 Pinocchio 导入顺序，cmeel 真 Pinocchio 路径优先于 .venv_ik 中同名包。
- [ ] 配置读取模块只做 stdlib 工作，导入不启动节点、不加载模型、不访问硬件。

### 3.3 CLI

~~~bash
./scripts/run_component.sh preflight
./scripts/run_component.sh arm
./scripts/run_component.sh camera --camera d455
./scripts/run_component.sh camera_tf
./scripts/run_component.sh detector --mode fruit
./scripts/run_component.sh detector --mode blue_block
./scripts/run_component.sh graspnet
./scripts/run_component.sh action
./scripts/run_component.sh gui
./scripts/run_component.sh camera --camera dabai
./scripts/run_component.sh calibrate-intrinsics --camera d455
./scripts/run_component.sh calibrate-intrinsics --camera dabai
./scripts/run_component.sh calibrate-handeye --camera d455
./scripts/run_component.sh calibrate-handeye --camera dabai
~~~

每个启动入口支持 --dry-run：只输出解释器、argv、配置和关键环境，不加载模型、不建输出目录、不启动组件、不访问 CAN/串口。preflight 检查当前部署依赖，分组件报告缺失；可以只读查询 live 接口，但结果区分“接口存在”和“模型就绪”。

手动命令默认前台监督：launcher 创建组件进程组、转发 SIGINT/TERM、wait 回收、返回真实退出码。校准命令保留终端 stdin/stdout。GUI 使用相同命令构造器在 ROS worker 里 background ensure，仅管理 camera、camera_tf、detector。

### 3.4 当前参数必须冻结

D455 手动和 GUI 共用：

~~~text
rgb_camera.color_profile:=640,480,30
depth_module.profile:=640,480,30
align_depth.enable:=true
publish_tf:=false
spatial_filter.enable:=true
temporal_filter.enable:=true
hole_filling_filter.enable:=true
~~~

水果：

~~~text
d455.enabled=true
dabai.enabled=false
show_gui_window=false
depth_pair_delay=true
d455.x_offset_m=-0.035
d455.y_offset_m=-0.020
d455.z_offset_m=0.030
d455.depth_offset_m=0.0
~~~

蓝方块：

~~~text
x_offset_m=-0.045
y_offset_m=-0.010
z_offset_m=0.052
~~~

用户早期说明中的水果 -0.015/-0.025 不是当前源码默认值，README 使用上面的真实基线。旧 use_depth_camera、无前缀水果补偿参数改写为 d455.*，不新增两套参数别名。

### 3.5 活动外参

YOLO、蓝方块、GraspNet、D455 TF 都读取同一个 config/calibration/d455_eye_to_hand.json：

~~~text
position = [0.6490691434509585, -0.05381643112008146, 0.5963265498171803]
orientation_xyzw = [-0.6924182483257096, -0.7044102901442802,
                    0.08488321314295422, 0.1309883683803036]
parent = base_link
child = camera_color_optical_frame
mode = eye_to_hand
p_base = T_base_camera @ p_camera
~~~

JSON 原样复制，语义写入独立 manifest。历史文件不具备新质量元数据，标为 legacy_production_baseline，不伪造其已通过新判据。默认 TF 入口仍保留 launch/eye_to_hand_camera_tf.launch.py；内部统一读配置，允许 calibration_file 显式覆盖用于离线/候选检查。

加载器验证 3 位位置、4 位 xyzw、有限数值、非零四元数和相机/方向。缺失/非法明确失败，不静默回退单位矩阵。正确归一化仅用于计算，不能重写原 JSON 改变基线哈希。

### 3.6 进程管理和就绪

- [ ] .runtime/ 存本机登记；组件键为 camera:d455、camera:dabai、camera_tf:d455、detector:d455。
- [ ] 登记 managed_workspace、initiator(manual/gui)、owner_session、PID、PGID、UID、/proc starttime、boot_id、record_id、入口和模式。
- [ ] 使用短原子登记锁；另用组件 transition 锁覆盖该组件 stop/start/ready，不持全局锁跨长等待。
- [ ] Popen 使用 argv、start_new_session=True；必要 shell 通过 exec 转交真实组件，不使用字符串拼接或 preexec_fn。
- [ ] 停止前逐项校验身份。SIGINT 等 5 秒，必要时 SIGTERM 等 5 秒，仍存活报告失败；不按文件名 pkill。
- [ ] 每次信号升级前重新校验身份；PGID必须来自本工具创建的session且等于子PID，拒绝PGID≤1或当前调用者进程组。
- [ ] 非父进程停止只轮询 /proc；原父进程负责 wait/reap，不能用 waitpid 回收不属于自己的进程。
- [ ] 旧监督进程退出只删除仍匹配 record_id 的登记，避免删掉模式切换的新进程记录。
- [ ] leader退出但launch子孙仍占相机/话题时报告“未完全退出”，不删状态后重启第二实例。
- [ ] 已就绪外部未登记节点可以复用；无法证明所有权时不停止。模式冲突提示操作者关闭旧终端。
- [ ] GUI 退出只清理本 session 创建的组件；复用 manual 组件不获得退出清理权。
- [ ] GUI 显式模式切换允许停止同 workspace 登记的 manual detector；旧 T3 监督终端随之结束，新 detector 归当前 GUI。README 必须说明。

就绪规则：

| 组件 | 检查 | 默认超时 |
|---|---|---:|
| D455/Dabai 相机 | RGB、对应深度、CameraInfo 接收年龄≤3s，配置分辨率吻合；D455还核验publish_tf=false | 30s |
| D455 TF | tf2 可查询且完整矩阵与活动值一致；static TF 不要求每3秒重发 | 10s |
| 水果检测 | /object_detector、/detection_info 水果格式、新消息年龄≤3s，detected=false合法 | 60s |
| 蓝方块 | /blue_block_detector、detections 列表、新消息年龄≤3s，空列表合法 | 30s |
| GraspNet | 托管进程本次日志出现模型加载完成及“就绪, 等待请求...” | 120s，可配置 |

蓝方块消息没有水果 header_stamp/camera 字段；新鲜度按接收 monotonic 算。GraspNet req/result 先于模型加载创建，topic存在只能报告接口存在。对外部手动 GraspNet，人工核查日志；不靠发真实抓取请求探测就绪。

模式切换完成必须是旧publisher消失、目标节点上线、根/detection_info恰有一个目标publisher，并收到切换后新消息；不能用缓存的旧JSON宣告ready。复用的外部D455若开启了TF或配置不匹配，报告冲突，不能再叠加第二个相机/TF修补。

启动/错误日志放 log/runtime/，GUI 总等待时间按 stop + camera + TF + detector 累计，避免旧 35s/65s 固定超时抢先失败。

**阶段完成条件：** 各组件 dry-run 命令正确，不依赖 cwd，环境隔离，进程模拟测试通过。

## 4. 任务 4：在线代码、GUI 与热更新

### 4.1 不变的对外接口

保留 /object_detector、/blue_block_detector、/detection_info 的原两种 JSON、图像/姿态/曝光接口和时间戳字段。保留 /sorting_cmds、/sorting_status、/sorting/cycle_result、/sorting/robot_status、/emergency_stop、/conveyor_status。

GraspNet 继续 std_msgs/msg/String：

~~~json
{"cmd": "get_grasp", "pick_x": 0.0, "pick_y": 0.0, "pick_z": 0.0}
~~~

结果保留 success、translation、quaternion、width、score、n_candidates/error。根目录 saved_poses.json、conveyor_calib.json 不迁路径、不覆盖，原 URDF/场景不替换。

- [ ] 保留 auto_sorting_action.py、sorting_gui_client.py、graspnet_service_node.py 主体，只调整路径、启动和指定修复。
- [ ] 保留 u=260 越线停带、最右侧选择、抓取后继续循环及原停止行为。
- [ ] 不改 GraspNet 模型推理、夹爪参数、逆解、轨迹和类别选择算法。

### 4.2 GUI

- [ ] 新增 GUI 单实例锁，在 spawn worker 前取得；第二个 GUI 明确提示并退出，避免第二 worker 抢串口并发布错误状态。
- [ ] 保留 Tk 主进程/ROS worker 架构，不在 Tk 主进程调用启动/停止工具。
- [ ] 自动管线线程在 rclpy.init 和 Node 创建之后开始；等待 ready 期间 spinner 保持运转。
- [ ] 自动链补齐 camera_tf，共用任务3配置、锁和就绪规则。
- [ ] requested_mode与active_mode分别保存，只在目标新消息ready后提交active_mode。失败保持已确认旧模式或明确offline，不提前让UI显示新模式。
- [ ] 单次抓取、自动分拣、动态抓取或控制busy时拒绝视觉切换，等待停止与状态确认后再处理。
- [ ] 正常退出通过队列请求 worker 停止原有监控、按现有语义处理传送带、清理本 session 子进程、释放串口并 ACK，再 join。
- [ ] 父进程退出 watchdog 同样走清理；最后才兜底 terminate，不在正常路径直接杀 worker 绕过 finally。
- [ ] 自动启动仅相机/TF/检测；机械臂、动作和 GraspNet 仍按明确终端入口启动。

### 4.3 旧入口兼容

- [ ] 新入口和软件回归成功后，Orange 两个在线脚本改为薄包装，调用主空间 canonical 入口并保留 argv、退出码、信号。
- [ ] dispatcher 最终执行 vision/ 下真脚本，避免 wrapper → launcher → wrapper 递归。
- [ ] setup_graspnet_env.sh 兼容共享 env；start_graspnet_pipeline.sh 更新只读检查和新命令。
- [ ] test_graspnet_hardware.sh 使用真实 req/result 话题协议，移除对旧 /generate_grasp_pose 的默认检查。
- [ ] 原根 Python 入口保留可用。

旧未登记的手动入口仍可运行但属于external，不取得GUI停止/切换权限；README写清这一差异。旧start脚本只读检查与打印命令，不自动激活CAN。

### 4.4 补偿热更新

- [ ] 水果仅热更新相机 x/y/z/depth_offset_m；蓝方块仅 x/y/z_offset_m。
- [ ] callback 原子验证有限数字，拒绝 NaN/Inf/非法类型；验证时不改 CameraState 或 track。
- [ ] 在下一帧开始读取已提交参数快照，一帧内全部点使用同一快照。
- [ ] 受影响相机的三维/深度平滑 track 清空，其他相机保留；确保下一帧体现完整补偿变化。
- [ ] depth_offset_m 保留“从原始深度减去”的语义，不能改成直接加减 base z。
- [ ] 热更新不自动写 YAML；README 说明临时/永久设置。
- [ ] 相机拓扑、话题和外参变化需要重启；不承诺所有已声明参数都支持热更新。

水果示例：

~~~bash
ros2 param set /object_detector d455.x_offset_m -0.035
ros2 param set /object_detector d455.y_offset_m -0.020
ros2 param set /object_detector d455.z_offset_m 0.030
ros2 param set /object_detector d455.depth_offset_m 0.0
~~~

Dabai 当前代码外参为占位固定矩阵且无图像时刻 TCP 接入。保留多相机结构，但本次生产 detector 对 dabai.enabled=true 明确拒绝启动并提示尚未完成动态坐标接入，避免输出占位 base 坐标。Dabai 标定入口不受此限制。

**阶段完成条件：** JSON/状态机回归无变化，GUI 管线和退出模拟通过，补偿下一帧完整生效。

## 5. 任务 5：双相机标定迁移、候选保护与教程

### 5.1 包与依赖

- [ ] 迁入 handeye_calibration.py、intrinsic_calibration.py、charuco_detect.py、axxb.py 和包资源。
- [ ] 保留 handeye_calibration、camera_intrinsic_calibration 两个 console scripts。
- [ ] package.xml 补 python3-scipy，记录 OpenCV≥4.7 新 Charuco API 要求。
- [ ] 标定使用 /usr/bin/python3 + 当前用户 OpenCV/NumPy/SciPy；不进入旧 handeye 的两个 venv。
- [ ] 单包构建：

~~~bash
cd /home/lxf/agx_arm_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select handeye_calibration_ros
~~~

运行不继承 PYTHONNOUSERSITE=1，否则会退回缺新 API 的系统 OpenCV 4.5.4。交互入口用真实 TTY、前台 ros2 run，不放 GUI 后台或重定向 stdin 的 ros2 launch。

### 5.2 数据目录与候选/批准

~~~text
config/calibration/
  d455_eye_to_hand.json        # 原有生产基线
  d455/intrinsics.yaml         # 存在时只能是已批准本相机内参
  dabai/intrinsics.yaml        # 不伪造
  dabai_eye_in_hand.json       # 当前不存在，不生成假结果
  manifest.json
data/calibration/
  d455/intrinsics/<会话>/      # 原始帧、候选YAML、质量报告
  d455/handeye/                # 候选JSON、raw_data、各采集会话
  dabai/intrinsics/<会话>/
  dabai/handeye/
  history/handeye/
  history/d455_intrinsics_legacy/
~~~

- [ ] 内参输出 out_yaml 指向本相机会话候选，不直接覆盖批准内参。
- [ ] 外参 result_save_path 指向该相机 handeye 数据目录，不覆盖生产 JSON。
- [ ] 每个候选带 camera_id、mode、分辨率、board、有效帧数、来源、质量状态。
- [ ] 手眼默认引用该相机批准内参；不存在时明确提示先采集/批准内参。
- [ ] 仅显式 --use-camera-info 允许用本相机 CameraInfo，并记录来源；指定文件非法、相机错误或尺寸不符不能静默 fallback。
- [ ] 生产检测仍使用原驱动 CameraInfo，迁入的新内参不自动改变当前抓取 K/D。
- [ ] 失败候选和原始数据允许保存，状态明确 failed/unverified；“文件生成”不等于验收通过。

提供最小推广工具 tools/calibration/approve_calibration.py，由入口调用：

~~~bash
./scripts/run_component.sh approve-intrinsics --camera d455 --candidate /绝对路径/候选.yaml
./scripts/run_component.sh approve-intrinsics --camera dabai --candidate /绝对路径/候选.yaml
./scripts/run_component.sh approve-handeye --camera d455 --candidate /绝对路径/候选.json --scene-verified
./scripts/run_component.sh approve-handeye --camera dabai --candidate /绝对路径/候选.json --scene-verified
~~~

推广要求相机、模式、帧端点、分辨率、质量和来源完整；先保存旧批准文件及 manifest，再原子替换并追加审计记录。--scene-verified 仅由已经完成现场坐标核查的操作者明确使用，不由标定程序自动追加。Dabai 推广只保存外参，仍不启用抓取或基座 TF。旧生产 D455 文件是明确保留的 legacy 基线，不经过新候选门禁倒推合格状态。

批准内参/外参会更新相机对应路径和manifest；已有运行节点不自动重载。操作者先结束任务，再重启受影响的检测/GraspNet/TF并重新核对完整矩阵。内参推广只供后续手眼使用，不顺带改写驱动CameraInfo。默认生产preflight不把尚未生成的Dabai结果或新标定内参当作D455抓取的必需依赖。

### 5.3 板、采样和质量

提供 tools/calibration/generate_charuco_board.py，复用 charuco_detect 的唯一板常量，生成 PNG 与尺寸说明：

~~~text
7×5，square=0.032m，marker=0.024m，DICT_5X5_1000
板面224×160mm，17个marker、24个Charuco角点
~~~

可生成带空白边缘的图片，并标明实际板面尺寸；打印关闭缩放，量尺确认32mm方格，固定在平整刚性载体。旧 marker582 是历史单 ArUco，不能当当前板。

内参批准门：

- ≥15有效帧；推荐20～25帧，多个倾角、距离和画面区域。
- 整体重投影 RMS <0.5px。
- 3×3 覆盖网格的中心和四角都被保留帧覆盖，不能用 coverage数量≥5代替。
- 每帧记录覆盖cells，d回退后按保留帧重新计算。
- 图像/CameraInfo/输出尺寸一致、数据有限、相机和板信息完整。

手眼保留原标准：

- ≥15有效姿态，推荐20～25。
- 每帧 markers≥14、corners≥20、RMS<0.5px、square≥40px。
- 停稳0.8s；时间差<0.05s；holdout=4。
- consistency_deg<0.02，median_rel_angle_deg≥25，n_pairs_ge25deg≥10。
- 留出验证确实执行，median_mm<1.0；p95/max同时报告，不擅自改为 max<1mm 的新硬门。

### 5.4 接收、新鲜度与坐标语义

- [ ] 手眼保留唯一后台spin；内参新增持续接收，不在同一 executor 再开第二spin点。
- [ ] max_message_age_s=1.0，按 monotonic 接收年龄验证图像/CameraInfo/机器人位姿。
- [ ] 用接收序号识别新图像，拒绝重复采样；header有效时同时记录，不能让无效零stamp把所有新帧都判同一帧。
- [ ] 一次采样取锁保护的输入快照，验证后才计数；失败不写入有效集合。
- [ ] 新 sync_time_source 默认 receive：SDK与相机时钟域尚未证明相同，按monotonic接收差<50ms；写入元数据，明确不是硬件同步。
- [ ] 显式 header 模式要求有效同域时间，header超限直接拒绝，不能再用接收近来掩盖；不得自动判定任何超限都代表不同域。
- [ ] 位置单位m、RPY来源rad、四元数xyzw；不再次除1000。

机器人反馈用 /feedback/tcp_pose，类型 geometry_msgs/msg/PoseStamped，数值语义为 base_link 下TCP位姿。当前 header.frame_id为空，允许已核查的legacy接口并显示说明；不大改控制发布器。

采样前查询并记录：

~~~bash
ros2 param get /agx_arm_ctrl_single_node tcp_offset
~~~

零offset时TCP对应link6；非零时结果相对实际TCP，metadata记录offset和端点，不能直接挂到link6。

### 5.5 双相机参数

| 项目 | D455 | Dabai DC1 |
|---|---|---|
| 安装 | 工作区外部固定 | 手腕固定 |
| 板 | 末端刚性固定 | 桌面固定不动 |
| mode | eye_to_hand | eye_in_hand |
| image_topic | /camera/camera/color/image_raw | /dabai/color/image_raw |
| camera_info_topic | /camera/camera/color/camera_info | /dabai/color/camera_info |
| optical frame | camera_color_optical_frame | dabai_color_optical_frame |
| piper_topic | /feedback/tcp_pose | /feedback/tcp_pose |
| 彩色配置 | 640×480@30 | 640×480@15，MJPG |
| 求解结果 | T_base_camera | T_tcp_camera |

Dabai 相机入口使用实际 dabai.launch.py，并先加载Orbbec overlay：

~~~bash
ros2 launch orbbec_camera dabai.launch.py \
  camera_name:=dabai depth_registration:=true align_mode:=HW \
  enable_point_cloud:=false enable_colored_point_cloud:=false enable_ir:=false \
  color_format:=MJPG color_width:=640 color_height:=480 color_fps:=15
~~~

不指定深度profile，不用默认frame_sync不兼容DC1的dabai_dcl.py。Dabai内部TF可保持驱动默认；此次不自动建立到机器人基座的连接。

### 5.6 README中的D455实际步骤

1. 操作者停止分拣、GUI、GraspNet与检测；固定相机不动，将正确ChArUco板刚性装在末端。
2. 启动arm和D455 camera；确认RGB/CameraInfo为640×480，记录TCP offset。
3. 执行 calibrate-intrinsics --camera d455；至少15帧，推荐20～25，覆盖四角/中心与倾角。
4. Enter采样、d回退、q计算，检查打印的候选路径和质量报告。
5. 用 approve-intrinsics --camera d455 --candidate <实际候选绝对路径> 批准内参。
6. 操作者扶稳机械臂后执行现有重力补偿工具：

~~~bash
source /home/lxf/agx_arm_ws/scripts/runtime_env.sh
/usr/bin/python3 /home/lxf/agx_arm_ws/enable_gravity_comp.py
~~~

7. 前台运行：

~~~bash
cd /home/lxf/agx_arm_ws
./scripts/run_component.sh calibrate-handeye --camera d455
~~~

默认展开为：

~~~bash
ros2 run handeye_calibration_ros handeye_calibration --ros-args \
  -p mode:=eye_to_hand -p piper_topic:=/feedback/tcp_pose \
  -p image_topic:=/camera/camera/color/image_raw \
  -p camera_info_topic:=/camera/camera/color/camera_info \
  -p intrinsics_yaml:=/home/lxf/agx_arm_ws/config/calibration/d455/intrinsics.yaml \
  -p result_save_path:=/home/lxf/agx_arm_ws/data/calibration/d455/handeye
~~~

8. 多轴改变姿态与位置，停稳后Enter，错误d，满足数量后q；检查候选判据。
9. 结果为T_base_camera；保留原生产外参。只有质量和现场坐标核查通过、操作者明确选用新结果，才调用approve-handeye。
10. 退出示教、移除板、恢复生产流程：

~~~bash
ros2 service call /exit_teach_mode std_srvs/srv/Empty
~~~

### 5.7 README中的Dabai实际步骤

1. Dabai刚性安装于手腕，板固定桌面；停止生产分拣/检测，采样全程板不动。
2. 启动arm与 camera --camera dabai，核对彩色/CameraInfo为640×480@15和TCP offset。
3. calibrate-intrinsics --camera dabai，覆盖/倾角采样，批准本相机内参。
4. 操作者进入示教，改变腕载相机位置与姿态，前台执行：

~~~bash
./scripts/run_component.sh calibrate-handeye --camera dabai
~~~

展开参数为eye_in_hand、Dabai image/info、相同TCP话题，intrinsics_yaml为config/calibration/dabai/intrinsics.yaml，result_save_path为data/calibration/dabai/handeye。

5. 采集20～25组，q求解，保存T_tcp_camera和offset/质量报告。当前没有已通过的Dabai外参，不能生成假结果。
6. 操作者退出示教。通过的候选可推广为标定资产；生产仍D455、dabai.enabled=false。

未来腕载坐标接入必须在图像时刻使用：

~~~text
T_base_camera(t) = T_base_tcp(t) @ T_tcp_camera
~~~

本次仅完成标定工具链，不把腕载标定当固定base外参。

### 5.8 TF方向与独立数学测试

用 ^A T_B 表示将B系点变换到A系。D455得到 ^base T_camera，Dabai得到 ^tcp T_camera；对应parent→optical TF直接使用结果R,t。

完整逆为 [Rᵀ, -Rᵀt]。删除旧教程“平移原样，仅四元数取共轭”的步骤；修正旧verify_handeye.launch.py把optical外参直接发布给camera_link的问题。D455复用统一TF入口；Dabai本次只做候选数学/端点验证，不发布固定base TF。

合成测试固定种子、非零平移、非平凡旋转、至少两非平行轴，独立设真值：

- 腕载：G_i=^base T_tcp(i)，X=^tcp T_cam，Z=^base T_board；生成 C_i=inv(X)@inv(G_i)@Z，求解G_i/C_i返回X。
- 外置：Y=^base T_cam，S=^tcp T_board；生成 C_i=inv(Y)@G_i@S，生产预处理传inv(G_i)/C_i，求解返回Y。
- 比较矩阵/平移，容差1e-4；q和-q等价，不按四元数分量误判。
- 特设非零t，验证完整求逆正确，“共轭而不改平移”必须失败。

方向说明可对照 [OpenCV官方手眼标定接口](https://docs.opencv.org/4.10.0/d9/d0c/group__calib3d.html)，同时以本机AX=XB输入处理与独立真值测试确认。

**阶段完成条件：** 包可发现、单包构建成功、纯离线数学/图像/候选测试通过，双相机教程完整。有效实机标定结果由现场采集判据决定，不能由代码迁移宣称已获得。

## 6. 任务 6：清理、归档与Git资产边界

### 6.1 归档

| 当前对象 | 目标 |
|---|---|
| _add_filter.py、_fix_filter.py、_fix_mask.py | archive/legacy/graspnet_patches/ |
| auto_sorting_action.txt、joint_space_sorting_action.py、sorting_action1.txt | archive/legacy/sorting/ |
| .make_docs.py | archive/legacy/document_generation/ |
| 旧标定教程、marker582、旧工具、.bak和TF图 | archive/legacy/handeye/ |

archive/COLCON_IGNORE避免旧源码被发现。归档记录原路径/目标/哈希/原因；同内容历史文件也保留原名称来源。

文档：

- PINOCCHIO_IK_GUIDE.md和STM32通信协议 → docs/guides/。
- 三份Piper DOCX、系统运行流程详解.html、项目进度报告 → docs/reports/。
- 智能仓储分拣系统_项目需求_最终版.pdf → docs/requirements/。
- 参赛工作日志、日志复制 → docs/worklogs/各自子目录。
- 原根README先保留到docs/guides/agx_driver_original_README.md，再重写根运行指南；迁移后修正文档有效链接。

### 6.2 删除白名单

- .pytest_cache/
- __pycache__/
- launch/__pycache__/
- tests/__pycache__/
- ch340-relay/__pycache__/
- 上述范围内可再生pyc。
- src/agx_arm_ctrl/agx_arm_ctrl和父src/agx_arm_ctrl当前为空；执行时仍空才分别rmdir，不用rmdir -p。

先列候选，核验备份，按确切路径处理。执行时新增内容或目录非空则保留并报告。

### 6.3 保留

build/install/log、.venv/.venv_ik/.venv_mjx、third_party/python和两处SDK引导、src/trac_ik、仿真源码和模型、test2.urdf、conveyor.scene、table.scene、saved_poses.json、conveyor_calib.json、当前使能/重力补偿/标定/离线IK/STM32/状态桥工具、.trae/.superpowers/.vscode。

不按文件名批删txt、URDF、mesh、权重或日志，不调用git clean -fdx或reset覆盖现场改动。

### 6.4 .venv_ik取消索引跟踪

目前跟踪21,289个文件。备份后：

- [ ] 保存物理manifest，增加.venv_ik/忽略规则。
- [ ] 仅执行限定路径 git rm -r --cached -- .venv_ik。
- [ ] 对比物理文件/pyvenv.cfg/链接，确认仍完整存在。
- [ ] 控制依赖预检查通过；不搬环境、不安装包、不改历史。

说明此操作不立即省磁盘，也不缩小历史Git；fresh clone不会带环境，必须恢复部署资产。本次只保证当前机器迁移后的工作流，不声称只凭Git即可在新机器复现全部运行环境。

**阶段完成条件：** 每项移动/归档/删除有记录，保留资产完整，更新后链接与资源路径无迁移引入的缺失。

## 7. 任务 7：README与操作文档

根README使用中文，成为日常入口；详细标定说明写docs/calibration.md，迁移证据写docs/migration_report.md。

必须包含环境解释、依赖路径、只读preflight、七终端顺序、GUI使用、模式切换、在线参数/永久参数、停止、日志排查、两个相机完整标定、候选批准、恢复生产、回退。

### 7.1 新七终端

每条在独立终端先cd /home/lxf/agx_arm_ws：

| 终端 | 命令 | 就绪 |
|---|---|---|
| T1 | ./scripts/run_component.sh arm | MoveIt、控制及机器人反馈 |
| T2 | ./scripts/run_component.sh camera --camera d455 | RGB、对齐深度、CameraInfo更新 |
| T2b | ./scripts/run_component.sh camera_tf | 完整base→optical矩阵正确 |
| T3 | ./scripts/run_component.sh detector --mode fruit | 检测消息更新，空场景合法 |
| T4 | ./scripts/run_component.sh graspnet | 模型就绪日志，不能只看话题 |
| T5 | ./scripts/run_component.sh action | 分拣执行节点 |
| T6 | ./scripts/run_component.sh gui | 图像/状态/控制正常 |

T1底层保留：

~~~bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=can0 arm_type:=piper effector_type:=agx_gripper
~~~

TF只读检查：

~~~bash
source /home/lxf/agx_arm_ws/scripts/runtime_env.sh
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
~~~

平移接近[0.649,-0.054,0.596]，同时核验完整旋转。

### 7.2 操作与停止

1. 等相机、检测和GraspNet加载就绪，再放物体。
2. 选择料框1/2，单次按钮执行GraspNet。
3. 自动分拣：左到右越u=260停带、最右目标、抓取放置、继续循环。
4. 停止按钮结束自动任务；明确当前在途动作按既有状态机处理，不新增未经定义的回零动作。
5. 模式切换按登记进程替换；manual T3被显式替换会结束该终端的监督命令。
6. 正常停止顺序：GUI停止自动任务→等待当前处理结束/状态稳定→正常退出GUI→Ctrl+C停止T5/T4→停止检测、TF、相机→最后停止T1；现场急停按既有设备流程。
7. GUI创建组件随GUI退出；GUI复用的manual组件由对应终端关闭。

错误表至少包括环境/资源缺失、DDS不一致、相机占用、未收CameraInfo、双TF、模式冲突、检测过期、GraspNet仍加载、标定分辨率错误、候选质量失败及串口占用。

**阶段完成条件：** 文档命令与命令构造测试一致，所有实际入口和输出路径明确，首次接手者能照步骤执行。

## 8. 任务 8：回归、实机验收与最终交付

### 8.1 软件检查

备份后迁移前记录原九个数值测试基线，迁移后同命令再执行：

~~~bash
/usr/bin/python3 -m pytest tests/test_conveyor_calib.py tests/test_velocity_predictor.py -q
~~~

新增测试目标：

| 文件/位置 | 必须断言 |
|---|---|
| tests/test_runtime_config.py | cwd独立、AGX_ARM_WS、缺资源、合法外参、基线哈希 |
| tests/test_runtime_launcher.py | 每组件argv/解释器/DDS/滤波/TF，dry-run零组件副作用，包装不递归 |
| tests/test_runtime_process_manager.py | 锁、PID复用、boot_id、record_id、所有权、外部复用、manual切换、并发/超时/回收 |
| tests/test_gui_runtime_lifecycle.py | 单实例、spinner保持、关闭ACK、created/reused清理差异、watchdog |
| tests/test_vision_offsets.py | 原子拒绝NaN/Inf，下一帧完整补偿，EMA隔离，depth符号 |
| 标定包tests | 独立双模式真值、逆变换、板识别、内参覆盖回退、失败候选、推广门禁、尺寸/相机/新鲜度/同步 |

额外检查：

- Python语法/编译、bash -n。
- 控制依赖检查真实Pinocchio导入路径和buildModelFromUrdf等符号，不能只看distribution版本；.venv_ik同名stub的metadata=0.4.3、实际__version__=0.1，真实cmeel pin=3.9.0。
- 在工作空间和/tmp分别运行所有组件--dry-run，关键值一致。
- build仅新标定包，ros2 pkg executables发现两个入口。
- 实际解释器下只读依赖检查；不import带main副作用的控制/GUI文件。
- axxb现有纯NumPy self-test；合成板检测，不开相机。
- 空场景格式、消息过期、模型未ready、缺资源的明确错误。
- 模式状态延迟提交、busy时禁切换、切换后的新消息/唯一publisher、外部错误TF配置拒绝。
- 参数拒绝不改状态，合法提交下一帧体现完整增量。
- 内参某角唯一覆盖帧撤回后重新判为不合格。
- receive同步报告语义；header超限但接收近仍拒绝。
- 活动外参、料框、传送带、模型、归档哈希不变。

测试采用模拟输入/进程，不自动启动arm或串口设备。原测试若已有失败，记录真实基线；迁移引入回归必须修复，不能只隐藏测试。

### 8.2 实机验收

由操作者在场执行，保存日志/目标/结果/异常与时间：

| 场景 | 通过条件 |
|---|---|
| T1到T6 | 顺序启动，每阶段真实ready |
| 通信/TF | DDS一致、单一正确D455 TF、源外参一致 |
| 空场景 | GUI/检测正常，空结果可就绪 |
| 单次GraspNet | 两料框各至少5次，与迁移前基线比较 |
| 自动分拣 | 至少连续10循环，停带/选择/放置/恢复检测正确 |
| 多目标 | 仍按最右规则选择 |
| 停止任务 | 不再进入新自动周期，在途动作按原语义 |
| 模式 | fruit→blue→fruit至少两轮，无重复检测/误杀 |
| 手动T3+GUI | 复用已启动组件，不抢相机 |
| GUI关闭/重开 | owned组件清理、manual组件保留、串口可恢复 |
| 双相机标定 | 输入/端点/独立输出/真实采样可运行，质量结果据实记录 |
| 标定后生产 | 退出示教、移除板，重新七终端，复验单次/自动 |

识别类别按当前部署模型与源码核对，不能把训练旧orange.yaml中的占位类别写成已支持。故障验证优先在无运动状态进行；不为验收自动发送抓取或示教命令。

“标定工具能运行”与“此次得到有效新标定”分别记录。现场数据不足时保留现有D455外参、记录Dabai候选未通过，不能编造有效结果。

### 8.3 回退

1. 正常停止任务和节点。
2. 在外部目录保存“迁移后状态保全”：新增/修改会话、结果、配置、saved_poses、conveyor_calib和运行日志，并生成manifest/哈希。
3. 优先按清单恢复原入口/配置与索引；若全量回退，先将当前空间重命名保全，再从已验证归档恢复到原绝对路径，避免直接覆盖丢新数据。
4. 原Orange/handeye保留，恢复旧wrapper原稿和旧配置引用。
5. 用户site-packages本次不修改，正常回退无需覆盖全局环境；仅确有环境事故时按备份恢复。
6. 校验资源哈希、解释器链接、活动外参、Git状态，再按原T1～T6复核。
7. 新候选留在保全目录，不自动混入活动配置。

### 8.4 最终交付与完成标准

- [ ] 完整备份和BACKUP_VERIFIED.md。
- [ ] 迁入源码、模型部署目录、配置、受跟踪资产manifest和外部依赖清单。
- [ ] 统一启动入口、GUI生命周期和补偿热更新。
- [ ] 可构建的标定包、板生成工具、候选/批准保护与两相机教程。
- [ ] 清理/归档逐文件记录和原路径映射。
- [ ] 根README、docs/calibration.md、docs/migration_report.md。
- [ ] 软件测试命令、结果与差异。
- [ ] 现场抓取、模式切换、标定工具和恢复生产验收记录。
- [ ] 回退步骤与备份实际位置。

实施完成要求：备份通过、清单落实、软件无迁移回归、README可按步骤运行、标定工具和双相机步骤完整、现有D455单次/自动抓取实机验收通过。缺实机验证时状态为“软件验证完成，现场验收待完成”；设计交付本身不代表以上实施项已完成。

## 9. 设计交付复核记录

本文件交付前进行了当前源码、路径、参数、资源及Git状态的只读复核，并由三项独立审查覆盖启动依赖、视觉/标定、备份/清理。

已纳入修正：

1. 内参候选/批准隔离、覆盖按保留帧重算。
2. 蓝方块无水果时间戳字段，按模式和接收时间判ready。
3. GraspNet话题存在不等于模型ready。
4. GUI自动线程在ROS初始化之后启动，等待不阻塞spin；双GUI单实例和正常关闭ACK。
5. 前台监督/后台创建、登记所有权、跨进程stop与父进程reap、record_id防删替换登记。
6. 备份包含整个GraspNet及ignored扩展；软链接原文校验和历史断链基线。
7. 原样保留活动D455、真实offset、模型；legacy标定不伪造新质量状态。
8. 显式同步时间来源、拒绝旧帧，保持原算法质量阈值。
9. Dabai无动态TCP时阻止生产占位base坐标，标定入口独立可用。
10. ignored资产、标定历史与Git索引清理分别验收，回退先保全新采集成果。

本轮仅交付该设计文档；没有备份/迁移业务目录、修改运行代码、构建包、运行项目测试或启动硬件。
