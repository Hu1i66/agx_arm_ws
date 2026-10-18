# Piper 工作空间整合迁移报告（证据记录）

- **依据计划：** `docs/superpowers/plans/2026-10-12-piper-workspace-integration.md`
- **执行时间：** 2026-10-13 ~ 2026-10-14（Asia/Shanghai）
- **备份：** `/home/lxf/agx_arm_ws_backups/20261013-214530/`（见其中 `BACKUP_VERIFIED.md`）
- **结论：** 任务 1~7 与任务 8 的软件检查完成；**现场实机验收待执行**。
  2026-10-14 后续审查已修正运行时、GUI、标定问题，补做遗留清理；最新 **213 passed**，详见
  [`review_2026-10-14.md`](review_2026-10-14.md)。

**2026-10-15 启动流程补正：** 用户指出硬件激活前置步骤被遗漏。现已在根 README、
`start_graspnet_pipeline.sh` 提示和双相机标定手册恢复：
**连接上电 → CAN 接口激活 → `activate_robot.py` → T1 → T2 → T2b → T3 → T4 → T5 → T6**。
启动提示中原先“先 T1 后使能”的说明已更正；本次没有执行硬件激活，也未改变激活脚本内容。

---

## 2026-10-15 后续进展

用户确认误按示教按钮导致的T1问题已排除，现已使能回零并跑通生产全流程，暂未发现问题。
这是用户现场反馈，未补充逐项计次记录，§9的严格验收记录仍待填写。

本轮完成双相机标定专项审查、独立逐帧推广门与板图DPI修正、新TCP固定点XYZ工具，
恢复根目录 `README_previous.md` 与用户习惯的完整CAN命令。软件回归更新为 **239 passed**。
完整记录见 [`calibration_review_2026-10-15.md`](calibration_review_2026-10-15.md)，
TCP操作见 [`tcp_calibration.md`](tcp_calibration.md)。原活动外参与模型哈希不变。

## 1. 备份与基线（任务 1）

| 项 | 结果 |
|---|---|
| 归档 | 6 个 tar（主空间/orange_dataset/handeye/graspnet/用户 site-packages/cyclonedds），约 19.18 GB |
| 清单 | 归档前后各一套 TSV（相对路径/类型/权限/字节/mtime/SHA256/软链接原文），源稳定性 5/5 `CONTENT_MATCH=YES` |
| 解包校验 | 全部解包到 `restore-check/` 后与归档前清单一致（5/5），归档 SHA256 自校验 6/6 |
| 差异说明 | 解包后 mtime 为秒级（tar 默认格式），内容 SHA256 全同 |
| 关键修正 | 清单工具初版遗漏"指向目录的符号链接"，修复后软链接数 282/8/0/268/0，与 tar 成员数一致 |
| 资产基线核对 | YOLO `9162f7af…`、checkpoint `60680087…`、活动外参 `2d0907d8…` 与计划表一致 |
| 已记录未修复 | `install/` 中两个历史断链（agx_arm_description_gazebo.xacro 等）如实记录 |

## 2. 迁移（任务 2）

| 来源 | 目标 | 核验 |
|---|---|---|
| orange_dataset `realsense_yolo_node.py` / `blue_block_detector.py` | `vision/` | 复制时 SHA256 一致；当前修订哈希另记 deployment_manifest 的 `review_current_*` |
| YOLO `best.pt` | `models/yolo/fruits12.pt` | 哈希 = 计划基线 |
| GraspNet `checkpoint-rs.tar` | `models/graspnet/checkpoint-rs.tar` | 哈希 = 计划基线 |
| `2026-09-15_21-25-46_calibration.json` | `config/calibration/d455_eye_to_hand.json` | 哈希 = 计划基线（生产外参未改变） |
| CycloneDDS XML | `config/dds/cyclonedds.xml` | 哈希一致 |
| `handeye_calibration_ros`（含 4 模块、未跟踪 axxb/charuco_detect/intrinsic_calibration、2 个 launch） | `src/handeye_calibration_ros` | 复制阶段 `diff -r` 一致；随后按计划接入共享配置并修正（排除 .git/缓存/.bak） |
| `handeye/result` 全部（含未完成/空会话） | `data/calibration/history/handeye` | `diff -r` 一致（26 项） |
| `标定/calib_results`（旧分辨率资料） | `data/calibration/history/d455_intrinsics_legacy` + `README_LEGACY.md` | `diff -r` 一致 |
| 旧教程/ArUco 工具与板样/TF 图/.bak | `archive/legacy/handeye/` | 见 `archive/README.md`（含哈希） |

- 未整体复制旧 `handeye/src`（避免 piper_description/piper_gazebo/piper_mujoco 重名）；`aruco_ros` 未进入默认构建。
- 旧 `orange_dataset` / `handeye` / `graspnet` 目录保留作回退来源。
- 新增 `config/deployment_manifest.json`（来源/目标/大小/SHA256/解释器/外部实现路径）。
- `.gitignore`：新增权重与 legacy 大图像忽略；**未**使用 `data/**` 静默排除全部标定成果。

## 3. 共享配置与启动入口（任务 3）

新增（仅 stdlib + 子进程 ros2）：

| 文件 | 作用 |
|---|---|
| `runtime_config.py` | `workspace_root/load_runtime_config/resolve_resource/load_handeye`；外参校验（3 位位置、4 位 xyzw、有限、非零四元数、相机/方向）；非法明确失败 |
| `runtime_process_manager.py` | 登记（PID/PGID/UID/starttime/boot_id/record_id/owner_session/initiator）、flock 原子锁、身份校验、受控停止、`cleanup_session` |
| `runtime_launcher.py` | `build_command(...)` 固定签名、环境构造、前台监督、后台确保、就绪检查、`--dry-run`、`preflight`、`stop/status` |
| `config/runtime.json` | 15 个命名资源键 + ROS/DDS/相机参数/就绪超时/质量门 |
| `config/vision/fruit.yaml`、`blue_block.yaml` | 冻结参数基线（不含外参数值） |
| `config/calibration/manifest.json` | 外参语义、legacy 状态、推广审计策略 |
| `scripts/runtime_env.sh` | ROS(+Orbbec)+overlay 加载、显式 DDS、兼容 `set -u`、不 source `~/.bashrc` |
| `scripts/run_component.sh` | 用户统一入口 |

外参统一：`vision/realsense_yolo_node.py`、`vision/blue_block_detector.py`、`graspnet_service_node.py`、
`launch/eye_to_hand_camera_tf.launch.py` 全部改读 `config/calibration/d455_eye_to_hand.json`
（保留 `calibration_file` 显式覆盖）。

## 4. 在线代码、GUI 与热更新（任务 4）

- **热更新（4.4）**：两节点新增 `add_on_set_parameters_callback`——原子验证（拒绝 NaN/Inf/非法类型）、
  验证期不改状态、合法值在**下一帧边界**快照提交、受影响相机 track 清空、`depth_offset_m` 保持"从原始深度减去"、
  不自动写 YAML。Dabai 生产检测对 `dabai.enabled=true` **明确拒绝**。
- **GUI（4.2）**：单实例锁（spawn worker 前取得）；自动管线在 `rclpy.init`+Node 之后启动；
  自动只覆盖 camera/camera_tf/detector；`requested_mode` 与 `detection_mode`(active) 分离，
  仅目标就绪才提交；busy 时拒绝切换；正常退出经队列请求 → worker 清理本会话组件+断串口 → ACK → join，
  `terminate` 仅兜底；watchdog 走同一清理；不再使用 `pkill`。
- **旧入口（4.3）**：orange_dataset 两个在线脚本改为 `os.execv` 薄包装（保留 argv/退出码/信号，不递归），
  原实现另存 `.orig`；`setup_graspnet_env.sh` 兼容共享 env；`start_graspnet_pipeline.sh` 改为只读检查+统一入口命令
  （**不自动激活 CAN**）；`test_graspnet_hardware.sh` 改用真实 `/graspnet/req|result` 协议。

## 5. 双相机标定（任务 5）

- `package.xml` 补 `python3-scipy` 并记录 OpenCV≥4.7 新 ChArUco API 要求；单包构建成功，
  `ros2 pkg executables` 发现 `camera_intrinsic_calibration` 与 `handeye_calibration`。
- 内参节点：候选 YAML（`candidate` 元数据段：camera_id/来源/分辨率/board/有效帧/覆盖cells/质量状态）、
  批准门（≥15 帧、RMS<0.5px、3×3 中心+四角覆盖、尺寸一致、有限数值）、
  覆盖按保留帧重算、`max_message_age_s`/接收序号拒重复帧/锁保护快照。
- 手眼节点：内参来源（默认要求已批准文件，缺失明确失败；仅显式 `use_camera_info=true` 可用驱动 CameraInfo 并记录来源）、
  `sync_time_source`（默认 `receive`，元数据注明"非硬件同步"；`header` 模式超限直接拒绝，不用接收近掩盖）、
  年龄/序号校验、候选 JSON 带完整元数据与质量状态、审计 `promotion_log`。
- 工具：`tools/calibration/approve_calibration.py`（门禁 + 先备份后原子替换 + 审计；手眼必须 `--scene-verified`）、
  `tools/calibration/generate_charuco_board.py`（复用唯一板常量，生成 PNG+说明）。
- 板：7×5 / square 32mm / marker 24mm / DICT_5X5_1000，板面 224×160mm，17 markers，24 corners；
  已生成 `config/calibration/board/charuco_7x5_32mm.png`（3118×2362px @300dpi）。
- `verify_handeye.launch.py` 修正：不再把 optical 外参发给 `camera_link`，改为正确 optical child frame，
  提供**完整逆** `[Rᵀ, -Rᵀt]`（`invert:=true`），删除旧教程错误的"平移原样仅四元数取共轭"。

## 6. 清理与归档（任务 6）

- 归档：`_add_filter/_fix_filter/_fix_mask` → `archive/legacy/graspnet_patches/`；
  `auto_sorting_action.txt/joint_space_sorting_action.py/sorting_action1.txt` → `archive/legacy/sorting/`；
  `.make_docs.py` → `archive/legacy/document_generation/`；旧标定资料 → `archive/legacy/handeye/`（任务 2）。
  `archive/COLCON_IGNORE` 已建；逐文件哈希与原因见 `archive/README.md`。
- 文档迁移：`docs/guides/`（含 `agx_driver_original_README.md` 原 README 稿）、`docs/reports/`、
  `docs/requirements/`、`docs/worklogs/{参赛工作日志,日志复制}`；docs 下共 26 个文件。
- `.venv_ik`：忽略规则已加；物理清单（22,642 文件 / 2,136 目录 / 4 链接 / 655,292,142 字节，
  `pyvenv.cfg` SHA256 `53467b25…`）存 `log/migration/venv_ik_manifest.txt`；
  控制依赖预检通过（真实 cmeel pinocchio 3.9.0 + `buildModelFromUrdf`；`.venv_ik` 内同名 stub `__version__=0.1`）。
- 清理记录：`log/migration/task6_cleanup.txt`（候选/备份核验/处置）。

## 7. 清理补做结果（2026-10-14）

此前审批超时的两类操作已在本次审查中执行：

| 项 | 当前结果 | 证据 |
|---|---|---|
| 五处可再生缓存目录与空 `src/agx_arm_ctrl` 层级 | 已核对归档后删除；路径复验不存在 | `log/migration/cleanup_review_20261014.json` |
| `.venv_ik` 取消 Git 跟踪 | 限定路径 `git rm -r --cached -- .venv_ik` 成功；`git ls-files -- .venv_ik` 为 0 | `log/migration/venv_ik_untrack_20261014.log` |
| `.venv_ik` 物理内容 | 前后清单一致；22,642 文件、4 链接、655,292,142 字节，无内容或 mtime 差异 | 审查备份的 `venv_ik_before.tsv` / `venv_ik_after.tsv` |

额外审查备份：`/home/lxf/agx_arm_ws_backups/20261014-022422-review/`（代码、配置、文档、测试、Git 索引及状态）。
Git 取消跟踪目前表现为暂存删除，磁盘环境完整保留；本轮未创建提交。

原始 6 个归档已重新验证 SHA256 全部通过。原 `BACKUP_VERIFIED.md` 的 SHA 清单路径有笔误：
清单实际在备份根目录，正确命令是：

```bash
cd /home/lxf/agx_arm_ws_backups/20261013-214530/archives
sha256sum -c ../SHA256SUMS
```

## 8. 软件检查结果（任务 8.1）

本节的 **147 passed / 13 组 dry-run** 为原实施交付记录。审查后的最新结果为 **213 passed / 16 组 dry-run**，
新增回归与真实模型加载证据见 [`review_2026-10-14.md`](review_2026-10-14.md)；实机状态仍按 §9。

```bash
# 原有数值测试（迁移后同命令；本环境需 -p no:anyio 规避用户 site-packages 中不兼容的 anyio pytest 插件）
/usr/bin/python3 -m pytest -p no:anyio tests/test_conveyor_calib.py tests/test_velocity_predictor.py -q
# → 9 passed（与计划"原九个数值测试"一致）

/usr/bin/python3 -m pytest -p no:anyio tests/ -q
# → 147 passed
```

新增测试文件与覆盖：

| 文件 | 覆盖 |
|---|---|
| `tests/conftest.py` | sys.path |
| `tests/test_runtime_config.py` | cwd 独立、AGX_ARM_WS、命名键缺失、外参合法/非法（NaN/长度/零四元数/mode/缺文件）、基线哈希（兼容合法推广） |
| `tests/test_runtime_launcher.py` | 各组件 argv/解释器/参数/冻结项、dry-run 零副作用、cwd 独立、就绪逻辑（相机含 publish_tf、TF 矩阵 q/-q、水果/蓝方块 JSON、GraspNet 接口 vs 模型就绪）、复用/模式冲突/外部相机与双 TF 拒绝 |
| `tests/test_runtime_process_manager.py` | 锁互斥、登记字段、身份校验、PID 复用防护、PGID≤1 拒绝、record_id 防误删、未完全退出、回收、工作空间隔离 |
| `tests/test_gui_runtime_lifecycle.py` | 单实例锁、锁先于 spawn、自动管线顺序与范围、关闭 ACK→兜底 terminate、session 清理差异（created vs reused/manual）、无 pkill、模式仅在成功后提交、busy 禁切换 |
| `tests/test_vision_offsets.py` | 真实模块热更新：拒绝 NaN/Inf/非法类型、原子性、下一帧完整补偿、EMA/track 隔离、depth 符号、Dabai 拒绝（22 项） |
| `tests/test_old_entry_wrappers.py` | execv 语义（argv/退出码/PID 不变）、缺 canonical 明确报错、`.orig` 保留 |
| `tests/test_handeye_math.py` | 腕载/外置双模式合成真值（容差 1e-4）、完整逆、`q/-q` 等价、axxb self-test |
| `tests/test_calibration_logic.py` | 同步语义（receive/header、超限不被接收近掩盖）、内参不静默回退、覆盖回退重算、合成内参标定+质量门、approve 接受/拒绝全套（32 项） |
| `tests/test_board_detection.py` | 板常量、合成板检测与位姿求解、方格像素随分辨率、内参 YAML 往返 |

其他检查：

- Python 编译 + `bash -n` 全通过。
- Pinocchio：真实 cmeel 3.9.0（`buildModelFromUrdf` 可用）；`.venv_ik` 同名 stub `__version__=0.1`（与计划描述一致）。
- **13 个组件在 `/home/lxf/agx_arm_ws` 与 `/tmp` 两个 cwd 下 `--dry-run` 输出逐字节一致**。
- 单包构建仅 `handeye_calibration_ros`；两个 console script 可发现。
- 保留资产哈希不变：YOLO `9162f7af…`、checkpoint `60680087…`、活动外参 `2d0907d8…`、
  `saved_poses.json`/`conveyor_calib.json`/`cyclonedds.xml` 与备份一致。
- 环境说明：本机 pytest 直接运行会因用户 site-packages 的 `anyio` 插件报 `_pytest.scope` 错误（**迁移前既有**），
  统一以 `-p no:anyio` 运行；该选项保留用户 site-packages，与生产依赖一致。

## 9. 现场实机验收清单（待执行）

以下场景需操作者在场，保存日志/目标/结果/异常与时间；缺失时状态为"软件验证完成，现场验收待完成"。

| 场景 | 通过条件 | 状态 |
|---|---|---|
| T0→T1→T6 | 先激活 CAN，再执行 activate_robot.py，随后依次启动七终端，每阶段真实 ready | 待验收 |
| 通信/TF | DDS 一致、单一正确 D455 TF、源外参一致 | 待验收 |
| 空场景 | GUI/检测正常，空结果可就绪 | 待验收 |
| 单次 GraspNet | 两料框各 ≥5 次，与迁移前基线比较 | 待验收 |
| 自动分拣 | ≥10 连续循环，停带/最右选择/放置/恢复检测正确 | 待验收 |
| 多目标 | 仍按最右规则选择 | 待验收 |
| 停止任务 | 不再进入新周期，在途动作按原语义 | 待验收 |
| 模式切换 | fruit→blue→fruit ≥2 轮，无重复检测/误杀 | 待验收 |
| 手动 T3 + GUI | 复用已启动组件，不抢相机 | 待验收 |
| GUI 关闭/重开 | owned 组件清理、manual 保留、串口可恢复 | 待验收 |
| 双相机标定 | 输入/端点/独立输出/真实采样可运行，质量据实记录 | 待验收（工具链软件验证已完成） |
| 标定后生产 | 退出示教、移除板，重新七终端，复验单次/自动 | 待验收 |

## 10. 回退

1. 正常停止任务与节点。
2. 外部目录保全迁移后新数据（会话/结果/配置/saved_poses/conveyor_calib/日志）+ manifest 哈希。
3. 全量回退：先重命名保全当前空间，再从已验证归档恢复到**原绝对路径**（详见 `BACKUP_VERIFIED.md` §6）。
4. 用户 site-packages 本次未修改，正常回退无需覆盖全局环境。
5. 校验资源哈希/解释器链接/活动外参/Git 状态后，按原 T1~T6 复核。

## 11. 最终交付清单（任务 8.4）

- [x] 完整备份与 `BACKUP_VERIFIED.md`
- [x] 迁入源码、模型部署目录、配置、受跟踪资产 manifest 与外部依赖清单
- [x] 统一启动入口、GUI 生命周期与补偿热更新
- [x] 可构建的标定包、板生成工具、候选/批准保护与两相机教程（`docs/calibration.md`）
- [x] 清理/归档逐文件记录与原路径映射（`archive/README.md`、`log/migration/`）
- [x] 根 `README.md`、`docs/calibration.md`、`docs/migration_report.md`
- [x] 软件测试命令、结果与差异（§8）
- [ ] 现场抓取、模式切换、标定工具与恢复生产验收记录（§9 待执行）
- [x] 回退步骤与备份实际位置（§10）
