# 标定专项审查、TCP工具与旧README恢复记录

日期：2026-10-15（Asia/Shanghai）。本轮修改前已备份到：
`/home/lxf/agx_arm_ws_backups/20261015-calibration-tcp-872fcfc0/`，
其中 `before.tar` 保存相关源码、测试、配置和文档，`board_before.tar` 保存原板图与尺寸说明，均有 SHA256 清单。

## 1. 用户现场反馈与本轮范围

用户确认此前 T1 不运动是误按示教按钮，退出后已能使能回零，并已跑通生产全流程，暂未发现问题。
本轮没有改动 T1 的运动、使能逻辑或自动分拣轨迹，也没有重新执行硬件激活/运动。
用户未提供分拣计次及逐项验收记录，因此该反馈按“全流程已跑通”记录，不补造原验收表要求的次数。

本轮交付：

- 审查 D455 眼在手外、Dabai 眼在手上的采集、求解、质量与推广链路。
- 新增用户选定的固定点法 TCP XYZ 标定，保留当前 RPY。
- 恢复之前的 README，并恢复用户习惯的 CAN 激活命令。

## 2. 两种相机标定的审查结论

令 `H_i = T_base_tcp`、`C_i = T_camera_board`：

| 模式 | 待求量 | 固定关系 | 结论 |
|---|---|---|---|
| D455 eye-to-hand | `Y = T_base_camera` | `inverse(H_i) · Y · C_i = T_tcp_board` | 采样先取机器人变换完整逆，AXXB 与 holdout 使用同一约定，正确 |
| Dabai eye-in-hand | `X = T_tcp_camera` | `H_i · X · C_i = T_base_board` | 主求解与 holdout 方向一致，正确 |

- 两相机默认入口分别选择正确的图像、CameraInfo、批准内参和候选目录；批准结果按相机分开。
- 当前驱动 `/feedback/tcp_pose` 已包含旧 TCP offset，手眼采集不应再次叠加；其位姿是米、xyzw 四元数。
- 内参批准、分辨率、同步/消息年龄、重复帧、TCP offset 端点记录和候选隔离沿用现有逻辑。
- 标定操作仍是**真实终端交互**，没有新增 GUI 采集入口。

### 已修正：独立的逐帧推广门

原来调低 `min_markers/min_corners/min_square_px` 或调高 `max_reproj_rms` 后，
候选即使未满足固定逐帧门，也可能凭求解质量报告进入审批。

现在：

1. 逐帧 RMS 和方格像素保留未四舍五入的原值，避免边界值误判。
2. `criteria.capture_frames` 保存每个保留帧的四个观测值；`acquisition_thresholds` 记录实际采样设置。
3. 候选生成与审批都按 **markers≥14、corners≥20、RMS<0.5px、square≥40px** 检查全部保留帧。
4. 生成质量状态的最少姿态数有15组硬下限；放宽采集参数只用于诊断，不会降低推广门。
5. 缺少逐帧质量的新格式以前的候选不能直接推广；原生产 legacy 外参保持原值、继续作为基线。

### 已修正：标定板物理尺寸

原 PNG 缺少 DPI，打印软件可能无法可靠解释“100%实际尺寸”。
现在写入约 **300 DPI**，明确默认整图 **264×200mm**（含四周20mm边缘）、
板面 **224×160mm**、方格 **32mm**，说明使用 A4 横向或更大纸张并实测。
新旧 PNG 像素内容一致，修改的是打印分辨率元数据。实际打印后仍需量尺确认。

D455 文档也已澄清：非零 TCP 只改变机器人反馈端点，D455 外参仍相对 base；
Dabai 外参相对采集时的实际 TCP。

## 3. TCP程序

入口：

```bash
cd /home/lxf/agx_arm_ws
./scripts/run_component.sh calibrate-tcp
```

源码：`src/handeye_calibration_ros/handeye_calibration_ros/tcp_calibration.py`。
说明：[`tcp_calibration.md`](tcp_calibration.md)。

- 固定点最小二乘联合求工具 XYZ 和基座中的固定点。
- 从带旧 offset 的 TCP 位姿**完整还原法兰**，包含旧平移和旋转；新 XYZ 为法兰下绝对偏移，RPY保留。
- 至少6组，建议10～15组；检查 rank、条件数、全部点的 RMS 与最差残差，拒绝单轴等退化数据。
- 采样检查新消息、停稳和重复姿态；支持 Enter、撤回、求解后继续采集、退出。
- 保存工作会话、每次求解的独立输入快照、输入 SHA256、实际质量阈值与候选。
- 实时路径只订阅位姿、读取参数；没有运动发布器、使能调用或参数写入。
- 离线可通过直接 Python 或统一入口 `calibrate-tcp -- --input ...` 复算。
- 不合格时 `tcp_offset=null`；合格时打印完整六维 T1 启动参数，由操作者核验后重启应用。

当前 T1 只在启动时给 SDK 应用 offset，所以文档明确不能使用 `ros2 param set tcp_offset` 在线应用。
Dabai 的 TCP 相对外参也依赖当时的 TCP 定义；建议先标定 TCP，再做 Dabai 手眼。

## 4. 旧README和CAN命令

[`README_previous.md`](../README_previous.md) 已恢复到根目录，顶部注明“之前的 README.md（旧版，迁移前原文）”。
正文与原完整备份/现存 `docs/guides/agx_driver_original_README.md` **逐字一致**，旧相对路径按历史资料保留。
当前 README 仍是日常入口，并链接到旧版。

当前 README、启动提示、入口注释与 TCP 手册已统一恢复用户习惯的完整命令：

```bash
bash ~/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh can0 1000000 && cansend can0 123#1122334455667788 && ip -s link show can0
```

随后仍运行 `activate_robot.py`，再启动 T1～T6。本轮只更新命令文本，没有代替用户发送该 CAN 帧。

## 5. 软件检查证据

完整测试记录：`log/migration/calibration_tcp_tests_20261015.txt`。

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest \
  -p no:anyio -p no:cacheprovider tests/ -q
# 239 passed（最终用时见日志）
```

| 检查 | 结果/证据 |
|---|---|
| 完整软件回归 | 239项通过，涵盖原有流程及新增TCP/逐帧推广/打印DPI用例 |
| 标定包构建 | 成功；`log/migration/calibration_tcp_build_20261015.txt` |
| 三个安装入口 | 内参、手眼、TCP均可发现；TCP `--help`成功；`calibration_tcp_entrypoints_20261015.txt` |
| 静态预检查 | `ok=True, missing=[]`；`calibration_tcp_preflight_20261015.json` |
| 标定命令干跑 | TCP实时/离线和两相机内参/手眼6组，两种cwd输出一致；`calibration_tcp_dryrun_20261015.txt` |
| TCP数学/文件 | 已知真值、旧XYZ/RPY还原、q/-q、噪声/误触、退化、离线、输入快照与非默认阈值复现通过 |
| 相机标定 | 双模式合成真值、原有数值/质量回归通过；新增逐帧固定门拒绝用例通过 |
| 保留资源 | 模型权重、活动D455外参、DDS哈希与迁移基线一致 |
| 板图与旧文档 | 旧README正文一致；新旧PNG像素一致且包含300DPI |

上述是软件检查。两相机真实采样质量、TCP固定点采集与独立姿态核验尚需操作者现场完成；
不会将合成样本结果冒充实测标定精度。
