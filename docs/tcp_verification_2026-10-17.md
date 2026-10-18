# Piper TCP 定点姿态验证交付与软件检查记录

日期：2026-10-17（Asia/Shanghai）。状态：实现及软件检查完成；现场规划、自动运动、实际取消、物理精度验证待操作者执行。

## 1. 用户确认与质量状态

用户已明确选择：“接受这次粗标定：允许明确标注质量豁免后用于验证”。据此增加仅限验证的例外，
默认仍拒绝 failed；原质量门、原始样本、原候选状态不改变。

会话 `data/calibration/tcp/2026-10-16_22-42-05_c3296802` 复核结果：10 组、rank=6、
condition=4.199976779490851、RMS=5.358227754611016 mm、max=8.718494213648277 mm。
原 RMS≤2 mm、max≤5 mm 门未通过。姿态分布可辨识，固定点残差不足以满足原门，不能从残差独立定位误差来源。
全部样本参加，未删样本或改为 passed。

粗拟合六维值（m/rad）：
`[0.014575441592672675, -0.005306490263008168, 0.1457583484569631, 0.0, 0.0, 0.0]`。
另存 `verification_acceptance.json`：`scope=verification_only`、`qualified=false`、`applied=false`。
原 `candidate.json` 仍 failed 且 `tcp_offset=null`。
实际使用仍需 `--allow-unqualified` 及有效 `--waiver-reason`；结构/秩/条件数/样本不足不能豁免。

## 2. 开工备份与保留项

备份目录：`/home/lxf/agx_arm_ws_backups/20261017-verify-tcp-5c7a1fdf/`。
`before.tar` 包含相关启动器、脚本、标定包、文档、测试及该 TCP 会话；SHA256：

```text
0656ae594f56db3ef88300339eea8db1baff75a43c1ab24e2f2b2418e5796eb0
```

已重新校验归档哈希，并逐字节比对备份里的：原 TCP 标定程序、frames.json、candidate.json、求解快照。
原始数据 SHA256：

| 文件 | SHA256 |
|---|---|
| frames.json | `4705ec56fa524b59a2b9b1f4c5dc94086f60f0c7b54cb61fe7c8703938225e7e` |
| candidate.json | `3486699467a7bbcc61129f95a42a0bb7db6bc8e68dcce6b2faf81b1043b56e83` |
| solve_input_680d217bd0414c34855999abb4020719.json | `a9f7ad04d70a2742567cf3a383bc9c6b7fd2e37c9fb00ae14bb00b6be609b3a2` |

本轮未修改 T1 驱动、MoveIt 配置、GUI/自动分拣代码、模型、相机外参、旧 README、CAN 激活脚本或已有虚拟环境。
主 README 和入口脚本中的原 CAN 命令保留。未应用 TCP 到 T1，未启动/停止用户硬件节点。
本轮备份覆盖范围内已有文件的变化仅为：runtime_launcher.py、scripts/run_component.sh、
标定包 setup.py/package.xml、docs/tcp_calibration.md、README.md；新增文件另列下一节。

## 3. 交付文件

| 文件 | 用途 |
|---|---|
| `src/handeye_calibration_ros/handeye_calibration_ros/tcp_verification.py` | 纯数学、候选重算、严格门/显式豁免、轨迹计时、默认零运动CLI、编排、内外部报告 |
| `src/handeye_calibration_ros/handeye_calibration_ros/tcp_verification_ros.py` | 只在 plan/execute 导入ROS；启动参数/模型/反馈一致性、碰撞/FK、笛卡尔规划、ExecuteTrajectory及取消 |
| `runtime_launcher.py` | 新增verify-tcp，dry-run调用纯数学检查，拒绝后台运行该工具 |
| `scripts/run_component.sh` | 帮助注释加入验证入口，原启动行为保留 |
| `src/handeye_calibration_ros/setup.py`、`package.xml` | 安装tcp_verification入口并声明已使用的ROS消息/接口依赖 |
| `tests/test_tcp_verification.py` | 42项离线测试，不建立ROS节点或访问设备 |
| `docs/tcp_calibration.md` | 第7节完整干跑、规划、执行、当前粗标定豁免、停止、报告及恢复流程 |
| `README.md` | 日常入口增加验证链接及零运动干跑命令 |
| `data/calibration/tcp/.../verification_acceptance.json` | 用户明确接受的本次验证范围和完整复算/哈希记录 |
| `data/calibration/tcp_verification/offline_review_2026-10-17/report.json` | 已运行的离线数学检查；起点为示例，没有现场反馈/运动 |
| `docs/superpowers/plans/2026-10-17-verify-tcp.md` | 本轮实现决策与核查清单 |

## 4. 实现要点及独立审查修正

- 直接对 MoveIt `tcp_link` 规划，SDK反馈已含offset，不重复补偿；法兰离线检查使用完整逆。
- 从新鲜真实反馈建立起点，默认工具本地X/Y轴±5°、每边5步、一次循环、速度/加速度缩放0.03。
- 检查managed T1驱动实际进程的启动参数文件、在线参数和MoveIt模型完整变换。
  在线param set不能代替SDK启动生效。支持Python console script实际argv中的解释器位置。
- 笛卡尔路径开启碰撞，fraction不完整拒绝；检查六轴限位、跳跃、FK、插值状态。
  轨迹零端v/a五次插值保守计时，实际起点衔接也计时并检查，禁止未检查的零时长起点跳变。
- guard和派发前、执行中复核哈希；派发紧前检查中断/参数变化；执行后核对新的真实反馈。
- 不依赖驱动未填充的err_status单字段；同时检查六轴限位/通讯故障数组。
- 只取消本工具的ExecuteTrajectory goal；延迟接受也有取消回调，结果异常/STATUS_UNKNOWN不能视为终态。
  未确认时保留executor与取消owner；即使报告写盘失败也继续处理。无附加回零/待机。
- 真实TTY最后确认，Ctrl+C无需补Enter即可退出确认等待。
- 内部反馈单独记录，外部物理观测默认未完成；可关联已执行步骤或已接受的中断执行的人工测量。

独立只读审查指出的起点零时长衔接、故障位遗漏、哈希变化、延迟接受/终态判断及报告失败影响取消等问题，
均已在本轮修正并加入对应离线边界测试。

## 5. 软件检查证据

| 检查 | 结果 |
|---|---|
| 完整 `tests/` 回归 | **281 passed in 8.92s**，退出码0 |
| 新增验证测试数量 | 42项；包含在完整回归中 |
| handeye_calibration_ros单包构建 | 成功，退出码0；未运行节点 |
| 安装入口发现 | 4个：camera_intrinsic_calibration、handeye_calibration、tcp_calibration、tcp_verification |
| 已安装验证入口 `--help` | 成功；不创建ROS节点 |
| ROS消息/接口导入 | rclpy action状态QoS、GetCartesianPath、GetPlanningScene、ExecuteTrajectory、AgxArmStatus可导入；没有rclpy.init |
| 根目录与/tmp的粗标定豁免dry-run | 输出逐字节一致（2615字节）；无报告写入、无节点/设备访问 |
| 默认failed候选 | 离线测试确认在ROS backend构造前拒绝 |
| 纯数学报告 | 成功，40目标、固定示例XYZ，状态math_only_no_motion |
| shell语法 | bash -n scripts/run_component.sh通过 |
| 备份/原数据完整性 | 归档SHA正确；原始数据、原候选、快照和原采集求解程序一致 |

完整回归命令：

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest -p no:anyio -p no:cacheprovider tests -q
```

本机用户site-packages中anyio pytest插件与当前环境不兼容，沿用已有`-p no:anyio`；未修改虚拟环境。
新增测试覆盖非零XYZ/RPY完整变换、双轴定点、候选拒绝/豁免、干跑零ROS零文件、plan零执行、
不完整路径、限位/五次插值内部v/a、实际起点衔接、碰撞/FK、故障数组、冻结时间戳、停止后续派发、
晚到接受、异常/未知终态、写盘失败保留取消owner、TTY中断、物理观测隔离。

## 6. 现场待验证清单

- [ ] 按文档结束抓取并关闭T5/T6；确认所有外部运动源退出。
- [ ] 操作者留出T1启动/回零空间，用本次验证值手动启动T1，核对真实SDK/URDF一致性。
- [ ] 真实工具点与参考点留隙对准，模型/工具/桌面场景符合现场。
- [ ] `--mode plan`通过；真实ROS接口名、状态QoS、控制器和路径返回符合预期。
- [ ] `--mode execute`现场输入确认后，以小角度低速完成第一轮；观察实际反馈与运动平滑性。
- [ ] 现场确认取消、超时或故障时控制器真实停止；软件取消不代替急停。
- [ ] 外部量具/相机测量真实尖端漂移，记录方法及最大值，判定是否满足任务需求。
- [ ] 恢复旧完整TCP启动参数，重新核验既有生产抓取流程；需要生产精度时重新采样/标定。

本轮没有执行上述现场步骤，不以软件回归或数学固定XYZ代替物理精度结论。
