# Piper TCP 位置标定（固定点法，仅 XYZ）

## 1. 适用范围与输出

本工具标定 **Piper 法兰 `link6` 到工具中心 `tcp_link` 的 XYZ 偏移**，单位米。
当前 `tcp_offset` 的后三项 roll/pitch/yaw（弧度）原样保留，**不宣称已标定工具朝向**。

操作者把工具上的同一点从不同方向对准同一个基座固定点，程序读取机器人位姿并求解：

```text
R_base_flange[i] · tcp_xyz + p_base_flange[i] = pivot_base
```

`tcp_xyz` 和固定点 `pivot_base` 一起做最小二乘求解。该方法的操作思路可参照
[Universal Robots 的固定点 TCP 位置教学说明](https://www.universal-robots.com/manuals/EN/HTML/SW5_22/Content/prod-usr-man/software/PolyScope/content/installation_g5/teaching_TCP_position.htm)。
本项目额外要求至少六组，并对退化姿态和所有观测残差做软件检查。

程序只有 `/feedback/tcp_pose` 订阅和读取 `tcp_offset` 的 GetParameters 客户端，
采集过程中不发送运动/使能指令，也不在线设置参数、不自动修改生产配置。

## 2. 先定义你要标定的“工具点”

- 若用于抓取中心，应使用能重复对齐**两指中心**的固定治具，并保持夹爪开口及工具安装状态一致。
- 也可以夹持标定针，让针尖对准固定锥尖；此时得到的是**针尖 TCP**。若针尖不是抓取中心，不能直接把它当作抓取中心使用。
- 固定点全程不能移动，工具/夹爪上的采样点不能更换。每次采样先停止运动。
- 仅平移、仅绕同一根轴旋转都不足以辨识全部 XYZ；须改变至少两个非平行轴的朝向。

## 3. 现场采集步骤

### 3.1 硬件与 T1

停止 GUI 自动/动态抓取、排队及分拣动作节点。按主 README 的习惯命令完成准备：

```bash
cd /home/lxf/agx_arm_ws
bash ~/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh can0 1000000 && cansend can0 123#1122334455667788 && ip -s link show can0
/usr/bin/python3 /home/lxf/agx_arm_ws/activate_robot.py
./scripts/run_component.sh arm
```

**T1 单独占一个终端**，保留它提供真实位姿反馈；相机、YOLO 和 GraspNet 不参与 TCP 固定点求解。
从新启动的 T1 开始采样，记录它的 `tcp_offset`；本轮内不要在线修改该参数。

当前驱动只在启动时把 TCP 参数传给 SDK，单独执行 `ros2 param set tcp_offset` 不会同步实际 SDK。
因此本文只采用**求解候选 → 现场核验 → 用完整六维参数重启 T1**的应用方式。

### 3.2 启动工具

在另一个真实终端执行：

```bash
cd /home/lxf/agx_arm_ws
./scripts/run_component.sh calibrate-tcp
```

启动时读取 `/agx_arm_ctrl_single_node/get_parameters` 中的六维 `tcp_offset`，打印来源和会话目录。
`/feedback/tcp_pose` 已带旧 TCP 偏移，程序先用完整变换还原法兰：

```text
T_base_flange = T_base_tcp_reported · inverse(T_flange_tcp_current)
```

这里同时处理旧 XYZ 和旧 RPY，不只是减去一个世界坐标平移，也不会把旧偏移重复叠加。

当前旧版驱动的 PoseStamped header.frame_id 可能为空。采样文件会记录这一情况，
按该 Piper 驱动 API 约定解释为 `base_link`；非空且不是 `base_link` 时拒绝采样。

### 3.3 采样、回退和求解

1. 使用你熟悉的手动控制或示教方式调整姿态，使同一工具点与固定点重合。
2. 停稳后按 **Enter**；程序等待 0.8 秒，并核对最近 0.25 秒的反馈窗口。
3. 改变工具朝向，再把同一点对准固定点。建议 **10～15 组**，至少 **6 组**；绕两个以上轴充分展开姿态，避免碰撞或关节极限。
4. `d`：撤回上一组并同步保存原始记录。
5. `q`：求解并保存候选，打印误差与未通过项；可以继续补采，再次 `q`。
6. `c` 或 Ctrl+C：退出，已落盘的样本保留。

采样要求新消息至少 10 条，最新消息接收年龄不超过 1 秒；窗口内位移不超过 1 mm、
转角不超过 0.3°。与已采集姿态近乎重复时拒绝添加，消息过期或未停稳时会提示原因。
这些稳定性检查只能识别机器人运动，不能替代操作者检查工具点是否与固定点重合。

### 3.4 查看候选质量

默认软件门如下（2026-10-17按用户粗标定需求由原2/5mm调整为5/10mm）。这是拟合残差标准，不是对实际工具精度的保证：

| 项 | 默认门 |
|---|---|
| 样本数 | ≥6，建议 10～15 |
| 联合方程秩 | 6；不足说明某些方向无法辨识 |
| 条件数 | ≤100，过大说明姿态分布差 |
| 所有点的三维残差 RMS | ≤5 mm |
| 最差单点残差 | ≤10 mm |

所有样本都参与拟合，不会自动删除误触点后假报通过。报告保存每个样本残差，便于重新对点采集。
任一质量门失败时 `quality_status=failed`，`tcp_offset=null`，不会打印可直接应用的参数。
第7节记录此前旧数据的历史豁免，原failed候选保留。第11节是本次15组数据按新5/10mm标准复算的结果。

在已有精度需求/现场测量依据时，可显式调整求解门，例如：

```bash
./scripts/run_component.sh calibrate-tcp --ros-args \
  -p max_rms_mm:=1.0 -p max_residual_mm:=3.0 -p max_condition:=50.0
```

## 4. 保存文件与离线复算

```text
data/calibration/tcp/<时间_UUID>/
  frames.json      # 原始TCP位姿、当前六维offset、接收时刻、header、稳定性记录
  solve_input_<UUID>.json  # 每次求解的独立原始输入快照，后续补采/撤回不会改写
  candidate.json   # 求解值、固定点、保留的RPY、逐点残差、秩/条件数/阈值/质量状态
```

候选 `session_file` 指向对应求解的独立输入快照，并保存 `input_sha256`；`working_session_file` 指向持续采集文件。
会话目录唯一。写入使用临时文件后原子替换；不会写 `config/calibration/d455_eye_to_hand.json`、
Dabai 外参或 T1 配置。候选中始终记录 `applied=false`、`orientation_calibrated=false`。

离线复算无需启动 ROS 或连接机械臂：

```bash
/usr/bin/python3 /home/lxf/agx_arm_ws/src/handeye_calibration_ros/handeye_calibration_ros/tcp_calibration.py \
  --input /绝对路径/frames.json \
  --output /绝对路径/candidate_recomputed.json
```

当前部署也可通过统一入口执行 `./scripts/run_component.sh calibrate-tcp -- --input /绝对路径/frames.json`；
直接 Python 方式无需加载 ROS 环境。

精确复现某次候选时，使用其 `session_file` 指向的 `solve_input_<UUID>.json`（包含当次质量阈值）；
工作 `frames.json` 用于按当前样本重新求解。

离线会从原始 TCP 位姿和当时的旧 offset 重新还原法兰再求解，不信任另存的派生法兰矩阵。
未提供 `--output` 时写同目录 `candidate.json`；不会覆盖输入 `frames.json`。

## 5. 核验并用于 T1

先检查实际工具点定义、残差、姿态分布，再用**未参加拟合的姿态**对准固定点，验证工具点位置。
通过后，使用候选打印的完整命令；以下仅是格式示例，数值须替换为你的真实候选：

```bash
# 结束当前动作，退出示教并按既有流程停掉旧T1，再在T1终端运行：
./scripts/run_component.sh arm -- 'tcp_offset:=[0.0, 0.0, 0.12, 0.0, 0.0, 0.0]'
```

若用过示教模式，恢复生产前必须退出。项目现有 `/exit_teach_mode` 服务会执行回零/复位序列，
由现场操作者确认路径后调用；TCP 标定工具本身不会调用它：

```bash
source /home/lxf/agx_arm_ws/scripts/runtime_env.sh
ros2 service call /exit_teach_mode std_srvs/srv/Empty
```

新 T1 会同时给驱动和 MoveIt 的 `tcp_link` 使用新参数。启动后核对参数、TF 和真实工具点，再恢复生产任务。
**改 TCP 会改变规划目标点及工具坐标语义**，需要复验原抓取流程；TCP 与手眼外参是不同标定量。

- D455 的外参是 `T_base_camera`，只改变 TCP 定义不会改变固定相机的物理安装变换。
- Dabai 的外参是相对**采集时 TCP**的 `T_tcp_camera`；TCP 改变后旧外参不能直接当作新 TCP 的外参。
  推荐先定好 TCP，再做 Dabai 手眼标定，并保留当时六维 offset。

## 6. 软件验证与重建

源码为 `src/handeye_calibration_ros/handeye_calibration_ros/tcp_calibration.py`。
复用现有 NumPy，无新增环境；ROS 导入只发生在实时采集路径。
安装入口随标定包构建：

```bash
source /home/lxf/agx_arm_ws/scripts/runtime_env.sh
PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select handeye_calibration_ros
source /home/lxf/agx_arm_ws/install/setup.bash
ros2 pkg executables handeye_calibration_ros
```

软件回归覆盖已知真值、非零旧位置/朝向还原、q/-q、噪声、错误接触点、单轴退化、
非有限值、离线文件与新鲜度/停稳检查。真实工具点采集与独立姿态核验需现场执行。


## 7. TCP 定点姿态运动验证

### 7.1 目标、坐标和证据边界

新增 `verify-tcp` 用于观察：**真实工具上的同一点是否在姿态变化时保持在同一基座位置**。
默认以启动时的新鲜真实 TCP 反馈为起点，绕**起始工具坐标系的 X、Y 两轴**分别
`0 → +5° → 0 → -5° → 0`，每条边分 5 步，总计 40 个有限步骤，每轴结束恢复起始朝向。
XYZ 始终取本次验证起点，不会拿原标定的固定点坐标作为运动起点。

- MoveIt 直接规划 `arm` 组的 `tcp_link`。该模型端点已包含 T1 的完整 TCP 变换，**不再重复手工补偏移**。
- `/feedback/tcp_pose` 是带 T1 启动偏移的真实 SDK 反馈；程序核对它与 MoveIt FK 的一致性。
- 离线法兰核对使用 `T_base_flange_target = T_base_tcp_target × inverse(T_flange_tcp)`，同时处理 XYZ 和 RPY。
- 这项验证不求解工具朝向，保持候选的原有 RPY 约定。
- 内部反馈偏差反映控制跟踪和模型一致性；同一模型里的 XYZ 一致，**不能证明物理工具尖端不漂移**。
  必须另用固定尖点、相机或量具观察/测量；未填写外部测量的报告始终写“未完成物理精度验证”。

默认 `math` 不创建 ROS 节点、服务或运动客户端。`plan` 访问现有 ROS 做只规划；只有 `execute`
会创建 ExecuteTrajectory 客户端，而且必须在真实终端再次输入确认文字。
工具不会启动 T1、激活 CAN、使能、回零或退出示教。

### 7.2 本次粗标定的状态与豁免

会话：`/home/lxf/agx_arm_ws/data/calibration/tcp/2026-10-16_22-42-05_c3296802/`。
本次复核仍为：

| 项目 | 复核值 | 原门 | 结果 |
|---|---:|---:|---|
| 样本数 | 10 | ≥6 | 通过 |
| 秩 | 6 | 6 | 通过 |
| 条件数 | 4.1999768 | ≤100 | 通过 |
| RMS | 5.3582278 mm | ≤2 mm | 未通过 |
| 最大残差 | 8.7184942 mm | ≤5 mm | 未通过 |

姿态分布足以辨识 XYZ，但固定点一致性不足；仅凭这些数据不能确认误差来自对点、工具松动还是运动学偏差。
原始 10 组全部参与，没有删样本、改门或把 failed 改成 passed。
已有 `candidate.json` 的 `tcp_offset` 保持 `null`。

用户明确接受“粗标定仅用于运动观察验证”后，另存
`verification_acceptance.json`，记录原状态、门、双 SHA256、用户授权范围和重新计算的六维值：

```text
tcp_offset = [0.014575441592672675, -0.005306490263008168, 0.1457583484569631, 0.0, 0.0, 0.0]
单位：前三项 m，后三项 rad
scope = verification_only，qualified = false，applied = false
```

**该记录不批准生产标定，也不会自动修改 T1 或生产配置。** 运行时仍需使用原 `candidate.json`
并显式传 `--allow-unqualified --waiver-reason '本次粗标定仅用于现场姿态验证'`。
缺少任一项时默认拒绝 failed 候选。格式错误、输入不完整、哈希不符、伪造 passed/数值、
秩不足、条件数过大或样本不足均不能豁免。每次加载按原始快照重算；旧2/5mm结果作为历史参照保留。
当前工作树的验证器按第11节5/10mm标准接受重新求解且passed的候选；旧failed文件仍需显式豁免或另存复算。

### 7.3 离线干跑与数学报告（无需机械臂）

```bash
cd /home/lxf/agx_arm_ws
# 零运动、零 ROS 节点、零报告文件：用示例姿态演示数学序列
./scripts/run_component.sh verify-tcp --dry-run

# 当前粗标定复核干跑：保留 failed，仅显示显式验证豁免
./scripts/run_component.sh verify-tcp --dry-run -- \
  --candidate /home/lxf/agx_arm_ws/data/calibration/tcp/2026-10-16_22-42-05_c3296802/candidate.json \
  --allow-unqualified --waiver-reason '本次粗标定仅用于现场姿态验证'

# 保存离线数学报告；这里的起点是离线示例，不是现场反馈
./scripts/run_component.sh verify-tcp -- \
  --mode math \
  --candidate /home/lxf/agx_arm_ws/data/calibration/tcp/2026-10-16_22-42-05_c3296802/candidate.json \
  --allow-unqualified --waiver-reason '本次粗标定仅用于现场姿态验证' \
  --output /home/lxf/agx_arm_ws/data/calibration/tcp_verification/math_report.json
```

统一启动器自己的 `--mode` 用于视觉模式，所以 **verify-tcp 的参数放在 `--` 后**。
`--dry-run` 放在 `--` 前；它会调用纯数学检查，而不只是打印启动命令。
`--dry-run` 与 `--mode plan/execute` 组合会报错。
也可用已安装的 `ros2 run handeye_calibration_ros tcp_verification --help` 查看参数；
纯 Python 离线入口在 `src/handeye_calibration_ros/handeye_calibration_ros/tcp_verification.py`。

### 7.4 现场准备：由操作者完成

1. 完成当前动作，停止自动分拣、动态抓取和排队任务，**关闭 T5 和 T6**。
   程序会拒绝检测到 `/piper_moveit_action_client`、`/sorting_headless_client_node` 的情况，
   也拒绝控制器或 MoveIt 尚有活动轨迹。节点名检查不能识别所有外部运动源，操作者须关闭其他控制客户端。
2. 记录生产 T1 的完整启动命令和旧六维 TCP。当前这组采样的旧值是六个零。
   若要使用本次粗标定做观察，先按现场既有流程停旧 T1，再**由操作者**用下面参数启动 T1。
   **T1 原本的使能/回零行为仍存在，启动时先清空路径，启动完成后再对准参考点。**
3. CAN 和激活沿用主 README 的原命令；本工具不会代执行。T1 必须用统一 `arm` 入口启动，
   以便核对实际驱动进程的启动参数文件。不能只用在线 `ros2 param set` 冒充 SDK 已应用。

```bash
# 操作者在独立 T1 终端执行；先确认启动/回零空间，旧 T1 已退出
cd /home/lxf/agx_arm_ws
./scripts/run_component.sh arm -- \
  'tcp_offset:=[0.014575441592672675, -0.005306490263008168, 0.1457583484569631, 0.0, 0.0, 0.0]'
```

4. 用你熟悉的手动方式把**同一个真实工具点**对准固定参考点；第一次留出可见的小间隙，
   从侧面观察。不要硬接触、强压参考尖点。保持夹爪开口和工具安装不变。
5. 确保机械臂回到正常 CAN 控制状态，停稳；如果在示教模式，请按既有现场步骤退出并确认其动作，
   本程序不会代退出。确认规划场景中的桌面、工具等障碍模型符合现场，急停可用。
6. 验证结束如需恢复原生产，先结束动作，再由操作者按旧六维值重启 T1 并复验原抓取流程。
   对原始零TCP部署，明确恢复命令是 `./scripts/run_component.sh arm -- 'tcp_offset:=[0.0,0.0,0.0,0.0,0.0,0.0]'`；（本轮arm默认已是新TCP，整体回退另见双相机教程）若旧生产用了覆盖参数，
   应恢复你记录的完整旧命令。不要把验证豁免当成生产精度验收。

### 7.5 只规划，不执行

T1 已运行并使用与候选一致的六维 TCP、T5/T6 已退出、真实起点已停稳后，在另一个终端：

```bash
cd /home/lxf/agx_arm_ws
./scripts/run_component.sh verify-tcp -- \
  --mode plan --tasks-stopped \
  --candidate /home/lxf/agx_arm_ws/data/calibration/tcp/2026-10-16_22-42-05_c3296802/candidate.json \
  --allow-unqualified --waiver-reason '本次粗标定仅用于现场姿态验证' \
  --angle-deg 5 --steps 5 --axes x y --cycles 1 --speed 0.03 \
  --output /home/lxf/agx_arm_ws/data/calibration/tcp_verification/plan_report.json
```

`plan` 没有创建运动 goal。它从当前真实反馈生成本轮起点，依次预规划所有小段，并保存分段轨迹、
关节限位、碰撞/FK检查、完整度等。下次执行会重新从当时的真实起点规划，不会盲目重播这个文件。

程序核对实际驱动的启动 `tcp_offset`、在线参数和 MoveIt URDF `link6 → tcp_link` 完整刚体变换，
并检查真实 SDK TCP 与 MoveIt FK、控制器关节状态的一致性。任一不符都应查清来源；不要绕过检查。

### 7.6 现场明确确认后执行

以下命令只交给在场操作者运行。本次开发没有执行它：

```bash
cd /home/lxf/agx_arm_ws
./scripts/run_component.sh verify-tcp -- \
  --mode execute --tasks-stopped --point-aligned-with-gap \
  --candidate /home/lxf/agx_arm_ws/data/calibration/tcp/2026-10-16_22-42-05_c3296802/candidate.json \
  --allow-unqualified --waiver-reason '本次粗标定仅用于现场姿态验证' \
  --angle-deg 5 --steps 5 --axes x y --cycles 1 --speed 0.03 \
  --output /home/lxf/agx_arm_ws/data/calibration/tcp_verification/onsite_report.json
```

所有小段规划成功后，终端显示候选原质量与豁免，并要求输入 **`EXECUTE TCP`**。
其他输入或 Ctrl+C 停止，不执行后续动作。无真实交互终端时拒绝 execute。
两个声明参数代表操作者已完成任务停止和留隙对点，不能替代现场检查。

| 参数 | 默认 | 限制/含义 |
|---|---:|---|
| `--angle-deg` | 5 | `(0,15]`；围绕起始工具轴 |
| `--steps` | 5 | 每条边 1～10 步，每步 ≤2.5° |
| `--axes` | x y | 至少两个不同轴，允许 x/y/z |
| `--cycles` | 1 | 1～3；总步骤 ≤240 |
| `--speed` | 0.03 | 速度/加速度缩放 0.005～0.1 |
| `--position-tolerance-mm` | 5 | 内部跟踪容差 1～10 mm；不是物理精度门 |
| `--output` | 唯一会话/report.json | 默认在 `data/calibration/tcp_verification/` |

### 7.7 规划与停止规则

- `GetCartesianPath` 使用 `tcp_link`、`avoid_collisions=true`；fraction 必须达到完整路径，
  IK失败、碰撞、非有限值或缺失轨迹均拒绝。不会为通过而关闭碰撞。
- 检查 URDF/MoveIt 关节位置、速度、加速度限制；连续点最大关节变化 0.12 rad。
  路径点以零端速度/加速度作保守五次插值计时，按配置的速度和加速度放慢，每个间隔至少 0.25 s，
  单段超过 60 s 拒绝。规划首点须与种子关节一致。
- 每次派发前，从实际关节到规划首点补上**有时间、有限位和碰撞/FK检查的衔接段**。
  检查插值状态（关节间隔 ≤0.01 rad）有效性，模型 TCP 位置偏差 ≤1 mm，末姿态偏差 ≤0.5°。
  这是离散路径检查，现场障碍物必须由正确的场景模型和操作者留隙共同保障。
- 拒绝超出 SRDF 正常允许对的场景碰撞放行；不会修改场景 ACM。
- 实际反馈须持续新鲜（≤1 s），重复的源时间戳不会刷新有效年龄；六轴通讯/限位故障也会中止。
- 每段前重查候选/输入双 SHA、参数变化、任务节点、控制器空闲、实际关节和控制状态；
  衔接前要求至少 0.3 秒、至少 3 个不同源时间戳的关节反馈窗口，六轴变化均不超过 0.0001 rad；
  最长等待 3 秒，未停稳或反馈冻结则拒绝。衔接核查后如起点变化超过 0.0001 rad，
  最多重新等待停稳、重建并复查衔接 3 次；持续变化时拒绝。不会直接忽略差异或发送未经验证的起点。
  执行中持续检查真实 TCP、双 SHA 和 TCP 参数变更。
- ExecuteTrajectory 返回成功且 MoveIt error_code 成功后，仍等待新的真实反馈验证终点；
  中断、超时、失败或过期停止后续派发，仅取消**本工具自己发出的 goal**。
- 若请求尚未被接受，保留晚到接受回调并取消；取消/终态未确认时程序持续保留接收和取消处理，
  打印提示并落盘状态。**不要关闭该终端**。软件取消不等于硬件急停；通信中断时由在场操作者确认停止，
  必要时使用现场急停。外部强杀进程或断电不在软件取消保证内。
- 停止不会自动回零、回待机或重新对点。

MoveIt 的轨迹需要时间参数才能执行，且限位来自模型及 joint_limits 配置；参见
[MoveIt Humble 轨迹时间处理说明](https://moveit.picknik.ai/humble/doc/concepts/trajectory_processing.html)。
本工具使用已安装 Humble 的 GetCartesianPath/ExecuteTrajectory 服务与动作定义，详见
[MoveGroupInterface 笛卡尔路径接口](https://moveit.picknik.ai/humble/api/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html)。

### 7.8 报告与外部物理测量

报告两类证据独立保存：

| 类别 | 字段与含义 |
|---|---|
| 内部反馈 | 起点真实反馈/时间戳、目标 TCP、逐段路径和关节时序、执行状态、MoveIt 返回码、真实反馈轨迹、位置/姿态偏差 |
| 外部观察 | 默认 `status=未完成物理精度验证`；只有操作者输入独立实测后才记录方法与最大漂移 |

姿态采样中的朝向误差是“当前反馈相对本段终点朝向”，不是时间对齐的瞬时朝向跟踪误差。
目标固定 XYZ 和 FK/SDK 推算值不能填作真实尖端漂移。中断时保留 `inflight_step` 及已采内部反馈，
接受/取消未确认也写入报告；`completed` 只表示这轮内部执行完成。

有实际完成步骤（或 goal 已接受的中断执行记录）且外部测量后，可**另存**带人工观察的报告（该命令不创建 ROS 节点、不运动）：

```bash
./scripts/run_component.sh verify-tcp -- \
  --record-observation /home/lxf/agx_arm_ws/data/calibration/tcp_verification/onsite_report.json \
  --max-drift-mm 2.0 --measurement-method '固定参考尖点旁标尺，侧视相机逐姿态读数，人工取最大偏离' \
  --output /home/lxf/agx_arm_ws/data/calibration/tcp_verification/onsite_with_observation.json
```

`2.0` 只是命令格式示例，请替换为你的真实读数。该记录标为 `operator_manual`，不会伪称已独立自动验证。
没有外部测量时保持默认状态。

### 7.9 软件检查与待现场验证

```bash
cd /home/lxf/agx_arm_ws
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest -p no:anyio -p no:cacheprovider tests/test_tcp_verification.py -q
```

覆盖：非零 XYZ/RPY 完整逆、定点双轴序列、候选哈希/质量/数值拒绝、明确豁免不改原文件、
零 ROS 干跑、只规划零派发、部分路径拒绝、起点衔接、碰撞/FK漂移拒绝、故障数组、冻结时间戳、
中断/超时/反馈失效停止后续派发、晚到接受取消和内外部证据区分。
软件检查与备份记录见 [`tcp_verification_2026-10-17.md`](tcp_verification_2026-10-17.md)。

待现场：真实 ROS 接口可用性、T1启动/SDK/URDF一致性、现有控制器的计划和取消行为、
空隙处小角度执行、外部工具点最大漂移、场景障碍精度、恢复旧 TCP 后原生产抓取流程。
这些项目没有被离线测试或构建结果代替。


## 8. “规划成功但没有运动”的排查（2026-10-17 修正）

- `--mode plan` 只规划，输出“已规划”不代表执行，也不会带动机械臂。
- `--mode execute` 完成全部规划、现场输入 `EXECUTE TCP` 后，才检查起点并发送当前段。
- 旧版若打印 `衔接验证后真实起点移动，须重新规划`，说明还没有派发该段。
  旧逻辑对两次真实关节读数超过 `0.0001 rad` 的变化直接退出，缺少停稳与重新核查步骤。
  不能从这条旧日志判断具体关节或确定物理原因，因为旧版没有记录偏差值。
- 修正版保留原门，新增上述停稳窗口与最多 3 次衔接重建。每次重建仍做计时、关节/碰撞/FK检查，
  不累加旧衔接、不跳过失败。日志打印变化量和重试次数；报告 `start_revalidation` 保留六轴差值和前后快照，
  `start_stability_checks` 保留窗口变化范围、时长和独立反馈数。
- 若仍连续失败，请查看新报告中的这两项，以及是否仍在运动、反馈是否冻结、外部控制端是否活动。
  不要反复放大起点阈值或关闭碰撞检查来强行执行。

保持 T1 运行、T5/T6 关闭且工具点留隙对准后，重跑第 7.6 节原 execute 命令即可；统一入口直接读取新源码。
若上次已执行一部分后中止，请先由操作者重新对点并停稳；下一次会以新的真实反馈为验证起点。
本次修正的软件/只读现场检查记录见 [`tcp_start_fix_2026-10-17.md`](tcp_start_fix_2026-10-17.md)。

## 9. 动作太慢、摆幅太小：连续摆动教程（`--sweep`）

### 9.1 原因及新旧模式

原来的 `--angle-deg 5 --steps 5 --speed 0.03` 每条边分5段、每段1°，两轴共40次执行。
旧模式每个路径点使用零速度/零加速度计时，每段前后还有反馈和停稳核查，适合初次小幅观察，节奏较慢。
用户本次现场报告已完成40步，第一步到最后一步约100秒，未出现执行失败；这不等于物理尖端精度已通过。

新增显式 **`--sweep`**：

- 每条边的多个小姿态保留为规划waypoints，合并为一程，不再每个小姿态单独发送运动goal。
- 每轴四程：`起始 → +角度 → 起始 → -角度 → 起始`；X、Y两轴共 **8程**。
- 每程内部使用连续速度的三次Hermite插值，首尾速度为零；中间点不统一强制停住。
- 保留每程前的停稳、起点衔接复核、碰撞/FK与反馈检查，以及现场确认和取消。
- 推荐明显观察用 **±15°、steps=6、speed=0.1、1轮**。角度是单侧幅度，负端到正端总跨度30°。
  `steps=6`仍表示每条边有6个姿态waypoints（间隔2.5°），不是6次独立启停。
- TCP尖端的目标仍固定在同一XYZ；观察夹爪/法兰整体的姿态摆动，以及真实尖端相对参考点的漂移。
  此工具不会为了增加“行程”而平移TCP。

旧模式和默认值保留。**必须加 `--sweep` 才启用连续摆动**；复制旧命令仍是原来的细分模式。

### 9.2 运行前准备

1. 保持已配置本次六维TCP的T1运行，结束抓取，关闭T5/T6及其他运动源。
2. 按第7.4节确认工具安装、夹爪开口与参考点，将同一工具点留小间隙对准并停稳。
3. 扩大角度会扩大夹爪/法兰扫过的空间，先确认周围间隙和场景模型。
4. 新程序不启动T1、CAN、使能或回零。若提示“真实反馈/控制状态过期”，先排查T1反馈；
   不要将没有反馈当作允许执行，也无需为此修改标定参数。

本轮开发对±15°做现场只规划尝试时未取得新鲜TCP反馈，因此没有获得该幅度的完整现场规划结果，
也没有自动执行运动。下面的plan/execute会在现场重新核验，不重播旧报告。

### 9.3 先干跑：不连接设备、不运动

```bash
cd /home/lxf/agx_arm_ws
./scripts/run_component.sh verify-tcp --dry-run -- \
  --sweep --angle-deg 15 --steps 6 --axes x y --cycles 1 --speed 0.1
```

应看到 `mode=math`、`execution_status=math_only_no_motion`、`target_count=8`、`sweep=true`。
这里用离线示例起点，只确认命令和数学序列，不能证明现场IK或碰撞可行。

### 9.4 只规划较大幅度，不执行

```bash
cd /home/lxf/agx_arm_ws
./scripts/run_component.sh verify-tcp -- \
  --mode plan --tasks-stopped --sweep \
  --candidate /home/lxf/agx_arm_ws/data/calibration/tcp/2026-10-16_22-42-05_c3296802/candidate.json \
  --allow-unqualified --waiver-reason '本次粗标定仅用于现场连续摆动验证' \
  --angle-deg 15 --steps 6 --axes x y --cycles 1 --speed 0.1 \
  --output /home/lxf/agx_arm_ws/data/calibration/tcp_verification/visible_sweep_plan_report.json
```

应完成8程规划。任何一程的IK、碰撞、关节限位、模型TCP定点误差或时间检查失败，整轮拒绝执行。
`speed=0.1`为相对于URDF/MoveIt限制的速度和加速度缩放，不是SDK速度百分数或固定角速度。

### 9.5 现场执行：±15°连续摆动

在场确认参考点留隙、扫掠空间和任务停止状态后执行：

```bash
cd /home/lxf/agx_arm_ws
./scripts/run_component.sh verify-tcp -- \
  --mode execute --tasks-stopped --point-aligned-with-gap --sweep \
  --candidate /home/lxf/agx_arm_ws/data/calibration/tcp/2026-10-16_22-42-05_c3296802/candidate.json \
  --allow-unqualified --waiver-reason '本次粗标定仅用于现场连续摆动验证' \
  --angle-deg 15 --steps 6 --axes x y --cycles 1 --speed 0.1 \
  --output /home/lxf/agx_arm_ws/data/calibration/tcp_verification/visible_sweep_onsite_report.json
```

本命令会重新规划8程，并显示轨迹时长合计；实际用时还包括起点复核、反馈等待和可能的重试。
然后在真实终端输入 **`EXECUTE TCP`**。
日志逐程显示“开始执行 1/8”“已完成 1/8”等进度，便于分辨规划与实际执行。
Ctrl+C停止后续派发并取消本工具的goal，不追加回零。

若希望先稍小一些，将角度和细分改为 `--angle-deg 10 --steps 4`，其余参数保持不变，仍是8程。
角度当前上限15°、速度缩放上限0.1；不要通过编辑门值来强行提高。

### 9.6 插值、检查与时长说明

连续模式发送位置和速度，`accelerations=[]`，要求运行中的
`/arm_controller` 参数 `interpolation_method=splines`；不符合时拒绝，不修改控制器参数。
位置+速度使用三次样条的规则可参照
[ROS2 Control Humble轨迹表示说明](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html)。

程序计算每个三次多项式区间的位置、速度和加速度极值，按配置的缩放限值统一拉伸时间；
检查实际三次曲线的碰撞与FK，而非只检查节点或直线弦。首尾零速度，保留中间速度连续性；
加速度在分段接点可以不连续，未声称具备jerk限制。

以已完成的同一±5°空间路径离线重计时对比：原轨迹时间合计约70.65秒；8程连续模式、speed=0.1约37.50秒。
这是离线轨迹计时比较，不是新模式的实机计时。**±15°路程更长，不能把37.50秒当作其实际用时**；
以现场重新规划打印值及实际报告为准。

软件测试覆盖8程分组、首尾零速/中间非零速度、区间内部超速/超加速度/越限拒绝、
实际曲线取样、重复点去除、干跑零ROS和原有中断/取消/候选检查。
本次变更记录见 [`tcp_sweep_2026-10-17.md`](tcp_sweep_2026-10-17.md)。

### 9.7 如何观察和记录

- 从侧面固定机位观察：工具整体姿态变化应明显；真实尖端应尽量停留在参考点上方。
- 使用参考尖点旁的标尺或固定相机测量最大漂移，保持工具、参考点、相机和夹爪开口不变。
- 本次粗标定的质量状态仍failed，豁免仅用于验证；更大摆幅有助暴露漂移，不会提高标定本身的精度。
- 结束后按第7.8节的`--record-observation`步骤，将报告路径替换为
  `/home/lxf/agx_arm_ws/data/calibration/tcp_verification/visible_sweep_onsite_report.json`，填写实际读数和方法。
  没有外部测量时报告仍为“未完成物理精度验证”。

## 10. ±15°整轮连续运动：18秒轨迹、20秒运动阶段超时

### 10.1 保持的幅度与计时含义

本节明确保持 **±15°**，不是±5°。两轴每轮都经过`+15° → 0 → -15° → 0`。
用户前次贴出的`--angle-deg 5 --steps 5 --speed 0.03`是旧40段模式，不能用它验证新模式时长。

新增`--continuous-round --motion-seconds 18`：X/Y全部姿态合成**一条ExecuteTrajectory goal**，
一程运动中不再逐段停稳、等待反馈或重新派发。必要的换向处瞬时速度为零，没有人为停留段。

- **轨迹时长18秒**，包含实际起点衔接；不会把连接段额外加在18秒之外。
- **20秒从运动请求派发前计时**，覆盖接受等待、执行和最终反馈确认；到期取消并报告失败。
- 规划、碰撞/FK核查、现场输入确认及派发前停稳检查另计。这不保证整条终端命令从启动到退出≤20秒。
- 网络/硬件故障下，取消和物理停止的确认可能延后；超时不会冒充完成，也不替代现场急停。
- 原40段和8程模式仍保留；本节必须显式使用`--continuous-round`。

### 10.2 本轮代码位于独立worktree

按并行开发约定，本轮仅修改：
`/home/lxf/.codex/worktrees/parallel-dev/agx_arm_ws`，分支`codex/parallel-dev`。
原`/home/lxf/agx_arm_ws`业务代码未覆盖。下面直接运行**工作树的新程序**，只读核对主目录中已运行的T1。
`--arm-workspace`只确定T1进程记录的读取位置，不在主目录创建/启动节点。

本机已构建离线时间参数化工具。需要重建时在该worktree中执行：

```bash
cd /home/lxf/.codex/worktrees/parallel-dev/agx_arm_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install \
  --packages-select agx_arm_msgs tcp_trajectory_tools handeye_calibration_ros
```

该工具仅做数学计算，不创建ROS节点或访问硬件。

### 10.3 为什么速度参数不同

本次在当前姿态只规划核算时，整轮`speed=0.2`的时间参数化曾给出约27.34秒的最短时长，程序拒绝将其伪装成18秒。
新整轮专用分支允许显式`speed=0.5`，仍按真实URDF/MoveIt的速度、加速度限位及二次插值极值检查。
这是**上限缩放**，不是固定以50%速度运行；可行时统一拉长到18秒，做不到就拒绝，不缩小幅度。
旧模式上限仍为0.1。

程序采用已安装MoveIt的时间最优路径参数化。真正反向处独立求解并无停留拼接；
使用小关节路径偏差余量，输出位置+速度并对实际三次曲线再次检查。
最终路径所有位置、速度、加速度、碰撞、1mm模型定点门和±15°姿态包络均需通过。

Humble的GetCartesianPath服务会在返回前自行执行TOTG；大整轮路径可能已经发生圆滑偏移。
因此新整轮模式使用MoveIt的`/compute_ik`逐点求解（角度间隔≤0.5°、`avoid_collisions=true`），
全部成功且连续后统一计时，再核查实际曲线。不是通过关闭碰撞来提速。
相关上游实现见[MoveIt Humble笛卡尔服务源码](https://github.com/moveit/moveit2/blob/humble/moveit_ros/move_group/src/default_capabilities/cartesian_path_service_capability.cpp)。

### 10.4 干跑与只规划

```bash
# 干跑：无ROS节点、无设备、无运动；应为1条整轮目标、幅度15°
/usr/bin/python3 /home/lxf/.codex/worktrees/parallel-dev/agx_arm_ws/src/handeye_calibration_ros/handeye_calibration_ros/tcp_verification.py \
  --dry-run --continuous-round --motion-seconds 18 \
  --angle-deg 15 --steps 6 --axes x y --cycles 1 --speed 0.5
```

T1使用本次TCP、T5/T6关闭、工具点留隙对准后，只规划可运行下一节命令，
将`--mode execute`换为`--mode plan`并去掉`--point-aligned-with-gap`。规划失败不会发送运动。

### 10.5 现场执行完整命令

保持现有T1正常，关闭T5/T6及其他运动源，确认±15°扫过空间和参考点间隙。
先加载当前主部署的消息/ROS/DDS环境，然后直接调用工作树源码：

```bash
source /home/lxf/agx_arm_ws/scripts/runtime_env.sh
/usr/bin/python3 /home/lxf/.codex/worktrees/parallel-dev/agx_arm_ws/src/handeye_calibration_ros/handeye_calibration_ros/tcp_verification.py \
  --mode execute --tasks-stopped --point-aligned-with-gap \
  --continuous-round --motion-seconds 18 \
  --arm-workspace /home/lxf/agx_arm_ws \
  --candidate /home/lxf/agx_arm_ws/data/calibration/tcp/2026-10-16_22-42-05_c3296802/candidate.json \
  --allow-unqualified --waiver-reason '本次粗标定仅用于15度整轮18秒现场验证' \
  --angle-deg 15 --steps 6 --axes x y --cycles 1 --speed 0.5 \
  --output /home/lxf/.codex/worktrees/parallel-dev/agx_arm_ws/data/calibration/tcp_verification/round18_onsite_report.json
```

全部路径检查完成后，核对日志为1条整轮、±15°、18秒，再在现场输入`EXECUTE TCP`。
Ctrl+C只取消本工具的goal，不追加回零。程序不会自动启动T1、使能、激活CAN或退出示教。

### 10.6 报告中的两种内部参考

为了从当前真实关节姿态原地开始，整轮规划的模型参考来自新鲜真实关节反馈的FK；
SDK真实TCP起点另外保存为`feedback_reference_pose`。模型定点和幅度门使用模型参考，
实际SDK反馈的跟踪偏差使用SDK参考，不把二者混作同一个读数。preflight仍检查模型/SDK一致性。
这些都属于内部证据；真实尖端最大漂移仍按第7.8节由外部观察记录。
原粗标定仍failed，仅验证豁免，不改阈值、不作为生产精度批准。

### 10.7 本次只规划结果与派发前核查

2026-10-17当前姿态只读核查：1条轨迹、905个计时点、18.000秒；
逐点IK全部成功，实际三次插值检查1809个状态。模型TCP位置偏差最大约0.048mm；
X轴实际模型姿态范围约[-15.000°, +15.000°]，Y轴约[-15.000°, +15.000°]，没有减小幅度。
规划及初始化约10.42秒，派发前停稳和一致性核查约0.32秒；未创建执行客户端、没有发送运动。
这些是本次姿态的规划证据，其他起点仍需重新规划，实机运动耗时和尖端漂移尚待现场验证。

若实际起点、全部非arm状态、场景几何/附着物/碰撞矩阵/固定TF及轨迹均未改变，派发前可复用
刚完成的路径证明，仍检查真实新鲜反馈、空闲、参数和停稳。任一项改变则完整复核，可能增加等待；
不会跳过碰撞或FK检查。仅更新关节的服务请求保留场景附着碰撞物。

本次交付和测试明细见[tcp_round18_2026-10-17.md](tcp_round18_2026-10-17.md)。


## 11. 本次15组TCP标定：按用户需求采用5/10mm粗标定标准

### 11.1 结果与含义

原始会话：`/home/lxf/agx_arm_ws/data/calibration/tcp/2026-10-17_20-42-56_0b43465c`。
本次全部15组参与最小二乘求解，没有删除第14组等高残差样本。
RMS表示这些姿态换算出的工具点对共同固定点的整体偏离程度；最大残差表示其中最差一次的偏离。
它们衡量采样与模型的一致性，不能直接等同于每次抓取的物理误差。

| 项目 | 结果 |
|---|---:|
| TCP X | 1.563113 mm |
| TCP Y | 0.075462 mm |
| TCP Z | 148.489633 mm |
| 保留RPY | [0, 0, 0] rad（本次未标定朝向） |
| 样本 / 秩 / 条件数 | 15 / 6 / 6.205581 |
| RMS / 最大单点残差 | 4.657449 / 9.016730 mm |
| 原2/5mm标准 | failed，历史结果保留 |
| 新5/10mm粗标定标准 | passed |

前12组的RMS/max约3.795/6.629mm，补至15组后约4.657/9.017mm；
样本增加并未降低拟合残差，第14组最大。此次按用户要求接受全部15组结果，不挑点优化报告。

新默认值已修改于本会话worktree的标定程序及验证程序；原主部署目录尚未整合，因此旧命令默认值可能仍是2/5mm。
样本数、秩、条件数、数据完整性和碰撞/运动限位检查保持不变。
新候选无需为这次5/10mm标准通过的结果再加`--allow-unqualified`；旧failed候选不自动改写。

### 11.2 文件与备份

本次复算和源码修改位于`/home/lxf/.codex/worktrees/parallel-dev/agx_arm_ws`，分支`codex/parallel-dev`。
原始会话和主目录旧candidate均保留。新结果文件：

```text
data/calibration/tcp/2026-10-17_20-42-56_0b43465c/
  frames.json               # 复制的原始15组采样
  candidate.json            # 新5/10mm标准下的TCP候选，applied=false
  solve_input_<UUID>.json   # 独立输入和实际采用的阈值
  calculation_report.json   # 新/旧标准对比、逐点误差、验证器读取结果
```

修改前备份：`.local_backups/tcp-quality-20261017-205528/before.tar`，170KiB，
包含原始会话及本轮修改前的标定/验证源码、测试和教程。

### 11.3 离线复算命令

无需ROS或硬件：

```bash
/usr/bin/python3 /home/lxf/.codex/worktrees/parallel-dev/agx_arm_ws/src/handeye_calibration_ros/handeye_calibration_ros/tcp_calibration.py \
  --input /home/lxf/.codex/worktrees/parallel-dev/agx_arm_ws/data/calibration/tcp/2026-10-17_20-42-56_0b43465c/frames.json \
  --output /home/lxf/.codex/worktrees/parallel-dev/agx_arm_ws/data/calibration/tcp/2026-10-17_20-42-56_0b43465c/candidate.json \
  --max-rms-mm 5 --max-residual-mm 10
```

离线`--max-rms-mm`/`--max-residual-mm`可覆盖输入快照内旧阈值；不指定时优先沿用快照的阈值，无阈值才采用当前默认值。
每次报告记录实际门限，真实误差不会因为放宽门限改变。

主目录原采集入口可通过显式ROS参数使用同样标准：

```bash
cd /home/lxf/agx_arm_ws
./scripts/run_component.sh calibrate-tcp --ros-args \
  -p max_rms_mm:=5.0 -p max_residual_mm:=10.0
```

### 11.4 将本次结果用于T1和整轮验证

本程序未写在线参数或让机械臂运动。后续由操作者按既有停止、CAN激活和激活脚本流程处理，
在T1终端使用本次完整参数启动（不能通过在线param set让SDK应用TCP）：

```bash
cd /home/lxf/agx_arm_ws
./scripts/run_component.sh arm -- \
  'tcp_offset:=[0.0015631133283368326,0.00007546221381904417,0.1484896328004612,0.0,0.0,0.0]'
```

启动T1可能使能或回零，需现场操作者确认后执行。运行第10节±15°、18秒整轮验证时，
必须调用工作树的新验证程序，并将candidate参数改为：

```text
--candidate /home/lxf/.codex/worktrees/parallel-dev/agx_arm_ws/data/calibration/tcp/2026-10-17_20-42-56_0b43465c/candidate.json
```

去掉旧`--allow-unqualified --waiver-reason ...`，其余运动参数和现场EXECUTE TCP确认保持第10节要求。
T1必须已使用本次新TCP；旧0偏移或上次14.575mm的X偏移均不匹配。

本次针对性检查：标定与验证测试合计93项通过；真实候选可由新版验证器读取为qualified=true，
quality_waiver=null，同时保留strict_quality_status=failed供历史参照。未执行实机运动。

本轮已将调试导入生成的`tcp_calibration.cpython-310.pyc`、`tcp_verification.cpython-310.pyc`和`__init__.cpython-310.pyc`移入Ubuntu回收站，共43157字节（约42.1KiB）。必要构建产物、备份和标定原始数据均保留。


## 12. 本轮已将15组TCP用于双相机抓取（2026-10-17）

本节为当前配置；前面第7节的10组failed/豁免示例是历史记录，不是当前活动TCP。
工作树`/home/lxf/.codex/worktrees/parallel-dev/agx_arm_ws`、分支`codex/parallel-dev`；主部署目录保留原流程。
用户已确认本次标定点是**两指尖之间的中点**。活动文件`config/calibration/tcp.json`明确使用第11节15组结果。

```bash
cd /home/lxf/.codex/worktrees/parallel-dev/agx_arm_ws
export AGX_ARM_WS="$PWD"
# 先按README原命令激活CAN并运行activate_robot.py，再在独立T1终端执行：
./scripts/run_component.sh arm
```

本次`arm`自动把活动六维TCP同时传给SDK和MoveIt；启动时仍有原来的使能/回零动作，先确认周围空间。
动作节点会核对SDK启动参数、在线TCP、MoveIt固定TCP变换及Dabai碰撞体；在线param set不能代替SDK启动应用。
需验证TCP固定点摆动时仍按第10节显式`verify-tcp --mode plan/execute`命令，候选用第11节新15组文件；不要误用旧10组结果。

抓取中指尖中点对准**估计物体三维几何中心**。视觉表面点、估计中心、网络前端点分别记录；不再添加64mm工具长度或类别XYZ经验补偿。
Dabai本次手眼parent为link6（采集时零TCP），所以`T_base_tcp_feedback × inverse(S) × T_link6_camera`；新TCP不会自动改变法兰外参。
完整双相机步骤、观察位、模型来源和回退见[双相机抓取教程](dual_camera_grasping.md)。
