# Piper 双相机抓取与统一 TCP 改造设计

日期：2026-10-17。历史设计；本轮已实施于独立工作树，具体状态见dual_camera_implementation_progress.md。未执行实机运动。

实施中用户追加要求：最终XYZ使用物体三维几何中心，GraspNet只提供朝向/宽度并用同帧RGB-D重估中心；用户授权按公开尺寸和手眼位置近似建立Dabai碰撞体。后续教程优先于本文旧的网络指尖目标描述。

## 1. 目标与已确认约定

用户要求：使用最新TCP、D455与Dabai各自的手眼结果；移除视觉和控制中为旧误差添加的手动空间偏置；验证两路相机能否分别抓取。
用户已明确：TCP固定点标定针对**两指尖之间的中点**，不是单侧手指尖或已拆除的针尖。

实施基线：现有工作树`/home/lxf/.codex/worktrees/parallel-dev/agx_arm_ws`，分支`codex/parallel-dev`。
主部署目录`/home/lxf/agx_arm_ws`有历史未提交/暂存改动，实施中保留；验收后只部署本次相关变更。
本设计采用现有ROS2 Humble、tf2、MoveIt、Pinocchio、YOLO/GraspNet和现有Python环境；不引入另一套机器人控制框架。

目标的实际含义：

- 标定负责“相机在哪里”“两指尖中点相对法兰在哪里”；物体点云/GraspNet仍负责“夹哪里、从什么方向接近”。
- 删除的是误差补偿和旧工具长度的近似换算；接近/抬起距离、碰撞模型、关节限位、夹爪开口控制是正常抓取策略，不应一并清零。
- 先完成两路相机各自的静态抓取，再回归D455自动/动态分拣。双相机融合和Dabai运动中追踪不作为首轮前提。
- 允许两路相机同时采集/显示，每次抓取只选择一个`active_camera`，共享一套动作执行器与一套GraspNet模型。

## 2. 已核实的标定资产

以下路径相对于本工作树，除非另有说明。

| 资产 | 使用的文件 | 当前事实 |
|---|---|---|
| 最新TCP | `data/calibration/tcp/2026-10-17_20-42-56_0b43465c/candidate.json` | 本树按5/10mm粗标定门passed；主部署同次旧候选仍failed/null，不能误读它 |
| D455手眼 | `data/calibration/d455/handeye/2026-10-17_21-32-01_calibration.json`及其`_accepted.json` | 已按用户现场接受，写入两树活动配置 |
| Dabai手眼 | `data/calibration/dabai/handeye/2026-10-17_22-53-28_calibration.json` | 22组，当前新标准passed，尚需明确推广为活动Dabai资产 |

### 2.1 TCP

```text
tcp_offset (m, rad):
[0.0015631133283368326, 0.00007546221381904417, 0.1484896328004612, 0, 0, 0]
```

15组，RMS=4.657449mm，max=9.016730mm。当前工具RPY沿用0，不是重新标定出的工具朝向。
启用必须同时进入SDK启动参数和MoveIt模型；驱动仅在T1启动时应用offset，不能用在线param set替代重启。
“最新”在本次实施中指上面的确定文件；不得每次按文件mtime自动选新结果，以免实验文件变动影响生产。

### 2.2 D455（眼在手外）

```text
parent: base_link
child: camera_color_optical_frame
position (m): [0.6163013695268662, -0.09175253586745957, 0.6072941375905626]
quaternion xyzw: [-0.6936988665644966, -0.7054387399658206, 0.12251711991558956, 0.07827912882855059]
```

相机固定，变换是常量。留出中位11.7811mm、max15.3025mm，已按用户需求接受，不改写原误差。

### 2.3 Dabai（眼在手上）

```text
parent: link6
child: dabai_color_optical_frame
position (m): [-0.06692139388230045, 0.003969724591065465, 0.046240533934709345]
quaternion xyzw: [-0.11836555672201625, 0.12249753614843861, -0.7020197435076555, 0.6914855228744087]
```

留出中位3.574122mm，P95=3.912608mm，max=3.941412mm；相对转角中位26.0015°。
**该次采集的tcp_offset为全零，robot_pose_frame和transform_parent都是link6。**
因此结果实际是法兰到相机，不能因为mode是eye_in_hand就强行解释为“启用新TCP后的tcp_link到相机”。
保留此原始参考系即可使用，不需要仅因启用TCP就重新采集手眼。

## 3. 统一坐标关系（最重要的实现约束）

定义`T_A_B`为“把B坐标表达转换为A坐标表达”的4×4刚体矩阵，含旋转和平移。
`B=base_link`，`F=link6`，`T=tcp_link`，`C=相机光学系`，`S=T_F_T`为最新TCP结果。

### 3.1 相机位置

D455：

```text
T_B_Cd455 = 活动D455手眼常量
```

Dabai：

```text
T_B_Cdabai(t_image) = T_B_F(t_image) × T_F_Cdabai
```

优先复用tf2按图像时刻查询base_link→link6；也可以从实际TCP反馈恢复法兰：

```text
T_B_F(t) = T_B_T_feedback(t) × inverse(S)
T_B_Cdabai(t) = T_B_T_feedback(t) × inverse(S) × T_F_Cdabai
```

`/feedback/tcp_pose`已经包含运行中T1的TCP偏移，不能把该反馈直接乘本次link6→相机结果。
如果必须生成TCP→相机表示，则`T_T_C=inverse(S)×T_F_C`，同时更正元数据；推荐保留原法兰表示，减少转换副本。
不得仅改JSON的parent字符串或只减世界坐标XYZ。

### 3.2 时间配对

Dabai随机械臂运动，曝光后到推理结束这段时间机械臂可能已改变姿态。因此：

- 图像、对齐深度、CameraInfo及图像时刻的法兰位姿一起构成快照，冻结本帧`T_B_C`。
- 所有点云变换、PCA方向、GraspNet结果都使用这份快照；不能用推理结束时最新的机械臂姿态。
- 复用tf2 Buffer/TransformListener，不另写一套TF系统。已有驱动发布了关节和TCP反馈时间戳；实施时检查相机与机器人戳是否同一ROS时域。
- 若时间戳不在同一时域或找不到历史变换，运动中检测拒绝用于抓取，不能静默取“latest”充当图像时刻。
- 首轮可采用显式的“停稳快照”模式：机械臂、物体/传送带停止，检查新鲜图像和停稳反馈窗口后冻结几何关系。该模式不宣称支持移动中视觉定位。
- 沿用现有消息新鲜度/同步设置，先测量实际时差再决定是否调整，不额外堆砌门禁。

### 3.3 最终抓取目标是两指尖中点

统一内部与请求协议中的抓取目标为`T_B_T_target`。

- MoveIt约束、IK、笛卡尔路径显式以`tcp_link`为目标时直接使用该矩阵，不再手工添加TCP。
- 现有Pinocchio仍以link6为末端，可以保留这个成熟求解器，在调用入口统一换算：

```text
T_B_F_target = T_B_T_target × inverse(S)
```

- ROS IK若继续使用法兰目标，必须显式设置`ik_link_name=link6`并传上述法兰目标；不能依赖group默认末端。
- 禁止把同一个未经标注的pose_dict同时交给tcp_link和link6接口。
- 配置SDK、MoveIt及转换器使用同一S；已有`test2.urdf`的Pinocchio模型/限位须与当前机械臂一致，消除写死主目录URDF的路径。
- 已记录的关节待机/放置姿态不因TCP参数改变而需要数值平移；旧笛卡尔记录则先确认它记录的是法兰还是TCP，再作完整转换。

### 3.4 GraspNet的translation不是指尖位置

本机已安装的graspnetAPI可视化中，指尖位于抓取系X方向的depth处。不能只保留旧translation并删除64mm加法。

```text
p_C_tip = translation_C + depth × R_C_G[:, 0]
p_B_tip = translation(T_B_C) + rotation(T_B_C) × p_C_tip
```

这是GraspNet预测的插入深度，不是额外手眼或TCP偏置。服务响应必须保留depth及明确的目标点语义。
旋转保留现有物理轴约定，并显式换算到TCP：

```text
R_B_F = R_B_C × [R_C_G[:,1], R_C_G[:,2], R_C_G[:,0]]
R_B_T = R_B_F × R_F_T
T_B_T_target = [R_B_T, p_B_tip]
```

当前R_F_T为单位矩阵；仍需覆盖非零RPY情况，避免以后再次出现只补XYZ的错误。
实施中用当前graspnetAPI的网格/可视化与Piper夹爪轴向核验该固定轴映射；保留从原始网络输出到TCP目标的可追踪数据。
首次静态双相机测试优先使用YOLO选物体+GraspNet给抓取姿态。YOLO像素中心反投影得到的是可见表面点，不能直接把它当成完整6D抓取姿态。

来源：本机`orange_dataset/.venv/lib/python3.10/site-packages/graspnetAPI/utils/utils.py`的`plot_gripper_pro_max`；
[GraspNet作者的API格式说明](https://github.com/graspnet/anygrasp_sdk/blob/main/grasp_detection/USAGE.md#predict-grasps)明确区分translation和指尖位置；
[graspnetAPI评估实现](https://github.com/graspnet/graspnetAPI/blob/master/graspnetAPI/utils/eval_utils.py)也使用旋转后的depth恢复抓取接触位置。
不替换成AnyGrasp模型或引入SDK授权依赖，只复用现有GraspNet模型和格式约定。

## 4. 手动偏置与正常抓取策略的处理清单

| 现有项 | 处理 |
|---|---|
| YOLO `d455.x/y/z_offset_m=-0.035/-0.020/+0.030` | 新抓取模式全设0，源码默认与fruit.yaml一致 |
| 两路`depth_offset_m` | 全设0；无有效深度时不退到人工类别尺寸补偿来执行抓取 |
| 控制`PICK_X_OFFSET/PICK_Y_OFFSET` | 删除活动计算分支，结果不叠加常量 |
| `GRIPPER_PICK_Z_OFFSET=0.064`及另一蓝方块分支0.10 | 由完整S换算替代，禁止继续加高度 |
| `PER_OBJECT_Z_CORRECTION` | 移除柠檬+25mm、青苹果-15mm、草莓+10mm等经验项 |
| `DYNAMIC_GRASP_Z_LIFT=0.005` | 移除经验抬升，动态分支统一用TCP目标 |
| 蓝方块`0.1358/0.1398-height`、50mm偏置下限 | 不再作为工具长度/抓取坐标修正；工具与台面关系按模型和标定检查 |
| 视觉“表面减预设半高、最高25mm”等旧推测 | 不进入新GraspNet目标路径；可保留显示字段，但不能暗中驱动运动 |
| 历史手动时延补偿`DESC_LEAD_S`等 | 静态模式不使用；动态回归时区分真实运动预测与经验常量，使用时间戳/实测延迟，另行记录 |
| GraspNet depth、方向轴映射 | 保留，是模型输出的几何含义，显式转换成指尖TCP目标 |
| 预抓取/抬起高度、夹爪宽度策略 | 保留，名称和日志注明“路径/开口策略”，不改写最终标定目标 |
| 台面防撞、关节限位、可达性与反馈检查 | 保留；目标危险时拒绝或重新规划，不用`max(z,某值)`静默改成另一个最终抓取点 |

视觉中既有坐标补偿也有测量/滤波/算法常数，不能全局搜索offset或数字后机械清零。
安全检查依赖真实桌面和夹爪几何；移除旧Z补偿前必须完成TCP换算，不能让原link6目标直接等于物体点。

## 5. 双相机接入方式

### 5.1 首轮推荐：双路采集、单路抓取

GUI增加“抓取相机：D455 / Dabai”。切换时：

1. 结束或取消当前任务，确认动作空闲；busy期间保持既有禁止切换逻辑。
2. 停止旧检测派发，清除旧图像配对、track、选中目标、队列与GraspNet待回包状态。
3. 切换图像/深度/CameraInfo来源及对应外参，等待新相机的第一份完整快照。
4. 确认就绪后再提交active_camera，显示来源和当前TCP文件，允许单次抓取。

一套YOLO模型按当前相机推理即可，不需要为了两个相机分别复制权重或常驻两个GPU模型。
保留既有D455话题和Dabai命名空间；GUI/动作的订阅或映射必须跟随active_camera，不能只换显示画面。
复用现有mode切换ACK/epoch机制；每个抓取请求与结果带上`camera_id/request_id/图像戳/目标frame`，晚到的旧相机结果不得执行。

### 5.2 Dabai TF

发布且仅发布标定确定的`link6 → dabai_color_optical_frame`静态边，基座到相机随机器人TF自动变化。
**不得发布固定的base_link→Dabai光学系。**
本机Orbbec`dabai.launch.py`确实提供`publish_tf`参数，默认true；实施时改为`publish_tf:=false`并由项目统一发布所需标定TF，避免光学帧重复父节点。
保留对齐深度到彩色的驱动设置，确认实际Image/CameraInfo.frame_id与`dabai_color_optical_frame`一致；如驱动名字不同，明确frame映射，不能给同一物理帧造两套冲突TF。
D455继续用`publish_tf:=false`与其原base→光学静态TF。
手眼验证TF节点与生产TF节点不同时发布相同父子边。
Dabai相机和安装支架随腕运动，需把真实外形或保守包围盒纳入现有MoveIt碰撞模型；尺寸来自实物/已有模型，不凭空填写，线缆活动范围也由操作者现场检查。

### 5.3 感知与服务

- 移除现有Dabai生产拒绝分支的前提是动态相机变换已接入并验证，不单独删除raise伪装支持。
- `graspnet_service_node.py`从固定D455话题/常量变换改为按请求相机选择快照；同一份RGB-D、内参、T_B_C贯穿推理。
- 结果标注目标frame=tcp_link，包含原始translation、depth、轴约定、最终TCP pose；动作端不重复做相机变换或深度位移。
- 物体点、PCA轴和抓取候选方向都要旋转到base，不能只转换质心位置。
- 采用Dabai观察位停稳→采样→计算基座目标→规划抓取。接近后视野变化不应令已冻结的基座目标跟着相机漂移。

## 6. 文件改造范围与完成条件

| 文件/区域 | 改造内容 | 完成条件 |
|---|---|---|
| `runtime_config.py`、`config/calibration/manifest.json` | 记录活动TCP，推广Dabai原link6外参，统一资产来源 | 两相机和TCP来源可打印；主目录旧failed TCP不能被误选 |
| `config/runtime.json`、`runtime_launcher.py`、`scripts/run_component.sh` | 扩展detector/graspnet/camera_tf的--camera，T1显式加载活动TCP，Orbbec TF控制 | 干跑分别展示两相机话题/TF/解释器；不再D455硬拒绝 |
| `launch/`中的相机TF入口 | 兼容D455 base静态与Dabai link6静态边 | TF没有同child重复父节点，没有静态base→Dabai |
| 规划新增轻量共享几何模块（如`runtime_geometry.py`） | 复用已有SE(3)函数、tf2；提供按时刻相机变换及TCP↔法兰转换 | YOLO、GraspNet和控制使用一致语义；不要复制三套矩阵常量 |
| `vision/realsense_yolo_node.py` | 动态Dabai变换、快照、零人工偏置、相机身份 | 两路相机输出基座点且来源明确；动态时态错误拒绝 |
| `graspnet_service_node.py` | 双路快照、depth指尖转换、明确TCP目标 | 不能混用D455图像和Dabai外参；旧请求不串包 |
| `auto_sorting_action.py` | 所有笛卡尔目标统一TCP；Pinocchio入口完整逆；移除旧位置修正 | sort/GraspNet/蓝方块/动态/过渡路径不再混用link6和tcp_link |
| `sorting_gui_client.py` | 相机选择、预览/检测/曝光路由、busy切换保护和清队列 | 显示相机等于本次动作来源，保留料框及停止功能 |
| `saved_poses.json`等历史笛卡尔资产 | 识别参考点语义，必要时一次性转换并备份 | 关节记录不误改；旧法兰pose不直接当TCP pose |
| README与标定文档 | 新T1参数、双相机运行流程、旧补偿移除清单、回退 | 操作者无需从不同历史说明拼凑命令 |

MoveIt位姿约束当前硬写link6，ROS IK未显式设ik_link_name，而SRDF链端为tcp_link；实施必须一起消除该混用。
现有ROS IK路径还有avoid_collisions=False，新的抓取执行路径应复用MoveIt碰撞场景，不能关闭碰撞以求跑通。
Pinocchio本身的IK成功不等于无碰撞，应在执行前经现有MoveIt状态/轨迹检查。

## 7. 实施顺序

### P0：先备份与冻结

备份本次将修改的源码、配置、saved_poses/conveyor_calib以及三个确定的标定结果，记录当前生产启动命令。
复用当前worktree，不重置主目录或其他会话分支；备份存/home分区项目内。
保留模型/现有虚拟环境只读；备份完成后再改程序。

### P1：活动资产和统一几何

1. 从本树accepted TCP生成明确活动TCP配置，不自动挑“最新文件”。
2. 用当前新版批准工具推广Dabai的passed候选，保留parent=link6；D455活动结果保持当前版本。
3. T1启动时同时给SDK与MoveIt应用S，启动日志明确XYZ/RPY和来源。
4. 完成共享SE(3)、图像时刻法兰变换，以及完整TCP↔法兰转换。

完成标志：给定同一已知法兰姿态，从TF和TCP反馈两种路径得到的Dabai相机矩阵一致。

### P2：D455先消除旧补偿

先完成TCP目标语义和控制入口换算，再把视觉/控制经验空间偏置归零或移除。
D455静态GraspNet先验证，再移植普通/蓝方块/动态分支；尚未迁移的按钮明确禁用，禁止悄悄回退旧偏置。
既有正常生产可通过备份/原分支回退，不在新模式里保留隐形补偿兜底。

完成标志：同一TCP目标通过MoveIt tcp_link与Pinocchio法兰转换得到一致末端位置；日志不再叠加64mm等旧项。

### P3：Dabai静态单次抓取

连接动态TF与RGB-D快照，配置观察位，加入GUI相机选择与请求身份，复用同一YOLO/GraspNet。
两相机都能独立得到完整base系TCP目标，再由操作者分别现场执行。

### P4：回归原工作流和部署

回归D455两料框、自动分拣、停止/重开；完成已纳入范围的旧分支迁移。
Dabai先保证静态物体和停止的传送带能抓取；Dabai随动动态抓取要在时间同步和移动相机验证后单独启用。
只整合本次相关文件，审核主目录现有未提交改动；不能整树复制覆盖。

## 8. 针对性验证与现场验收

### 软件检查（实施时执行，本次设计阶段没有运行测试）

- 非零XYZ和非零RPY的TCP完整逆变换；TCP/法兰往返一致，无世界坐标简单减法。
- Dabai旧标定offset=0配合新TCP：`T_B_T×S^-1×T_F_C`与`T_B_F×T_F_C`一致。
- 同一静止世界点，在机械臂不同姿态/图像时刻换算后仍为同一base点；使用错误“最新姿态”的反例应暴露偏差。
- GraspNet指尖depth沿接近方向转换、轴映射和RPY一次应用；缺depth/未知目标参考点拒绝新接口执行。
- 人工偏置为零覆盖默认、YAML、GUI和动作路径；只查活动路径，不误删正常规划距离或预训练算法常数。
- camera/request/切换序号不匹配或图像/变换过期时不派发；busy时切换不生效，停止后不派发后续轨迹。
- 两相机CLI干跑、TF唯一性、原D455工作流回归。只运行受影响测试，不把无关全套失败当成本轮改造前置。

### 现场检查

1. 保留用户习惯的CAN激活命令与activate_robot.py流程；操作者现场启动加载新TCP的T1。
2. 查看双相机图像/深度和TF，确认Dabai光学帧随手腕运动，D455保持固定。
3. 同一静止物体，分别从两相机观察；保存base点/姿态、图像时刻和法兰状态，比较差异，真实误差如实报告。
4. 单次D455：先只规划，再留隙到预抓取位，最后按现场确认执行；Dabai同样流程。
5. 每相机先做5次单次抓取（覆盖视野不同位置），再测两料框与相机切换；记录成功/失败原因，不新设用户未要求的精度硬门。
6. 回归D455自动分拣≥10轮；不以Dabai标定已完成为理由直接开启其动态追踪。
7. 内部SDK/FK误差与外部工具点/实物偏差分开记录；成功抓住物体也不代表全工作区达到某个毫米级精度。

## 9. 预期运行界面与命令

下列是**实现后的目标接口，目前不要直接照此启动实机**：

```bash
# 用户按既有CAN激活、activate_robot.py流程准备后，在现场启动T1
./scripts/run_component.sh arm
# 预期默认读取明确批准的active TCP；不会依赖在线param set

./scripts/run_component.sh camera --camera d455
./scripts/run_component.sh camera --camera dabai
./scripts/run_component.sh camera_tf --camera d455
./scripts/run_component.sh camera_tf --camera dabai
./scripts/run_component.sh detector --camera d455 --mode fruit
./scripts/run_component.sh graspnet --camera d455
./scripts/run_component.sh action
./scripts/run_component.sh gui
```

GUI选择Dabai时通过现有组件管理器完成相机/检测来源切换；GraspNet保持单实例，当前源参数可切换并受请求身份约束。
`graspnet --camera dabai`用于启动时选择Dabai，不允许两套无命名空间服务同时抢同一个/graspnet/req。
README最终必须给出D455单独、Dabai单独、双相机采集切换三套明确步骤和安全停止顺序。
所有生产资产必须存在于部署项目中，运行时不依赖Codex进程或临时worktree路径；工作树只用于隔离开发与交付。

## 10. 本次交付边界

本次交付为设计文件。未推广Dabai、未切换活动TCP、未清零生产偏置、未启动硬件或执行抓取。
本轮只读取已有源码/标定结果并新增本文，没有新增临时脚本、编译产物或实验数据；需清理临时文件0KB。
