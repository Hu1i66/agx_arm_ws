# 双相机与TCP抓取改造交付记录

实施时间：2026-10-17～2026-10-18。状态：**实现与软件检查完成，实机抓取验收待现场执行**。

## 1. 工作边界和备份

- 工作树：`/home/lxf/.codex/worktrees/parallel-dev/agx_arm_ws`
- 分支：`codex/parallel-dev`；本轮变更保留在工作文件中，尚未合并/覆盖主部署目录。
- 主部署目录`/home/lxf/agx_arm_ws`原有源码、索引和生产工作流未由本轮整目录覆盖。
- 开工备份：`.local_backups/dual-camera-20261017-231534/before.tar`，757760字节。
- 扩展MoveIt builder前另备份：同目录`moveit_builder_before.tar`。
- 本机`/home`在独立数据盘；新增代码、备份、依赖副本、测试临时文件和参考资料均在本树。
- 沿用现有ROS/Python/视觉虚拟环境和权重，没有pip升级、系统安装、CAN激活、驱动使能或实机运动。

## 2. 已采用的结果

| 项目 | 采用值/来源 | 状态 |
|---|---|---|
| TCP | 15组`2026-10-17_20-42-56_0b43465c`，XYZ=[1.563113,0.075462,148.489633]mm，RPY保持0 | 用户接受5/10mm标准，活动配置显式选定 |
| D455 | 21:32:01接受版，parent=base_link | 沿用用户已现场核查结果 |
| Dabai | 22:53:28，parent=link6，22组 | 按用户要求启用；现场抓取核查未完成 |

直接核对活动JSON与所选来源的position/orientation/TCP数组一致，没有把结果手工修正成另一套参数。
旧10组failed候选没有被当作本轮生产TCP；原始实验数据保留。

## 3. 主要修改

| 文件 | 本轮作用 |
|---|---|
| `runtime_geometry.py` | 完整刚体变换/TCP检查、RGB-D中心估计、Dabai近似碰撞模型与视觉命令鲜度检查 |
| `camera_geometry.py` | 图像时刻TF、必要时TCP反馈插值并去偏移一次、兼容相机发布QoS |
| `config/calibration/tcp.json`、Dabai活动外参与manifest | 明确采用本次结果与原坐标父节点，保留审计 |
| `runtime_launcher.py`、相机TF launch | T1注入同一TCP，双相机入口/TF，Dabai驱动publish_tf=false，检测就绪检查相机来源 |
| 两个`vision/`节点 | 输出物体三维几何中心、单列表面点、移除人工XYZ/深度纠偏，无输入/TF时发布失效目标 |
| `graspnet_service_node.py` | 一套模型/双路缓存，按相机+请求ID路由，按track ID在本次RGB-D快照重估中心，网络提供姿态/宽度 |
| `auto_sorting_action.py` | MoveIt显式tcp_link，Pinocchio用实际TCP检查目标；普通/蓝方块/两阶段/动态分支统一中心，无旧工具高度/类别Z加法 |
| `sorting_gui_client.py` | 相机选择、Dabai独立启动、观察位、切换占用保护、队列与旧回包隔离、过期目标拒绝、旧法兰记录点要求重采 |
| MoveIt `_moveit_config_builder.py`、`config/robot/dabai_mount.json` | 由本次手眼定位相机和支架近似包围体，纳入规划碰撞 |
| README、`docs/calibration.md`、`docs/tcp_calibration.md` | 更新当前TCP、双相机、零人工偏置和回退说明 |
| `docs/dual_camera_grasping.md` | 本轮完整可执行教程和现场验收表 |

按用户最后补充，最终XYZ采用物体几何中心，不能把可见表面点当成中心。
中心由周边实测支撑平面、物体点云水平范围及高度估计，不恢复历史类别补偿；单视角没有测量质量分布，指标不冒充真实质量质心。
Dabai相机/支架外形按用户授权估算，非实测CAD，具体来源与尺寸假设在教程第9节。

## 4. 软件验证证据

### 4.1 针对性自动化测试

本轮最终相关集合：**138 passed**（6.23秒；重复运行时间略有不同），不是声称全仓库所有测试通过。

```bash
TMPDIR=$PWD/.runtime PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest \
  -p no:anyio -p no:cacheprovider \
  tests/test_dual_camera_geometry.py tests/test_dual_camera_pipeline.py \
  tests/test_dual_camera_ik.py tests/test_runtime_launcher.py \
  tests/test_gui_runtime_lifecycle.py tests/test_gui_review_regressions.py \
  tests/test_runtime_review_regressions.py tests/test_vision_offsets.py -q --tb=short
```

覆盖：非零XYZ/RPY完整变换、Dabai法兰外参兼容新TCP、GraspNet depth格式、中心与表面区分、相机/支架模型、
新鲜同帧快照、错误/过期/混尺寸快照拒绝、请求ID/相机匹配、零偏置规则、GUI与动作队列过期检查、
按正确track选择物体、Dabai独立GUI启动、原GUI生命周期回归。
`test_dual_camera_ik.py`使用真实Pinocchio模型，检查活动TCP和另一套非零XYZ/RPY的FK/IK往返；没有创建ROS节点或动作客户端。

`-p no:anyio`沿用本机已有插件兼容处理，未修改共享Python环境。

### 4.2 ROS构建

```bash
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install \
  --packages-up-to agx_arm_ctrl agx_arm_moveit handeye_calibration_ros trac_ik_kinematics_plugin
```

最终输出：**8 packages finished**。包括控制/消息/描述/MoveIt、标定、已有TCP轨迹工具，以及TRAC-IK库和插件。
TRAC-IK源码使用主空间现有版本的独立副本，存`src/trac_ik/`（既有忽略规则），没有链接或改写主空间build/install。
第一次编译插件出现上游tf2_kdl.h弃用头文件warning；构建成功，未为此修改外部依赖。

### 4.3 入口、依赖和模型

- 两相机×7个运行组件共14组真实CLI `--dry-run`均退出0，在`/home/lxf`作为cwd调用，未启动ROS节点。
- `gui --camera dabai --dry-run`明确传递`sorting_gui_client.py --camera dabai`；`--help`可直接读取，无Tk窗口/设备连接。
- D455与Dabai只读`preflight`均返回`ok=True, missing=[]`，只验证资源/依赖，不启动相机或机械臂。
- 用实际MoveItConfigsBuilder离线生成URDF/SRDF：TCP固定偏移为本次六维值，arm链端为tcp_link，含相机/支架两个collision box。
- 实际builder仍打印其既有“Cannot infer SRDF ... using config/agx_arm.srdf”提示；本轮明确加载xacro后生成的arm链端检查为tcp_link。
- 10个本轮生产Python文件AST解析、4个关键JSON解析通过；`git diff --check`无空白错误。

## 5. 独立审查与修复

按requesting-code-review技能进行独立只读审查，发现1条P1：视觉断流/TF失效后旧目标仍可能被派发。
修复及回归：

1. 视觉输入/TF失效发布无效目标；GUI派发前检查当前检测来源和时间。
2. 确认对话框后按原track取当前中心，不误用当前主物体；排队任务保留原时间戳。
3. 动作节点在ROS接收和实际出队两处拒绝过期视觉目标。
4. GraspNet按所选track取得接近本次图像时间的bbox，并使用自己的冻结RGB-D/T_B_C重新求中心。

该问题的对应测试先失败，再修复通过，最终相关集合138项通过。审查过程没有硬件动作。

## 6. 尚待现场完成

- 两路真实RGB-D对齐、光学帧命名、相机/机器人时间戳一致性、Dabai图像时刻TF可用性。
- 近似相机与支架碰撞体在RViz与实物的对照，线缆活动空间；Dabai观察位的真实视野。
- 物体中心与外部量具/实物核查；单次两料框、D455自动/动态、相机切换与GUI重开回归。
- 直接关节轨迹新增碰撞检查的实际耗时、动态预测时序和新TCP下原抓取动作的实机效果。

Dabai首轮仅停带静态单次，未宣称移动中抓取已验收。内部FK/IK一致与软件通过不能替代物理中心和成功率测量。

## 7. 清理

已通过`gio trash`将本轮pytest临时目录`.runtime/pytest-of-lxf`（磁盘占用1.3M，约1.3MB）移入Ubuntu回收站，返回0，原路径已不存在；未清空回收站。
必要的build/install、日志、源依赖副本、权重链接、标定数据、备份及官方参考手册保留。


## 2026-10-18 主目录整合补记

用户在主目录运行Dabai TF时仍遇到旧D455限制，已按用户要求将双相机抓取相关源码/配置按文件整合到`/home/lxf/agx_arm_ws`。
主目录修改前备份`.local_backups/dabai-enable-20261018-001451/before.tar`。主目录README与双相机教程已更新，不再要求用户切换工作树运行。
本段更新后的状态优先于本文早先“未合并/覆盖主目录”的历史交付描述；其他不相关历史变更不在本次整合范围。
