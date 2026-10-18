# 多会话使用 Git worktree 开发

## 1. 本项目采用的方式

用户已确认后续多会话通过worktree隔离。主目录`/home/lxf/agx_arm_ws`保留当前部署、现场运行和待整合改动。
各开发会话分配独立目录及`codex/`分支。源码、配置和暂存区各自独立，任务完成后按提交整合。
Worktree可避免会话直接覆盖同一份工作文件；并行修改同一文件仍可能在合并时产生冲突。

本次从当前工作文件制作开发基线`codex/worktree-base-20261017`，纳入尚未跟踪的迁移和TCP最新代码，
排除Git已忽略的模型、虚拟环境和构建产物。使用临时索引生成本地快照，保留原main指针和原暂存区。
这是一份本地开发起点，不是发布或实机验收结论；未推送远程。

## 2. 新会话怎么开始

在Codex中为每个任务选择独立worktree，或向agent说明：

> 从codex/worktree-base-20261017创建本会话独立worktree，在自己的codex/分支开发。先读AGENTS.md及docs/parallel_development.md，保留主目录和其他会话的改动。

Agent应先检查是否已位于linked worktree；已有隔离就复用。创建优先使用Codex原生worktree工具，
显式指定上述ref，不使用旧main作为最新代码起点。工具返回路径后，所有操作使用新路径。
原生工具若创建detached HEAD，可在该新目录中创建独立`codex/<任务名>`分支。

两个任务示例：相机标定会话使用一个worktree/分支，GUI改进会话使用另一个worktree/分支。
只在各自目录启动编辑器和终端，不同时把它们指向主目录。普通本地模式的两个会话仍会共享主目录文件。

## 3. 权重与环境

模型大文件未进入Git，新树需要连接现有只读资产。以下在**新worktree目录**执行；目的路径已存在时先检查，
不要使用覆盖参数替换已有资产：

```bash
# 先 cd 到Codex返回的新worktree路径
mkdir -p models/yolo models/graspnet
ln -s /home/lxf/agx_arm_ws/models/yolo/fruits12.pt models/yolo/fruits12.pt
ln -s /home/lxf/agx_arm_ws/models/graspnet/checkpoint-rs.tar models/graspnet/checkpoint-rs.tar
```

权重链接属于已忽略文件，不提交；不要通过链接写入共享权重。若要训练或替换权重，在本树单独目录中保存并调整本树配置。
现有视觉解释器仍是`config/runtime.json`声明的绝对路径`/home/lxf/orange_dataset/.venv/bin/python3`，
GraspNet及IK的外部路径也沿用该配置。把它们作为只读依赖使用，避免某个会话升级包影响其他会话。
需要不同依赖版本时，为该会话创建独立环境，并仅修改本树的解释器配置。
原`.venv_ik`目录保留在主工作空间，不复制进Git或新worktree。

## 4. ROS构建与运行

`build/`、`install/`、`log/`、`.runtime/`各树独立；**不要软链接主目录的这些目录**。
主install中的symlink可能指回主源码，共享会导致运行的代码与当前worktree不一致。

按任务构建所需包，例如仅需标定包时：

```bash
# 当前目录为本会话worktree
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select handeye_calibration_ros
source install/setup.bash
```

需要驱动和MoveIt时应在自己的树构建对应包及依赖；不要未经安排启动实机。
切换worktree终端时清除主目录遗留的`AGX_ARM_WS`，或显式设为当前目录：

```bash
export AGX_ARM_WS="$PWD"
```

共享启动脚本读取该变量；若它仍指向主目录，可能误运行主目录代码。
新树默认没有构建产物；离线Python开发不需要为了满足目录存在而复制生产install。
被忽略的`src/trac_ik`是外部嵌套仓库，只有任务确实需要它时才另行准备，避免共享修改其源码。

## 5. 基线检查

技能要求创建后确认基线；本项目可在各worktree执行：

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest -p no:anyio -p no:cacheprovider tests -q
```

`-p no:anyio`沿用本机已有插件兼容处理，不能仅为这个参数修改共享环境。
如基线失败，报告具体失败并查清后再开始业务修改。首次配置不自动pip install或改变共享环境。

## 6. 多会话约定与硬件

- 会话只修改自己的目录和分支；各自提交时先看`git diff`及`git diff --cached`。
- 分配清楚任务边界，尽量让两个会话不同时重写同一模块；合并冲突在整合阶段解决。
- `.git`公共对象和分支引用由Git管理；worktree暂存区独立，但stash、全局配置等仍可能共享，避免跨会话使用共享stash转移改动。
- 独立目录不隔离CAN口、相机、ROS2同名节点和真实机械臂。实机验证同一时刻只由一个经用户确认的会话执行。
- 仿真或独立ROS图可由每个终端显式选择不同ROS_DOMAIN_ID；这不会隔离物理CAN或授权同时控制机器人。
- 标定候选、新日志及采集数据保存到本树目录；生产标定配置在审查后统一应用，避免不同会话轮流覆盖活动参数。

## 7. 整合与清理

会话完成后交付worktree路径、分支、提交、修改范围及检查结果。把相关提交按顺序整合到用户确认的集成分支，
检查冲突与构建后，再安排更新主部署目录。主目录当前有历史暂存和未提交改动，整合前先处理这些边界，
不能直接用reset覆盖，也不能将工作树整目录覆盖回主目录。

后续共同开发起点更新时，更新AGENTS.md所引用的集成ref；已经开始的任务按需合并该集成分支。
不要强制移动其他会话正在使用的分支。
清理优先使用Codex归档worktree工具，确认需要保留的忽略文件已单独保存；不要直接rm -rf工作树。
