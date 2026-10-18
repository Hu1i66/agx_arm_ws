# Pinocchio IK 集成指南

## 概述

本工作项添加了一个基于 Pinocchio + CasADi 的 IK（逆向运动学）求解器，内嵌在 `auto_sorting_action.py` 中，作为 MoveIt2 规划器的关节空间求解方案。

> ⚠️ **架构说明（2026-08 更新）**：独立节点 `pinocchio_ik_node.py` 已移除。当前唯一实现是 `auto_sorting_action.py` 中的内嵌副本 `PinocchioIKSolver`（第 78~246 行）。不再使用 ROS2 消息通信，直接在同一进程内调用。

## 实现位置

### `PinocchioIKSolver` 类（`auto_sorting_action.py` 第 78~246 行）

- 使用 Pinocchio 加载机器人 URDF（`test2.urdf`）
- 求解 6 个关节角（joint1~joint6），末端 frame = `link6`
- 不依赖 ROS2 消息，直接函数调用

**调用入口**（`auto_sorting_action.py` 第 116 行）：

```python
def get_ik_solution(
    target_pos,       # [x, y, z] 目标位置（米）
    target_quat,      # [qx, qy, qz, qw] 目标姿态四元数
    initial_guess,    # 6 维关节角初始猜测（当前关节状态）
    max_iter=500,
    tol=1e-4
) -> (success: bool, q_sol: np.ndarray, error: float, time: float)
```

## 工作流程

`auto_sorting_action.py` 的分拣步骤中，机械臂移动优先使用内嵌 IK：

1. **IK 求解**：给定抓取/放置位姿，`PinocchioIKSolver` 求解 6 关节角
2. **成功** → 关节空间目标直接交给 MoveIt2 规划执行（`move_arm_joint`）
3. **失败** → 回退到姿态约束 + MoveIt2 规划（`move_arm_cartesian`）

IK 与 MoveIt2 的分工：

- **Pinocchio IK**：计算"末端到目标位姿时 6 个关节各转多少度"（静态解）
- **MoveIt2**：从当前关节角规划到目标关节角的时序轨迹（避障、限速、平滑）

## IK 算法说明

`get_ik_solution` 采用**随机采样 + 随机游走**策略（非 CasADi IPOPT）：

| 阶段 | 说明 |
|------|------|
| Phase 1 随机采样 | 500 次，70% 在初始猜测附近高斯采样（σ 0.15→0.30 rad），30% 全局均匀采样 |
| Phase 2 随机游走 | 6000 次，σ 随当前误差自适应（`max(0.003, err×6.0)`） |
| 提前终止 | `pe<0.005 且 ze<0.15 且 yaw_err<0.25` 时提前退出 |

**误差函数** `_combined_error`（第 141~169 行）：

```
total = 位置误差 + 0.06×Z轴夹角误差 + 0.05×偏航误差 + 0.01×joint6 偏差
```

- 位置误差 `pe`：末端平移与目标的欧氏距离
- Z 轴夹角 `ze`：末端 Z 轴与目标 Z 轴的夹角（0=对齐，π=相反）
- 偏航误差 `yaw_err`：末端 X 轴与目标 X 轴的夹角（取绝对值）
- joint6 偏差：只惩罚腕关节，避免冗余关节翻转 90° 的解

## 安全约束

IK 成功判定（第 237 行）：

```python
success = bool(final_pe < 0.015 and final_ze < 0.52)
```

- **位置误差 < 15mm**（`final_pe < 0.015`）
- **Z 轴方向误差 < 30°**（`final_ze < 0.52 rad`）

> ⚠️ Z 轴约束的意义：当物体超出工作空间时，只有末端朝上的构型可达（位置准但 Z 轴朝上）。若只判位置会接受该解 → 机械臂朝上抓（危险）。加 `final_ze` 约束强制拒绝朝上解，物体超出工作空间时 IK 返回失败而非危险解。

## 辅助方法

### `get_frame_position(q_joints, frame_name)`（第 248~273 行）

用 FK 计算指定 frame（如 `link6`, `gripper_link1`）在 base 坐标系中的位置，用于料框坐标计算和抓取验证。

## 故障排除

### 问题：启动报错 "Pinocchio not available"

```bash
pip install pinocchio casadi numpy
```

### 问题：IK 求解超时 / 无解

- 检查 `initial_guess` 是否传入当前关节状态（好的初值显著提高成功率）
- 增加采样次数（第 186 行 `n_samples`）或随机游走次数（第 214 行 `n_walk`）
- 验证 URDF 路径正确（`test2.urdf`）
- 确认目标位置在工作空间内（超出工作空间时 IK 返回失败是安全行为）

### 问题：IK 精度不足 (error > 0.015)

- 检查目标四元数是否归一化
- 使用更好的初值猜测（当前关节状态）
- 调整成功阈值（第 237 行）——注意降低阈值会削弱安全约束

## 下一步改进

- [ ] 支持多个 IK 初值候选（例如，当前位置 + 几个预定义姿态）
- [ ] 实现更快的数值 IK（基于雅可比的增量式求解）
- [ ] 集成 MoveIt2 的 RobotState 用作更好的初值
- [ ] 添加速度/加速度约束的 IK 求解
- [ ] 性能监控和日志记录（各步骤耗时统计）

---

**最后更新**: 2026-08-11
