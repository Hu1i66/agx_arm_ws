# Grasp Cycle Optimization V2 — 途径点无停顿优化

**Date**: 2026-07-26  
**Status**: Draft  
**Scope**: `auto_sorting_action.py` only  
**Depends on**: `grasp-cycle-optimization` (V1, already implemented)

---

## Why

V1 优化将周期从 ~17s 降到 ~14.3s，但用户反馈：
1. 夹爪闭合后等待仍然偏长
2. PICK_UP（夹取位上方）和 BIN_ABOVE（料框上方）是两个安全途径点，机械臂不需要在此停留——按顺序途径即可，无需每步之间停止

## What Changes

- 夹取位上方（PICK_UP）和料框上方（BIN_ABOVE）改为途径点：相邻步骤间从 `continuous=False` 改为 `continuous=True`，消除 0.15s 的步骤间稳定等待
- 夹爪闭合后的 `GRIPPER_SETTLE_SEC` 从 0.15s 降到 0.05s
- 夹爪闭合后的 `planning_scene` 更新等待从 0.15s 降到 0.05s
- 夹爪释放后的 `GRIPPER_SETTLE_SEC` 随之上限（共用同一个常量）
- step4 抬起改为直接去 BIN_ABOVE，跳过 PICK_UP 中间停留

## Impact

- Affected specs: grasp-cycle-optimization
- Affected code: `auto_sorting_action.py` sort 序列中 6 处 `continuous=` 参数 + 2 处 sleep 常量

---

## ADDED Requirements

### Requirement: PICK_UP 和 BIN_ABOVE 作为途径点
相邻的关节空间移动步骤 SHALL 使用 `continuous=True`，使机械臂在步骤间不停止（仅 0.05s 极短稳定），PICK_UP 和 BIN_ABOVE 作为轨迹途径点而非停留点。

#### Scenario: step1→step2 途径 PICK_UP
- **GIVEN** 机械臂在待机位
- **WHEN** 执行 step1（移动到 PICK_UP）使用 `continuous=True`
- **THEN** step1 完成后仅等 0.05s，立即开始 step2（下降抓取）的 IK 求解和规划

#### Scenario: step4 抬起后直接去 BIN_ABOVE
- **GIVEN** 夹爪在 PICK 位置闭合完成
- **WHEN** 执行 step4 抬起
- **THEN** 抬起目标改为 BIN_ABOVE_JOINTS（关节空间），直接从 PICK → BIN_ABOVE，绕过 PICK_UP 中间停留
- **AND** 使用 `continuous=True`

#### Scenario: step5→step6 途径 BIN_ABOVE
- **GIVEN** 机械臂到达 BIN_ABOVE
- **WHEN** step5 使用 `continuous=True`
- **THEN** step5 完成后仅等 0.05s，立即执行 step6（下降到放置位）

#### Scenario: step8→step9 途径 BIN_ABOVE 回待机位
- **GIVEN** 夹爪已在放置位释放
- **WHEN** step8（BIN_ABOVE）和 step9（STANDBY）均使用 `continuous=True`
- **THEN** 两步间仅等 0.05s，机械臂几乎不停顿地回到待机位

### Requirement: 夹爪闭合后等待进一步缩短
夹爪闭合后的稳定等待 SHALL 从 0.15s 降到 0.05s，planning_scene 更新等待从 0.15s 降到 0.05s。

#### Scenario: 闭合夹爪后快速转入抬起
- **GIVEN** step3 夹爪闭合完成
- **WHEN** `GRIPPER_SETTLE_SEC = 0.05`
- **THEN** 仅等 0.05s 夹爪稳定 + 0.05s planning_scene 更新，总计 0.10s 后进入 step4

---

## MODIFIED Requirements

### Requirement: GRIPPER_SETTLE_SEC
**V1 值**: 0.15s  
**V2 值**: 0.05s  
夹爪物理响应 < 0.1s，闭合/释放后 0.05s 足以稳定。

---

## 不改动的内容

- 重试流程中的步骤（保留 `continuous=False`，重试需要确定性）
- step2 下降抓取（已是 `continuous=True`）
- step4 抬起脱离的原有逻辑（保持 `continuous=True`）
- 所有 IK、规划、碰撞检测逻辑
- `check_grasp_success` timeout（V1 已优化到 0.3s）
- 周期结束 0.5s sleep

---

## 预期效果

| 指标 | V1 优化后 | V2 优化后 | 变化 |
|------|-----------|-----------|------|
| 单周期时间 | ~14.3s | ~12.5s | -13% |
| 步骤间稳定等待 | 4 × 0.15s | 0s (换为 0.05s) | -0.4s |
| 夹爪闭合后等待 | 0.15+0.15s | 0.05+0.05s | -0.2s |
| step4 跳过 PICK_UP 直接去 BIN_ABOVE | — | 省一次移动 + 等待 | -1.2s |
