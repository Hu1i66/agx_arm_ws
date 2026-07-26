# 机械臂状态 MQTT 广播 Spec

## Why
智能仓储分拣系统项目要求机械臂在每个分拣周期结束后，通过 MQTT 协议向 B 组后端上报动作状态。当前 auto_sorting_action.py 已有抓取流程及所需数据，但缺少对外通信能力。需要一个独立节点在分拣周期完成时收集动作状态并以标准 JSON 格式向 MQTT Broker 广播。

## What Changes
- 在 auto_sorting_action.py 中新增 `/sorting/robot_status` topic 发布者，在每个分拣周期结束时发布完整的 RobotPayload 数据
- 新建 `robot_status_bridge.py` 独立 ROS2 节点，订阅 `/sorting/robot_status`，将 JSON 通过 MQTT 转发到外部 Broker
- 新增 `config/robot_status_bridge.yaml` MQTT 参数配置文件
- 新建 `launch/robot_status_bridge.launch.py` 启动文件

## Impact
- Affected specs: 无（新功能）
- Affected code: `auto_sorting_action.py`（新增一个 publisher），新建 `robot_status_bridge.py` 及相关配置文件

## ADDED Requirements

### Requirement: 分拣周期状态上报
系统 SHALL 在每个分拣周期结束后通过 `/sorting/robot_status` topic 发布 RobotPayload 数据。

#### Scenario: 分拣成功
- **WHEN** 机械臂完成一次成功的抓取+放置周期
- **THEN** `/sorting/robot_status` 发布 JSON，包含：
  - `action` = `"sort_complete"`
  - `cargo_type` = 检测到的物体类别（如 `"lemon"`）
  - `confidence` = YOLO 检测置信度 (0-1)
  - `source_position` = 抓取坐标字符串 `"(x, y, z)"`（米，3位小数）
  - `target_position` = 目标料框坐标字符串 `"(x, y, z)"`（料框放置位）
  - `actual_position` = 实际放置坐标字符串（成功时与 target_position 一致）
  - `grasp_success` = `true`
  - `joint_angles` = 6 个关节角度（度）
  - `tcp_position` = 末端执行器坐标 `[x, y, z]`（mm）
  - `error_code` = `0`

#### Scenario: 分拣失败
- **WHEN** 机械臂分拣周期执行失败（抓取失败/放置失败/IK失败等）
- **THEN** `/sorting/robot_status` 发布 JSON，包含：
  - `action` = `"sort_failed"`
  - `grasp_success` = `false`
  - `actual_position` = `"NONE"`
  - `error_code` 为对应的分类错误码

### Requirement: MQTT 桥接转发
系统 SHALL 提供独立的 `robot_status_bridge` 节点，订阅 `/sorting/robot_status` 并将数据封装为 RobotData 格式通过 MQTT 发布。

#### Scenario: 成功转发
- **WHEN** 收到 `/sorting/robot_status` 消息
- **THEN** 桥接节点：
  - 将 payload JSON 解析为 RobotPayload
  - 封装为 RobotData（添加 msg_type="robot_action", device_id, device_type, timestamp, seq）
  - 通过 MQTT 发布到配置的 topic

#### Scenario: MQTT 断连
- **WHEN** MQTT Broker 不可达
- **THEN** 桥接节点日志记录错误，不影响 `/sorting/robot_status` 订阅和后续重连

### Requirement: 非侵入式设计
通信节点 SHALL NOT 干扰现有抓取流程。

#### Scenario: 桥接节点崩溃
- **WHEN** robot_status_bridge 节点崩溃或未启动
- **THEN** auto_sorting_action.py 的抓取流程正常运行，`/sorting/robot_status` 正常发布

#### Scenario: MQTT 发布阻塞
- **WHEN** MQTT 连接慢或 Broker 响应延迟
- **THEN** auto_sorting_action.py 的抓取流程不受阻塞影响

### Requirement: 可配置化
MQTT 连接参数 SHALL 通过 YAML 配置文件管理。

#### Scenario: 参数覆盖
- **WHEN** 启动 robot_status_bridge
- **THEN** 读取 YAML 配置文件中的 broker_host、broker_port、topic、client_id、username、password，支持 ROS2 参数覆盖

### Requirement: 数据字段定义

#### RobotPayload 字段来源映射
| 字段 | 数据来源 | 转换说明 |
|------|---------|---------|
| action | 周期成功/失败 | 成功=`"sort_complete"`, 失败=`"sort_failed"` |
| cargo_type | `_latest_detection` → object_name | 去除 ` (detected)` 后缀 |
| confidence | `_latest_detection` → confidence | 保留原始浮点值 |
| source_position | pick_pose (x,y,z) | 格式化为 `"(x, y, z)"` 米 |
| target_position | BIN_PLACE_JOINTS → FK 坐标 | 格式化为 `"(x, y, z)"` 米 |
| actual_position | 成功=target_position, 失败=`"NONE"` | 同上 |
| grasp_success | check_grasp_success 返回值 | bool |
| joint_angles | current_joints (弧度→度) | 6个关节，取 joint1~joint6 |
| tcp_position | TF lookup base_link→link6 | 米→毫米 (×1000) |
| error_code | 分类映射 | 0=成功, 1=抓取失败, 2=放置失败, 3=IK/规划失败, 4=超时, 99=未知 |

#### RobotData 封装字段
| 字段 | 值 |
|------|-----|
| msg_type | `"robot_action"` |
| device_id | 可配置，默认 `"ROBOT-01"` |
| device_type | `"robot"` |
| timestamp | ISO 8601 格式，如 `"2026-07-26T15:30:00+08:00"` |
| payload | RobotPayload 完整数据 |
| seq | Unix 秒级时间戳 (int) |

## MODIFIED Requirements
无现有需求被修改。

## REMOVED Requirements
无现有需求被移除。
