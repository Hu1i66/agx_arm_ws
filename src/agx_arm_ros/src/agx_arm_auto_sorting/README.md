# agx_arm_auto_sorting

自动分拣与采集包，包含两个节点：

- sorting_cycle_server: 提供一次完整分拣循环的触发服务，底层使用 MoveIt2 MoveGroup Action。
- dataset_collection_runner: 自动遍历 Gazebo 场景参数，调用分拣触发服务，完成三重校验并写入 CSV。

说明：如果当前 Gazebo 环境未提供 entity state 查询服务（如 `/gazebo/get_entity_state`），采集节点会自动降级运行，不会崩溃；此时仅保留可用校验项，缺失校验项按失败计入标签。

## 关键接口

- 触发服务: /sorting/trigger_once (std_srvs/Trigger)
- 状态话题: /sorting/state (std_msgs/String)
- 结果话题: /sorting/cycle_result (std_msgs/String, JSON)

## 配置文件

默认配置文件：

- config/sorting_dataset_config.yaml

可配置项包括：

- 分拣动作参数（速度、加速度、位姿、工作空间限位）
- Gazebo 场景服务名
- 果实参数遍历（直径、姿态）
- 光照强度与角度
- 数据集 CSV 路径、断点续传开关

## 运行

1) 编译

```
cd /home/lxf/agx_arm_ws
colcon build --packages-select agx_arm_auto_sorting
source install/setup.bash
```

2) 启动分拣服务端（依赖 MoveIt2 已启动）

```
ros2 run agx_arm_auto_sorting sorting_cycle_server
```

3) 启动采集脚本（依赖 Gazebo + sorting_cycle_server 已启动）

```
ros2 run agx_arm_auto_sorting dataset_collection_runner
```

4) 一键启动两个节点

```
ros2 launch agx_arm_auto_sorting auto_sorting_pipeline.launch.py
```
