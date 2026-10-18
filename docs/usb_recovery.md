# USB掉线后的STM32和机械臂恢复

日期：2026-10-18。

## 本次故障证据

两种报错来自设备连接失效：

- 内核日志记录 `usb 1-2: USB disconnect` 多次发生，同一集线器下的相机、CH340串口、CAN适配器均重新枚举。
- GUI旧进程持有 `/dev/ttyUSB1 (deleted)`，当前设备已经是 `/dev/ttyUSB0`。旧句柄写入报 `Input/output error`。
- `ip -details -statistics link show can0` 显示 `DOWN`、`STOPPED`、收发计数为0。
- T1日志持续出现 `Agx_arm is not connected, cannot control`。缺少真实关节反馈时，TF树也可能断开。

这说明本轮静止抓取的 `CONTROL_FAILED(-4)` 有明确的硬件通信故障证据，不能通过改抓取姿态、放宽规划条件来恢复。
集线器断开的具体原因尚未确认，可能涉及线缆、接触或供电；不能仅凭本次日志断言是哪一种。
建议CAN适配器优先直连电脑，STM32与相机尽量不要与CAN共用同一个外接集线器。

## 已修复的软件问题

原代码遇到串口异常后只等待，没有关闭失效句柄；GUI只在启动时记录一次连接成功标志。
现在改为：

1. 读/写遇到串口或系统I/O错误后关闭旧连接，丢弃旧速度和状态。
2. GUI后台每秒尝试重新连接，重新枚举当前设备，优先使用`/dev/serial/by-id/`稳定路径。
3. 重连不发送`S/P/F/R`，不会自动启动或换向，也不会补发失败的旧命令。
4. 收到真实状态帧后才显示速度、物体和电机状态；未连接或状态过期显示`--`。
5. 传送带错误只显示在传送带区域，不再覆盖机械臂服务器状态。

STM32速度仍沿用原始数值及原有滑动平均，没有增加速度比例。
自动重连属于GUI后台行为；单独使用`STM32Conveyor`时，掉线后可再次调用`connect()`。

## 本次只读确认

通过新版控制器连接当前STM32，收到：

```text
V:0.0000,O:1,M1:0,M2:0
```

解析为：连接正常、速度0m/s、电机停止、无物体。整个读取过程没有发送控制命令。
这证明当前串口能读取真实数据，尚不能证明电机启动、运行速度读数或热插拔恢复已完成现场验收。

操作者重新执行T0并启动T1后，进一步只读确认：

- CAN恢复UP，bitrate为1000000，收发计数增加，错误计数为0。
- 3秒内收到601条不同时间戳的真实关节反馈，以及600条TCP反馈。
- `base_link → tcp_link` TF已连通。
- `arm_controller`、`gripper_controller`和`joint_state_broadcaster`均为active。

CAN中的`ERROR-ACTIVE`是可正常收发的状态名称，不等于设备报错。
这些确认没有发送运动命令；传送带开关、运转速度和静止抓取结果仍需操作者现场复验。

## 恢复顺序

1. 停止自动分拣、动态抓取及排队任务，退出T5/T6和T1。
2. 重新插稳USB线，等待设备识别。不要在旧动作节点仍持有任务时直接恢复CAN。
3. 使用原来的T0命令激活接口及机械臂：

```bash
cd /home/lxf/agx_arm_ws
bash ~/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh can0 1000000 && cansend can0 123#1122334455667788 && ip -s link show can0
/usr/bin/python3 /home/lxf/agx_arm_ws/activate_robot.py
```

激活脚本可能使能机械臂；T1/T5启动也保留原有回零/待机行为，运行前留出活动空间。

4. 按README启动T1。仅重新激活CAN并不保证旧T1连接会恢复，需要重新启动T1。
5. 相机若也掉线、没有新图像，应按README恢复T2/T2b/T3。之后启动T5和新版T6。
6. 先确认静止抓取，再在现场手动点击传送带开/关，观察速度和电机状态对应。最后再尝试动态抓取。

只读检查命令：

```bash
ip -details -statistics link show can0
ls -l /dev/serial/by-id/
source /opt/ros/humble/setup.bash
source /home/lxf/agx_arm_ws/install/setup.bash
ros2 topic hz /feedback/joint_states
# Ctrl+C结束监听后
ros2 run tf2_ros tf2_echo base_link tcp_link
```

CAN应为UP、bitrate 1000000，真实反馈应持续更新，TF应能连续输出。
若整个USB集线器再次掉线，应先处理连接稳定性；串口重连不能代替CAN重新激活和T1恢复。

不要为修复这次USB断线修改手眼、TCP、深度或抓取偏置。
