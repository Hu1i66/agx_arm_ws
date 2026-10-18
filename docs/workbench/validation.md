# 工作台验证记录

环境：Ubuntu22.04、ROS2 Humble，工作树 `codex/workbench-design-20261019`。未启动实机 CAN、机械臂或相机驱动；未升级共享 Python / ROS / CUDA。

## 针对性自动验证

2026-10-19 主体提交前的相关检查：255 passed，7.15 秒。覆盖工作台后台、双路 ROS 输入、来源会话、恢复编排、放置快照、原图 PNG/ZIP、新帧等待、PTY交互、标定原文和实际生效比较，以及受影响的运行配置、视觉、手眼、TCP原有检查。最终修复后的计数见本文后续补充。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export AGX_ARM_WS="$PWD" ROS_DOMAIN_ID=223 PYTHONPATH="$PWD:$PYTHONPATH"
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest -p no:anyio -p no:cacheprovider tests/test_workbench_*.py tests/test_runtime_config.py tests/test_runtime_launcher.py tests/test_runtime_review_regressions.py tests/test_dual_camera_geometry.py tests/test_vision_offsets.py tests/test_calibration_logic.py tests/test_calibration_review_regressions.py tests/test_tcp_calibration.py tests/test_tcp_verification.py tests/test_handeye_math.py tests/test_handeye_receiver.py -q
```

Qt工作台包 `colcon build --symlink-install --packages-select piper_workbench` 构建成功。后台连接专用 C++ 生命周期回归100次正常；AddressSanitizer（检测越界与释放后访问的工具）在旧析构版本复现错误，在修复版本通过。整图形程序的 ASAN 启动受显卡库开销影响，采用小范围连接回归和原生图形程序直接运行，未以完整 ASAN 通过作结论。

较早一次扩大检查发现旧 `test_old_entry_wrappers.py` 的部署目录假设：外部 `/home/lxf/orange_dataset` 包装器固定委托主部署目录，测试要求当前工作树路径，独立工作树下不满足。未修改该外部包装器或主目录来迁就测试。旧 runtime-config 对现场外参/Dabai未启用的断言已更新为当前确认基线；标定逻辑子进程需要项目根 PYTHONPATH，按实际启动环境复测。现场标定退出已有回归发现的Ctrl-C重复shutdown已修复。

## 实际原生界面验证

六个页面实际启动、截图、退出均完成。1600×900、比例2；另核对1920×1080、比例约1.6。截图中出现的未启动/等待数据是真实离线状态。发现并修复的绘制和生命周期问题见 `implementation.md`；视觉结果见 `visual-comparison.md`。

隔离 ROS domain231 只发布六个已知测试关节角 `[0.2,1.0,-1.2,0.3,0.2,-0.4]`，UI实际反馈与RViz监视器状态均逐项相等，交互末端数量1，窗口退出码0。记录 `assets/offline-model-feedback.json`。这是实际ROS消息到模型的离线验证，不是实机追随精度测量。

通过安装器创建用户应用菜单和桌面入口，使用 `gio launch` 连续打开两次，均正常渲染和退出；两次后台PID相同，无重复启动。专用截图环境变量 `PIPER_WORKBENCH_SCREENSHOT` 仅用于离线验收，普通打开无自动退出。检查结束会停止本次domain231后台，避免用户正常domain0启动时复用离线进程。

## 待用户现场核对

- 两路相机和共享YOLO同时启用后的真实帧率、检测周期、动态抓取表现。
- 三种模式各抓一轮，确认示教途经点、释放点和各模式返程；连续模式在当前物体后暂停。
- 末端球／旋转环、关节与TCP输入规划；实际执行、停止、全局急停和失败显示。
- 以近期实际标定方式各采样、撤回、求解，应用后核对数值与实际TF；TCP以原sweep验证精度。
- 机械臂实体示教按钮与读取反馈；由用户主动触发的碰撞保护恢复，确认旧目标取消、重启后保持暂停。

以上需要现场设备、观测或物理运动，当前软件验证不替代这些结果。开发期间不强制把这些检查做成工作台门禁。

## 最终修复后的验证

整分支审查发现的问题已按 `review.md` 修复。最终相关Python检查 **268 passed，7.32秒**，无失败；Qt包最终构建通过 **16.3秒**；C++状态回归与100次后台连接生命周期检查均返回0。语法检查解析15个受影响Python模块，无新字节码缓存。最终后台只读协议核对和更新后界面再次打开／退出正常。
