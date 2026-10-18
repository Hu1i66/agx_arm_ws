# 工作台实现记录

计划：docs/superpowers/plans/2026-10-19-piper-workbench-plan.md

## 任务 1：现场基线

抓取基线 100977f4；标定使用用户确认的主目录现场版本。主目录只读。

迁入以下不同文件：
- src/handeye_calibration_ros/handeye_calibration_ros/handeye_calibration.py
- src/handeye_calibration_ros/handeye_calibration_ros/intrinsic_calibration.py
- src/handeye_calibration_ros/handeye_calibration_ros/tcp_calibration.py
- src/handeye_calibration_ros/handeye_calibration_ros/tcp_verification.py
- src/handeye_calibration_ros/handeye_calibration_ros/tcp_verification_ros.py
- src/handeye_calibration_ros/handeye_calibration_ros/validation.py
- src/handeye_calibration_ros/package.xml
- tools/calibration/approve_calibration.py
- tests/test_tcp_calibration.py
- tests/test_tcp_verification.py
- tests/test_calibration_review_regressions.py

保留发布版中未被使用的 camera_qos.py，以免破坏旧入口；现场采集改用 qos_profile_sensor_data。未同步主目录文档/PDF删除，未引入已删除的 TCP 整轮工具。当前活动配置与主目录相同，未重写原候选或质量标签。

## 集成实现

工作台为 Qt5 Widgets 原生应用，内嵌现有 RViz / MoveIt。Python 常驻后台通过工作树内 Unix socket 传输控制与状态，图像直接订阅 ROS 原图。UI 关闭后请求当前物体后暂停，已有节点保持运行，重开不会续抓。未增加全组件在线、标定质量或并发调试门禁。

已有能力复用：原 launcher/ProcessManager、CAN 脚本和激活程序、T1 / T5、动态拦截与随动算法、两路共享 YOLO、标定采集求解与 TCP sweep。新代码负责界面、协议、两路缓存、放置快照、采图导出及日志汇聚。TCP 数值选择保留原候选质量；生效比较 SDK 启动值、在线参数、MoveIt 模型和工作台模型。手眼比较检测器和实际 TF；内参比较检测器已加载 K/D。

离线验证发现并修复：后台 socket 销毁回调越界；嵌套透明动画导致按钮绘制缺失；原生 GL 视图跨页重设父对象导致黑屏；隐藏的原 RViz 规划面板在无 MoveIt 时创建额外客户端引发退出竞争。现在复用同一原生三维视图，中文手动控制持有自己的规划客户端；隐藏面板不建立第二条规划连接。

工作树内 TRAC-IK 是从现有外部只读资产取得的独立副本，用于构建此树插件，未修改共享环境或权重。ROS 223 / 231 仅用于本次隔离数据与界面验证；没有启动 CAN、相机驱动或真实机械臂。
