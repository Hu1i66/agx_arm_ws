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
