# Dabai主目录启用记录（2026-10-18）

原因：用户日常运行目录`/home/lxf/agx_arm_ws`仍是旧入口，`camera_tf --camera dabai --dry-run`复现“生产视觉仅支持D455”。此前双相机代码只在工作树。

处理：确认主目录抓取/视觉/启动关键文件与上轮修改前基线一致，按文件整合已检查的双相机代码，不重置Git或暂存区。
备份：`/home/lxf/agx_arm_ws/.local_backups/dabai-enable-20261018-001451`，含`before.tar`、`files.json`、`calibration_docs_before.tar`。
Dabai活动外参为22:53:28结果，parent=link6；最新15组TCP通过独立活动配置选用，原始failed候选与采样不覆盖。
主目录原D455登记和不相关runtime设置保留，CAN激活命令不改。没有名为“生产状态”的运行开关需要用户解除。

验证：主目录相关测试138 passed（6.02秒）；agx_arm_moveit重建成功；Dabai的TF、detector、graspnet、gui、arm五入口dry-run均退出0。
未启动机械臂、相机、GUI或TF节点，未发运动命令。旧进程不会自动换版本：仅运行TF可直接用新命令；开始新TCP抓取流程前按教程正常重启旧T1及相关应用。
教程：`docs/dual_camera_grasping.md`，README和标定文档已追加当前主目录说明。
清理：本次`.runtime/dabai-enable-checks`（磁盘占用420K）已通过gio trash移入回收站，原路径不存在，未清空回收站；保留必要构建、备份和实验数据。
