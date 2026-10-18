# 双相机TCP改造实施记录

设计：dual_camera_tcp_grasping_2026-10-17.md。实施跨2026-10-17～18。

工作树`/home/lxf/.codex/worktrees/parallel-dev/agx_arm_ws`，分支`codex/parallel-dev`；未合并主部署目录。
备份`.local_backups/dual-camera-20261017-231534/before.tar`，另有MoveIt builder修改前备份。

- P0 完成：备份、确认数据盘与工作树边界。
- P1 完成：指定15组TCP、D455与Dabai活动外参，完整刚体转换与T1启动配置核对。
- P2 完成软件：控制/视觉均用TCP与几何中心，删除人工位置偏置，保持碰撞检查。
- P3 完成软件：图像时刻Dabai变换、双路快照、单模型服务、GUI相机切换与Dabai独立启动/观察位。
- P4 完成软件：138项针对性测试、8包构建、14组CLI干跑、独立审查修复及教程。实机验收、主部署整合待现场完成。

## 实施决定

1. 保留一套GraspNet模型，同时缓存两相机，请求指定相机；GUI只切换检测组件。
2. 用户追加要求：最终抓取XYZ为物体三维几何中心。共享RGB-D几何模块估计中心，GraspNet取本次快照重估所选track中心并提供姿态/宽度；网络前端点仅记录。
3. 用户无法提供支架尺寸，授权查官方模型/按尺寸与标定位置近似定义；添加可配置相机与支架包围体，明确非实测CAD，待RViz/实物核对。
4. 独立审查P1视觉目标过期已修复，包括动作内部队列出队重新查戳；回归先红后绿。
5. 原主目录已部署流程保留；熟悉的CAN命令不改。新教程明确先退出旧节点，再在本树运行，不自动触碰硬件。

交付详见`../dual_camera_implementation_report.md`及`../dual_camera_grasping.md`。
