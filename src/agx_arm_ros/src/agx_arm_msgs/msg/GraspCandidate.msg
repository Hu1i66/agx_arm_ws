# 单个抓取候选 (GraspNet 输出)
geometry_msgs/PoseStamped grasp_pose      # 抓取位姿 (base_link frame, 已完成坐标变换)
float32 score                             # GraspNet 分数 [0,1], 越高越好
float32 approach_angle_deg                # 夹爪接近方向与 -z 的夹角 (度), 0=纯顶部抓取
