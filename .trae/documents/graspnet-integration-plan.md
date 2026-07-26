# GraspNet 集成到抓取流程 — 实现计划

## Context

当前抓取流程使用 YOLO 检测 + D455 深度测距 + PCA 短轴对齐来确定抓取位姿。用户已验证 GraspNet 能生成高质量的 6DoF 抓取位姿（短边对齐 + 中心抓取），现在要将 GraspNet 集成到实际抓取流程中，替代现有的深度测距 + PCA 步骤。

**用户决策：**
- 在观察位运行 GraspNet（D455 在 0.5-0.8m 深度可靠）
- 失败直接报错跳过，不回退到 sort_verify
- 独立服务节点（避免 .venv_ik 与 orange_dataset/.venv 冲突）

## 架构

```
[realsense_yolo_node] --/detection_info--> [auto_sorting_action]
                                                  |
                                        sort_graspnet 命令
                                                  |
                                    1. 确保观察位 + 张开夹爪
                                    2. 发请求 /graspnet/req
                                                  v
[graspnet_service_node] <--- /graspnet/req (JSON)
   3. 快照最新 depth + tcp_pose + cam_info
   4. T_cam_to_base = T_robot_pose @ T_cam_to_ee (手眼标定)
   5. depth → 点云 → GraspNet 推理
   6. 过滤: 接近方向 + 邻近目标 + 短边+中心
   7. 最优位姿 → base 系 → 列重排 → 四元数
   8. 发布 /graspnet/result (JSON)
                                                  |
[auto_sorting_action] <--- /graspnet/result ------/
   9. POSE_PICK = GraspNet translation + Z offset
  10. 朝向 = GraspNet quaternion
  11. 夹爪宽度 = GraspNet width
  12. 下降抓取 (IK) → 闭合 → 抬起 → 放料 → 回观察位
```

## 文件 1: `/home/lxf/agx_arm_ws/graspnet_service_node.py` (新建)

ROS2 节点，运行在 orange_dataset/.venv 中。启动时加载 GraspNet 模型，持续订阅深度图 + tcp_pose，按需推理。

### 关键组件

**sys.path 注入** (复用 verify_grasp_pose.py L37-42):
```python
ROOT = "/home/lxf/graspnet/graspnet-baseline"
for _sub in ["", "models", "utils", "pointnet2", "knn"]:
    sys.path.insert(0, os.path.join(ROOT, _sub) if _sub else ROOT)
```

**手眼标定加载** (复用 grasp_pose_node.py L91-130):
- JSON 路径: `/home/lxf/handeye/result/2026-06-06_04-12-35_calibration.json` (与 YOLO 节点一致)
- 返回 4x4 矩阵 T_cam_to_ee (camera_color_optical_frame → end_effector)
- 包含 `_quat_to_rotmat(qx,qy,qz,qw)` 辅助函数

**可复用函数** (从 verify_grasp_pose.py 复制，不导入):
- `depth_to_pointcloud(depth, color, intrinsics, depth_offset_m)` — 注意 mask 在校正前计算
- `load_graspnet(checkpoint_path, num_view, device)` — checkpoint=/home/lxf/graspnet/checkpoints/checkpoint-rs.tar
- `run_inference(net, pcd, num_point, device)` — num_point=20000
- `segment_object_points(pcd)` — RANSAC + 深度过滤 + DBSCAN
- `analyze_object_shape(object_cloud)` — PCA 短轴/长轴
- `filter_grasps_short_edge_center(gg, shape)` — 短边+中心打分重排

**GraspNetServiceNode 类:**
- `__init__`: 加载模型，订阅 depth/color/camera_info/tcp_pose，订阅 /graspnet/req，发布 /graspnet/result
- 使用 `MultiThreadedExecutor` (推理 2-3s 期间不阻塞 depth 回调)
- `_depth_cb` / `_color_cb` / `_cam_info_cb` / `_tcp_pose_cb`: 加锁缓存最新数据
- `_req_cb(msg)`: 解析请求，调用 `_handle_get_grasp`，发布结果
- `_handle_get_grasp(pick_base)`: 主推理流水线 (见下)

**_handle_get_grasp 流水线:**
1. 快照最新 depth + tcp_pose + cam_info (加锁)
2. 构建变换链: `T_cam_to_base = T_robot_pose @ T_cam_to_ee`
   - T_robot_pose 从 PoseStamped 转换 (position + quaternion → 4x4)
   - T_cam_to_ee 从手眼标定 JSON 加载
3. depth → 点云 (depth_offset=0.04, mask 在校正前)
4. GraspNet 推理 → GraspGroup
5. **接近方向过滤** (相机系): 保留 `R[2,0] > 0.3` 的候选 (approach 沿相机 Z 正向 = 朝下)
6. **邻近过滤**: pick_base 转相机系，保留距离 < 0.08m 的候选 (只抓目标物体)
7. **短边+中心过滤**: segment_object_points + filter_grasps_short_edge_center
8. 取 Top-1，变换到 base 系: `T_base_grasp = T_cam_to_base @ T_cam_grasp`
9. **列重排** (GraspNet → 机器人约定):
   - robot_R[:,0] (opening) ← gn_R[:,1] (closing)
   - robot_R[:,1] (ortho) ← gn_R[:,2] (ortho)
   - robot_R[:,2] (approach) ← gn_R[:,0] (approach)
10. **安全检查**: `robot_R[2,2] < -0.5` (approach 在 base 系朝下，与 IK 约束 final_ze<0.52 一致)
11. robot_R → 四元数 (复用 _matrix_to_quaternion 算法)
12. 返回 JSON: `{success, translation:[x,y,z], quaternion:[qx,qy,qz,qw], width, score}`

**通信协议:**
- 请求 /graspnet/req (String JSON): `{"cmd":"get_grasp", "pick_x":.., "pick_y":.., "pick_z":..}`
- 响应 /graspnet/result (String JSON): `{"success":true, "translation":[..], "quaternion":[..], "width":.., "score":..}`

## 文件 2: `/home/lxf/agx_arm_ws/auto_sorting_action.py` (修改)

### 修改 1: 添加 GraspNet 服务客户端 (MoveItActionClient.__init__, ~L260 后)

```python
self._graspnet_req_pub = self.create_publisher(String, '/graspnet/req', 10)
self._latest_graspnet_result = None
self._graspnet_event = threading.Event()
self._graspnet_result_sub = self.create_subscription(
    String, '/graspnet/result', self._graspnet_result_cb, 10)
```

### 修改 2: 添加回调和调用方法 (~L278 后)

```python
def _graspnet_result_cb(self, msg):
    self._latest_graspnet_result = json.loads(msg.data)
    self._graspnet_event.set()

def call_graspnet_service(self, pick_pose, timeout_s=15.0):
    req = {"cmd":"get_grasp", "pick_x":float(pick_pose['x']),
           "pick_y":float(pick_pose['y']), "pick_z":float(pick_pose['z'])}
    self._graspnet_event.clear()
    msg = String(); msg.data = json.dumps(req)
    self._graspnet_req_pub.publish(msg)
    # 轮询等待结果 (与 _two_stage_refine 相同模式)
    ...
    return self._latest_graspnet_result or {"success":False, "error":"timeout"}
```

### 修改 3: 分离 sort_graspnet 命令 (L1302)

```python
elif data.get("cmd") in ("sort", "sort_verify", "sort_graspnet"):
    cmd_type = data.get("cmd")
    two_stage = (cmd_type == "sort_verify")
    use_graspnet = (cmd_type == "sort_graspnet")
```

### 修改 4: GraspNet 抓取流程 (在 two_stage 块之后, 第零步之前插入)

```python
if use_graspnet:
    # a. 确保观察位 + 张开夹爪 + 等待数据刷新
    node.move_arm_joint(JOINT_OBSERVE, "GraspNet: 回到观察位")
    node.operate_gripper(dynamic_open, "GraspNet: 张开夹爪")
    time.sleep(0.5)

    # b. 请求 GraspNet
    result = node.call_graspnet_service(pick_pose, timeout_s=15.0)

    # c. 失败: 报错跳过 (无回退)
    if not result.get("success"):
        print(f"❌ GraspNet 失败 ({result.get('error')}), 跳过")
        node.move_arm_joint(JOINT_OBSERVE, "GraspNet失败→回观察位")
        success = False; break

    # d. 成功: 用 GraspNet 位姿覆盖 POSE_PICK
    gn_t = result["translation"]
    gn_q = result["quaternion"]
    gn_w = float(result.get("width", 0.06))
    for p in (POSE_PICK, POSE_PICK_UP, POSE_LIFT_SOFT):
        p['x'] = float(gn_t[0]); p['y'] = float(gn_t[1])
    POSE_PICK['z'] = max(float(gn_t[2]), MIN_GRASP_Z) + GRIPPER_PICK_Z_OFFSET
    POSE_PICK_UP['z'] = POSE_PICK['z'] + 0.13
    POSE_LIFT_SOFT['z'] = POSE_PICK['z'] + 0.035
    graspnet_quat = Quaternion(x=gn_q[0], y=gn_q[1], z=gn_q[2], w=gn_q[3])
    dynamic_open, dynamic_close = compute_gripper_targets(gn_w)
```

### 修改 5: 下降步骤使用 GraspNet 朝向 (L1447-1481)

```python
if use_graspnet:
    active_ori = graspnet_quat
    print("📐 下降抓取使用 GraspNet 6DoF 姿态")
else:
    # 原有 PCA 逻辑不变
    ...

# IK 候选: GraspNet 优先, 失败时加 yaw 变体
if use_graspnet:
    pick_orientations = [graspnet_quat] + node._build_pick_orientations_multi(POSE_PICK)
else:
    pick_orientations = node._build_pick_orientations_multi(POSE_PICK)
    if pca_ori: pick_orientations = [pca_ori] + pick_orientations
```

**后续步骤 (闭合/抬起/放料/回观察位) 无需修改** — 复用现有 dynamic_close, pick_up_joints, BIN_JOINTS, JOINT_OBSERVE。

## 文件 3: `/home/lxf/agx_arm_ws/sorting_gui_client.py` (修改)

sort_graspnet 命令已存在于 GUI (项目记忆: "sort_graspnet 命令重定向到 sort_verify")。只需确保 GUI 发送 `{"cmd":"sort_graspnet", ...}` 而非重定向到 sort_verify。检查并移除重定向逻辑。

## 坐标系约定映射 (已验证)

GraspNet 旋转矩阵 R 列含义 (通过合成物体测试验证):
- R[:,0] = approach (接近方向)
- R[:,1] = closing (闭合/开合方向, 已验证对齐短轴 align=0.989)
- R[:,2] = orthogonal (正交方向)

机器人旋转矩阵列含义 (from _build_pick_orientation L601-616):
- R[:,0] = x_axis = opening (径向水平, 夹爪开合方向)
- R[:,1] = y_axis = orthogonal
- R[:,2] = z_axis = approach ([0,0,-1] 朝下)

**映射**: `robot_R = [gn_R[:,1], gn_R[:,2], gn_R[:,0]]`

变换链 (复用 grasp_pose_node.py L670):
```
T_base_grasp = T_robot_pose @ T_cam_to_ee @ T_cam_grasp
```
- T_robot_pose: from /feedback/tcp_pose (PoseStamped → 4x4)
- T_cam_to_ee: from handeye JSON (constant)
- T_cam_grasp: GraspNet 输出 (translation + rotation_matrix)

## 启动命令

GraspNet 服务节点 (新终端):
```bash
source /home/lxf/orange_dataset/.venv/bin/activate
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/lxf/ROS2-Gazebo-GO2/src/docker/cyclonedds.xml
cd /home/lxf/agx_arm_ws
python3 graspnet_service_node.py
```

## 验证步骤

1. **服务节点冒烟测试**: 单独启动服务节点，`ros2 topic pub --once /graspnet/req '{"cmd":"get_grasp","pick_x":0.2,"pick_y":0.0,"pick_z":0.05}'`，检查 /graspnet/result 返回合法 JSON
2. **位姿质量验证**: 放香蕉在相机下，发请求，检查返回的 translation 接近 YOLO 的 base_position_m，quaternion 转回 R 后 R[:,2] ≈ [0,0,-1]
3. **端到端**: 启动全部节点 (相机+YOLO+GraspNet服务+action)，放物体在传送带，发 sort_graspnet 命令，验证抓取成功
4. **失败路径**: 遮挡相机，发 sort_graspnet，验证 action 节点报错跳过不崩溃
