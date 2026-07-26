# D455 + PCA 短轴对齐 — GraspNet 移除收尾实施计划

> 本计划是 `d455-stereo-grasp-pca-short-axis.md` 的**收尾续作**。
> 上一会话已完成 `realsense_yolo_node.py`（camera_info 订阅 + PCA 短轴对齐 + 发布字段），
> 并部分修改了 `auto_sorting_action.py`（移除了 imports / service client init / 命令分发 / 路径初始化块）。
>
> **当前 `auto_sorting_action.py` 处于损坏状态**：GraspNet init 已删，但 8 处仍引用未定义的
> `graspnet_ok` / `graspnet_candidates` / `use_graspnet_path` → 运行即 `NameError`。
> 本计划完成剩余移除 + PCA 朝向接入 + GUI 清理 + 验证。

---

## 一、现状分析（基于 Phase 1 实际代码读取）

### 1.1 `realsense_yolo_node.py`（✅ 已完成，无需改动）
- L5 `CameraInfo` import、L151 `_cam_info_received` 标志、L308-313 `sub_cam_info` 订阅
- L825-846 `camera_info_cb`（首帧覆盖 fx/fy/cx/cy/camera_matrix/dist_coeffs）
- L855-953 `_compute_grasp_orientation`（bbox+30%margin 裁剪 → 向量化反投影 → cam→ee→base → 高度带通±3cm → PCA on XY → 短轴 → Z朝下+X对齐短轴 四元数；elongation<1.5 返回 None）
- L1195-1200 `primary_info['grasp_orientation'/'elongation_ratio']` 赋值
- L1253-1257 `info_dict` 发布 `grasp_orientation` + `elongation_ratio` + `velocity_mps=None`（随动接口占位）

### 1.2 `auto_sorting_action.py`（⚠️ 损坏，需收尾）
**已完成**：L27-28 imports 注释化、service client init 删除、L1419-1425 命令分发简化（`sort_graspnet`→`two_stage=True` 回退）、L1471 路径初始化块替换为注释。

**剩余损坏引用**（运行即 NameError）：
| 行 | 代码 | 问题 |
|----|------|------|
| L641-717 | `_call_grasp_service_async` 方法 | 引用已删的 `self.grasp_client`/`self.graspnet_timeout_s`/`_GRASPNET_MSGS_AVAILABLE`/`self.apply_z_correction_in_graspnet`；死代码 |
| L719-762 | `_build_pick_orientations_with_graspnet` 方法 | 死代码 |
| L1474 | `if graspnet_ok:` | `graspnet_ok` 未定义 |
| L1493 | `if two_stage and not graspnet_ok:` | 同上 |
| L1513-1600 | GraspNet 近距离精定位块 (`if use_graspnet_path and not graspnet_ok:`) | 同上，整块死且损坏 |
| L1608 | `if two_stage and not graspnet_ok:` | 同上 |
| L1632-1635 | `if graspnet_ok and graspnet_candidates:` 下降抓取朝向 | 同上 |
| L1647-1651 | `if graspnet_ok and graspnet_candidates:` IK 候选 | 同上 |
| L1659 | `'ik_solution' if not graspnet_ok else 'graspnet_ik'` | 同上 |

### 1.3 `sorting_gui_client.py`（❌ 未改动）
- L148 注释 "GraspNet 必需"
- L234-239 `self.use_graspnet` BooleanVar + `latest_detection_stamp`
- L359 标签 "两阶段精定位 / GraspNet 抓取"
- L360-366 GraspNet checkbox UI + `graspnet_status_var`
- L613-664 `_pick_from_detection_two_stage`（`use_gn` 分支）
- L903-938 `_dispatch_sort_cmd`（`use_gn` 分支）
- L940-948 `_on_graspnet_toggle` 方法

---

## 二、目标架构（最终态）

```
/detection_info (realsense_yolo_node):
  base_position_m  +  grasp_orientation (PCA短轴四元数)  +  elongation_ratio  +  velocity_mps(null)
        │                          │
        ▼                          ▼
auto_sorting_action (sort_verify / sort_graspnet 统一为两阶段):
  两阶段精定位 (close-range YOLO xy)  →  下降抓取朝向 = PCA 朝向 (替代 GraspNet 候选)
                                         IK 候选首位插入 PCA 朝向 + yaw 约束引导
                                         position_only 45° 兜底 + FK 检查 (安全防线不变)

sorting_gui_client:
  移除 GraspNet checkbox/开关 → 两阶段精定位夹取按钮统一发 sort_verify
```

**精度来源**：camera_info 准确内参（消除 ~8.5mm）+ D455 立体深度（消除单目 ~10%）+ EMA 滤波 + 移除 GraspNet（消除 ~27mm xy 偏移）。
**短轴对齐来源**：bbox 内 D455 深度点云 → base 系 PCA → 短轴 → 夹爪 X 轴对齐短轴。

---

## 三、实施步骤

### Part A：`auto_sorting_action.py` — 删死方法 + 修损坏引用 + PCA 接入

#### A1. 删除两个死方法（L641-762，约 122 行）
删除 `_call_grasp_service_async`（L641-717）与 `_build_pick_orientations_with_graspnet`（L719-762）整块。
- **方法**：用 Shell+Python 脚本按方法签名定位删除（行范围太大，Edit 重现易错）。
- 删除范围：从 `    def _call_grasp_service_async(` 行 → 到 `        return graspnet_orientations + keep_default` 行（含两者间空行）。
- 删除后保留 `return orientations`（L639，`_build_pick_orientations_multi` 末尾）与 `def _build_goal_constraints`（L764）间的一个空行分隔。

#### A2. 简化 mode_label（L1474-1479）
```python
# 从:
                if graspnet_ok:
                    mode_label = " [GraspNet 抓取位姿]"
                elif two_stage:
                    mode_label = " [两阶段精定位]"
                else:
                    mode_label = ""
# 改为:
                if two_stage:
                    mode_label = " [两阶段精定位]"
                else:
                    mode_label = ""
```

#### A3. 简化两阶段条件（L1491-1493）
```python
# 从:
                    # ── 两阶段精定位 (仅 sort_verify 或 GraspNet 回退时) ──
                    # GraspNet 成功路径不走两阶段 (已有精确位姿)
                    if two_stage and not graspnet_ok:
# 改为:
                    # ── 两阶段精定位 (sort_verify / sort_graspnet 统一路径) ──
                    if two_stage:
```

#### A4. 删除 GraspNet 近距离精定位块（L1513-1600，约 88 行）
删除从 `                    # ── GraspNet 近距离精定位 (sort_graspnet) ──` 注释行
到 `                                          f"(GraspNet z={clamping_z:.3f} + offset={GRIPPER_PICK_Z_OFFSET})")` 行（含尾部空行）。
- **方法**：Shell+Python 脚本按起止字符串定位删除。删除后 `p['y'] = refine_result['y']`（两阶段块末尾）后接 `# 【第零步】`。

#### A5. 简化第一步条件（L1605-1609）
```python
# 从:
                    # 【第一步】 抓取过渡(防止碰桌面) — IK 优先
                    # 两阶段精定位: 已在验证位(物体上方25cm), 可跳过预备位直接下降
                    # GraspNet 近距离: 已在验证位, 但 xy 可能与 GraspNet 结果不同, 需过渡点调整
                    if two_stage and not graspnet_ok:
                        print("⚡ 两阶段模式：已在物体正上方，直接下降抓取")
                    elif not node.move_arm_pose(POSE_PICK_UP, "抓取位上方过渡点", continuous=False, planning_mode='normal'):
# 改为:
                    # 【第一步】 抓取过渡(防止碰桌面) — IK 优先
                    # 两阶段精定位: 已在验证位(物体上方), 可跳过预备位直接下降
                    if two_stage:
                        print("⚡ 两阶段模式：已在物体正上方，直接下降抓取")
                    elif not node.move_arm_pose(POSE_PICK_UP, "抓取位上方过渡点", continuous=False, planning_mode='normal'):
```

#### A6. 下降抓取朝向：GraspNet 候选 → PCA 朝向（L1629-1638）
```python
# 从:
                    pick_fallback_ori = node._build_pick_orientation(POSE_PICK)[0]
                    # GraspNet 成功时: 优先用 best 候选姿态 (X 轴已对齐物体短轴) 作为下降目标朝向,
                    # 使夹爪夹住物体较短方向; 否则回退到 PICK_UP 的实际末端姿态
                    if graspnet_ok and graspnet_candidates:
                        bq = graspnet_candidates[0].grasp_pose.pose.orientation
                        active_ori = [bq.x, bq.y, bq.z, bq.w]
                        print("📐 下降抓取使用 GraspNet 短轴对齐姿态")
                    else:
                        active_ori = getattr(node, 'last_successful_orientation', pick_fallback_ori)
                    pose_pick_msg = node._create_pose(POSE_PICK, active_ori)
# 改为:
                    pick_fallback_ori = node._build_pick_orientation(POSE_PICK)[0]
                    # PCA 短轴对齐: 优先用 /detection_info.grasp_orientation (D455 深度点云 PCA)
                    # 作为下降目标朝向, 使夹爪 X 轴(开合方向)对齐物体短轴;
                    # 无 PCA 朝向 (圆物体/失败) 时回退到径向朝下
                    pca_ori = None
                    if getattr(node, '_latest_detection', None):
                        pca_ori = node._latest_detection.get('grasp_orientation')
                    if pca_ori is not None and len(pca_ori) == 4:
                        active_ori = pca_ori
                        print("📐 下降抓取使用 PCA 短轴对齐姿态")
                    else:
                        active_ori = getattr(node, 'last_successful_orientation', pick_fallback_ori)
                        print("📐 下降抓取使用径向朝下姿态 (物体近圆形或无 PCA 朝向)")
                    pose_pick_msg = node._create_pose(POSE_PICK, active_ori)
```

> **`_latest_detection` 来源说明**：两阶段路径中 `_two_stage_refine` 会清空并等待新鲜近距离检测，返回后 `_latest_detection` 即近距离检测（含 PCA 朝向）；非两阶段 `sort` 路径下 `_latest_detection` 为最近一次观察位检测。两种情况下 PCA 朝向均有效（静态物体短轴帧间不变）。`getattr` + `None` 守卫保证失败时安全回退。

#### A7. IK 候选分支：GraspNet 候选 → PCA 朝向（L1644-1659）
```python
# 从:
                    elif node.enable_ik and node.ik_solver is not None:
                        # 直线插补失败，尝试 IK 多姿态求解（以当前关节角为种子，保证解靠近当前构型）
                        # GraspNet 成功时: 用 GraspNet 候选 orientation + 默认多 yaw (去重)
                        if graspnet_ok and graspnet_candidates:
                            pick_orientations = node._build_pick_orientations_with_graspnet(
                                POSE_PICK, graspnet_candidates
                            )
                            ik_desc = "下降抓取 (GraspNet orientation + IK多姿态)"
                        else:
                            pick_orientations = node._build_pick_orientations_multi(POSE_PICK)
                            ik_desc = "下降抓取 (IK多姿态)"
                        ik_ok, ik_joints = node.move_arm_via_ik(POSE_PICK, pick_orientations, ik_desc, continuous=True)
                        if ik_ok:
                            descent_success = True
                            cycle_strategies.append('ik_joint_space')
                            cycle_profiles.append('ik_solution' if not graspnet_ok else 'graspnet_ik')
# 改为:
                    elif node.enable_ik and node.ik_solver is not None:
                        # 直线插补失败，尝试 IK 多姿态求解（以当前关节角为种子，保证解靠近当前构型）
                        # PCA 朝向优先: 插入候选首位, IK yaw 约束引导匹配短轴对齐
                        pick_orientations = node._build_pick_orientations_multi(POSE_PICK)
                        if pca_ori is not None and len(pca_ori) == 4:
                            pick_orientations = [pca_ori] + pick_orientations
                            ik_desc = "下降抓取 (PCA朝向 + IK多姿态)"
                        else:
                            ik_desc = "下降抓取 (IK多姿态)"
                        ik_ok, ik_joints = node.move_arm_via_ik(POSE_PICK, pick_orientations, ik_desc, continuous=True)
                        if ik_ok:
                            descent_success = True
                            cycle_strategies.append('ik_joint_space')
                            cycle_profiles.append('pca_ik' if pca_ori is not None else 'ik_solution')
```

> `pca_ori` 在 A6 同一 `for _ in range(1):` 循环体内定义，A7 可直接访问。

### Part B：`sorting_gui_client.py` — 移除 GraspNet UI/开关 + 简化按钮

#### B1. L148 注释更新
```python
# 从: # align_depth.enable:=true: 启用深度图与 RGB 对齐 (GraspNet 必需 /camera/camera/aligned_depth_to_color/image_raw)
# 改: # align_depth.enable:=true: 启用深度图与 RGB 对齐 (D455 双目深度 + PCA 短轴对齐必需)
```

#### B2. L234-239 移除 `use_graspnet`，保留 `latest_detection_stamp`
```python
# 从:
            # ── GraspNet 抓取位姿生成开关 ──
            # use_graspnet=True: sort_graspnet 命令 (D455+GraspNet)
            # use_graspnet=False: sort_verify 命令 (单目+两阶段精定位, 旧路径)
            self.use_graspnet=tk.BooleanVar(value=True)
            # 缓存最近一次 /detection_info 的 header_stamp (浮点秒数), 用于 GraspNet service 时间同步
            self.latest_detection_stamp=0.0
# 改为:
            # ── GraspNet 已移除: 统一使用 D455 双目深度 + PCA 短轴对齐 (两阶段精定位) ──
            # 缓存最近一次 /detection_info 的 header_stamp (浮点秒数), 保留备用
            self.latest_detection_stamp=0.0
```

#### B3. L359 标签更新
```python
# 从: tk.Label(df,text=" =单次检测直接抓取 |  =两阶段精定位 / GraspNet 抓取",font=("Arial",8),fg="gray").pack()
# 改: tk.Label(df,text=" =单次检测直接抓取 |  =两阶段精定位 (D455深度+PCA短轴对齐)",font=("Arial",8),fg="gray").pack()
```

#### B4. L360-366 删除 GraspNet checkbox UI 块
整块删除（`# ── GraspNet 抓取位姿生成开关 ──` + `gf=...` + Checkbutton + `graspnet_status_var` + Label）。

#### B5. L613-664 `_pick_from_detection_two_stage` 简化为纯两阶段
删除 `use_gn = bool(...)` 行与整个 `if use_gn:` 分支，仅保留原 `else:` 分支（两阶段）内容，并把二次检测说明改为 "D455 深度 + PCA 短轴对齐"：
```python
            dia_str=f"物体直径: {dia} m\n" if dia else ""
            txt=(f" 两阶段精定位夹取确认\n\n观察位检测坐标:\n  ({pp['x']:.3f}, {pp['y']:.3f}, {pp['z']:.3f})\n\n"
                 f"-> 机械臂移动到物体正上方\n-> YOLO 近距离二次检测 (D455 深度)\n-> PCA 短轴对齐生成夹爪朝向\n-> 以二次结果为准直接抓取\n\n"
                 f"Place: 料框{bin_num} (关节空间预设点位)\n"
                 f"{dia_str}\n确定发送？")
            if not messagebox.askyesno("确认两阶段精定位夹取",txt): return
            cmd={"cmd":"sort_verify","pick":pp,"bin":bin_num,"pick_name":f"{d.get('object_name','物体')} (two-stage)","place_name":f"料框{bin_num}"}
            if dia: cmd["object_diameter_m"]=float(dia)
            sent_msg=f"两阶段精定位指令已发送！\n检测物体 -> 【料框{bin_num}】"
            queue_msg="两阶段精定位任务已加入排队队列。"
```

#### B6. L903-938 `_dispatch_sort_cmd` 简化为纯 sort_verify
- 改 docstring：`"""构造分拣命令并发送: 统一使用 sort_verify (两阶段精定位 + PCA 短轴对齐)。"""`
- 删除 `use_gn = bool(...)` 与整个 `if use_gn:` 分支，仅保留原 `else:` 分支：
```python
            dia = obj.get('size_m', {}).get('diameter')
            cmd = {"cmd": "sort_verify", "pick": pick, "bin": bin_num,
                   "pick_name": f"{name} (auto-sort{'-retry' if retry else ''})",
                   "place_name": f"料框{bin_num}"}
            method_tag = "two-stage"
            if dia: cmd["object_diameter_m"] = round(float(dia), 3)
```

#### B7. L940-948 删除 `_on_graspnet_toggle` 方法
整方法删除。

### Part C：验证

#### C1. 语法检查
```bash
python3 -m py_compile /home/lxf/agx_arm_ws/auto_sorting_action.py
python3 -m py_compile /home/lxf/agx_arm_ws/sorting_gui_client.py
python3 -m py_compile /home/lxf/orange_dataset/realsense_yolo_node.py
```

#### C2. grep 验证无残留 GraspNet 运行时引用
```bash
grep -n -E "graspnet_ok|graspnet_candidates|use_graspnet_path|_GRASPNET_MSGS_AVAILABLE|self\.grasp_client|_call_grasp_service_async|_build_pick_orientations_with_graspnet|self\.use_graspnet|_on_graspnet_toggle|graspnet_status_var|apply_z_correction_in_graspnet" \
  /home/lxf/agx_arm_ws/auto_sorting_action.py /home/lxf/agx_arm_ws/sorting_gui_client.py
```
预期：**无输出**（注释中的 "GraspNet 已移除" 说明文字不计入上述模式）。

#### C3. 重启节点
- `realsense_yolo_node`：重启以加载 camera_info + PCA 代码（上一会话改了文件，运行中进程仍是旧码）
- `auto_sorting_action.py`：**必须重启**（当前损坏，无法运行）
- `sorting_gui_client.py`：重启 GUI

#### C4. 功能测试
1. **细长物体回归**（香蕉/黄瓜）：检测 → 点 "两阶段精定位夹取" → 日志应出现 `📐 下降抓取使用 PCA 短轴对齐姿态`，夹爪夹住窄腰（X 轴对齐短轴）。
2. **圆物体回归**（苹果/橙子）：日志应出现 `📐 下降抓取使用径向朝下姿态 (物体近圆形或无 PCA 朝向)`，无短轴对齐。
3. **安全回归**：确认无向上/水平抓取姿态（FK 检查 + position_only 45° 兜底 + IK `final_ze<0.52` 约束不变）。
4. `realsense_yolo_node` 启动日志确认 `camera_info` 覆盖内参（fx≈384.8）。

---

## 四、假设与决策

1. **`sort_graspnet` 命令保留向后兼容**：L1419-1425 已将其映射为 `two_stage=True`，旧 GUI/外部调用不会报错，统一走两阶段。不改。
2. **`latest_detection_stamp` 保留**：GUI 仍缓存（L489），虽 GraspNet 移除后不再被读取，但无害且为随动抓取预留。不删。
3. **PCA 朝向取自 `node._latest_detection`**（auto_sorting_action 进程的 `/detection_info` 订阅缓存），而非 GUI 传入的 pick 字典。理由：朝向是 base 系四元数，与位置同源；静态物体短轴帧间稳定；两阶段下 `_two_stage_refine` 保证新鲜近距离检测。失败有 `None` 守卫安全回退。
4. **不动 IK 安全约束**：`_combined_error` 的 Z轴+yaw 约束、`final_ze<0.52`、FK 检查、position_only 45° 兜底全部保留——这是拒绝向上抓取的核心防线，与 GraspNet 移除无关。
5. **大块删除用 Shell+Python 脚本**（A1/A4）：行数多（122/88 行），Edit 重现易错；按方法签名/起止字符串定位删除最稳。小改动用 Edit。
6. **realsense_yolo_node.py 不再改动**：已完成且验证通过，仅 C1 语法检查 + C3 重启。

---

## 五、执行顺序

1. A1（删两死方法）→ A2/A3/A5（简化条件）→ A4（删 GraspNet 近距离块）→ A6/A7（PCA 接入）
2. B1-B7（GUI 清理）
3. C1 语法检查 → C2 grep 验证 → C3 重启 → C4 功能测试
