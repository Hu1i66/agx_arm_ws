#!/usr/bin/env python3
"""修复 verify_grasp_pose.py 的深度 mask bug + 聚类加体素降采样

Bug: depth_to_pointcloud 中深度校正把 depth=0 clip 成 1,
     mask=depth_uint16>0 把无效像素全变成有效点 (307200=全部像素),
     产生10万虚假近相机点, 污染 RANSAC 和 PCA.

修复1: mask 在深度校正之前计算
修复2: segment_object_points 的 DBSCAN 前加体素降采样 (加速)
"""
import sys

F = "/home/lxf/graspnet/verify_grasp_pose.py"
with open(F, "r", encoding="utf-8") as fh:
    src = fh.read()

# ── 修复1: mask 在校正前计算 ──
OLD1 = (
    "    # D455 近距离偏差校正 (grasp_pose_node.py:490-494)\n"
    "    if depth_offset_m != 0.0:\n"
    "        correction_mm = int(round(depth_offset_m * 1000))\n"
    "        depth_uint16 = np.clip(\n"
    "            depth_uint16.astype(np.int32) - correction_mm, 1, 65535\n"
    "        ).astype(np.uint16)\n"
    "\n"
    "    # GraspNet CameraInfo: scale = 1000 (mm -> m)\n"
    "    cam = GraspNetCameraInfo(\n"
    "        float(width), float(height), float(fx), float(fy), float(cx), float(cy), 1000.0\n"
    "    )\n"
    "    depth_m = depth_uint16.astype(np.float32)  # GraspNet 内部除以 scale\n"
    "    cloud = create_point_cloud_from_depth_image(depth_m, cam, organized=True)\n"
    "\n"
    "    # 构造 Open3D 点云 (只保留 depth>0 的点)\n"
    "    mask = depth_uint16 > 0\n"
    "    points = cloud[mask]\n"
)
NEW1 = (
    "    # ⚠️ mask 必须在深度校正之前计算!\n"
    "    # 校正会把 depth=0 clip 成 1, 若之后再算 mask 会把无效像素全变成有效点.\n"
    "    mask = depth_uint16 > 0\n"
    "\n"
    "    # D455 近距离偏差校正 (grasp_pose_node.py:490-494)\n"
    "    if depth_offset_m != 0.0:\n"
    "        correction_mm = int(round(depth_offset_m * 1000))\n"
    "        depth_uint16 = np.clip(\n"
    "            depth_uint16.astype(np.int32) - correction_mm, 1, 65535\n"
    "        ).astype(np.uint16)\n"
    "\n"
    "    # GraspNet CameraInfo: scale = 1000 (mm -> m)\n"
    "    cam = GraspNetCameraInfo(\n"
    "        float(width), float(height), float(fx), float(fy), float(cx), float(cy), 1000.0\n"
    "    )\n"
    "    depth_m = depth_uint16.astype(np.float32)  # GraspNet 内部除以 scale\n"
    "    cloud = create_point_cloud_from_depth_image(depth_m, cam, organized=True)\n"
    "\n"
    "    # 构造 Open3D 点云 (只保留原始 depth>0 的有效点)\n"
    "    points = cloud[mask]\n"
)

# ── 修复2: DBSCAN 前加体素降采样 ──
OLD2 = (
    "    # Step 3: DBSCAN 聚类, 取最大簇\n"
    "    labels = np.array(object_cloud.cluster_dbscan(eps=eps, min_points=min_points))\n"
)
NEW2 = (
    "    # Step 3: 体素降采样 (加速 DBSCAN, 5mm 精度足够 PCA)\n"
    "    if len(object_cloud.points) > 5000:\n"
    "        object_cloud = object_cloud.voxel_down_sample(0.005)\n"
    "        print(f\"  体素降采样(5mm): → {len(object_cloud.points)} 点\")\n"
    "\n"
    "    # Step 4: DBSCAN 聚类, 取最大簇\n"
    "    labels = np.array(object_cloud.cluster_dbscan(eps=eps, min_points=min_points))\n"
)

for label, old, new in [("mask修复", OLD1, NEW1), ("体素降采样", OLD2, NEW2)]:
    cnt = src.count(old)
    if cnt != 1:
        print(f"❌ [{label}] 期望匹配 1 次, 实际 {cnt} 次, 中止")
        sys.exit(1)
    src = src.replace(old, new)
    print(f"✅ [{label}] 替换成功")

with open(F, "w", encoding="utf-8") as fh:
    fh.write(src)
print("\n✅ 全部修复完成")
