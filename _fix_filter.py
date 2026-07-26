#!/usr/bin/env python3
"""修复 verify_grasp_pose.py 的两个 Bug:
1. segment_object_points: 加深度过滤+DBSCAN聚类, 去除背景点污染
2. print_poses / visualize: 去掉 sort_by_score(), 保留过滤排序"""
import sys

F = "/home/lxf/graspnet/verify_grasp_pose.py"
with open(F, "r", encoding="utf-8") as fh:
    src = fh.read()

# ── 修复1: 替换 segment_object_points (加深度过滤+聚类) ──
OLD1 = (
    "def segment_object_points(pcd, distance_thresh=0.01, ransac_n=3, num_iterations=2000):\n"
    "    \"\"\"RANSAC 去桌面平面, 返回物体点云 (非桌面点)\"\"\"\n"
    "    plane_model, inliers = pcd.segment_plane(\n"
    "        distance_thresh, ransac_n, num_iterations\n"
    "    )\n"
    "    object_cloud = pcd.select_by_index(inliers, invert=True)\n"
    "    print(\n"
    "        f\"  去桌面: {len(pcd.points)} → {len(object_cloud.points)} 个物体点 \"\n"
    "        f\"(平面法向量: [{plane_model[0]:.2f}, {plane_model[1]:.2f}, {plane_model[2]:.2f}])\"\n"
    "    )\n"
    "    return object_cloud\n"
)
NEW1 = (
    "def segment_object_points(pcd, distance_thresh=0.01, ransac_n=3,\n"
    "                          num_iterations=2000, max_depth=0.8,\n"
    "                          eps=0.012, min_points=20):\n"
    "    \"\"\"RANSAC 去桌面 + 深度过滤 + DBSCAN 聚类, 返回最大物体簇\n"
    "\n"
    "    三步分割避免背景点(墙壁/传送带)污染 PCA:\n"
    "      1. RANSAC 去桌面平面\n"
    "      2. 深度过滤: 只保留 z<max_depth 的近处点 (相机系)\n"
    "      3. DBSCAN 聚类: 取最大簇 = 目标物体\n"
    "    \"\"\"\n"
    "    # Step 1: RANSAC 去桌面\n"
    "    plane_model, inliers = pcd.segment_plane(\n"
    "        distance_thresh, ransac_n, num_iterations\n"
    "    )\n"
    "    object_cloud = pcd.select_by_index(inliers, invert=True)\n"
    "    print(\n"
    "        f\"  去桌面: {len(pcd.points)} → {len(object_cloud.points)} 个非平面点 \"\n"
    "        f\"(法向量: [{plane_model[0]:.2f}, {plane_model[1]:.2f}, {plane_model[2]:.2f}])\"\n"
    "    )\n"
    "\n"
    "    # Step 2: 深度过滤 (去远处背景)\n"
    "    pts = np.asarray(object_cloud.points)\n"
    "    depth_mask = pts[:, 2] < max_depth\n"
    "    object_cloud = object_cloud.select_by_index(np.where(depth_mask)[0])\n"
    "    print(f\"  深度过滤 (<{max_depth}m): → {len(object_cloud.points)} 个近处点\")\n"
    "\n"
    "    if len(object_cloud.points) < 50:\n"
    "        print(\"  ⚠️ 近处点过少, 跳过聚类\")\n"
    "        return object_cloud\n"
    "\n"
    "    # Step 3: DBSCAN 聚类, 取最大簇\n"
    "    labels = np.array(object_cloud.cluster_dbscan(eps=eps, min_points=min_points))\n"
    "    n_clusters = labels.max() + 1 if labels.max() >= 0 else 0\n"
    "    if n_clusters == 0:\n"
    "        print(f\"  ⚠️ DBSCAN 无簇 (eps={eps}), 用深度过滤后的点云\")\n"
    "        return object_cloud\n"
    "\n"
    "    unique, counts = np.unique(labels[labels >= 0], return_counts=True)\n"
    "    largest_label = int(unique[np.argmax(counts)])\n"
    "    cluster_mask = labels == largest_label\n"
    "    cluster_cloud = object_cloud.select_by_index(np.where(cluster_mask)[0])\n"
    "    print(\n"
    "        f\"  DBSCAN: {n_clusters} 簇 → 最大簇 {len(cluster_cloud.points)} 点 \"\n"
    "        f\"(label={largest_label}, eps={eps})\"\n"
    "    )\n"
    "    return cluster_cloud\n"
)

# ── 修复2: print_poses 去掉 sort_by_score ──
OLD2 = (
    "def print_poses(gg, top_n):\n"
    "    \"\"\"终端打印 top-N 位姿详情 (位置坐标 + 姿态信息)\"\"\"\n"
    "    gg.sort_by_score()\n"
    "    n = min(top_n, len(gg))\n"
)
NEW2 = (
    "def print_poses(gg, top_n):\n"
    "    \"\"\"终端打印 top-N 位姿详情 (位置坐标 + 姿态信息)\n"
    "\n"
    "    ⚠️ 不调 sort_by_score(): 保留 filter_grasps_short_edge_center 的排序结果.\n"
    "    若未经过滤 (--no_filter), main 中已调 gg.sort_by_score().\n"
    "    \"\"\"\n"
    "    n = min(top_n, len(gg))\n"
)

# ── 修复3: visualize 去掉 sort_by_score (保留 nms 去重) ──
OLD3 = (
    "    gg.nms()\n"
    "    gg.sort_by_score()\n"
    "    gg = gg[:top_n]\n"
)
NEW3 = (
    "    # nms 去重后直接取前 top_n (保留过滤排序, 不重新 sort_by_score)\n"
    "    gg.nms()\n"
    "    gg = gg[:top_n]\n"
)

for label, old, new in [("segment聚类", OLD1, NEW1),
                         ("print_poses去sort", OLD2, NEW2),
                         ("visualize去sort", OLD3, NEW3)]:
    cnt = src.count(old)
    if cnt != 1:
        print(f"❌ [{label}] 期望匹配 1 次, 实际 {cnt} 次, 中止")
        sys.exit(1)
    src = src.replace(old, new)
    print(f"✅ [{label}] 替换成功")

with open(F, "w", encoding="utf-8") as fh:
    fh.write(src)
print("\n✅ 全部 3 处修复完成")
