# 归档记录（archive/legacy/）

本次迁移归档的对象、原路径、目标、SHA256 与原因。原文件内容未修改，仅位置变更（tracked 文件用 git mv 保留历史）。

## graspnet_patches（一次性补丁脚本，已完成使命）

| 原路径 | 目标 | SHA256 | 原因 |
|---|---|---|---|
| _add_filter.py | archive/legacy/graspnet_patches/_add_filter.py | 01fb8118d072b748543538f2e52b4da0d1c71497bf794469a8d0159f2d809254 | 旧深度滤波链一次性补丁脚本，非运行依赖 |
| _fix_filter.py | archive/legacy/graspnet_patches/_fix_filter.py | 1d73e0efed8cbb24a669a5f52523d64e039b76abfccd2535df322ec32c1caaf6 | 旧深度滤波链一次性补丁脚本，非运行依赖 |
| _fix_mask.py | archive/legacy/graspnet_patches/_fix_mask.py | bec409f460fc001ae440252dab66a36272e3dfca749ab45854d68e486e34c659 | 旧深度滤波链一次性补丁脚本，非运行依赖 |

## sorting（旧分拣实现/文本备份）

| 原路径 | 目标 | SHA256 | 原因 |
|---|---|---|---|
| auto_sorting_action.txt | archive/legacy/sorting/auto_sorting_action.txt | 56ba8b0c2c9c657b2a7e7019db0736b48f0c039274413a2605130a9470bb73e7 | 旧分拣历史稿；现役实现为根目录 auto_sorting_action.py |
| joint_space_sorting_action.py | archive/legacy/sorting/joint_space_sorting_action.py | babf15d9153fc2a292e8219e9b53a34bec6646b591e210643757e0d95b3f2a32 | 旧分拣历史稿；现役实现为根目录 auto_sorting_action.py |
| sorting_action1.txt | archive/legacy/sorting/sorting_action1.txt | babf15d9153fc2a292e8219e9b53a34bec6646b591e210643757e0d95b3f2a32 | 旧分拣历史稿；现役实现为根目录 auto_sorting_action.py |

## document_generation（文档生成脚本）

| 原路径 | 目标 | SHA256 | 原因 |
|---|---|---|---|
| .make_docs.py | archive/legacy/document_generation/.make_docs.py | 5bb9c1a6324c3ce5c22c65ba6136fd5c13f7f36a75f2b948f0d1fe75c18890a0 | 一次性文档生成脚本 |

## handeye（旧教程 / ArUco 工具与板样 / TF 图 / .bak）

| 原路径 | 目标 | SHA256 | 原因 |
|---|---|---|---|
| 标定教程_内参与手眼标定.md | archive/legacy/handeye/标定教程_内参与手眼标定.md | f196a431e836cfe5ef765a8da57815dbee63159c6ecd88f5ec192349e958d097 | 旧教程/工具/历史板样/TF图/.bak；当前教程重写，板用 ChArUco 7x5 |
| EYE_TO_HAND_CALIBRATION.md | archive/legacy/handeye/EYE_TO_HAND_CALIBRATION.md | e09e12cc30129df41a7923d14e63bea38fb85fa2a8d3110490ed43fbd7b8390d | 旧教程/工具/历史板样/TF图/.bak；当前教程重写，板用 ChArUco 7x5 |
| HANDEYE_CALIBRATION_GUIDE.md | archive/legacy/handeye/HANDEYE_CALIBRATION_GUIDE.md | 13f8a53eef367ef491c8dc744a7385512ef1c4842a8838e7af878257bab32c91 | 旧教程/工具/历史板样/TF图/.bak；当前教程重写，板用 ChArUco 7x5 |
| generate_marker.py | archive/legacy/handeye/generate_marker.py | 55fe7aa87180d1ac6e211646b049ecb584bb651326bfedeb49ed06d0dac271bf | 旧教程/工具/历史板样/TF图/.bak；当前教程重写，板用 ChArUco 7x5 |
| collect_images.py | archive/legacy/handeye/collect_images.py | cd4866b9c9448d9501f77b4eaec9c54e310efe8c31f0f2c24924e2ca958a0e11 | 旧教程/工具/历史板样/TF图/.bak；当前教程重写，板用 ChArUco 7x5 |
| marker_582_100mm.pdf | archive/legacy/handeye/marker_582_100mm.pdf | ed21810f50fda02494d73cf844f74cf0b5723e94fc3a6d720952adb970477e38 | 旧教程/工具/历史板样/TF图/.bak；当前教程重写，板用 ChArUco 7x5 |
| marker_582_30mm.pdf | archive/legacy/handeye/marker_582_30mm.pdf | 9230c833cae4e20b058a3915eab1dd3e694d1d68b59860a7e94d40306203ccae | 旧教程/工具/历史板样/TF图/.bak；当前教程重写，板用 ChArUco 7x5 |
| frames_2026-04-10_17.09.53.gv | archive/legacy/handeye/frames_2026-04-10_17.09.53.gv | 6a11520214acf73e0639efcc13d3bcb2f8a0fae161decb1d393c3d5237c50553 | 旧教程/工具/历史板样/TF图/.bak；当前教程重写，板用 ChArUco 7x5 |
| frames_2026-04-10_17.09.53.pdf | archive/legacy/handeye/frames_2026-04-10_17.09.53.pdf | 9555d3530e7cf53d205a360e2037b2f2ffe955af9e7f223363f4c38bd734b609 | 旧教程/工具/历史板样/TF图/.bak；当前教程重写，板用 ChArUco 7x5 |
| frames_2026-05-16_01.54.01.gv | archive/legacy/handeye/frames_2026-05-16_01.54.01.gv | 6b39acd304aa786c77b70ae87014b42feca2797a8a185280c576f238fb10ccf4 | 旧教程/工具/历史板样/TF图/.bak；当前教程重写，板用 ChArUco 7x5 |
| frames_2026-05-16_01.54.01.pdf | archive/legacy/handeye/frames_2026-05-16_01.54.01.pdf | 686b4336ab1e948317a5e0b1ec137aa564116e89ad25c57b8647dbd1bbaeedc1 | 旧教程/工具/历史板样/TF图/.bak；当前教程重写，板用 ChArUco 7x5 |
| frames_2026-05-16_02.21.48.gv | archive/legacy/handeye/frames_2026-05-16_02.21.48.gv | 4c586bd6e212afc998f37441d4d0259a36bf99f6dddab93e6063988bf20c8876 | 旧教程/工具/历史板样/TF图/.bak；当前教程重写，板用 ChArUco 7x5 |
| frames_2026-05-16_02.21.48.pdf | archive/legacy/handeye/frames_2026-05-16_02.21.48.pdf | f3f50e565c37c1cdf6a05df3dd24cada34fe29787fac260b36a70a5c100f01a0 | 旧教程/工具/历史板样/TF图/.bak；当前教程重写，板用 ChArUco 7x5 |
| handeye_calibration.py.bak | archive/legacy/handeye/handeye_calibration.py.bak | 857e0b7b0a76ecc589f97791a79028ccbeeb50f350319d85e524a81897ce2ee3 | 旧教程/工具/历史板样/TF图/.bak；当前教程重写，板用 ChArUco 7x5 |

## 说明
- `archive/COLCON_IGNORE` 避免 colcon 发现旧源码。
- 同内容历史文件（joint_space_sorting_action.py 与 sorting_action1.txt 哈希相同）保留各自原名称与来源。
- 归档对象均不参与运行；现役入口见根 README。
