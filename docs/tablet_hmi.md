# Piper 平板上位机

版本：`0.3.2`

目标：iPad Safari 优先，兼容 Android Chrome，实验室同一局域网使用。

需求基线见 [tablet_hmi_design_v0.3.md](tablet_hmi_design_v0.3.md)。

平板是便携 HMI，只负责显示和发送白名单命令。ROS 2、YOLO、MoveIt、标定和抓取状态机继续由电脑运行。电脑本地工作台始终拥有完整权限。

## 进程

| 进程 | 入口 | 说明 |
|---|---|---|
| 工作台后台 | `python3 -m workbench.service` | 设备、组件、任务状态的唯一权威 |
| 平板网关 | `.venv_tablet/bin/python -m workbench.gateway` | HTTPS、PWA、配对、WebSocket、控制租约、操作代理 |
| 平板媒体 | `.venv_tablet/bin/python -m workbench.media` | ROS 图像订阅、WebRTC H.264、MJPEG/JPEG 降级 |

网关和媒体服务分别登记到现有 `runtime_launcher`，可在电脑工作台“调试 → 平板上位机”中单独启停和重启。任一服务退出不会停止 T1、T5、相机或电脑工作台。

媒体进程由轻量 supervisor 管理。编码器发生原生退出时，supervisor 会按 1～10 秒退避重新创建媒体 worker，并继续向网关提供在线状态；停止媒体服务时会同时结束 supervisor 与 worker。

## 安装与启动

```bash
cd /home/lxf/.codex/worktrees/f142/agx_arm_ws
./scripts/setup_tablet_hmi.sh
./scripts/start_tablet_hmi.sh
```

`setup_tablet_hmi.sh` 会在项目内创建 `.venv_tablet`，安装网关和媒体依赖，然后调用 `scripts/build_tablet_robot_model.py` 生成连杆 GLB，最后构建 `tablet_hmi/dist`。不会修改系统 Python、ROS 或 Orbbec 环境。

单独前台运行：

```bash
./scripts/start_tablet_gateway.sh
./scripts/start_tablet_media.sh
```

默认端口：

- `https://<电脑局域网 IP>:8443/`：平板入口。
- `http://<电脑局域网 IP>:8080/`：重定向到 HTTPS。
- `127.0.0.1:8090`：媒体服务，仅供本机网关代理，不直接暴露给平板。

## 首次配对

1. 打开电脑工作台，进入“调试 → 平板上位机”。
2. 启动网关与媒体服务。
3. 选择默认角色并点击“生成新配对码”。
4. 平板扫描二维码，或用 Safari 打开页面并输入六位配对码。
5. 第一次使用 HTTPS 时，在电脑管理页“服务与本地证书”中打开 CA 文件，按设备提示安装并信任。该 CA 只用于当前工作树的局域网地址。

配对码十分钟有效且只能使用一次。设备令牌保存在 HttpOnly、Secure、SameSite Cookie 中，网关只保存令牌摘要。设备撤销后，现有 WebSocket 连接会立即关闭。

## 控制权限

- 观察者：状态、相机、模型和流程只读。
- 操作者：取得租约后可发出抓取和连接命令。
- 默认租约 15 秒，平板每 5 秒续约。
- 同一时刻只允许一个平板持有租约。
- 电脑管理页可以立即收回租约。
- 页面切到后台后平板主动释放租约；即使释放请求被系统挂起，服务端也会按 TTL 自动过期。
- 租约丢失不会触发回零、松爪或已开始动作的中断。

## 平板功能

抓取页：

- D455 固定相机与 Dabai 腕载相机。
- WebRTC H.264 主链路；建连、解码或无新帧时自动切换 MJPEG。
- Canvas 检测框叠加，点击画面或目标列表选择 `track_id`。
- Three.js 显示当前 URDF 连杆，位置和四元数来自 `base_link → link1…tcp_link` 的服务器 TF。
- 当前目标位置、TCP、J1～J6、抓取流程和后台任务状态。
- D455 动态、D455 静态、Dabai 静态。
- 放置方案选择、连续开始、所选目标单次抓取、本轮后停止。
- 待机位、观察位、复位、急停。

连接页：

- CAN 激活、机械臂激活、T1、D455、Dabai、双路 YOLO、T5、传送带。
- 独立启动和停止；已经运行的组件由后台真实状态维护按钮。
- 一键启动全部、关闭所有连接、碰撞后恢复。
- 高风险操作使用二次确认；相机、检测和动作节点之间没有额外互锁门禁。

## API

平板接口：

```text
POST /api/v1/pair
GET  /api/v1/session
GET  /api/v1/health
POST /api/v1/lease
POST /api/v1/actions
GET  /api/v1/media/token
GET  /ws/v1/events
GET  /ws/v1/jpeg/{camera}
POST /api/v1/media/whep/{camera}
DELETE /api/v1/media/whep/{camera}/{session}
GET  /media/{camera}/mjpeg
GET  /media/{camera}/snapshot
```

平板命令白名单：

```text
status.get
motion.start
motion.pause_after_cycle
motion.send
system.start_missing
system.cancel
system.stop_all
arm.recover
component.start
component.stop
```

`motion.send.cmd` 只允许 `standby`、`observe`、`reset`、`emergency_stop`。

电脑管理代理：

```text
tablet.status.get
tablet.start_all
tablet.stop_all
tablet.gateway.start / stop / restart
tablet.media.start / stop / restart
tablet.pair.create / revoke_all
tablet.device.revoke
tablet.lease.revoke
tablet.certificate.info / regenerate / download
tablet.stream.keyframe
```

所有平板动作使用客户端 `request_id`，网关缓存 60 秒。断线重试同一请求返回原结果，不会重复执行机械臂命令。

## 视频

媒体服务直接订阅：

- `/camera/camera/color/image_raw`
- `/dabai/color/image_raw`

WebRTC 使用 aiortc 发送 H.264，默认 15 fps，两路最高目标合计约 3～5 Mbps。当前默认使用 libx264；在 iPad Safari 双流实测中 NVENC 路径曾触发 `libnvcuvid.so` 原生崩溃，因此不再默认启用。`config/tablet_hmi.json` 可将 `media.h264_encoder` 切换为 `nvenc` 做独立验证。JPEG 主循环默认 6 fps，质量 72，通过同源 WebSocket 发送；WebSocket 也不可用时退回 MJPEG。每路只保留最新帧，不建立历史队列。

在平板地址后加 `?transport=jpeg` 可直接验证低带宽 JPEG 模式，例如 `https://10.217.81.33:8443/?transport=jpeg#/grab`。

WebRTC 信令通过网关同源转发，媒体 HTTP 端口只绑定回环地址。短时媒体令牌默认 60 秒，绑定设备与相机。检测框由平板 Canvas 叠加，不烧录进视频。

## 三维模型

`scripts/build_tablet_robot_model.py` 从当前 `test2.urdf` 的 visual mesh 生成：

```text
tablet_hmi/public/models/
  model_manifest.json
  base_link.glb
  link1.glb … link6.glb
  gripper_base.glb
  gripper_link1.glb
  gripper_link2.glb
```

网关通过 `tablet.telemetry` 以约 15 Hz 读取 TF，平板对位置做 lerp、对四元数做 slerp。模型不可用时自动退回真实 FK 骨架，而不是隐藏机械臂状态。

Assimp 导出的 GLB 同时带场景根节点和网格节点的 Z-up→Y-up 转换。Three.js 端对这层重复旋转保留一次净 −90° 轴变换，再把 TF 位姿转换到 Three.js 坐标；如果按普通“抵消两次旋转”处理，各连杆会散开。该组合已通过相邻连杆最近点距离验证。

## 服务重启与断线

- WebSocket 断线按 0.8 秒到 8 秒指数退避重连。
- 重连后先重新拉取 snapshot，再接收增量事件。
- WebRTC 失败或无新帧自动切换 JPEG，并保留最后一帧。
- 页面隐藏时停止 Three.js 渲染和 WebRTC 编码。
- 网关退出只让平板离线，不影响电脑工作台或已开始的机械臂动作。

## 验证

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv_tablet/bin/python -m pytest \
  -p no:cacheprovider tests/test_tablet_hmi.py -q
npm --prefix tablet_hmi run build
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select piper_workbench
```

实机验收仍需分别在 iPad Safari 和 Android Chrome 上检查：CA 信任、两路视频同时播放、WebRTC 到 JPEG 降级、后台恢复、租约过期、网关/媒体独立重启，以及至少一小时连续运行。

2026-10-19 当前工作树已完成：

- 电脑工作台相关 Python 回归 117 项通过，平板上位机专项 6 项通过。
- Vue/TypeScript 生产构建和 `piper_workbench` Qt 构建通过。
- HTTPS 配对、观察者设备、会话快照和控制租约实际通过。
- 双路 MJPEG、同源 WebSocket JPEG 实际收到 JPEG 帧。
- 经网关 WHEP 建连实际协商 H.264，收到 640×480 视频帧。
- 平板页面在 1024×768 横屏和 768×1024 竖屏完成实际浏览器渲染检查。

## 已知边界

- 当前 WebRTC 使用本机 host ICE candidate，面向同一局域网。跨 VLAN 时应增加 TURN，不能直接把媒体端口暴露到公网。
- 当前 GLB 使用 assimp 转换现有 DAE，未做 Meshopt 二次压缩；资源量约 9 MiB。
- iPad 后台会暂停页面脚本，租约由服务端 TTL 兜底，不能依赖浏览器定时器。
- 本地 CA 必须在每台新平板上安装并信任；更换证书后需要重新信任。
