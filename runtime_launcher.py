#!/usr/bin/env python3
"""统一启动入口：命令构造、前台监督、后台组件确保、只读检查。

计划 3 要求：
  - build_command(component, *, camera, mode, ros_args) -> list[str]（固定接口）
  - 每个入口支持 --dry-run：只输出解释器、argv、配置和关键环境，
    不加载模型、不建输出目录、不启动组件、不访问 CAN/串口
  - 手动命令默认前台监督：创建组件进程组、转发 SIGINT/TERM、wait 回收、返回真实退出码
  - GUI 在 ROS worker 里 background ensure，仅管理 camera、camera_tf、detector
  - 就绪规则与超时见 config/runtime.json readiness 段
  - 启动/错误日志放 log/runtime/
"""
from __future__ import annotations

import argparse
import datetime
import json
import os
import pathlib
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
import uuid

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import runtime_config as rc  # noqa: E402
from runtime_process_manager import (  # noqa: E402
    ProcessError, ProcessManager, _boot_id, _parse_stat,
)

MANUAL = "manual"
GUI = "gui"

# 组件键（计划 3.6）
KEY_CAMERA = "camera:{camera}"
KEY_CAMERA_TF = "camera_tf:{camera}"
KEY_DETECTOR = "detector:{camera}"

LAUNCHABLE = (
    "arm", "camera", "camera_tf", "detector", "graspnet", "action", "gui", "workbench", "workbench-service",
    "calibrate-intrinsics", "calibrate-handeye", "calibrate-tcp", "verify-tcp",
    "approve-intrinsics", "approve-handeye", "generate-board",
)
COMPONENTS = LAUNCHABLE + ("preflight", "stop", "status")
STOPPABLE = ("arm", "camera", "camera_tf", "detector", "graspnet", "action", "gui", "workbench", "workbench-service")


class LauncherError(Exception):
    pass


# ========================================================================
# 命令构造（固定公共接口）
# ========================================================================
def build_command(component: str, *, camera: str = "d455", mode: str = "fruit",
                  ros_args=(), session_dir=None) -> list:
    """构造组件启动 argv（列表，不字符串拼接）。

    ros_args: 额外 ROS 参数/参数覆盖，原样追加（如 "calibration_file:=/path"）。
    session_dir: 内部参数，标定组件复用已创建的会话目录（避免二次时间戳）。
    """
    ros_args = [str(a) for a in ros_args]
    ws = rc.workspace_root()
    if component == "arm":
        cmd = ["ros2", "launch", "agx_arm_ctrl",
               "start_single_agx_arm_moveit.launch.py",
               "can_port:=can0", "arm_type:=piper", "effector_type:=agx_gripper"]
        if not any(a.startswith("tcp_offset:=") for a in ros_args):
            from runtime_geometry import load_tcp
            cmd.append("tcp_offset:="+json.dumps(load_tcp()))
    elif component == "workbench":
        cmd = ["ros2", "launch", "piper_workbench", "workbench.launch.py"]
    elif component == "workbench-service":
        cmd = [str(rc.resolve_resource("system_python")), "-m", "workbench.service"]
    elif component == "camera":
        if camera == "d455":
            # 冻结参数（计划 3.4）：手动与 GUI 共用
            cmd = ["ros2", "launch", "realsense2_camera", "rs_launch.py",
                   "rgb_camera.color_profile:=640,480,30",
                   "depth_module.profile:=640,480,30",
                   "align_depth.enable:=true",
                   "publish_tf:=false",
                   "spatial_filter.enable:=true",
                   "temporal_filter.enable:=true",
                   "hole_filling_filter.enable:=true"]
        elif camera == "dabai":
            # 计划 5.5：不指定深度 profile；不用 dabai_dcl.py 的 frame_sync
            cmd = ["ros2", "launch", "orbbec_camera", "dabai.launch.py",
                   "camera_name:=dabai", "publish_tf:=false",
                   "depth_registration:=true",
                   "align_mode:=HW",
                   "enable_point_cloud:=false",
                   "enable_colored_point_cloud:=false",
                   "enable_ir:=false",
                   "color_format:=MJPG",
                   "color_width:=640",
                   "color_height:=480",
                   "color_fps:=15"]
        else:
            raise LauncherError(f"未知相机: {camera}")
    elif component == "camera_tf":
        cmd = ["ros2", "launch", str(ws / "launch" / "eye_to_hand_camera_tf.launch.py"), "camera:="+camera]
    elif component == "detector":
        if mode == "fruit":
            cmd = [str(rc.resolve_resource("vision_python")),
                   str(ws / "vision" / "realsense_yolo_node.py"),
                   "--ros-args", "--params-file",
                   str(ws / "config" / "vision" / "fruit.yaml")]
        elif mode == "blue_block":
            cmd = [str(rc.resolve_resource("vision_python")),
                   str(ws / "vision" / "blue_block_detector.py"),
                   "--ros-args", "--params-file",
                   str(ws / "config" / "vision" / "blue_block.yaml")]
        else:
            raise LauncherError(f"未知检测模式: {mode}")
        if mode == "blue_block":
            cmd += ["-p", "camera_id:="+camera]
        if camera == "dual" and mode == "fruit":
            cmd += ["-p", "d455.enabled:=true", "-p", "dabai.enabled:=true", "-p", "show_gui_window:=false"]
            settings_file=ws/"config/workbench/settings.json"
            if settings_file.is_file():
                values=json.loads(settings_file.read_text()).get("vision",{})
                for name,value in values.items():cmd += ["-p",name+":="+str(value).lower()]
        if camera == "dabai" and mode == "fruit":
            cmd += ["-p", "d455.enabled:=false", "-p", "dabai.enabled:=true",
                    "-r", "/dabai/detection_info:=/detection_info",
                    "-r", "/dabai/yolo/annotated_image/compressed:=/yolo/annotated_image/compressed"]
    elif component == "graspnet":
        cmd = [str(rc.resolve_resource("vision_python")),
               str(ws / "graspnet_service_node.py"), "--ros-args", "-p", "camera_id:="+camera]
    elif component == "action":
        cmd = [str(rc.resolve_resource("system_python")),
               str(ws / "auto_sorting_action.py")]
    elif component == "gui":
        cmd = [str(rc.resolve_resource("system_python")),
               str(ws / "sorting_gui_client.py"), "--camera", camera]
    elif component == "calibrate-intrinsics":
        cfg = rc.load_runtime_config()
        cam_cfg = cfg["cameras"][camera]
        board = cfg["calibration"]["board"]
        session = pathlib.Path(session_dir) if session_dir \
            else calibration_session_dir(camera, "intrinsics")
        cmd = ["ros2", "run", "handeye_calibration_ros",
               "camera_intrinsic_calibration", "--ros-args",
               "-p", f"camera_id:={camera}",
               "-p", f"image_topic:={cam_cfg['rgb_topic']}",
               "-p", f"camera_info_topic:={cam_cfg['camera_info_topic']}",
               "-p", f"out_yaml:={session / 'candidate.yaml'}",
               "-p", f"squares_x:={board['squares_x']}",
               "-p", f"squares_y:={board['squares_y']}",
               "-p", f"square_length:={board['square_length_m']}",
               "-p", f"marker_length:={board['marker_length_m']}",
               "-p", f"aruco_dict:={board['aruco_dict']}"]
    elif component == "calibrate-handeye":
        cfg = rc.load_runtime_config()
        cam_cfg = cfg["cameras"][camera]
        man = json.loads((ws / "config" / "calibration" / "manifest.json")
                         .read_text(encoding="utf-8"))
        mode_he = man["cameras"][camera]["mode"]
        intr = ws / "config" / "calibration" / camera / "intrinsics.yaml"
        result_dir = ws / "data" / "calibration" / camera / "handeye"
        cmd = ["ros2", "run", "handeye_calibration_ros", "handeye_calibration",
               "--ros-args",
               "-p", f"camera_id:={camera}",
               "-p", f"mode:={mode_he}",
               "-p", "piper_topic:=/feedback/tcp_pose",
               "-p", f"image_topic:={cam_cfg['rgb_topic']}",
               "-p", f"camera_info_topic:={cam_cfg['camera_info_topic']}",
               "-p", f"intrinsics_yaml:={intr}",
               "-p", f"result_save_path:={result_dir}",
               "-p", f"min_square_px:={cfg['calibration']['handeye_gate']['min_square_px']}"]
    elif component == "calibrate-tcp":
        if any(arg == "--input" or arg.startswith("--input=") for arg in ros_args):
            cmd = [str(rc.resolve_resource("system_python")),
                   str(ws / "src/handeye_calibration_ros/handeye_calibration_ros/tcp_calibration.py")]
        else:
            cmd = ["ros2", "run", "handeye_calibration_ros", "tcp_calibration", "--ros-args",
                   "-p", f"result_save_path:={ws / 'data' / 'calibration' / 'tcp'}"]
    elif component == "verify-tcp":
        cmd = [str(rc.resolve_resource("system_python")),
               str(ws / "src/handeye_calibration_ros/handeye_calibration_ros/tcp_verification.py")]
    elif component == "approve-intrinsics":
        cmd = [str(rc.resolve_resource("system_python")),
               str(ws / "tools" / "calibration" / "approve_calibration.py"),
               "intrinsics", "--camera", camera]
    elif component == "approve-handeye":
        cmd = [str(rc.resolve_resource("system_python")),
               str(ws / "tools" / "calibration" / "approve_calibration.py"),
               "handeye", "--camera", camera]
    elif component == "generate-board":
        cmd = [str(rc.resolve_resource("system_python")),
               str(ws / "tools" / "calibration" / "generate_charuco_board.py")]
    else:
        raise LauncherError(f"未知组件: {component}")
    if "--ros-args" in cmd and ros_args[:1] == ["--ros-args"]:
        ros_args = ros_args[1:]
    return cmd + ros_args


def component_key(component: str, *, camera: str = "d455",
                  mode: str = "fruit") -> str:
    if component == "camera":
        return KEY_CAMERA.format(camera=camera)
    if component == "camera_tf":
        return "camera_tf:"+camera
    if component == "detector":
        return KEY_DETECTOR.format(camera=camera)
    return component


def calibration_session_dir(camera: str, kind: str) -> pathlib.Path:
    """标定会话候选目录：data/calibration/<cam>/<kind>/<时间戳>_session/"""
    ws = rc.workspace_root()
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    return ws / "data" / "calibration" / camera / kind / f"{stamp}_session"


# ========================================================================
# 环境构造（读取 source 后的完整环境，再叠加项目 DDS）
# ========================================================================
_ENV_CACHE: dict = {}


def build_env(camera: str = "d455") -> dict:
    """返回组件子进程环境：source ROS(+Orbbec)+主空间 overlay，显式 DDS。

    不依赖调用方是否 source；不 source ~/.bashrc；不设置 PYTHONNOUSERSITE；
    不全局注入 IK site-packages；ROS_DOMAIN_ID 保留调用者值。
    """
    cache_key = (camera, str(rc.workspace_root()), os.environ.get("ROS_DOMAIN_ID"))
    if cache_key in _ENV_CACHE:
        return dict(_ENV_CACHE[cache_key])
    cfg = rc.load_runtime_config()
    ros_setup = rc.resolve_resource("ros_setup")
    ws_setup = rc.resolve_resource("workspace_setup")
    dds_xml = rc.resolve_resource("dds_xml")
    rmw = cfg["ros"]["rmw_implementation"]
    parts = [f"source {shlex.quote(str(ros_setup))}"]
    if camera in ("dabai", "dual"):
        orbbec = rc.resolve_resource("orbbec_setup")
        if orbbec.is_file():
            parts.append(f"source {shlex.quote(str(orbbec))}")
        else:
            print(f"[launcher][WARN] Orbbec overlay 不存在: {orbbec}", file=sys.stderr)
    if ws_setup.is_file():
        parts.append(f"source {shlex.quote(str(ws_setup))}")
    else:
        print(f"[launcher][WARN] 主空间 overlay 不存在: {ws_setup}", file=sys.stderr)
    parts.append(f"export RMW_IMPLEMENTATION={shlex.quote(rmw)}")
    parts.append(f"export CYCLONEDDS_URI={shlex.quote('file://' + str(dds_xml))}")
    parts.append("/usr/bin/python3 -c 'import os,json; print(json.dumps(dict(os.environ)))'")
    script = " && ".join(parts)
    try:
        proc = subprocess.run(["bash", "-c", script], capture_output=True,
                              text=True, timeout=90)
    except subprocess.TimeoutExpired as e:
        raise LauncherError(f"环境构造超时: {e}") from e
    if proc.returncode != 0:
        raise LauncherError(f"环境构造失败: {proc.stderr.strip()[:500]}")
    lines = [l for l in proc.stdout.strip().splitlines() if l.strip()]
    if not lines:
        raise LauncherError("环境构造输出为空")
    try:
        env = json.loads(lines[-1])
    except json.JSONDecodeError as e:
        raise LauncherError(f"环境构造输出解析失败: {e}") from e
    if env.get("RMW_IMPLEMENTATION") != rmw:
        env["RMW_IMPLEMENTATION"] = rmw
    if env.get("CYCLONEDDS_URI") != f"file://{dds_xml}":
        env["CYCLONEDDS_URI"] = f"file://{dds_xml}"
    env.pop("PYTHONNOUSERSITE", None)
    env["PYTHONUNBUFFERED"] = "1"
    env["AGX_ARM_WS"] = str(rc.workspace_root())
    env["PYTHONPATH"] = str(rc.workspace_root()) + (
        os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
    _ENV_CACHE[cache_key] = env
    return dict(env)


# ========================================================================
# 只读查询助手（ros2 CLI）
# ========================================================================
def _ros2(args, camera="d455", timeout=15):
    env = build_env(camera)
    argv = ["ros2"] + [str(a) for a in args]
    # tf2_echo is continuous. Run the binary directly so timeout kills it,
    # rather than only killing a ros2-run parent and orphaning its child.
    if list(args[:3]) == ["run", "tf2_ros", "tf2_echo"]:
        argv = [str(rc.resolve_resource("ros_setup").parent /
                    "lib" / "tf2_ros" / "tf2_echo")] + list(args[3:])
    try:
        proc = subprocess.run(argv,
                              capture_output=True, text=True,
                              timeout=timeout, env=env)
        return proc.returncode, proc.stdout, proc.stderr
    except subprocess.TimeoutExpired as exc:
        def decoded(value):
            return value.decode("utf-8", errors="replace") if isinstance(value, bytes) else (value or "")
        return 124, decoded(exc.stdout), decoded(exc.stderr) or "timeout"
    except OSError as exc:
        return 127, "", str(exc)


def _echo_once(topic, camera="d455", timeout=15, field=None):
    """订阅并取一条新消息；返回 (ok, text)。field 限定提取字段。"""
    args = ["topic", "echo", topic, "--once"]
    if field:
        args += ["--field", field]
    rc_, out, err = _ros2(args, camera=camera, timeout=timeout)
    return rc_ == 0 and bool(out.strip()), out


def _unquote_scalar(raw: str):
    raw = raw.strip()
    if len(raw) >= 2 and raw[0] == "'" and raw[-1] == "'":
        return raw[1:-1].replace("''", "'")
    if len(raw) >= 2 and raw[0] == '"' and raw[-1] == '"':
        return raw[1:-1].replace('\\"', '"')
    return raw


def _parse_echo_string_field(text):
    """从 ros2 topic echo --field data 输出提取 std_msgs/String 的 data 值。"""
    if not text.strip():
        return None
    # --field data 输出为 YAML 标量（可能带引号/块标记）
    lines = []
    for line in text.strip().splitlines():
        if line.strip() in ("---", "..."):
            break
        lines.append(line)
    if lines and lines[0].strip() in ("|", ">", "|-", ">-"):
        body = "\n".join(lines[1:])
    else:
        body = "\n".join(lines)
    return _unquote_scalar(body.strip())


def _topic_publisher_count(topic, camera="d455"):
    rc_, out, _ = _ros2(["topic", "info", topic], camera=camera, timeout=10)
    if rc_ != 0:
        return None
    m = re.search(r"Publisher count:\s*(\d+)", out)
    return int(m.group(1)) if m else None


def _detector_nodes_online(camera="d455"):
    """当前在线的检测节点集合（/object_detector、/blue_block_detector）。

    用节点列表而非 `ros2 topic info`（Humble 默认不打印发布者节点名，
    需 --verbose；节点列表更直接可靠）。
    """
    rc_, out, _ = _ros2(["node", "list"], camera=camera, timeout=10)
    if rc_ != 0:
        return set()
    known = {"/object_detector", "/blue_block_detector"}
    return {l.strip() for l in out.splitlines() if l.strip() in known}


def _node_exists(node, camera="d455"):
    rc_, out, _ = _ros2(["node", "list"], camera=camera, timeout=10)
    if rc_ != 0:
        return False
    return any(l.strip() == node for l in out.splitlines())


# ========================================================================
# 就绪检查（计划 3.6）
# ========================================================================
def check_ready(component: str, *, camera: str = "d455", mode: str = "fruit",
                record: dict | None = None) -> dict:
    """返回 {ready: bool|None, detail: str, checks: [...]}。

    区分"接口存在"与"模型就绪"；不发送真实抓取请求。
    """
    if component in ("workbench", "workbench-service"):
        return {"ready": False, "checks": [], "detail": "使用本工作树进程登记与本地套接字"}
    if component == "detector" and camera == "dual":
        # Do not reuse a single-camera node just because its root output exists.
        enabled=True
        for name in ("d455.enabled","dabai.enabled"):
            code,out,_=_ros2(["param","get","/object_detector",name],camera="dual",timeout=3)
            enabled=enabled and code==0 and "true" in out.lower()
        enabled=enabled and (_topic_publisher_count("/dabai/detection_info",camera="dual") or 0)>0
        return {"ready": enabled, "checks": [], "detail": "双路检测配置" if enabled else "旧单路检测器需要显式停止再启动双路"}
    cfg = rc.load_runtime_config()
    checks = []

    def _add(name, ok, detail=""):
        checks.append({"check": name, "ok": bool(ok), "detail": detail})
        return ok

    if component == "camera":
        cam_cfg = cfg["cameras"][camera]
        timeout = cfg["readiness"]["camera"]["max_age_s"]
        max_age = cfg["readiness"]["camera"]["max_age_s"]
        for topic in (cam_cfg["rgb_topic"], cam_cfg["depth_topic"],
                      cam_cfg["camera_info_topic"]):
            cnt = _topic_publisher_count(topic, camera)
            _add(f"publisher {topic}", cnt is not None and cnt >= 1,
                 f"publisher_count={cnt}")
        if not all(c["ok"] for c in checks):
            return {"ready": False, "detail": "相机输入发布者不完整", "checks": checks}
        ok, out = _echo_once(cam_cfg["camera_info_topic"], camera, timeout=timeout)
        if ok:
            w = re.search(r"^width:\s*(\d+)", out, re.M)
            h = re.search(r"^height:\s*(\d+)", out, re.M)
            if camera == "d455":
                expected = tuple(map(int, cam_cfg["color_profile"].split(",")[:2]))
            else:
                expected = (cam_cfg["color_width"], cam_cfg["color_height"])
            size_ok = bool(w and h) and (int(w.group(1)), int(h.group(1))) == expected
            _add("camera_info 尺寸", size_ok,
                 f"{w.group(1) if w else '?'}x{h.group(1) if h else '?'}")
        else:
            _add("camera_info 尺寸", False, "未收到 camera_info")
        for label, topic in (("RGB", cam_cfg["rgb_topic"]), ("深度", cam_cfg["depth_topic"])):
            started = time.monotonic()
            ok, _ = _echo_once(topic, camera, timeout=max_age, field="header")
            received_in_time = ok and time.monotonic() - started <= max_age + .1
            _add(f"{label} 新消息", received_in_time,
                 "收到本次订阅的新消息" if received_in_time else "订阅超时")
        if camera == "d455":
            rc_, out, _ = _ros2(["param", "get", "/camera/camera", "publish_tf"],
                                camera=camera, timeout=10)
            tf_off = rc_ == 0 and bool(re.search(r"\bfalse\b", out, re.I))
            _add("publish_tf=false", tf_off, out.strip()[:80])
    elif component == "camera_tf":
        timeout = min(3, cfg["readiness"]["camera_tf"]["timeout_s"])
        try:
            he = rc.load_handeye(camera)
        except rc.ConfigError as e:
            return {"ready": False, "detail": f"活动外参不可用: {e}", "checks": checks}
        rc_, out, err = _ros2(["run", "tf2_ros", "tf2_echo", he["parent"], he["child"]],
                              camera=camera, timeout=timeout)
        # tf2_echo 持续输出直到超时(rc=124)；有输出即视为可查询
        queryable = "Translation:" in out
        if not queryable:
            _add("tf2_echo 可查询", False, (err.strip()[:120] or "无输出"))
        else:
            _add("tf2_echo 可查询", True, "")
            tm = re.search(r"Translation:\s*\[([^\]]+)\]", out)
            qm = re.search(r"Rotation:[^\[]*\[([^\]]+)\]", out)
            if tm:
                t = [float(x) for x in tm.group(1).split(",")]
                dt = max(abs(a - b) for a, b in zip(t, he["position"]))
                _add("TF 平移与活动值一致", dt < 1e-3, f"max_diff={dt:.2e}")
            else:
                _add("TF 平移与活动值一致", False, "未解析到 Translation")
            if qm:
                q = [float(x) for x in qm.group(1).split(",")]  # xyzw
                qxyzw = list(he["quaternion_normalized"])
                dq = max(abs(a - b) for a, b in zip(q, qxyzw))
                dq_neg = max(abs(a - b) for a, b in zip(q, [-v for v in qxyzw]))
                _add("TF 旋转与活动值一致(q/-q 等价)", min(dq, dq_neg) < 1e-3,
                     f"max_diff={min(dq, dq_neg):.2e}")
            else:
                _add("TF 旋转与活动值一致", False, "未解析到 Rotation")
    elif component == "detector":
        if mode == "fruit":
            node, other, timeout = "/object_detector", "/blue_block_detector", \
                cfg["readiness"]["detector_fruit"]["timeout_s"]
        else:
            node, other, timeout = "/blue_block_detector", "/object_detector", \
                cfg["readiness"]["detector_blue_block"]["timeout_s"]
        topic = "/detection_info"
        online = _detector_nodes_online(camera)
        _add(f"节点 {node} 在线", node in online, f"detectors={sorted(online)}")
        cnt = _topic_publisher_count(topic, camera)
        # 根 /detection_info 恰有一个目标 publisher：目标在线、另一模式不在线、发布者恰为 1
        exactly_one = (cnt == 1 and node in online and other not in online)
        _add(f"{topic} 恰有一个目标发布者({node})", exactly_one,
             f"publisher_count={cnt} detectors={sorted(online)}")
        if not exactly_one:
            return {"ready": False, "detail": "目标检测节点/唯一发布者未就绪", "checks": checks}
        ok, out = _echo_once(topic, camera,
                             timeout=cfg["readiness"]["detector_" + mode]["max_age_s"],
                             field="data")
        if not ok:
            _add("收到新消息", False, "echo --once 超时")
        else:
            _add("收到新消息", True, "")
            data = _parse_echo_string_field(out)
            try:
                payload = json.loads(data) if data else None
            except json.JSONDecodeError:
                payload = None
            if isinstance(payload,dict):
                _add("检测来源相机一致",payload.get('camera','d455')==camera,str(payload.get('camera','d455')))
                _add("输入与图像时刻变换有效",payload.get('valid') is not False,str(payload.get('invalid_reason','有效')))
            if payload is None:
                _add("消息 JSON 可解析", False, (data or "")[:80])
            elif mode == "fruit":
                has_detected = isinstance(payload, dict) and "detected" in payload
                _add("水果 JSON 格式(detected 字段)", has_detected,
                     f"detected={payload.get('detected') if isinstance(payload, dict) else '?'}")
            else:
                dets = payload.get("detections") if isinstance(payload, dict) else None
                _add("蓝方块 JSON 格式(detections 列表)", isinstance(dets, list),
                     f"n={len(dets) if isinstance(dets, list) else '?'}（空列表合法）")
    elif component == "graspnet":
        cfg_g = cfg["graspnet"]
        req_code, req_out, _ = _ros2(["topic", "info", cfg_g["req_topic"]], camera=camera)
        req_match = re.search(r"Subscription count:\s*(\d+)", req_out)
        req_cnt = int(req_match.group(1)) if req_code == 0 and req_match else 0
        res_cnt = _topic_publisher_count(cfg_g["result_topic"], camera)
        interface = (req_cnt is not None and req_cnt >= 1
                     and res_cnt is not None and res_cnt >= 1)
        _add("接口存在(req/result 话题)", interface,
             f"req={req_cnt} result={res_cnt}")
        model_ready = None
        if record and record.get("log_file"):
            try:
                with open(record["log_file"], "rb") as stream:
                    stream.seek(record.get("log_offset", 0))
                    text = stream.read().decode("utf-8", errors="replace")
            except OSError:
                text = ""
            loaded = cfg_g["model_loaded_log"] in text
            ready_mark = cfg_g["model_ready_log"] in text
            model_ready = loaded and ready_mark
            _add("模型就绪(托管进程日志)", bool(model_ready),
                 f"loaded={loaded} ready_mark={ready_mark}")
        if interface and model_ready is None:
            return {"ready": False,
                    "detail": "接口存在；模型就绪无法自动判定（外部手动进程，请人工核查其终端日志中的"
                              f"'{cfg_g['model_ready_log']}'）",
                    "checks": checks}
        if interface and model_ready is False:
            return {"ready": False,
                    "detail": "接口存在；模型仍在加载（未出现就绪日志）",
                    "checks": checks}
    else:
        return {"ready": None, "detail": f"组件 {component} 无就绪检查定义",
                "checks": checks}

    ready = all(c["ok"] for c in checks) if checks else False
    failed = [c for c in checks if not c["ok"]]
    detail = "全部就绪" if ready else "未就绪: " + "; ".join(
        f"{c['check']}({c['detail']})" for c in failed)
    return {"ready": ready, "detail": detail, "checks": checks}


# ========================================================================
# preflight（分组件报告缺失；可选 live 只读查询）
# ========================================================================
def preflight(*, camera: str = "d455", mode: str = "fruit",
              live: bool = False) -> dict:
    ws = rc.workspace_root()
    report = {"workspace": str(ws), "camera": camera, "mode": mode,
              "components": {}, "missing": []}
    env = build_env(camera)

    def _need(path, label, component):
        p = pathlib.Path(path)
        ok = p.exists()
        report["components"].setdefault(component, []).append(
            {"item": label, "path": str(p), "ok": ok})
        if not ok:
            report["missing"].append(f"{component}: {label} ({p})")

    for key in rc.RESOURCE_KEYS:
        if key == "orbbec_setup" and camera != "dabai":
            continue
        if key.startswith("graspnet_"):
            # GraspNet 已退出主抓取流程，缺资源不再算 preflight 失败。
            report.setdefault("optional", []).append(
                {"item": f"资源 {key}", "path": str(rc.resolve_resource(key))})
            continue
        _need(rc.resolve_resource(key), f"资源 {key}", "config")
    _need(ws / "launch" / "eye_to_hand_camera_tf.launch.py", "TF launch", f"camera_tf:{camera}")
    _need(ws / "vision" / "realsense_yolo_node.py", "水果检测脚本", f"detector:{camera}")
    _need(ws / "vision" / "blue_block_detector.py", "蓝方块脚本", f"detector:{camera}")
    _need(ws / "config" / "vision" / "fruit.yaml", "水果参数", f"detector:{camera}")
    _need(ws / "config" / "vision" / "blue_block.yaml", "蓝方块参数", f"detector:{camera}")
    report.setdefault("optional", []).append(
        {"item": "GraspNet 脚本(按需)", "path": str(ws / "graspnet_service_node.py")})
    _need(ws / "auto_sorting_action.py", "分拣动作脚本", "action")
    _need(ws / "sorting_gui_client.py", "GUI 脚本", "gui")
    if camera == "dabai":
        _need(rc.resolve_resource("orbbec_setup"), "Orbbec overlay", "camera:dabai")

    # 解释器可用性
    for key, component in (("vision_python", f"detector:{camera}"),
                           ("system_python", "action")):
        p = rc.resolve_resource(key)
        ok = p.is_file() and os.access(p, os.X_OK)
        report["components"].setdefault(component, []).append(
            {"item": f"解释器可执行 {key}", "path": str(p), "ok": ok})
        if not ok:
            report["missing"].append(f"{component}: 解释器不可执行 {key} ({p})")

    # 关键 Python 依赖（ROS 环境下按各自解释器只读 import 检查）
    ros_setup = rc.resolve_resource("ros_setup")
    dep_checks = [
        ("vision_python", f"detector:{camera}", ["cv2", "numpy", "ultralytics"]),
        ("system_python", "action", ["rclpy", "numpy"]),
    ]
    for key, component, mods in dep_checks:
        py = rc.resolve_resource(key)
        if not py.is_file():
            continue
        for m in mods:
            proc = subprocess.run([str(py), "-c", f"import {m}"],
                                  capture_output=True, text=True, timeout=90, env=env)
            ok = proc.returncode == 0
            report["components"].setdefault(component, []).append(
                {"item": f"import {m} ({key})", "path": str(py), "ok": ok})
            if not ok:
                report["missing"].append(
                    f"{component}: import {m} 失败 ({py}) {proc.stderr.strip()[:120]}")

    # 标定包可发现性（任务 5 构建后）
    proc = subprocess.run(
        ["ros2", "pkg", "prefix", "handeye_calibration_ros"],
        capture_output=True, text=True, timeout=60, env=env)
    pkg_ok = proc.returncode == 0
    report["components"].setdefault("calibrate", []).append(
        {"item": "handeye_calibration_ros 包可发现", "path": "-", "ok": pkg_ok})
    if not pkg_ok:
        report["missing"].append(
            "calibrate: handeye_calibration_ros 未构建（任务 5："
            "colcon build --packages-select handeye_calibration_ros）")

    _need(ws / "config/calibration/tcp.json", "活动TCP", "action")
    _need(ws / "config/robot/dabai_mount.json", "腕载碰撞体配置", "arm")

    # 活动外参可加载
    try:
        rc.load_handeye(camera)
        report["components"].setdefault(f"camera_tf:{camera}", []).append(
            {"item": "活动外参可加载", "path": "-", "ok": True})
    except rc.ConfigError as e:
        report["components"].setdefault(f"camera_tf:{camera}", []).append(
            {"item": "活动外参可加载", "path": "-", "ok": False, "detail": str(e)})
        report["missing"].append(f"camera_tf:d455: 活动外参不可用 ({e})")

    report["ok"] = not report["missing"]

    if live:
        live_report = {}
        for comp, cam in (("camera", camera), ("camera_tf", camera),
                          ("detector", camera)):
            try:
                manager = _manager()
                record = manager.get_record(component_key(comp, camera=cam, mode=mode))
                if record is not None and not manager.verify_identity(record)[0]:
                    record = None
                r = check_ready(comp, camera=cam, mode=mode, record=record)
                live_report[comp] = {"ready": r["ready"], "detail": r["detail"]}
            except Exception as e:  # noqa: BLE001
                live_report[comp] = {"ready": None, "detail": f"查询失败: {e}"}
        report["live"] = live_report
    return report


# ========================================================================
# 前台监督 / 后台确保
# ========================================================================
def _log_runtime(component_key: str, message: str):
    ws = rc.workspace_root()
    cfg = rc.load_runtime_config()
    log_dir = ws / cfg.get("runtime_log_dir", "log/runtime")
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    line = f"[{stamp}] [{component_key}] {message}\n"
    with open(log_dir / "launcher.log", "a", encoding="utf-8") as f:
        f.write(line)


def _manager() -> ProcessManager:
    cfg = rc.load_runtime_config()
    return ProcessManager(rc.workspace_root(),
                          cfg.get("runtime_state_dir", ".runtime"))


def _runtime_log_dir() -> pathlib.Path:
    cfg = rc.load_runtime_config()
    d = rc.workspace_root() / cfg.get("runtime_log_dir", "log/runtime")
    d.mkdir(parents=True, exist_ok=True)
    return d


def _make_record(key: str, argv, pid: int, *, initiator: str, mode, camera,
                 log_file) -> dict:
    return {
        "component_key": key,
        "managed_workspace": str(rc.workspace_root()),
        "initiator": initiator,
        "owner_session": f"{initiator}:{os.getpid()}",
        "pid": pid,
        "pgid": pid,  # start_new_session=True → 子进程为 session/组 leader
        "uid": os.getuid(),
        "proc_starttime": (_parse_stat(pid) or {}).get("starttime"),
        "boot_id": _boot_id(),
        "record_id": uuid.uuid4().hex,
        "entry": argv[0],
        "argv": argv,
        "mode": mode,
        "camera": camera,
        "started_at": time.time(),
        "log_file": str(log_file),
    }


def run_foreground(component: str, *, camera: str = "d455", mode: str = "fruit",
                   ros_args=()) -> int:
    """手动命令：前台监督，转发 SIGINT/TERM，wait 回收，返回真实退出码。"""
    key = component_key(component, camera=camera, mode=mode)
    env = build_env(camera)
    log_dir = _runtime_log_dir()
    log_file = log_dir / f"{key.replace(':', '_')}.log"
    session_dir = None

    # 标定命令前置校验与会话目录（真实运行才做；dry-run 不建目录不校验）
    if component == "calibrate-intrinsics":
        session_dir = calibration_session_dir(camera, "intrinsics")
        session_dir.mkdir(parents=True, exist_ok=True)
    if component == "calibrate-handeye":
        intr = (rc.workspace_root() / "config" / "calibration" / camera
                / "intrinsics.yaml")
        use_camera_info = "use_camera_info:=true" in ros_args
        if not intr.is_file() and not use_camera_info:
            raise LauncherError(
                f"相机 {camera} 尚未批准内参: {intr} 不存在。\n"
                f"请先执行: ./scripts/run_component.sh calibrate-intrinsics --camera {camera}\n"
                f"完成采集与批准（approve-intrinsics）后再运行手眼标定。")

    argv = build_command(component, camera=camera, mode=mode,
                         ros_args=ros_args, session_dir=session_dir)
    _log_runtime(key, f"START manual argv={argv}")
    print(f"[launcher] 组件 {key} 以前台监督方式启动")
    print(f"[launcher] argv: {' '.join(argv)}")
    print(f"[launcher] 日志: {log_file}")

    manager = _manager()
    with manager.transition_lock(key):
        existing = manager.get_record(key)
        if existing:
            valid, _ = manager.verify_identity(existing)
            if valid or manager.group_alive(key):
                raise LauncherError(f"组件 {key} 已启动或仍有子进程，拒绝重复启动")
            manager.unregister(key, existing["record_id"])
        if component == "camera":
            topic = rc.load_runtime_config()["cameras"][camera]["rgb_topic"]
            if (_topic_publisher_count(topic, camera) or 0) > 0:
                raise LauncherError(f"外部相机已在发布 {topic}，拒绝重复启动")
        if component == "detector":
            if _detector_nodes_online(camera) or (_topic_publisher_count("/detection_info", camera) or 0):
                raise LauncherError("外部检测节点已启动，拒绝重复启动")
        if component == "camera_tf":
            found = check_ready("camera_tf", camera=camera)
            if any(c["check"] == "tf2_echo 可查询" and c["ok"] for c in found["checks"]):
                raise LauncherError("外部相机TF已存在，拒绝重复发布")
        if component == "arm" and _node_exists("/agx_arm_ctrl_single_node"):
            raise LauncherError("机械臂控制节点已启动，拒绝重复启动")
        if component == "graspnet" and (_topic_publisher_count("/graspnet/result") or 0):
            raise LauncherError("GraspNet节点已启动，拒绝重复启动")
        if component == "action" and _node_exists("/piper_moveit_action_client"):
            raise LauncherError("分拣执行节点已启动，拒绝重复启动")
        log_offset = log_file.stat().st_size if log_file.exists() else 0
        proc = subprocess.Popen(
            argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            stdin=None, cwd=str(rc.workspace_root()), env=env,
            start_new_session=True)
        record = _make_record(key, argv, proc.pid, initiator=MANUAL,
                              mode=mode if component == "detector" else None,
                              camera=camera if component in ("camera", "detector") else None,
                              log_file=log_file)
        record["log_offset"] = log_offset
        manager.register(record)

    logf = open(log_file, "ab", buffering=0)

    def _pump():
        try:
            for line in iter(lambda: os.read(proc.stdout.fileno(), 4096), b""):
                try:
                    sys.stdout.buffer.write(line)
                    sys.stdout.buffer.flush()
                except Exception:  # noqa: BLE001
                    pass
                logf.write(line)
        finally:
            try:
                proc.stdout.close()
            except Exception:  # noqa: BLE001
                pass

    pump_thread = threading.Thread(target=_pump, daemon=True)
    pump_thread.start()

    def _forward(signum, _frame):
        try:
            if manager.verify_identity(record)[0]:
                os.killpg(proc.pid, signum)
        except OSError:
            pass

    old_handlers = {}
    for s in (signal.SIGINT, signal.SIGTERM):
        try:
            old_handlers[s] = signal.signal(s, _forward)
        except ValueError:
            pass
    try:
        rc_code = proc.wait()
    finally:
        for s, h in old_handlers.items():
            try:
                signal.signal(s, h)
            except ValueError:
                pass
        pump_thread.join(timeout=3)
        logf.close()

    result = manager._finalize_stop(key, record, is_parent=True)
    _log_runtime(key, f"EXIT code={rc_code} finalize={result.get('result')}")
    if result.get("result") == "incomplete_exit":
        print(f"[launcher][WARN] {result['detail']} 存活成员: {result.get('alive_members')}")
    print(f"[launcher] 组件 {key} 已退出，退出码 {rc_code}")
    return rc_code


def ensure_background(component: str, *, camera: str = "d455", mode: str = "fruit",
                      initiator: str = GUI, owner_session: str | None = None,
                      ros_args=(), wait_ready: bool = False, cancel_event=None) -> dict:
    """后台确保（GUI 用）：复用已登记/已就绪外部节点，否则启动并登记。"""
    key = component_key(component, camera=camera, mode=mode)
    cfg = rc.load_runtime_config()
    manager = _manager()
    with manager.transition_lock(key):
        if cancel_event is not None and cancel_event.is_set():
            return {"result": "cancelled", "ready": False, "component_key": key}
        record = manager.get_record(key)
        if record is not None:
            ok, reason = manager.verify_identity(record)
            if ok:
                if (component == "detector" and record.get("mode")
                        and record["mode"] != mode):
                    return {
                        "result": "mode_conflict",
                        "component_key": key,
                        "registered_mode": record.get("mode"),
                        "requested_mode": mode,
                        "initiator": record.get("initiator"),
                        "detail": "已登记进程为其他检测模式；GUI 显式切换请先停止旧 detector"
                                  "（stop --allow-cross-initiator），或提示操作者关闭旧终端",
                    }
                res = {"result": "reused", "component_key": key,
                       "record": record, "detail": "复用已登记进程"}
                if wait_ready:
                    res["ready"], res["ready_detail"] = _wait_background_ready(
                        manager, record, component, camera, mode, cancel_event)
                _log_runtime(key, f"ENSURE reused initiator={initiator}")
                return res
            if manager.group_alive(key):
                return {"result": "incomplete_exit", "ready": False,
                        "detail": "登记leader失效但组内仍有存活成员，拒绝重复启动",
                        "component_key": key}
            manager.unregister(key, record.get("record_id"))
            _log_runtime(key, f"ENSURE stale_record_removed: {reason}")

        # 无登记：检查外部未登记节点是否已就绪（只读；无法证明所有权，不停止）
        r = check_ready(component, camera=camera, mode=mode)
        if r["ready"]:
            _log_runtime(key, f"ENSURE external_ready initiator={initiator}")
            return {"result": "reused_external", "component_key": key,
                    "ready": True, "ready_detail": r["detail"],
                    "detail": "复用外部未登记但已就绪的节点（无停止权）"}

        # camera：外部未登记相机已在发布但未就绪（可能开启自带 TF 或配置不符）→
        # 报告冲突，**不叠加第二个相机**（计划 3.6）
        if component == "camera":
            cam_cfg = cfg["cameras"][camera]
            pubs = _topic_publisher_count(cam_cfg["rgb_topic"], camera)
            if (pubs or 0) >= 1:
                _log_runtime(key, "ENSURE external_camera_conflict")
                return {
                    "result": "external_conflict",
                    "component_key": key,
                    "detail": f"检测到外部未登记的相机已在发布 {cam_cfg['rgb_topic']}，"
                              "但就绪检查未通过（可能未收到 CameraInfo、开启了自带 TF 或"
                              "分辨率/源配置不符）。为避免叠加第二个相机/TF 修补，不重复启动；"
                              "请检查该相机是否按统一入口配置运行，或关闭后重试。",
                }

        # camera_tf：若 base→optical TF 已存在但与活动值不一致（外部错误 TF）→ 拒绝叠加
        if component == "camera_tf":
            he = rc.load_handeye(camera)
            rc_, out, _ = _ros2(["run", "tf2_ros", "tf2_echo",
                                 he["parent"], he["child"]],
                                camera=camera, timeout=3)
            if "Translation:" in out:
                _log_runtime(key, "ENSURE external_tf_conflict")
                return {
                    "result": "external_conflict",
                    "component_key": key,
                    "detail": f"已存在 {he['parent']}→{he['child']} 的 TF，"
                              "但与活动外参不一致（外部错误 TF）。为避免双 TF，拒绝叠加；"
                              "请停止外部 TF 发布或改用统一入口。",
                }

        # detector：无登记且目标未就绪时，若另一模式检测节点或未知发布者仍在，
        # 拒绝叠加第二个检测节点，提示操作者关闭旧终端
        if component == "detector":
            other_node = "/blue_block_detector" if mode == "fruit" else "/object_detector"
            online = _detector_nodes_online(camera)
            cnt = _topic_publisher_count("/detection_info", camera)
            if online or (cnt or 0) >= 1:
                foreign = sorted(online) or [f"/detection_info publishers={cnt}"]
                _log_runtime(key, f"ENSURE external_conflict foreign={foreign}")
                return {
                    "result": "external_conflict",
                    "component_key": key,
                    "foreign_publishers": foreign,
                    "detail": f"检测到外部未登记的 {'/'.join(foreign)} 仍在发布 /detection_info；"
                              "无法证明所有权故不停止。请操作者关闭其终端后重试，"
                              "或使用统一入口启动以便受控切换。",
                }

        argv = build_command(component, camera=camera, mode=mode, ros_args=ros_args)
        env = build_env(camera)
        if cancel_event is not None and cancel_event.is_set():
            return {"result": "cancelled", "ready": False, "component_key": key}
        log_file = _runtime_log_dir() / f"{key.replace(':', '_')}.log"
        log_offset = log_file.stat().st_size if log_file.exists() else 0
        record = manager.spawn(key, argv, initiator=initiator, mode=mode,
                               camera=camera if component in ("camera", "detector") else None,
                               log_file=log_file, env=env,
                               owner_session=owner_session)
        record["log_offset"] = log_offset
        manager.register(record)
        _log_runtime(key, f"ENSURE spawned initiator={initiator} pid={record['pid']}")
        res = {"result": "spawned", "component_key": key, "record": record}
        if wait_ready:
            res["ready"], res["ready_detail"] = _wait_background_ready(
                manager, record, component, camera, mode, cancel_event)
        return res


def _wait_background_ready(manager, record, component, camera, mode, cancel_event):
    deadline = time.monotonic() + _ready_timeout(component, mode)
    last = "就绪检查超时"
    while time.monotonic() < deadline:
        if cancel_event is not None and cancel_event.is_set():
            return False, "会话正在关闭"
        valid, reason = manager.verify_identity(record)
        if not valid:
            return False, f"进程提前退出或身份失效: {reason}"
        checked = check_ready(component, camera=camera, mode=mode, record=record)
        if checked["ready"]:
            return True, checked["detail"]
        last = checked["detail"]
        time.sleep(1.)
    return False, last


def _ready_timeout(component: str, mode: str) -> float:
    cfg = rc.load_runtime_config()
    table = {
        "camera": cfg["readiness"]["camera"]["timeout_s"],
        "camera_tf": cfg["readiness"]["camera_tf"]["timeout_s"],
        "detector:fruit": cfg["readiness"]["detector_fruit"]["timeout_s"],
        "detector:blue_block": cfg["readiness"]["detector_blue_block"]["timeout_s"],
        "graspnet": cfg["readiness"]["graspnet"]["timeout_s"],
    }
    if component == "detector":
        return table[f"detector:{mode}"]
    return table.get(component, 30.0)


def stop_component(component: str, *, camera: str = "d455", mode: str = "fruit",
                   initiator: str | None = None,
                   owner_session: str | None = None,
                   allow_cross_initiator: bool = False) -> dict:
    key = component_key(component, camera=camera, mode=mode)
    manager = _manager()
    with manager.transition_lock(key):
        res = manager.stop(key, require_owner=initiator,
                           owner_session=owner_session,
                           allow_cross_initiator=allow_cross_initiator)
    _log_runtime(key, f"STOP initiator={initiator} "
                      f"cross={allow_cross_initiator} result={res.get('result')}")
    return res


# ========================================================================
# dry-run 与状态
# ========================================================================
def dry_run(component: str, *, camera: str = "d455", mode: str = "fruit",
            ros_args=()) -> int:
    ws = rc.workspace_root()
    cfg = rc.load_runtime_config()
    key = component_key(component, camera=camera, mode=mode)
    print("=" * 64)
    print(f"DRY-RUN 组件: {component}  (键 {key})")
    print("=" * 64)
    preview_session = ws / 'data' / 'calibration' / camera / 'intrinsics' / '<new-session>'
    if component in LAUNCHABLE:
        argv = build_command(component, camera=camera, mode=mode, ros_args=ros_args,
                             session_dir=preview_session if component == 'calibrate-intrinsics' else None)
        print(f"解释器/入口: {argv[0]}")
        print(f"argv: {' '.join(argv)}")
        print(f"cwd: {ws}")
    print("关键环境:")
    print(f"  RMW_IMPLEMENTATION = {cfg['ros']['rmw_implementation']}")
    print(f"  CYCLONEDDS_URI     = file://{rc.resolve_resource('dds_xml')}")
    print(f"  ROS_DOMAIN_ID      = {os.environ.get('ROS_DOMAIN_ID', '(调用者值; 未设置则 ROS 默认 0)')}")
    if camera == "dabai":
        print(f"  Orbbec overlay     = {rc.resolve_resource('orbbec_setup')}")
    print(f"  workspace overlay  = {rc.resolve_resource('workspace_setup')}")
    print("关键配置:")
    if component == "detector":
        print(f"  params_file = {ws / 'config' / 'vision' / ('fruit.yaml' if mode == 'fruit' else 'blue_block.yaml')}")
    if component in ("camera_tf", "detector", "graspnet"):
        try:
            he = rc.load_handeye(camera)
            print(f"  活动外参 = {he['source']} (mode={he['mode']}, "
                  f"parent={he['parent']} → child={he['child']})")
        except rc.ConfigError as e:
            print(f"  活动外参 = 不可用: {e}")
    if component == "calibrate-handeye":
        print(f"  intrinsics_yaml = {ws / 'config' / 'calibration' / camera / 'intrinsics.yaml'}"
              " （不存在时真实运行将明确失败）")
        print(f"  result_save_path = {ws / 'data' / 'calibration' / camera / 'handeye'}")
    if component == "calibrate-intrinsics":
        print(f"  out_yaml(会话候选) = {preview_session / 'candidate.yaml'}"
              " （真实运行时按当前时间创建会话目录）")
    print("dry-run 不加载模型、不建输出目录、不启动组件、不访问 CAN/串口。")
    if component == "verify-tcp":
        sys.stdout.flush()
        env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
        return subprocess.run(argv + ["--dry-run"], cwd=str(ws), env=env, timeout=30).returncode
    return 0


def show_status(component: str | None = None, *, camera: str = "d455",
                mode: str = "fruit") -> int:
    manager = _manager()
    reg = manager.read_registry()
    keys = [component_key(component, camera=camera, mode=mode)] if component else sorted(reg)
    if not keys:
        print("无登记组件。")
    for key in keys:
        record = reg.get(key)
        print(f"── {key} ──")
        if record is None:
            print("  未登记")
            continue
        ok, reason = manager.verify_identity(record)
        print(f"  登记: pid={record.get('pid')} pgid={record.get('pgid')} "
              f"initiator={record.get('initiator')} owner={record.get('owner_session')} "
              f"mode={record.get('mode')} camera={record.get('camera')}")
        print(f"  身份校验: {'通过' if ok else '失效 - ' + reason}")
        print(f"  日志: {record.get('log_file')}")
    return 0


# ========================================================================
# CLI
# ========================================================================
def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Piper 工作空间统一启动入口",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("component", choices=COMPONENTS,
                        help="组件名；stop/status 后跟目标组件，如: stop camera --camera d455")
    parser.add_argument("target", nargs="?", default=None,
                        help="stop/status 的目标组件（arm/camera/camera_tf/detector/"
                             "graspnet/action/gui）")
    parser.add_argument("--camera", default="d455", choices=["d455", "dabai", "dual"])
    parser.add_argument("--mode", default="fruit", choices=["fruit", "blue_block"])
    parser.add_argument("--dry-run", action="store_true",
                        help="只输出解释器/argv/配置/关键环境，无任何副作用")
    parser.add_argument("--background", action="store_true",
                        help="后台确保方式启动（GUI 使用；登记但不前台监督）")
    parser.add_argument("--initiator", default=MANUAL, choices=[MANUAL, GUI])
    parser.add_argument("--owner-session", default=None,
                        help="GUI 会话标识（后台停止/清理按会话归属）")
    parser.add_argument("--allow-cross-initiator", action="store_true",
                        help="GUI 显式模式切换：允许停止同 workspace 登记的 manual 组件")
    parser.add_argument("--wait-ready", action="store_true",
                        help="后台启动后等待就绪（受 readiness 超时约束）")
    parser.add_argument("--live", action="store_true",
                        help="preflight 附加只读 live 接口查询")
    parser.add_argument("--calibration-file", default=None,
                        help="显式覆盖外参文件（离线/候选检查；camera_tf 透传）")
    parser.add_argument("--candidate", default=None,
                        help="approve-*/verify-tcp: 候选文件绝对路径")
    parser.add_argument("--scene-verified", action="store_true",
                        help="approve-handeye: 已由操作者完成现场坐标核查（必须显式）")
    parser.add_argument("ros_args", nargs="*",
                        help="额外 ROS 参数，原样追加（如 key:=value）")
    raw_args = list(sys.argv[1:] if argv is None else argv)
    passthrough = []
    for marker in ("--ros-args", "--"):
        if marker in raw_args:
            index = raw_args.index(marker)
            passthrough = raw_args[index:] if marker == "--ros-args" else raw_args[index + 1:]
            raw_args = raw_args[:index]
            break
    args = parser.parse_args(raw_args)

    ros_args = list(args.ros_args) + passthrough
    if args.calibration_file:
        ros_args.append(f"calibration_file:={args.calibration_file}")
    if args.component in ("approve-intrinsics", "approve-handeye"):
        if not args.candidate:
            print("[launcher][ERROR] approve-* 需要 --candidate <绝对路径>",
                  file=sys.stderr)
            return 2
        ros_args += ["--candidate", os.path.expanduser(args.candidate)]
        if args.scene_verified:
            ros_args.append("--scene-verified")

    if args.component == "verify-tcp":
        if args.background:
            print("[launcher][ERROR] verify-tcp 仅允许前台交互运行", file=sys.stderr)
            return 2
        if args.candidate:
            ros_args += ["--candidate", os.path.expanduser(args.candidate)]
    try:
        if args.dry_run:
            return dry_run(args.component, camera=args.camera, mode=args.mode,
                           ros_args=ros_args)
        if args.component == "preflight":
            report = preflight(camera=args.camera, mode=args.mode, live=args.live)
            print(json.dumps(report, ensure_ascii=False, indent=2))
            return 0 if report["ok"] else 1
        if args.component == "stop":
            if args.target not in STOPPABLE:
                print(f"[launcher][ERROR] stop 需要目标组件: {'/'.join(STOPPABLE)}",
                      file=sys.stderr)
                return 2
            res = stop_component(args.target, camera=args.camera, mode=args.mode,
                                 initiator=args.initiator,
                                 owner_session=args.owner_session,
                                 allow_cross_initiator=args.allow_cross_initiator)
            print(json.dumps(res, ensure_ascii=False, indent=2))
            return 0 if res.get("result") in ("stopped", "not_registered",
                                              "stale_record_removed",
                                              "already_exited") else 1
        if args.component == "status":
            return show_status(args.target, camera=args.camera, mode=args.mode)
        if args.background:
            res = ensure_background(args.component, camera=args.camera,
                                    mode=args.mode, initiator=args.initiator,
                                    owner_session=args.owner_session,
                                    ros_args=ros_args, wait_ready=args.wait_ready)
            print(json.dumps(res, ensure_ascii=False, indent=2, default=str))
            successful = res.get("result") in ("spawned", "reused", "reused_external")
            return 0 if successful and (not args.wait_ready or res.get("ready")) else 1
        return run_foreground(args.component, camera=args.camera, mode=args.mode,
                              ros_args=ros_args)
    except (LauncherError, rc.ConfigError, ProcessError) as e:
        print(f"[launcher][ERROR] {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
