"""Pure validation shared by acquisition/load and promotion; no ROS or hardware."""
import math

try:
    from .axxb import CONSISTENCY_MAX_DEG, MEDIAN_REL_ANGLE_MIN_DEG, MIN_USABLE_PAIRS, HOLDOUT_MAX_MM
except ImportError:
    from axxb import CONSISTENCY_MAX_DEG, MEDIAN_REL_ANGLE_MIN_DEG, MIN_USABLE_PAIRS, HOLDOUT_MAX_MM

MIN_HANDEYE_SQUARE_PX = 25.0


class CalibrationValidationError(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise CalibrationValidationError(message)


def number(value, name):
    require(not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value), f"{name} 必须为有限数字")
    return float(value)


def vector(values, length, name):
    require(isinstance(values, (list, tuple)) and len(values) == length,
            f"{name} 必须包含 {length} 个数值")
    return [number(v, name) for v in values]


def positive_int(value, name, minimum=1):
    require(type(value) is int and value >= minimum, f"{name} 必须为整数且 >= {minimum}")
    return value


def camera_optical_frame(camera):
    require(camera in ("d455", "dabai"), f"未知相机 {camera}")
    return "camera_color_optical_frame" if camera == "d455" else "dabai_color_optical_frame"


def validate_board(board):
    require(isinstance(board, dict), "缺少板信息")
    sizes = board.get("squares")
    require(isinstance(sizes, list) and len(sizes) == 2, "板 squares 非法")
    for value in sizes:
        positive_int(value, "板方格数", 2)
    square = number(board.get("square_m"), "板方格边长")
    marker = number(board.get("marker_m"), "板标记边长")
    require(0 < marker < square, "板标记边长应小于正方格边长")
    require(isinstance(board.get("dict"), str) and board["dict"].startswith("DICT_"),
            "板字典缺失")


def quality_metadata(meta, kind, camera):
    require(isinstance(meta, dict) and meta.get("kind") == kind, "候选元数据 kind 非法")
    require(meta.get("camera_id") == camera, "候选相机与请求不一致")
    require(meta.get("quality_status") == "passed", "候选质量未通过")
    checks = meta.get("quality_checks")
    require(isinstance(checks, list) and checks and all(
        isinstance(c, dict) and c.get("ok") is True for c in checks),
        "候选质量检查缺失或不全部通过")
    resolution = meta.get("resolution")
    require(isinstance(resolution, list) and len(resolution) == 2, "候选分辨率缺失")
    for value in resolution:
        positive_int(value, "分辨率")
    validate_board(meta.get("board"))
    return resolution


def validate_intrinsics(data, camera, *, approved=False):
    require(isinstance(data, dict), "内参顶层必须为对象")
    meta = data.get("candidate")
    resolution = quality_metadata(meta, "intrinsics_candidate", camera)
    if approved:
        require(meta.get("approved") is True, "内参文件未批准")
    require(meta.get("source"), "内参来源缺失")
    frames = positive_int(meta.get("valid_frames"), "有效帧数", 15)
    require(data.get("frames") == frames, "内参有效帧数与数据不一致")
    width = positive_int(data.get("image_width"), "image_width")
    height = positive_int(data.get("image_height"), "image_height")
    require(resolution == [width, height], "内参分辨率不一致")
    rms = number(data.get("reprojection_error"), "重投影RMS")
    require(0 <= rms < .5, "内参重投影RMS未通过 <0.5px")
    cells = meta.get("coverage_cells")
    require(isinstance(cells, list) and all(type(c) is int and 0 <= c < 9 for c in cells)
            and {0, 2, 4, 6, 8}.issubset(set(cells)), "内参未覆盖中心和四角")
    matrix = data.get("camera_matrix")
    require(isinstance(matrix, dict) and matrix.get("rows") == 3
            and matrix.get("cols") == 3, "内参矩阵必须3×3")
    k = vector(matrix.get("data"), 9, "内参K")
    require(k[0] > 0 and k[4] > 0 and abs(k[6]) < 1e-9
            and abs(k[7]) < 1e-9 and abs(k[8] - 1) < 1e-9, "内参K结构非法")
    distortion = data.get("distortion_coefficients")
    require(isinstance(distortion, dict) and distortion.get("rows") == 1, "畸变数组非法")
    count = distortion.get("cols")
    require(type(count) is int and count in (4, 5, 8, 12, 14), "畸变数组长度非法")
    vector(distortion.get("data"), count, "畸变系数")
    return meta


def validate_capture_frames(rows, count):
    """Fixed promotion gates, independent of tunable acquisition settings."""
    require(isinstance(rows, list) and len(rows) == count, "逐帧质量记录缺失或数量不一致")
    for i, row in enumerate(rows):
        require(isinstance(row, dict), f"第{i+1}帧质量记录非法")
        positive_int(row.get("markers"), f"第{i+1}帧markers", 14)
        positive_int(row.get("corners"), f"第{i+1}帧corners", 20)
        require(0 <= number(row.get("reproj_rms_px"), "逐帧重投影RMS") < .5,
                f"第{i+1}帧重投影RMS未通过 <0.5px")
        require(number(row.get("square_px"), "逐帧方格像素") >= MIN_HANDEYE_SQUARE_PX,
                f"第{i+1}帧方格像素不足{MIN_HANDEYE_SQUARE_PX:g}px")
    return rows


def validate_handeye(data, camera):
    require(isinstance(data, dict), "手眼顶层必须为对象")
    meta = data.get("metadata")
    quality_metadata(meta, "handeye_candidate", camera)
    mode = "eye_to_hand" if camera == "d455" else "eye_in_hand"
    require(meta.get("mode") == mode, "手眼模式不匹配")
    position = vector(data.get("position"), 3, "position")
    quaternion = vector(data.get("orientation"), 4, "orientation")
    require(math.sqrt(sum(q*q for q in quaternion)) > 1e-9, "手眼四元数为零")
    frames = positive_int(meta.get("frames_used"), "有效姿态数", 15)
    require(meta.get("session_dir") and meta.get("intrinsics_source"), "采集/内参来源缺失")
    criteria = data.get("criteria")
    require(isinstance(criteria, dict), "手眼实际判据缺失")
    require(criteria.get("mode") == mode and criteria.get("frames_used") == frames,
            "手眼判据模式/帧数不一致")
    validate_capture_frames(criteria.get("capture_frames"), frames)
    require(0 <= number(criteria.get("consistency_deg"), "双算法一致性") < CONSISTENCY_MAX_DEG,
            "双算法一致性未通过")
    require(number(criteria.get("median_rel_angle_deg"), "相对转角") >= MEDIAN_REL_ANGLE_MIN_DEG,
            "相对转角不足")
    positive_int(criteria.get("n_pairs_ge25deg"), "可用运动对", MIN_USABLE_PAIRS)
    holdout = criteria.get("holdout")
    require(isinstance(holdout, dict), "未执行留出验证")
    positive_int(holdout.get("n"), "留出数量", 4)
    require(0 <= number(holdout.get("median_mm"), "留出误差") < HOLDOUT_MAX_MM,
            "留出误差未通过")
    for key in ("p95_mm", "max_mm"):
        require(number(holdout.get(key), key) >= 0, "留出报告非法")
    offset = vector(meta.get("tcp_offset"), 6, "tcp_offset")
    require(meta.get("tcp_offset_source"), "TCP offset 来源缺失")
    tool = "link6" if not any(offset) else "tcp_link"
    optical = camera_optical_frame(camera)
    require(meta.get("robot_base_frame") == "base_link"
            and meta.get("robot_pose_frame") == tool, "机器人位姿端点不一致")
    require(meta.get("camera_optical_frame") == optical
            and meta.get("transform_child") == optical, "相机光学端点不一致")
    require(meta.get("transform_parent") == ("base_link" if camera == "d455" else tool),
            "手眼变换父端点不一致")
    require(meta.get("units") == "m" and meta.get("quaternion_order") == "xyzw",
            "手眼单位/四元数顺序缺失或不匹配")
    return meta
