"""Small, dependency-light tracking helpers shared by the vision node."""
from collections import Counter, deque
from dataclasses import dataclass
import math

import cv2
import numpy as np


def iou_xyxy(a, b):
    x1 = max(a[0], b[0]); y1 = max(a[1], b[1])
    x2 = min(a[2], b[2]); y2 = min(a[3], b[3])
    inter = max(0., x2 - x1) * max(0., y2 - y1)
    area_a = max(0., a[2] - a[0]) * max(0., a[3] - a[1])
    area_b = max(0., b[2] - b[0]) * max(0., b[3] - b[1])
    return inter / max(area_a + area_b - inter, 1e-9)


def intersection_over_smaller(a, b):
    x1 = max(a[0], b[0]); y1 = max(a[1], b[1])
    x2 = min(a[2], b[2]); y2 = min(a[3], b[3])
    inter = max(0., x2 - x1) * max(0., y2 - y1)
    area_a = max(0., a[2] - a[0]) * max(0., a[3] - a[1])
    area_b = max(0., b[2] - b[0]) * max(0., b[3] - b[1])
    return inter / max(min(area_a, area_b), 1e-9)


@dataclass
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    class_id: int

    @property
    def bbox(self):
        return self.x1, self.y1, self.x2, self.y2

    @property
    def center(self):
        return (self.x1 + self.x2) / 2., (self.y1 + self.y2) / 2.


def deduplicate_detections(detections, iou_threshold=0.45,
                           containment_threshold=0.60):
    """Keep one highest-confidence box when classes disagree on the same fruit."""
    kept = []
    for candidate in sorted(detections, key=lambda d: d.confidence, reverse=True):
        duplicate = False
        for chosen in kept:
            if iou_xyxy(candidate.bbox, chosen.bbox) >= iou_threshold:
                duplicate = True; break
            if intersection_over_smaller(candidate.bbox, chosen.bbox) >= containment_threshold:
                duplicate = True; break
        if not duplicate:
            kept.append(candidate)
    return kept


def expand_detection(detection, ratio, width, height):
    dx = (detection.x2 - detection.x1) * ratio
    dy = (detection.y2 - detection.y1) * ratio
    return Detection(
        max(0., detection.x1 - dx), max(0., detection.y1 - dy),
        min(float(width), detection.x2 + dx), min(float(height), detection.y2 + dy),
        detection.confidence, detection.class_id)


def merge_fragment_detections(detections, max_gap_px=8.,
                              center_ratio=1.25, min_area_ratio=.03):
    """Fuse same-class boxes that cover different parts of one nearby object."""
    parent = list(range(len(detections)))

    def root(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def join(a, b):
        ra, rb = root(a), root(b)
        if ra != rb:
            parent[rb] = ra

    def gap(a, b):
        dx = max(a.x1, b.x1) - min(a.x2, b.x2)
        dy = max(a.y1, b.y1) - min(a.y2, b.y2)
        return max(0., dx), max(0., dy)

    for i, a in enumerate(detections):
        for j in range(i):
            b = detections[j]
            if a.class_id != b.class_id:
                continue
            area_a = max(0., (a.x2 - a.x1) * (a.y2 - a.y1))
            area_b = max(0., (b.x2 - b.x1) * (b.y2 - b.y1))
            if min(area_a, area_b) / max(area_a, area_b, 1.) < min_area_ratio:
                continue
            a_center = a.center; b_center = b.center
            center_inside = ((b.x1 <= a_center[0] <= b.x2 and b.y1 <= a_center[1] <= b.y2) or
                             (a.x1 <= b_center[0] <= a.x2 and a.y1 <= b_center[1] <= a.y2))
            if center_inside:
                join(i, j)
                continue
            gx, gy = gap(a, b)
            distance = math.hypot((a.x1 + a.x2 - b.x1 - b.x2) / 2.,
                                  (a.y1 + a.y2 - b.y1 - b.y2) / 2.)
            size = max(a.x2 - a.x1, a.y2 - a.y1, b.x2 - b.x1, b.y2 - b.y1)
            if math.hypot(gx, gy) <= max_gap_px and distance <= center_ratio * size:
                join(i, j)

    groups = {}
    for i, detection in enumerate(detections):
        groups.setdefault(root(i), []).append(detection)
    merged = []
    for group in groups.values():
        best = max(group, key=lambda d: d.confidence)
        merged.append(Detection(
            min(d.x1 for d in group), min(d.y1 for d in group),
            max(d.x2 for d in group), max(d.y2 for d in group),
            best.confidence, best.class_id))
    return sorted(merged, key=lambda d: d.confidence, reverse=True)


def nearest_cross_camera_match(position, candidates, max_distance=0.07):
    """Find a confident object from the other camera at the same base position."""
    if not position:
        return None
    best = None
    best_distance = max_distance
    for candidate in candidates:
        base = candidate.get('base_position_m')
        if not base or candidate.get('confidence', 0.) < .55:
            continue
        dx = float(base['x']) - float(position[0])
        dy = float(base['y']) - float(position[1])
        dz = float(base['z']) - float(position[2])
        distance = math.sqrt(dx * dx + dy * dy + .35 * dz * dz)
        if distance < best_distance:
            best_distance = distance
            best = candidate
    return best


def refine_bbox_grabcut(image, bbox, pad_ratio=.45, iterations=3):
    """Expand an under-sized fruit box to its visible color boundary."""
    height, width = image.shape[:2]
    x1, y1, x2, y2 = (int(round(v)) for v in bbox)
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(width, x2), min(height, y2)
    box_width, box_height = x2 - x1, y2 - y1
    if box_width < 8 or box_height < 8:
        return bbox
    pad = max(16, int(round(pad_ratio * max(box_width, box_height))))
    rx1, ry1 = max(0, x1 - pad), max(0, y1 - pad)
    rx2, ry2 = min(width, x2 + pad), min(height, y2 + pad)
    roi = image[ry1:ry2, rx1:rx2]
    if roi.size == 0:
        return bbox
    mask = np.full(roi.shape[:2], cv2.GC_PR_BGD, np.uint8)
    mask[4:max(4, roi.shape[0] - 4), 4:max(4, roi.shape[1] - 4)] = cv2.GC_PR_FGD
    mask[y1 - ry1:y2 - ry1, x1 - rx1:x2 - rx1] = cv2.GC_PR_FGD
    mask[:3, :] = mask[-3:, :] = cv2.GC_BGD
    mask[:, :3] = mask[:, -3:] = cv2.GC_BGD
    bgd = np.zeros((1, 65), np.float64)
    fgd = np.zeros((1, 65), np.float64)
    try:
        cv2.grabCut(roi, mask, None, bgd, fgd, iterations, cv2.GC_INIT_WITH_MASK)
    except cv2.error:
        return bbox
    foreground = np.where((mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD), 255, 0).astype(np.uint8)
    foreground = cv2.morphologyEx(foreground, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(foreground)
    candidates = [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= 20]
    if not candidates:
        return bbox
    center_x, center_y = (x1 + x2) / 2., (y1 + y2) / 2.
    component = min(
        candidates,
        key=lambda i: ((stats[i, cv2.CC_STAT_LEFT] + stats[i, cv2.CC_STAT_WIDTH] / 2. + rx1 - center_x) ** 2 +
                       (stats[i, cv2.CC_STAT_TOP] + stats[i, cv2.CC_STAT_HEIGHT] / 2. + ry1 - center_y) ** 2))
    nx1 = rx1 + stats[component, cv2.CC_STAT_LEFT]
    ny1 = ry1 + stats[component, cv2.CC_STAT_TOP]
    nx2 = nx1 + stats[component, cv2.CC_STAT_WIDTH]
    ny2 = ny1 + stats[component, cv2.CC_STAT_HEIGHT]
    old_area = max(1, box_width * box_height)
    new_area = max(1, (nx2 - nx1) * (ny2 - ny1))
    if not (.65 <= new_area / old_area <= 2.8):
        return bbox
    if nx2 - nx1 > box_width * 2.2 or ny2 - ny1 > box_height * 2.2:
        return bbox
    return float(nx1), float(ny1), float(nx2), float(ny2)


class ClassVoteStabilizer:
    """Temporal class vote with hysteresis so a single frame cannot relabel fruit."""
    def __init__(self, initial, window=15, switch_margin=1):
        self.labels = deque([initial], maxlen=window)
        self.stable = initial
        self.switch_margin = switch_margin

    def update(self, label):
        self.labels.append(label)
        counts = Counter(self.labels)
        challenger, votes = max(counts.items(), key=lambda item: (item[1], item[0] == self.stable))
        if challenger != self.stable and votes > counts.get(self.stable, 0) + self.switch_margin:
            self.stable = challenger
        return self.stable

    @property
    def stability(self):
        return Counter(self.labels)[self.stable] / max(len(self.labels), 1)


class ScalarKalmanCV:
    """Constant-velocity Kalman filter for one scalar coordinate."""
    def __init__(self, value, measurement_var=9., process_var=4.):
        self.x = np.array([float(value), 0.], dtype=float)
        self.P = np.diag([max(measurement_var, 1e-6), 100.])
        self.measurement_var = float(measurement_var)
        self.process_var = float(process_var)

    def update(self, value, dt=0.1):
        dt = float(np.clip(dt, 0.02, 0.5))
        F = np.array([[1., dt], [0., 1.]])
        H = np.array([[1., 0.]])
        q = self.process_var
        Q = q * np.array([[dt ** 4 / 4., dt ** 3 / 2.],
                          [dt ** 3 / 2., dt ** 2]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        innovation = float(value) - float((H @ self.x)[0])
        S = float((H @ self.P @ H.T)[0, 0]) + self.measurement_var
        K = (self.P @ H.T / S).reshape(2)
        self.x += K * innovation
        self.P = (np.eye(2) - K.reshape(2, 1) @ H) @ self.P
        return float(self.x[0])

    @property
    def value(self):
        return float(self.x[0])


class PointKalmanCV:
    """Independent constant-velocity filters for x, y and z coordinates."""
    def __init__(self, value, measurement_var=4e-4, process_var=2e-3):
        self.filters = [ScalarKalmanCV(v, measurement_var, process_var) for v in value]

    def update(self, value, dt=0.1):
        return [f.update(v, dt) for f, v in zip(self.filters, value)]
