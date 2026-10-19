"""Small, dependency-light tracking helpers shared by the vision node."""
from collections import Counter, deque
from dataclasses import dataclass
import math

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
