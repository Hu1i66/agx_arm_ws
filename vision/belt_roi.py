"""Green conveyor-region extraction shared by fixed and moving cameras."""
import json
import time

import cv2
import numpy as np


DEFAULT_LOWER_HSV = [65, 45, 35]
DEFAULT_UPPER_HSV = [100, 255, 225]


def build_belt_mask(image, lower_hsv=None, upper_hsv=None, min_area_ratio=.015,
                    min_row_fraction=.30):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = np.array(lower_hsv or DEFAULT_LOWER_HSV, dtype=np.uint8)
    upper = np.array(upper_hsv or DEFAULT_UPPER_HSV, dtype=np.uint8)
    raw = cv2.inRange(hsv, lower, upper)
    raw = cv2.morphologyEx(raw, cv2.MORPH_CLOSE,
                           cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25)))
    raw = cv2.morphologyEx(raw, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(raw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None, False
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < image.shape[0] * image.shape[1] * min_area_ratio:
        return None, None, False
    row_counts = np.count_nonzero(raw, axis=1)
    rows = np.flatnonzero(row_counts >= image.shape[1] * min_row_fraction)
    if rows.size:
        band = np.zeros_like(raw)
        band[rows.min():rows.max() + 1] = raw[rows.min():rows.max() + 1]
        points = cv2.findNonZero(band)
        hull = cv2.convexHull(points) if points is not None else cv2.convexHull(contour)
    else:
        hull = cv2.convexHull(contour)
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.fillPoly(mask, [hull], 255)
    return mask, hull.reshape(-1, 2).astype(int).tolist(), True


def calibration_payload(image, lower_hsv=None, upper_hsv=None):
    mask, polygon, valid = build_belt_mask(image, lower_hsv, upper_hsv)
    if not valid:
        raise ValueError('未找到足够大的绿色传送带区域')
    return {
        'schema_version': 1,
        'created_at': time.strftime('%Y-%m-%d %H:%M:%S'),
        'image_width': int(image.shape[1]),
        'image_height': int(image.shape[0]),
        'lower_hsv': list(lower_hsv or DEFAULT_LOWER_HSV),
        'upper_hsv': list(upper_hsv or DEFAULT_UPPER_HSV),
        'polygon': polygon,
        'area_pixels': int(np.count_nonzero(mask)),
    }


def polygon_mask(polygon, width, height, source_size=None):
    if not polygon:
        return None
    points = np.asarray(polygon, dtype=np.float32)
    if source_size:
        points[:, 0] *= width / float(source_size[0])
        points[:, 1] *= height / float(source_size[1])
    points = np.round(points).astype(np.int32).reshape(-1, 1, 2)
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(mask, [points], 255)
    return mask


def load_calibration(path):
    try:
        with open(path, encoding='utf-8') as stream:
            data = json.load(stream)
        if not data.get('polygon'):
            return None
        return data
    except (OSError, ValueError):
        return None


def filter_detections_in_mask(detections, mask):
    if mask is None:
        return [], len(detections)
    kept = []
    for detection in detections:
        center_u, center_v = (int(round(v)) for v in detection.center)
        if (0 <= center_u < mask.shape[1] and 0 <= center_v < mask.shape[0]
                and mask[center_v, center_u] > 0):
            kept.append(detection)
    return kept, len(detections) - len(kept)
