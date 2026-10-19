import math

import cv2
import numpy as np

from vision.tracker import (ClassVoteStabilizer, Detection, ScalarKalmanCV,
                            deduplicate_detections, expand_detection, iou_xyxy)
from vision.tracker import nearest_cross_camera_match
from vision.tracker import refine_bbox_grabcut
from vision.belt_roi import (build_belt_mask, calibration_payload,
                             filter_detections_in_mask, polygon_mask)


def test_cross_class_overlapping_boxes_are_deduplicated():
    high = Detection(10, 10, 110, 110, .72, 4)
    low = Detection(14, 12, 108, 112, .55, 0)
    separate = Detection(300, 300, 360, 360, .50, 7)
    kept = deduplicate_detections([low, separate, high])
    assert kept == [high, separate]
    assert iou_xyxy(high.bbox, low.bbox) > .8


def test_class_votes_need_more_than_one_frame_to_switch():
    votes = ClassVoteStabilizer('green_pear')
    assert votes.update('apple') == 'green_pear'
    assert votes.update('apple') == 'green_pear'
    assert votes.update('apple') == 'apple'
    assert 0. < votes.stability <= 1.


def test_kalman_reduces_jitter_instead_of_following_each_spike():
    kalman = ScalarKalmanCV(100., measurement_var=25., process_var=1.)
    filtered = [kalman.update(value, .1) for value in (100, 112, 100, 112)]
    assert all(math.isfinite(value) for value in filtered)
    assert max(filtered) - min(filtered) < 12


def test_dabai_box_expansion_is_clamped_to_image():
    box = Detection(2, 3, 38, 40, .8, 4)
    expanded = expand_detection(box, .1, 40, 42)
    assert expanded.x1 == 0 and expanded.y1 == 0
    assert expanded.x2 == 40 and expanded.y2 <= 42


def test_nearby_same_class_fragments_merge_into_one_box():
    from vision.tracker import merge_fragment_detections
    fragments = [
        Detection(258, 162, 345, 258, .28, 4),
        Detection(268, 254, 366, 356, .31, 4),
        Detection(367, 297, 419, 372, .25, 4),
    ]
    merged = merge_fragment_detections(fragments)
    assert len(merged) == 1
    assert merged[0].bbox == (258, 162, 419, 372)


def test_small_duplicate_inside_large_fruit_box_is_merged():
    from vision.tracker import merge_fragment_detections
    merged = merge_fragment_detections([
        Detection(100, 100, 180, 210, .84, 4),
        Detection(145, 145, 175, 190, .22, 4),
    ])
    assert len(merged) == 1
    assert merged[0].bbox == (100, 100, 180, 210)


def test_confident_cross_camera_class_matches_same_base_object():
    candidates = [{
        'object_name': 'orange', 'confidence': .87,
        'base_position_m': {'x': .42, 'y': -.22, 'z': 0.},
    }]
    assert nearest_cross_camera_match((.423, -.218, .002), candidates)['object_name'] == 'orange'


def test_grabcut_expands_bbox_to_full_visible_fruit():
    image = np.zeros((160, 180, 3), np.uint8)
    cv2.circle(image, (90, 75), 38, (120, 210, 160), -1)
    refined = refine_bbox_grabcut(image, (60, 70, 120, 110))
    assert refined[1] < 70 and refined[3] > 110
    assert refined[0] < 60 or refined[2] > 120


def test_green_conveyor_mask_fills_fruit_holes():
    image = np.zeros((180, 260, 3), np.uint8)
    image[30:160, 20:240] = (50, 220, 0)
    cv2.circle(image, (130, 95), 24, (20, 40, 220), -1)
    mask, polygon, valid = build_belt_mask(image)
    assert valid and polygon
    assert mask[130, 50] == 255 and mask[95, 130] == 255
    calibrated = calibration_payload(image)
    assert polygon_mask(calibrated['polygon'], 260, 180, (260, 180))[130, 50] == 255
    mask = np.zeros((180, 260), np.uint8);mask[30:160, 20:240] = 255
    kept, rejected = filter_detections_in_mask([
        Detection(40, 60, 80, 100, .8, 0),
        Detection(210, 150, 255, 175, .7, 0),
    ], mask)
    assert len(kept) == 1 and rejected == 1
