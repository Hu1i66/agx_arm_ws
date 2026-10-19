import math

from vision.tracker import (ClassVoteStabilizer, Detection, ScalarKalmanCV,
                            deduplicate_detections, iou_xyxy)


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
