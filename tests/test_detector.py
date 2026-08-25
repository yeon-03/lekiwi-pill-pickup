from lekiwi_pill_pickup.detector import Detection, best_detection, parse_detections


def test_parse_detections_builds_detection_list():
    dets = parse_detections(
        boxes_xyxy=[(10, 20, 30, 60)], confidences=[0.9],
        class_names=['bottle'], class_ids=[0])
    assert len(dets) == 1
    assert dets[0].label == 'bottle'
    assert dets[0].confidence == 0.9


def test_detection_center_and_height_properties():
    d = Detection(label='bottle', confidence=0.9, x1=10, y1=20, x2=30, y2=60)
    assert d.center_x == 20
    assert d.center_y == 40
    assert d.height_px == 40


def test_best_detection_filters_by_label_and_confidence():
    dets = [
        Detection(label='bottle', confidence=0.9, x1=0, y1=0, x2=10, y2=10),
        Detection(label='cup', confidence=0.95, x1=0, y1=0, x2=10, y2=10),
        Detection(label='bottle', confidence=0.3, x1=0, y1=0, x2=10, y2=10),
    ]
    best = best_detection(dets, target_label='bottle', min_confidence=0.5)
    assert best is not None
    assert best.confidence == 0.9


def test_best_detection_returns_none_when_no_match():
    dets = [Detection(label='cup', confidence=0.9, x1=0, y1=0, x2=10, y2=10)]
    assert best_detection(dets, target_label='bottle', min_confidence=0.5) is None


def test_best_detection_picks_highest_confidence_among_multiple():
    dets = [
        Detection(label='bottle', confidence=0.6, x1=0, y1=0, x2=10, y2=10),
        Detection(label='bottle', confidence=0.85, x1=0, y1=0, x2=10, y2=10),
    ]
    best = best_detection(dets, target_label='bottle', min_confidence=0.5)
    assert best.confidence == 0.85
