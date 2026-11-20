"""Reference pipeline: coordinate semantics, ROI restoration and fail-closed geometry."""
from dataclasses import replace
from unittest.mock import Mock
import cv2
import numpy as np
import pytest
from pi_vision.detectors.hsv_material_detector import HSVMaterialDetector
from pi_vision.detectors.morphology_ring_detector import MorphologyRingDetector, RingRegionDetector
from pi_vision.material_region import MaterialRegionDetector
from pi_vision.position_digits import NumberedRingDetector
from pi_vision.vision_ros import parse_args
from pi_vision.vision_backend import VisionBackend


def rings():
    image = np.full((300, 650, 3), 255, np.uint8)
    for x, digit in ((140, 1), (490, 3)):
        for r in (45, 54, 63, 72, 81):
            for start in range(0, 360, 40):
                cv2.ellipse(image, (x, 150), (r, r), 0, start, start+31, (0, 0, 0), 2)
        cv2.putText(image, str(digit), (x-12, 165), cv2.FONT_HERSHEY_SIMPLEX,
                    1.3, (0, 0, 0), 3)
    return image


def test_material_centroid_changes_only_geometry_and_restores_roi():
    image = np.full((240, 300, 3), 255, np.uint8)
    cv2.fillPoly(image, [np.array([(60, 30), (230, 100), (65, 210)])], (0, 0, 220))
    masks_a, masks_b = {}, {}
    box = HSVMaterialDetector().detect(image, diagnostics=masks_a)
    centroid = HSVMaterialDetector(center_method='centroid').detect(image, diagnostics=masks_b)
    assert len(box) == len(centroid) == 1
    a, b = box[0], centroid[0]
    assert a.color_id == b.color_id == 1
    assert a.confidence == b.confidence
    assert a.center_x-b.center_x > 15
    assert b.point_kind == 'contour_area_centroid'
    assert all(np.array_equal(masks_a[key], masks_b[key]) for key in masks_a)
    full = np.full((400, 500, 3), 255, np.uint8)
    full[70:310, 90:390] = image
    found = MaterialRegionDetector(HSVMaterialDetector(center_method='centroid'), (90, 70, 300, 240)).detect(full)
    assert found[0] == replace(b, center_x=b.center_x+90, center_y=b.center_y+70)


def test_morphology_localizes_numbered_dashed_targets_and_returns_stages():
    stages = {}
    detector = MorphologyRingDetector()
    base = detector.detect(rings(), diagnostics=stages)
    assert set(stages) == {'binary', 'dilated', 'blurred', 'eroded'}
    assert len(base) == 2
    assert all(r.radial_support_layers >= 3 for r in base)
    results = NumberedRingDetector(detector, 0).detect(rings())
    assert [(r.position_id, r.center_x, r.center_y) for r in results] == [(1, 140, 150), (3, 490, 150)]
    assert all(r.center_method == 'morphology_external_contour_centroid' for r in results)


def test_ring_roi_restores_global_coordinates_before_digit_reading():
    image = np.full((500, 900, 3), 255, np.uint8)
    image[80:380, 100:750] = rings()
    detector = RingRegionDetector(MorphologyRingDetector(), (100, 80, 650, 300))
    found = NumberedRingDetector(detector, 0).detect(image)
    assert [(r.position_id, r.center_x, r.center_y) for r in found] == [(1, 240, 230), (3, 590, 230)]


@pytest.mark.parametrize('kind', ['blank', 'disk', 'one_ring', 'rectangle', 'clipped'])
def test_morphology_rejects_non_concentric_or_clipped_shapes(kind):
    image = np.full((300, 400, 3), 255, np.uint8)
    if kind == 'disk':
        cv2.circle(image, (200, 150), 70, (0, 0, 0), -1)
    elif kind == 'one_ring':
        cv2.circle(image, (200, 150), 70, (0, 0, 0), 3)
    elif kind == 'rectangle':
        cv2.rectangle(image, (90, 90), (300, 210), (0, 0, 0), -1)
    elif kind == 'clipped':
        for r in (45, 54, 63, 72, 81):
            cv2.circle(image, (20, 150), r, (0, 0, 0), 2)
    assert MorphologyRingDetector().detect(image) == []


def test_reference_backend_requires_explicit_roi():
    with pytest.raises(SystemExit):
        parse_args(['--camera', 'not-opened', '--material-backend', 'hsv', '--ring-backend', 'morphology'])
    args, _ = parse_args(['--camera', 'not-opened', '--material-backend', 'hsv', '--ring-backend', 'morphology',
                         '--ring-roi', '0', '0', '400', '300', '--material-center', 'centroid'])
    assert args.material_center == 'centroid'


def test_ros_preserves_centroid_definition_for_calibration_validation():
    source = Mock()
    source.get_state.return_value = dict(session_id='s', hsv_profile_sha256='hash',
        material_observation=dict(valid=True, status='ok', source_sequence=1, acquired_unix_s=100.),
        preview=dict(source_width=400, source_height=300),
        materials=[dict(color_id=1, center_x=123, center_y=90, point_kind='contour_area_centroid')])
    backend = VisionBackend(source, 'cam', wall_clock=lambda: 100., clock=lambda: 10.)
    backend.command(dict(schema_version=1, simulation=False, run_id='r', command_id='c',
        operation='scan_material', params=dict(zone='raw', color_id=1), issued_at_unix_s=100., timeout_s=3.))
    data = backend.poll()[0]['data']
    assert data['targets'][0]['point_kind'] == 'contour_area_centroid'
    assert data['metric_coordinates_valid'] is False
    assert 'box_center' not in data['coordinate_semantics']


def test_old_diagnostics_replay_ignores_session_tracks_but_preserves_point_kind(tmp_path):
    import json
    from dataclasses import asdict
    from pi_vision.hsv_profile import profile_hash
    from tools.hsv_replay import replay
    image = np.full((200, 250, 3), 255, np.uint8)
    cv2.circle(image, (100, 100), 40, (0, 0, 220), -1)
    detector = HSVMaterialDetector()
    recorded = [asdict(m) for m in detector.detect(image)]
    for item in recorded:
        item.pop('point_kind')  # Historic schema predates explicit point definition.
        item['track_id'] = 23
    cv2.imwrite(str(tmp_path/'sample.png'), image)
    evidence = dict(backend='HSVMaterialDetector', valid=True, profile=detector.profile,
                    profile_sha256=profile_hash(detector.profile), image='sample.png',
                    frame_size=[250, 200], roi=None, materials=recorded, source_sequence=1)
    (tmp_path/'sample_1.json').write_text(json.dumps(evidence))
    assert replay(tmp_path)['baseline_exact_matches'] == 1
    evidence['materials'][0]['center_x'] += 10
    (tmp_path/'sample_1.json').write_text(json.dumps(evidence))
    assert replay(tmp_path)['baseline_exact_matches'] == 0
