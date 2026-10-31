"""Digit identity and position freshness; generated figures, not field accuracy."""

from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock
import cv2
import numpy as np
import pytest
from pi_vision.position_digits import DigitReader, NumberedRingDetector
from pi_vision.types import Ring
from pi_vision.vision_backend import VisionBackend
from pi_vision.vision_ros import parse_args
from pi_vision.config import default_config
from pi_vision.pipeline import VisionPipeline


def figure(digit='1', rotation=0):
    image = np.full((240, 240, 3), 255, np.uint8)
    (w, h), _ = cv2.getTextSize(digit, cv2.FONT_HERSHEY_SIMPLEX, 1.5, 3)
    cv2.putText(image, digit, (120-w//2, 120+h//2), cv2.FONT_HERSHEY_SIMPLEX,
                1.5, (0, 0, 0), 3)
    return np.rot90(image, rotation//90).copy()


@pytest.mark.parametrize('digit', [1, 2, 3])
@pytest.mark.parametrize('orientation', [0, 90, 180, 270])
def test_direct_digit_at_explicit_orientation(digit, orientation):
    reader = DigitReader(orientation)
    assert reader.read(figure(str(digit), orientation), Ring(1, 120, 120, 90, 80))[0] == digit


@pytest.mark.parametrize('digit', ['', '4', '7', 'X'])
def test_blank_or_unrecognized_symbol_is_not_numbered(digit):
    assert DigitReader(0).read(figure(digit), Ring(1, 120, 120, 90, 80))[0] is None


def test_crop_truncation_and_multi_components_are_uncertain():
    reader = DigitReader(0)
    assert reader.read(figure(), Ring(1, 5, 5, 90, 80))[0] is None
    image = figure()
    cv2.rectangle(image, (90, 92), (100, 136), (0, 0, 0), -1)
    assert reader.read(image, Ring(1, 120, 120, 90, 80))[0] is None


def test_digit_ids_are_independent_of_left_right_order_and_legacy_ring_no():
    image = np.concatenate([figure('3'), figure('1'), figure('2')], axis=1)
    detector = SimpleNamespace(detect=lambda _: [Ring(1, x, 120, 90, 80) for x in (120, 360, 600)])
    results = NumberedRingDetector(detector, 0).detect(image)
    assert [r.position_id for r in results] == [3, 1, 2]
    assert [r.ring_no for r in results] == [1, 1, 1]


def test_duplicate_digits_do_not_get_inferred_alternate_ids():
    image = np.concatenate([figure('1'), figure('1')], axis=1)
    detector = SimpleNamespace(detect=lambda _: [Ring(1, x, 120, 90, 80) for x in (120, 360)])
    results = NumberedRingDetector(detector, 0).detect(image)
    assert all(r.position_id is None and r.position_status == 'duplicate_digit_conflict' for r in results)


def test_pdf_plan_layout_three_concentric_targets_and_independent_digit_rotations():
    image = np.full((720, 1280, 3), 255, np.uint8)
    expected = [(3, 180), (2, 180), (1, 0)]
    centers = [(250, 360), (640, 360), (1030, 360)]
    for (x, y), (digit, rotation) in zip(centers, expected):
        for radius in (45, 55, 65, 75, 85):
            for start in range(0, 360, 45):
                cv2.ellipse(image, (x, y), (radius, radius), 0, start, start+32, (0, 0, 0), 2)
        patch = figure(str(digit), rotation)
        image[y-40:y+40, x-40:x+40] = patch[80:160, 80:160]
    detector = VisionPipeline(default_config(), material_backend='hsv').ring_detector
    results = NumberedRingDetector(detector, 0).detect(image)
    assert [(r.position_id, r.center_x, r.center_y) for r in results] == [
        (3, 250, 360), (2, 640, 360), (1, 1030, 360)]
    assert [r.digit_orientation_deg for r in results] == [180, 180, 0]


@pytest.mark.parametrize('options', [['--position-digits'], ['--position-orientation', '0']])
def test_enable_requires_explicit_orientation(options):
    with pytest.raises(SystemExit):
        parse_args(['--camera', 'not-opened', '--material-backend', 'hsv']+options)


@pytest.fixture
def adapter():
    now = [100.]
    source = Mock()
    b = VisionBackend(source, 'camera', wall_clock=lambda: now[0], clock=lambda: now[0])
    ring = dict(ring_no=1, position_id=2, center_x=500, center_y=300, position_status='candidate',
                point_kind='ring_candidate_center', digit_score=.9, digit_margin=.2,
                digit_orientation_deg=180, radius=70.)
    state = dict(position_identity_enabled=True, session_id='session', hsv_profile_sha256='hash',
                 preview=dict(source_width=1280, source_height=720), rings=[ring],
                 ring_observation=dict(valid=True, status='ok', source_sequence=1, acquired_unix_s=100.))
    source.get_state.return_value = state
    cmd = dict(schema_version=1, simulation=False, run_id='run', command_id='request',
               operation='locate_position', params=dict(zone='rough', position_id=2, layer=0),
               issued_at_unix_s=100., timeout_s=5.)
    return b, source, state, now, cmd


def test_position_needs_three_fresh_frames_and_minimum_span(adapter):
    b, source, state, now, cmd = adapter
    assert b.command(cmd)[0]['status'] == 'accepted'
    source.set_scan_detection.assert_called_with(2)
    for i in range(3):
        now[0] = 100.+i*.15
        state['ring_observation'].update(source_sequence=i+1, acquired_unix_s=now[0])
        data = b.poll()[0]['data']
        assert data['status'] == ('ok' if i == 2 else 'no_target')
        assert b.poll() == []
    assert data['targets'][0]['position_id'] == 2
    assert data['targets'][0]['point_kind'] == 'ring_candidate_center'
    assert data['targets'][0]['digit_orientation_deg'] == 180
    assert data['metric_coordinates_valid'] is False
    assert data['coordinate_semantics'] == 'concentric_ring_center_candidate_not_grasp_point'


def test_occlusion_clears_identity_instead_of_reusing_a_previous_label(adapter):
    b, _, state, now, cmd = adapter
    b.command(cmd)
    for i in range(3):
        now[0] = 100.+i*.15
        state['ring_observation'].update(source_sequence=i+1, acquired_unix_s=now[0])
        b.poll()
    now[0] += .15
    state['rings'][0]['position_id'] = None
    state['ring_observation'].update(source_sequence=4, acquired_unix_s=now[0])
    assert b.poll()[0]['data']['targets'] == []
    now[0] += .15
    state['rings'][0]['position_id'] = 2
    state['ring_observation'].update(source_sequence=5, acquired_unix_s=now[0])
    assert b.poll()[0]['data']['position_reason'] == 'position_unconfirmed'


def test_scoring_ring_number_never_becomes_position_id(adapter):
    b, _, state, _, cmd = adapter
    state['rings'][0]['position_id'] = None
    state['rings'][0]['ring_no'] = 2
    b.command(cmd)
    assert b.poll()[0]['data']['targets'] == []


def test_stack_base_not_inferred_from_requested_color(adapter):
    b, _, _, _, cmd = adapter
    cmd['params'].update(layer=1, color_id=5)
    assert b.command(cmd)[0]['data']['reason'] == 'stack_base_verification_not_implemented'
    assert b.pending is None


def test_old_request_and_stale_frames_cannot_confirm_position(adapter):
    b, _, state, now, cmd = adapter
    b.command(cmd)
    state['ring_observation']['acquired_unix_s'] = 99.
    assert b.poll() == []
    state['ring_observation'].update(valid=False, status='stale', acquired_unix_s=100.)
    assert b.poll() == []
    now[0] = 106.
    assert b.poll()[0]['status'] == 'failed'
    assert b.pending is None
