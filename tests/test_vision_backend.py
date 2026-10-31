"""Correlation, freshness and headless acquisition regression; no physical devices."""

from copy import deepcopy
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest

from pi_vision.vision_backend import VisionBackend
from pi_vision.stream import FrameStreamer
from pi_vision.mission import TaskCoordinator


def command(operation='scan_material', command_id='1', **params):
    return dict(schema_version=1, simulation=False, run_id='run', command_id=command_id,
                operation=operation, params=params or dict(zone='raw', color_id=1),
                issued_at_unix_s=100., timeout_s=10.)


@pytest.fixture
def setup_backend():
    source = Mock()
    now = [100., 10.]
    backend = VisionBackend(source, 'camera', wall_clock=lambda: now[0], clock=lambda: now[1])
    state = dict(session_id='camera_session', hsv_profile_sha256='hash',
                 preview=dict(source_width=1280, source_height=720),
                 materials=[dict(color_id=1, center_x=510, center_y=200, confidence=90, track_id=7,
                                 tracking_status='updated')],
                 qr_candidate='156+123+516+231', task={'text': 'old_latched_task'})
    obs = dict(status='ok', valid=True, source_sequence=1, acquired_unix_s=100.)
    state['material_observation'] = deepcopy(obs)
    state['qr_observation'] = deepcopy(obs)
    source.get_state.return_value = state
    return backend, source, now, state


def test_material_output_full_image_and_duplicate_suppression(setup_backend):
    backend, source, _, state = setup_backend
    assert backend.command(command())[0]['status'] == 'accepted'
    reply = backend.poll()[0]
    assert reply['command_id'] == '1'
    assert reply['data']['targets'] == [dict(u=510, v=200, color_id=1, confidence=90,
                                              track_id=7, tracking_status='updated',
                                              point_kind='contour_bounding_box_center')]
    assert reply['data']['coordinate_semantics'] == 'full_frame_contour_box_center_not_grasp_point'
    assert reply['data']['units'] == 'pixels'
    assert reply['data']['pixel_origin'] == 'top_left'
    assert reply['data']['metric_coordinates_valid'] is False
    assert reply['data']['calibration_required'] is True
    assert reply['data']['width'] == 1280
    assert backend.poll() == []
    state['session_id'] = 'new_camera_session'
    assert backend.poll()[0]['data']['source_session_id'] == 'new_camera_session'


def test_empty_and_ambiguous_results_remain_honest(setup_backend):
    backend, _, _, state = setup_backend
    backend.command(command())
    state['materials'] = [dict(color_id=3, center_x=40, center_y=20)]
    assert backend.poll()[0]['data']['status'] == 'no_target'
    state['material_observation']['source_sequence'] += 1
    state['materials'] = [dict(color_id=1, center_x=40, center_y=20)] * 2
    assert len(backend.poll()[0]['data']['targets']) == 2


@pytest.mark.parametrize('change', [dict(simulation=True), dict(schema_version=True),
    dict(issued_at_unix_s=101.), dict(issued_at_unix_s=89.), dict(timeout_s=float('nan')),
    dict(timeout_s=True), dict(run_id=''), dict(params=[]), dict(command_id=None)])
def test_bad_requests_do_not_change_camera_mode(setup_backend, change):
    backend, source, _, _ = setup_backend
    assert backend.command(command() | change) == []
    source.set_scan_detection.assert_not_called()


def test_replayed_command_cannot_reset_confirmation(setup_backend):
    backend, source, _, _ = setup_backend
    cmd = command('read_task')
    backend.command(cmd)
    source.reset_mock()
    assert backend.command(cmd) == []
    source.set_scan_detection.assert_not_called()


def test_stop_cancels_active_request_and_never_claims_robot_stopped(setup_backend):
    backend, source, _, _ = setup_backend
    backend.command(command())
    assert backend.command(command('stop', 'stop_request')) == []
    assert backend.pending is None
    assert backend.poll() == []
    source.set_scan_detection.assert_called_with(None)
    # A delayed duplicate request cannot restart acquisition after stop.
    source.reset_mock()
    assert backend.command(command()) == []
    source.set_scan_detection.assert_not_called()


def test_older_source_sequence_is_rejected(setup_backend):
    backend, _, _, state = setup_backend
    backend.command(command())
    state['material_observation']['source_sequence'] = 5
    assert backend.poll()
    state['material_observation']['source_sequence'] = 4
    assert backend.poll() == []


def test_prior_request_frame_and_invalid_result_are_not_forwarded(setup_backend):
    backend, _, _, state = setup_backend
    backend.command(command())
    obs = state['material_observation']
    obs['acquired_unix_s'] = 99.9
    assert backend.poll() == []
    obs.update(acquired_unix_s=100., valid=False, status='stale')
    assert backend.poll() == []
    obs.update(valid=True, status='ok')
    assert backend.poll()[0]['data']['valid'] is True


@pytest.mark.parametrize('status', ['camera_error', 'camera_stale', 'error', 'timeout'])
def test_fault_clears_request_without_coordinates(setup_backend, status):
    backend, source, _, state = setup_backend
    backend.command(command())
    state['material_observation'].update(valid=False, status=status)
    reply = backend.poll()[0]
    assert reply['status'] == 'failed'
    assert 'targets' not in reply['data']
    assert backend.pending is None
    source.set_scan_detection.assert_called_with(None)


def test_timeout_uses_remaining_budget_and_stop_is_not_robot_ack(setup_backend):
    backend, _, now, _ = setup_backend
    now[0] = 108.
    backend.command(command())
    now[1] = 12.
    assert backend.poll()[0]['data']['reason'] == 'vision_request_timeout'
    now[0] = 109.
    assert backend.command(command('stop', '3') | {'issued_at_unix_s': 109.}) == []
    assert backend.pending is None


@pytest.mark.parametrize('cmd', [command('locate_position'),
    command(zone='rough', color_id=1, position_id=1), command(color_id=7, zone='raw')])
def test_unimplemented_position_or_color_cannot_succeed(setup_backend, cmd):
    backend, _, _, _ = setup_backend
    assert backend.command(cmd)[0]['status'] == 'failed'
    assert backend.poll() == []


def test_qr_candidates_need_new_frames_and_no_target_resets_task_confirmation(setup_backend):
    backend, _, now, state = setup_backend
    engine = TaskCoordinator(wall_clock=lambda: now[0], clock=lambda: now[1])
    engine.start()
    cmd = engine.drain_commands()[0]
    backend.command(cmd)
    obs = state['qr_observation']
    for i, raw in enumerate(['156+123+516+231', None, '156+123+516+231',
                             '156+123+516+231', '156+123+516+231']):
        now[0] = 100.+i*.15
        obs.update(source_sequence=i+1, acquired_unix_s=now[0], status='ok' if raw else 'no_target')
        state['qr_candidate'] = raw
        assert engine.feedback(backend.poll()[0]) is (i == 4)
        assert backend.poll() == []
    assert str(engine.task) == '156+123+516+231'
    assert engine.pending.operation == 'display_task'


def test_headless_capture_produces_raw_frames_without_encoding():
    camera = Mock()
    pipeline = SimpleNamespace(qr_camera=camera)
    source = FrameStreamer(camera, pipeline, enable_qr=False, enable_materials=False,
                           enable_rings=False, preview_enabled=False)
    def read():
        if source._raw_sequence >= 2:
            source._running = False
        return True, np.zeros((40, 60, 3), np.uint8), time.time()
    camera.read.side_effect = read
    source._running = True
    try:
        with patch('pi_vision.stream.cv2.imencode') as encode:
            source._capture_loop()
        encode.assert_not_called()
        assert source._raw_sequence == 3
        assert source._latest_raw.shape == (40, 60, 3)
        camera.release.assert_called_once()
    finally:
        source.stop()
