from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tools.qr_station_work import FreshQRSession, QRStationWork
from pi_vision.task_confirmation import parse_competition_task

CAMERA = '/dev/v4l/by-id/usb-icSpring_icspring_camera-video-index0'
CODE = parse_competition_task('156+123+516+231')


def ready_worker(now=10.0):
    worker = QRStationWork(dict(camera=CAMERA), clock=lambda: now)
    worker.process = Mock()
    worker.process.is_alive.return_value = True
    worker.heartbeat = SimpleNamespace(value=now - .1)
    worker.connection = Mock()
    worker.connection.poll.return_value = False
    return worker


def test_start_requires_live_prewarmed_frames_and_only_once():
    worker = QRStationWork(dict(camera=CAMERA))
    with pytest.raises(RuntimeError, match='not prewarmed'):
        worker.start()
    worker = ready_worker()
    worker.start()
    assert worker.connection.send.call_args.args[0] == dict(action='scan', started_at=10)
    with pytest.raises(RuntimeError, match='already started'):
        worker.start()


@pytest.mark.parametrize('age', [1, 2, -1])
def test_stale_or_future_prewarm_never_starts(age):
    worker = ready_worker()
    worker.heartbeat.value = 10-age
    with pytest.raises(RuntimeError, match='stale'):
        worker.start()
    worker.connection.send.assert_not_called()


def test_gate_discards_queued_and_inflight_frames_and_rejects_duplicate_sequence():
    session = FreshQRSession(10, 15)
    for sequence in range(1, 5):
        assert not session.accepts(sequence, 10.01, 10.02)
    assert not session.accepts(5, 9.99, 10.1)
    assert not session.accepts(5, 10.1, 10.2)
    assert session.accepts(6, 10.1, 10.2)
    assert session.confirmation.count == 0


def test_original_confirmation_requires_three_frames_and_point_two_seconds():
    session = FreshQRSession(10, 15)
    assert session.observe(CODE, 10.1, 10.11, 10.12) is None
    assert session.observe(CODE, 10.15, 10.16, 10.17) is None
    assert session.observe(CODE, 10.2, 10.21, 10.22) is None
    result = session.observe(CODE, 10.4, 10.41, 10.42)
    assert result['raw'] == str(CODE) and result['matching_frames'] == 4


def test_invalid_or_missing_code_resets_and_next_route_never_inherits_result():
    session = FreshQRSession(10, 15)
    session.observe(CODE, 10.1, 10.11, 10.12)
    session.observe(None, 10.2, 10.21, 10.22)
    assert session.confirmation.count == 0
    session.observe(CODE, 10.3, 10.31, 10.32)
    assert session.confirmation.count == 1
    assert FreshQRSession(20, 15).confirmation.count == 0


@pytest.mark.parametrize('stage', ['read', 'decode'])
def test_late_frames_or_decodes_cannot_become_success(stage):
    session = FreshQRSession(10, 1)
    with pytest.raises(TimeoutError):
        if stage == 'read':
            session.accepts(1, 10.1, 11)
        else:
            session.observe(CODE, 10.1, 10.2, 11)


@pytest.mark.parametrize('problem', ['invalid', 'error', 'old_frame', 'late_frame', 'too_few'])
def test_parent_rejects_bad_result(problem):
    worker = ready_worker()
    worker.start()
    result = dict(type='task_code',status='ok',confirmed=True,raw=str(CODE),
                  matching_frames=3,first_read_started_at=10.1,acquired_at=10.4,confirmed_at=10.42)
    if problem == 'invalid': result['raw'] = '111+123+111+231'
    if problem == 'error': result.update(status='error',confirmed=False)
    if problem == 'old_frame': result['first_read_started_at'] = 9
    if problem == 'late_frame': result['confirmed_at'] = 26
    if problem == 'too_few': result['matching_frames'] = 2
    worker.connection.poll.return_value = True
    worker.connection.recv.return_value = result
    with pytest.raises((RuntimeError,ValueError)):
        worker.poll()


def test_success_closes_camera_and_duplicate_poll_returns_only_its_result():
    worker = ready_worker()
    worker.start()
    worker.connection.poll.return_value = True
    worker.connection.recv.return_value = dict(type='task_code',status='ok',confirmed=True,
        raw=str(CODE),matching_frames=3,first_read_started_at=10.1,acquired_at=10.4,confirmed_at=10.42)
    worker.process.is_alive.return_value = False
    result = worker.poll()
    assert worker.closed and result['raw'] == str(CODE)
    worker.connection.close.assert_called_once()
    assert worker.poll() == result
    worker.close()
    worker.connection.close.assert_called_once()


def test_hung_capture_is_terminated_and_killed_on_close():
    worker = ready_worker()
    worker.start()
    worker.close()
    worker.process.terminate.assert_called_once()
    worker.process.kill.assert_called_once()


def test_parent_timeout_even_when_capture_or_decoder_hangs():
    worker = ready_worker()
    worker.started_at = -10
    with pytest.raises(TimeoutError):
        worker.poll()


@pytest.mark.parametrize('config', [dict(camera='0'),dict(camera=CAMERA,timeout_s=float('nan'))])
def test_invalid_config(config):
    with pytest.raises(ValueError):
        QRStationWork(config)


def test_attempt_releases_prewarm_if_planning_raises(monkeypatch):
    import tools.chassis_button_service as service
    worker = Mock()
    monkeypatch.setattr('tools.qr_station_work.QRStationWork', lambda _: worker)
    def fail(*args):
        raise ValueError('planning failed')
    monkeypatch.setattr(service,'_run_attempt',fail)
    with pytest.raises(ValueError,match='planning failed'):
        service.run_attempt(None,dict(qr_station=dict(camera=CAMERA)),None,lambda **_:None,lambda:False)
    worker.prewarm.assert_called_once()
    worker.close.assert_called_once()


def test_capture_never_decodes_prewarm_and_releases_before_returning_success(monkeypatch):
    import numpy as np
    import tools.qr_station_work as module
    from pi_vision.config import CameraConfig
    state = SimpleNamespace(now=10.0, sequence=0, commanded=False, released=False)
    decoded = []
    messages = []
    camera = Mock()
    def read():
        state.now += .1
        state.sequence += 1
        return True,np.zeros((8,8,3),dtype=np.uint8),0
    camera.read.side_effect = read
    camera.release.side_effect = lambda: setattr(state,'released',True)
    connection = Mock()
    connection.poll.side_effect = lambda: state.sequence >= 5 and not state.commanded
    def receive():
        state.commanded = True
        return dict(action='scan',started_at=state.now)
    connection.recv.side_effect = receive
    def send(result):
        assert state.released
        messages.append(result)
    connection.send.side_effect = send
    def decode(*args):
        decoded.append(state.sequence)
        return CODE,'candidate'
    monkeypatch.setattr(module.time,'monotonic',lambda:state.now)
    monkeypatch.setattr('pi_vision.camera.Camera',lambda _:camera)
    monkeypatch.setattr('pi_vision.config.load_config',lambda _:SimpleNamespace(camera=CameraConfig()))
    monkeypatch.setattr('pi_vision.detectors.qr_detector.QRDetector',lambda:Mock())
    monkeypatch.setattr('pi_vision.qr_check.inspect_frame',decode)
    module.capture_worker(dict(vision_config='x',camera=CAMERA,timeout_s=15),
                          connection,SimpleNamespace(value=0))
    assert decoded and min(decoded) >= 10  # 5 warm frames + 4 discarded gate frames.
    assert len(messages) == 1 and messages[0]['confirmed'] is True
    assert messages[0]['first_read_started_at'] > 10.5
    camera.release.assert_called_once()
