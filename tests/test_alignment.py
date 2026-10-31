"""Alignment state, bounded correction and evidence validity, with virtual clocks."""

from copy import deepcopy
from dataclasses import replace
import math
import pytest
from pi_vision.alignment import AlignmentCalibration, AlignmentController
from pi_vision.alignment_demo import AlignmentPlant, simulation_calibration, simulate


@pytest.fixture
def rig():
    now = [1000.]
    engine = AlignmentController(simulation_calibration(), simulation=True,
                                 clock=lambda: now[0], wall_clock=lambda: now[0])
    plant = AlignmentPlant()
    return engine, plant, now


def observe(rig):
    e, plant, now = rig
    e.start(1)
    cmd = e.drain_commands()[0]
    plant.command(cmd['channel'], cmd['command'])
    channel, reply = plant.update(now[0])[0]
    assert e.feedback(channel, reply)
    now[0] += .11
    e.tick()
    cmd = e.drain_commands()[0]
    plant.command(cmd['channel'], cmd['command'])
    return plant.update(now[0])[0][1]


@pytest.mark.parametrize('failure', [None, 'target_lost', 'motion_failed', 'motion_timeout', 'stop_timeout'])
def test_complete_simulation_and_faults(failure):
    r = simulate(failure)
    assert r['ok']
    if failure is None:
        assert r['state']['adjustments'] == 2
        assert r['state']['stable_frames'] == 3
    else:
        assert r['state']['target'] is None


def test_real_motion_refused_without_real_calibration():
    for config in (None, simulation_calibration()):
        e = AlignmentController(config)
        with pytest.raises(ValueError):
            e.start(1)
        assert e.drain_commands() == []


@pytest.mark.parametrize('fields', [dict(width=True), dict(reference_pixel=(-1., 2.)),
    dict(correction_matrix_mm_per_pixel=((1., 2.), (2., 4.))),
    dict(correction_matrix_mm_per_pixel=((math.nan, 0.), (0., 1.))), dict(pose_id='')])
def test_bad_calibration(fields):
    with pytest.raises(ValueError):
        replace(simulation_calibration(), **fields)


def test_correction_is_bounded_and_requires_stopped_pose_ack(rig):
    e, _, _ = rig
    reply = observe(rig)
    assert e.feedback('vision', reply)
    items = e.drain_commands()
    assert [x['command']['operation'] for x in items] == ['stop', 'adjust_xy']
    params = items[-1]['command']['params']
    assert math.hypot(params['dx_mm'], params['dy_mm']) <= e.max_step_mm+1e-9
    assert params['target_expires_at_unix_s'] > reply['data']['acquired_unix_s']
    ack = AlignmentPlant.reply(items[-1]['command'], 'done', dict(stopped=False))
    e.feedback('motion', ack)
    assert e.phase == 'stopping'


@pytest.mark.parametrize('change', [dict(width=640), dict(camera_id='other'),
    dict(hsv_profile_sha256='other'), dict(units='mm'), dict(zone='rough'), dict(camera_role='qr')])
def test_calibration_identity_must_match(rig, change):
    e, _, _ = rig
    reply = observe(rig)
    reply['data'].update(change)
    e.feedback('vision', reply)
    assert e.phase == 'stopping'
    assert e.error == 'calibration_observation_mismatch'
    assert not any(i['command']['operation'] == 'adjust_xy' for i in e.drain_commands())


def test_old_simulated_or_wrong_command_feedback_cannot_move(rig):
    e, _, now = rig
    reply = observe(rig)
    for field, value in [('simulation', False), ('command_id', 'old'), ('run_id', 'old')]:
        bad = deepcopy(reply); bad[field] = value
        assert e.feedback('vision', bad) is False
    reply['data']['acquired_unix_s'] = now[0]-2.
    assert e.feedback('vision', reply) is False
    assert e.drain_commands() == []


def test_new_request_only_after_motion_and_settle_then_new_frame(rig):
    e, plant, now = rig
    reply = observe(rig)
    e.feedback('vision', reply)
    items = e.drain_commands()
    for item in items:
        plant.command(item['channel'], item['command'])
    channel, ack = plant.update(now[0])[0]
    e.feedback(channel, ack)
    assert e.phase == 'settling'
    assert e.drain_commands() == []
    assert not e.feedback('vision', reply)
    now[0] += .11
    e.tick()
    new = e.drain_commands()[0]['command']
    old = deepcopy(reply)
    old.update(command_id=new['command_id'])
    assert not e.feedback('vision', old)
    assert e.drain_commands() == []


def test_high_fps_confirmation_keeps_minimum_span_and_expires(rig):
    e, plant, now = rig
    reply = observe(rig)
    reply['data']['targets'][0].update(u=640., v=360.)
    for i in range(6):
        reply['data'].update(source_sequence=i+1, acquired_unix_s=now[0])
        e.feedback('vision', deepcopy(reply))
        if i < 4:
            assert e.phase == 'observing'
        now[0] += .05
        if e.phase == 'aligned':
            break
    assert e.phase == 'aligned'
    now[0] += 1.
    assert e.snapshot()['phase'] == 'expired'
    assert e.target is None


def test_no_target_ambiguity_or_bad_point_stops(rig):
    e, _, _ = rig
    reply = observe(rig)
    reply['data']['targets'] *= 2
    e.feedback('vision', reply)
    assert e.phase == 'stopping'
    assert e.target is None


def test_stop_timeout_blocks_restart(rig):
    e, _, now = rig
    observe(rig)
    e.cancel()
    e.drain_commands()
    now[0] += 5.
    e.tick()
    assert e.phase == 'fault'
    assert not e.stop_confirmed
    with pytest.raises(ValueError):
        e.start(1)


@pytest.mark.parametrize('limit', ['travel', 'count'])
def test_nonconvergence_cannot_exceed_motion_limits(rig, limit):
    e, _, _ = rig
    reply = observe(rig)
    if limit == 'travel':
        e.max_total_mm = 1.
    else:
        e.adjustments = e.max_adjustments
    e.feedback('vision', reply)
    assert e.phase == 'stopping'
    assert not any(i['command']['operation'] == 'adjust_xy' for i in e.drain_commands())


def test_camera_session_change_cannot_confirm_a_previous_target(rig):
    e, _, now = rig
    reply = observe(rig)
    reply['data']['targets'][0].update(u=640., v=360.)
    e.feedback('vision', deepcopy(reply))
    now[0] += .1
    reply['data'].update(source_session_id='reopened', source_sequence=1, acquired_unix_s=now[0])
    e.feedback('vision', reply)
    assert e.error == 'camera_session_changed'
    assert e.phase == 'stopping'


def test_real_vision_adapter_contract_connects_without_hardware():
    from unittest.mock import Mock
    from pi_vision.vision_backend import VisionBackend
    now = [1000.]
    cal = replace(simulation_calibration(), simulation_only=False)
    e = AlignmentController(cal, clock=lambda: now[0], wall_clock=lambda: now[0])
    source = Mock()
    backend = VisionBackend(source, cal.camera_id, clock=lambda: now[0], wall_clock=lambda: now[0])
    e.start(1)
    hold = e.drain_commands()[0]['command']
    e.feedback('motion', AlignmentPlant.reply(hold, 'done', dict(stopped=True,
        pose_id=cal.pose_id, calibration_id=cal.calibration_id)))
    now[0] += .11
    e.tick()
    scan = e.drain_commands()[0]['command']
    backend.command(scan)
    for i in range(3):
        source.get_state.return_value = dict(session_id='real-contract-test',
            hsv_profile_sha256=cal.hsv_profile_sha256,
            material_observation=dict(valid=True, status='ok', acquired_unix_s=now[0], source_sequence=i+1),
            preview=dict(source_width=1280, source_height=720),
            materials=[dict(color_id=1, center_x=640, center_y=360)])
        e.feedback('vision', backend.poll()[0])
        now[0] += .15
    assert e.phase == 'aligned'
    stop = e.drain_commands()[0]['command']
    assert backend.command(stop) == []
    assert backend.pending is None
