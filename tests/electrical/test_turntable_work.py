import pytest

from test_arm_alignment import Plant, configuration
from tools.turntable_work import TurntableWork


def setup(calibrated=True):
    plant = Plant()
    selected, published = [], []
    alignment = configuration()
    alignment['stations'] = {key: value for key, value in alignment['stations'].items() if key in ('102', '105')}
    config = dict(enabled=True, camera_source='/dev/v4l/by-id/test-camera',
        station_timeout_s=90, observation_timeout_s=1, confirm_frames=3, confirm_span_s=.2,
        max_frame_gap_s=.7, alignment=alignment if calibrated else None)
    def snapshot():
        state = plant.snapshot()
        color = selected[-1]
        state.update(material_selection='unique_color_in_search_window', material_search_color=color)
        for item in state['materials']:
            item['color_id'] = color
        return state
    work = TurntableWork(plant, config, snapshot, selected.append, published.append, clock=lambda: plant.now)
    qr = dict(confirmed=True, route_id=7, raw='231+312+321+231')
    return plant, work, selected, published, qr


def test_turntable_waits_then_aligns_qr_order_without_grasp_commands():
    plant, work, selected, published, qr = setup()
    plant.problem = 'lost'
    work.start(plant.status, qr)
    for _ in range(12):
        assert work.poll(plant.status) is None
        plant.advance()
    assert selected == [2] and plant.actions == []
    plant.problem = None
    for _ in range(120):
        result = work.poll(plant.status)
        if result is not None:
            break
        plant.advance()
    else:
        raise AssertionError('sequence never finished')
    assert selected == [2, 3, 1]
    assert result['aligned_colors'] == [2, 3, 1]
    assert result['physical_grasp_completed'] is False
    assert [kind for kind, _ in plant.actions] == ['align', 'align', 'align', 'finish']
    assert len(work.results) == 1


def test_uncommissioned_station_observes_yellow_but_never_moves_or_completes():
    plant, work, selected, published, qr = setup(False)
    work.start(plant.status, qr)
    for _ in range(8):
        assert work.poll(plant.status) is None
        plant.advance()
    assert published[-1]['phase'] == 'calibration_required'
    assert selected == [2] and plant.actions == [] and work.results == []


@pytest.mark.parametrize('problem', ['old', 'restart', 'camera'])
def test_invalid_camera_never_moves_turntable_arm(problem):
    plant, work, selected, published, qr = setup()
    plant.problem = problem
    work.start(plant.status, qr)
    with pytest.raises(RuntimeError):
        for _ in range(10):
            work.poll(plant.status)
            plant.advance()
    assert plant.actions == []


def test_task_must_be_bound_to_current_route():
    plant, work, selected, published, qr = setup()
    qr['route_id'] = 8
    with pytest.raises(RuntimeError, match='QR'):
        work.start(plant.status, qr)
    assert selected == []


def test_future_numbered_ring_station_has_no_simulated_release():
    plant, work, selected, published, qr = setup()
    plant.status['work_id'] = 103
    with pytest.raises(RuntimeError, match='数字圆环'):
        work.start(plant.status, qr)
    assert plant.actions == []
