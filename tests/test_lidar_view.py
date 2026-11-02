import json
import pytest
from tools.lidar_view import ScanState


def scan(seq=1, points=None):
    return 'SCAN_JSON ' + json.dumps(dict(seq=seq, frequency_hz=10, points=[[0, 1, 3]] if points is None else points))


def test_old_frames_and_faults_cannot_retain_coordinates():
    state = ScanState()
    assert state.snapshot(now=0)['frame'] is None
    state.ingest(scan(), now=10)
    assert state.snapshot(now=10.999)['valid']
    assert state.snapshot(now=11)['frame'] is None
    assert state.snapshot(now=11)['status'] == 'expired'
    with pytest.raises(ValueError):
        state.ingest(scan(), now=11)
    assert not state.snapshot(now=11)['valid']
    state.ingest(scan(2), now=12)
    state.invalidate('scan_error')
    assert state.snapshot(now=12)['frame'] is None
    state.ingest(scan(3), now=13)
    assert state.snapshot(now=13)['valid']


@pytest.mark.parametrize('points', [[[0, float('nan'), 0]], [[0, 0, 0]], [[4, 1, 0]], [[0, 13, 0]], [[0, 1]]])
def test_rejects_invalid_distance_and_angle(points):
    state = ScanState()
    with pytest.raises(ValueError):
        state.ingest(scan(points=points))
    assert state.snapshot()['frame'] is None


def test_start_profiles_switch_position_and_route_start_without_changing_scan_direction():
    import math
    common=dict(pose_source='estimated_start',angle_sign=-1,evidence_ttl_s=2)
    profiles={'1':dict(common,pose=[2.15,2.2,-93.52],start=8),
              '2':dict(common,pose=[2.15,.2,-93.52],start=2)}
    state=ScanState({'map_id':'test'},profiles)
    state.configure(dict(start_zone=1))
    state.ingest(scan(1,[[0,.1,10]]),now=1)
    state.configure(dict(start_zone=2))
    result=state.snapshot(now=1)
    assert not result['navigation']['occupied_cells']
    settings=result['navigation']['settings']
    assert settings['start_zone']==2 and settings['start']==2
    assert settings['pose']==[2.15,.2,-93.52] and settings['angle_sign']==-1
    # Field sample from the unchanged cylinder at cell12 after moving to start2.
    x,y=-.8663,.9973
    for seq in range(2,5):
        state.ingest(scan(seq,[[math.atan2(y,x),math.hypot(x,y),10]]),now=1+seq*.1)
    assert state.snapshot(now=1.4)['navigation']['occupied_cells']==[12]
    before=dict(state.navigation.settings)
    with pytest.raises(ValueError):
        state.configure(dict(start_zone=3))
    assert state.navigation.settings==before
    state.configure(dict(start_zone=1))
    assert state.navigation.settings['pose']==profiles['1']['pose']
    assert state.navigation.settings['start']==8
    assert not state.navigation.history
    assert state.snapshot(now=1.4)['navigation']['route']['valid_for_motion'] is False


def test_unconfigured_zone_does_not_invent_a_pose():
    state=ScanState({'map_id':'test'})
    with pytest.raises(ValueError):
        state.configure(dict(start_zone=2))
    assert state.navigation.settings['pose'] is None
