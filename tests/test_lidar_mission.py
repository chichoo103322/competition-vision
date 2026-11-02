import copy
import json
import math
import time
from pathlib import Path

import pytest
from tools.lidar_navigation import NavigationPreview
from tools.lidar_mission import build_mission_plan, mission_status
from tools.lidar_view import ScanState


CONFIG=json.loads(Path('config/field_map_national_2027.json').read_text())


def point(pose,xy):
    x,y=xy[0]-pose[0],xy[1]-pose[1]
    return [math.atan2(y,x)-math.radians(pose[2]),math.hypot(x,y),20]


def configured(zone=1):
    nav=NavigationPreview(CONFIG)
    nav.configure(dict(pose=[2.15,2.2 if zone==1 else .2,0],pose_source='estimated_start',
                       start_zone=zone,start=8 if zone==1 else 2,self_mask_radius_m=.3,
                       evidence_ttl_s=2,far_evidence_ttl_s=4))
    return nav


def test_self_returns_do_not_trap_start_but_external_returns_in_same_cell_still_block():
    nav=configured()
    own=point(nav.settings['pose'],[2.05,2.05])
    for seq in range(1,4):
        nav.ingest(dict(seq=seq,points=[own]),seq*.1)
    result=nav.snapshot(.31,True)
    assert result['home_cell']==4 and result['occupied_cells']==[]
    assert result['blocked_edges']==[]
    assert result['route']['status']=='path'
    external=point(nav.settings['pose'],[1.86,2.0])
    for seq in range(4,7):
        nav.ingest(dict(seq=seq,points=[external]),seq*.1)
    result=nav.snapshot(.61,True)
    assert 4 not in result['occupied_cells']  # User requested display suppression only.
    assert result['grid_votes'][4]==3
    assert 11 in result['blocked_edges']  # The whole cell is not declared free.


def test_far_isolated_hits_accumulate_and_expire_without_filling_empty_cells():
    nav=configured()
    for seq,t in enumerate([0,.9,2.1],1):
        nav.ingest(dict(seq=seq,points=[point(nav.settings['pose'],[.28,.28])]),t)
    result=nav.snapshot(2.1,True)
    assert result['occupied_cells']==[20]
    assert result['corner_diagnostics']['24']['status']=='unknown'
    assert nav.snapshot(2.1,False)['occupied_cells']==[]
    assert nav.snapshot(6.2,True)['occupied_cells']==[]


@pytest.mark.parametrize('zone',[1,2])
def test_eight_segments_share_one_snapshot_and_avoid_blocked_center_edge(zone):
    nav=configured(zone)
    for seq in range(1,4):
        nav.ingest(dict(seq=seq,points=[point(nav.settings['pose'],[1.2,1.65])]),seq*.1)
    evidence=nav.snapshot(.31,True)
    assert 8 in evidence['blocked_edges']
    plan=build_mission_plan(nav,evidence,3,'session')
    assert plan['status']=='ready' and len(plan['segments'])==8
    home=8 if zone==1 else 2
    assert plan['segments'][0]['nodes'][0]==home
    assert plan['segments'][-1]['nodes'][-1]==home
    assert plan['total_length_m']==pytest.approx(sum(s['length_m'] for s in plan['segments']))
    assert all(8 not in s['edges'] for s in plan['segments'])
    assert all(s['nodes'][-1]==t['nodes'][0] for s,t in zip(plan['segments'],plan['segments'][1:]))
    assert not plan['valid_for_motion']
    original=copy.deepcopy(plan)
    used=next(e for s in plan['segments'] for e in s['edges'])
    changed=dict(evidence,blocked_edges=evidence['blocked_edges']+[used])
    status=mission_status(plan,changed,True)
    assert status['evidence_status']=='new_obstacle_replan_required'
    assert plan==original
    assert mission_status(plan,evidence,False)['evidence_status']=='scan_unavailable'


def test_impossible_batch_has_no_total_and_missing_pose_is_rejected():
    nav=configured()
    for seq in range(1,4):
        nav.ingest(dict(seq=seq,points=[]),0)
    evidence=nav.snapshot(0,True)
    blocked=dict(evidence,blocked_edges=list(range(12)))
    plan=build_mission_plan(nav,blocked,1,'session')
    assert plan['status']=='no_path' and plan['total_length_m'] is None
    assert len(plan['segments'])==8
    with pytest.raises(ValueError):
        build_mission_plan(nav,nav.snapshot(0,False),1,'session')


def test_http_state_batch_action_freezes_and_configuration_clears():
    state=ScanState(CONFIG,{'1':dict(pose=[2.15,2.2,0],pose_source='estimated_start',start=8)})
    state.configure(dict(start_zone=1))
    with pytest.raises(ValueError):
        state.configure(dict(action='plan_mission'))
    for seq in range(1,4):
        state.ingest('SCAN_JSON '+json.dumps(dict(seq=seq,frequency_hz=6,points=[])))
    state.configure(dict(action='plan_mission'))
    plan_id=state.snapshot()['mission_plan']['plan_id']
    state.ingest('SCAN_JSON '+json.dumps(dict(seq=4,frequency_hz=6,points=[])))
    assert state.snapshot()['mission_plan']['plan_id']==plan_id
    state.configure(dict(clearance_m=.19))
    assert state.snapshot()['mission_plan'] is None


@pytest.mark.parametrize('zone',[1,2])
@pytest.mark.parametrize('xy,cell,edges', [([.1,2.3],24,{6,10}),([.1,.1],20,{0,1})])
def test_occupied_corner_blocks_roads_even_outside_point_clearance(zone,xy,cell,edges):
    nav=configured(zone)
    for seq in range(1,4):
        nav.ingest(dict(seq=seq,points=[point(nav.settings['pose'],xy)]),seq*.1)
    evidence=nav.snapshot(.31,True)
    assert evidence['occupied_cells']==[cell]
    assert evidence['point_blocked_edges']==[]  # Reproduces the old missing-block case.
    assert set(evidence['cell_blocked_edges'])==edges
    plan=build_mission_plan(nav,evidence,3,'session')
    assert plan['status']=='ready'
    assert all(not edges.intersection(s['edges']) for s in plan['segments'])
    assert nav.snapshot(5,True)['cell_blocked_edges']==[]
    assert nav.snapshot(.31,False)['blocked_edges']==[]


def test_new_corner_obstacle_invalidates_old_batch_and_replanned_batch_avoids_it():
    nav=configured(2)
    for seq in range(1,4):nav.ingest(dict(seq=seq,points=[]),seq*.1)
    plan=build_mission_plan(nav,nav.snapshot(.31,True),3,'s')
    assert any(6 in s['nodes'] for s in plan['segments'])
    for seq in range(4,7):
        nav.ingest(dict(seq=seq,points=[point(nav.settings['pose'],[.1,2.3])]),seq*.1)
    evidence=nav.snapshot(.61,True)
    assert mission_status(plan,evidence,True)['evidence_status']=='new_obstacle_replan_required'
    updated=build_mission_plan(nav,evidence,6,'s')
    assert all(6 not in s['nodes'] for s in updated['segments'])
    assert updated['segments'][4]['nodes']==[3,4,7]
