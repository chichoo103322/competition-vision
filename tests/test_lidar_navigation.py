import math
import pytest
from tools.lidar_navigation import NavigationPreview, grid_id, lane_graph, astar, local_candidates
from tools.lidar_view import ScanState
import json


def test_grid_numbering_follows_reference_and_edges_do_not_cover_yellow():
    assert grid_id(2.2,.2)==0
    assert grid_id(1.2,1.65)==13
    assert grid_id(.2,2.2)==24
    assert grid_id(2.4,1) is None
    nodes,edges=lane_graph()
    assert len(nodes)==9 and len(edges)==12
    for edge in edges:
        a,b=nodes[edge['a']]['xy'],nodes[edge['b']]['xy']
        for k in range(101):
            p=[a[i]+(b[i]-a[i])*k/100 for i in (0,1)]
            assert not any(x<=p[0]<=x+.45 and y<=p[1]<=y+.45 for x in (.55,1.4) for y in (.55,1.4))


def test_astar_reroutes_and_reports_no_path():
    n,e=lane_graph()
    route=astar(n,e,8,4)
    assert route['nodes'][0]==8 and route['nodes'][-1]==4
    blocked=route['edges'][:1]
    detour=astar(n,e,8,4,blocked)
    assert not(set(detour['edges'])&set(blocked))
    assert astar(n,e,8,4,[x['id'] for x in e])['status']=='no_path'


def test_footprint_cannot_fit_narrow_lane_even_without_live_obstacles():
    from pathlib import Path
    config=json.loads(Path('config/field_map_national_2027.json').read_text())
    nav=NavigationPreview(config)
    assert not nav.snapshot(0,False)['static_blocked_edges']
    nav.configure(dict(clearance_m=.25))
    result=nav.snapshot(0,False)
    assert result['static_blocked_edges']
    assert result['route']['status']=='no_path'


def test_scan_votes_require_new_frames_and_expire_unknown():
    nav=NavigationPreview({})
    nav.configure(dict(pose=[1.2,1.2,0],pose_source='manual_stationary'))
    frame=dict(seq=1,points=[[math.pi/2,.45,1]])
    nav.ingest(frame,1)
    nav.ingest(frame,1.1)
    assert not nav.snapshot(1.1,True)['occupied_cells']
    for i,t in [(2,1.2),(3,1.3)]:
        nav.ingest(dict(frame,seq=i),t)
    live=nav.snapshot(1.3,True)
    assert 13 in live['occupied_cells']
    assert live['blocked_edges']
    assert live['route']['valid_for_motion'] is False
    assert nav.snapshot(2.3,True)['occupied_cells']==[]
    assert nav.snapshot(1.4,False)['occupied_cells']==[]
    nav.configure(dict(pose=[2.25,2.25,180],pose_source='estimated_start'))
    assert nav.snapshot(1.4,True)['unique_scans']==0


def test_missing_pose_still_allows_live_local_scan_but_no_field_claims():
    state=ScanState({'map_id':'test'})
    state.ingest('SCAN_JSON '+json.dumps(dict(seq=1,frequency_hz=10,points=[[0,.3,1],[.04,.3,1],[.08,.3,1]])),now=1)
    live=state.snapshot(now=1)
    assert live['valid'] and live['candidates']
    assert live['navigation']['obstacle_map_status']=='pose_unknown'
    assert not live['navigation']['occupied_cells']
    assert live['navigation']['route']['scope']=='static_nominal_preview'
    expired=state.snapshot(now=2)
    assert expired['frame'] is None and expired['candidates']==[]
    assert not expired['navigation']['blocked_edges']


def test_field_motion_from_14_to_13_uses_configured_angle_direction():
    nav=NavigationPreview({})
    # Live paired-placement measurements, keeping the same sensor placement.
    pose=[2.15,2.2,-93.52]
    nav.configure(dict(pose=pose,pose_source='estimated_start',angle_sign=-1))
    for cell,(x,y) in [(14,(.182652,.841832)),(13,(.5807575,.817315))]:
        nav.history.clear()
        for k in range(3):
            nav.ingest(dict(seq=cell*10+k,points=[[math.atan2(y,x),math.hypot(x,y),20]]),1+k*.1)
        result=nav.snapshot(1.2,True)
        assert result['occupied_cells']==[cell]
        assert result['route']['valid_for_motion'] is False
    nav.configure(dict(angle_sign=1))
    assert nav.snapshot(1.2,True)['occupied_cells']==[]


def test_sparse_distant_hits_accumulate_but_removal_and_disconnect_clear():
    nav=NavigationPreview({})
    nav.configure(dict(pose=[1.2,1.2,0],pose_source='estimated_start',evidence_ttl_s=2))
    # Isolated hits spanning more than one second must still accumulate.
    for seq,t in enumerate([0,.6,1.2],1):
        nav.ingest(dict(seq=seq,points=[[math.pi/2,.45,20]]),t)
    assert nav.snapshot(1.2,True)['occupied_cells']==[13]
    assert nav.snapshot(1.2,False)['occupied_cells']==[]
    # New, valid empty scans do not perpetuate old occupied evidence.
    for seq in range(4,26):
        nav.ingest(dict(seq=seq,points=[]),1.2+(seq-3)*.1)
    assert nav.snapshot(3.4,True)['occupied_cells']==[]


@pytest.mark.parametrize('command',[dict(pose=[0,0,float('nan')]),dict(start=99),dict(clearance_m=-1),dict(pose=[2.25,2.25,0]),dict(angle_sign=0),dict(angle_sign=True),dict(evidence_ttl_s=10)])
def test_bad_config_is_atomic(command):
    nav=NavigationPreview({})
    before=dict(nav.settings)
    with pytest.raises(ValueError):
        nav.configure(command)
    assert before==nav.settings


def test_boundary_fill_does_not_discard_occupancy_or_road_blocking():
    nav=NavigationPreview({})
    nav.configure(dict(pose=[1.2,1.2,0],pose_source='estimated_start'))
    # Same return patch straddles x=1.4: only 6's boundary band is hit.
    points=[[math.atan2(y-1.2,x-1.2),math.hypot(x-1.2,y-1.2),20]
            for x,y in [(1.36,1.61),(1.405,1.61),(1.21,1.41)]]
    for seq in range(1,4):nav.ingest(dict(seq=seq,points=points),seq*.1)
    result=nav.snapshot(.31,True)
    assert result['occupied_cells']==[6,13]
    assert result['interior_evidence_cells']==[13]
    assert result['boundary_only_cells']==[6]
    assert 8 in result['blocked_edges']
    assert result['route']['valid_for_motion'] is False
    assert nav.snapshot(2,True)['boundary_only_cells']==[]
    assert nav.snapshot(.31,False)['boundary_only_cells']==[]
    # Actual returns in the second cell's interior must keep its red fill.
    for seq in range(4,7):
        nav.ingest(dict(seq=seq,points=points+[[math.pi/4,math.sqrt(2)*.4,20]]),seq*.1)
    assert nav.snapshot(.61,True)['boundary_only_cells']==[]


def test_far_boundary_display_uses_same_expiry_as_far_occupancy():
    nav=NavigationPreview({})
    nav.configure(dict(pose=[2.2,2.2,0],pose_source='estimated_start',
                       evidence_ttl_s=1,far_evidence_ttl_s=4))
    x,y=.56,.3
    point=[math.atan2(y-2.2,x-2.2),math.hypot(x-2.2,y-2.2),20]
    for seq in range(1,4):nav.ingest(dict(seq=seq,points=[point]),seq*.1)
    assert nav.snapshot(2,True)['boundary_only_cells']==[19]
    assert nav.snapshot(4.4,True)['occupied_cells']==[]
    assert nav.snapshot(4.4,True)['boundary_only_cells']==[]
