"""Eight-stage, immutable-at-request route preview; no actuator commands."""
import copy
import time
import uuid

try:
    from .lidar_navigation import astar
except ImportError:
    from lidar_navigation import astar


DEFAULT_STATIONS = {'qr':5,'material':7,'processing':1,'storage':3}
LABELS = {'home':'启停','qr':'扫码','material':'原料','processing':'加工','storage':'暂存'}
STAGES = ('home','qr','material','processing','storage','material','processing','storage','home')


def build_mission_plan(nav, evidence, seq, session_id):
    if evidence['unique_scans']<3 or not nav.settings['pose'] or nav.settings['start_zone'] not in (1,2):
        raise ValueError('需要新鲜扫描和已确认的起始区配置')
    stations=dict(nav.config.get('mission_stations',DEFAULT_STATIONS))
    if set(stations)!=set(DEFAULT_STATIONS) or any(type(n) is not int or n not in range(9) for n in stations.values()):
        raise ValueError('任务接近点配置无效')
    stations['home']=8 if nav.settings['start_zone']==1 else 2
    blocked=sorted(set(evidence['blocked_edges']+evidence['static_blocked_edges']))
    segments=[]
    for index,(a,b) in enumerate(zip(STAGES,STAGES[1:]),1):
        route=astar(nav.nodes,nav.edges,stations[a],stations[b],blocked,nav.settings['turn_penalty_m'])
        segments.append(dict(route,index=index,label=LABELS[a]+' → '+LABELS[b],
                             from_station=a,to_station=b,valid_for_motion=False))
    ready=all(s['status']=='path' for s in segments)
    return dict(plan_id=uuid.uuid4().hex,created_unix_s=time.time(),source_seq=seq,session_id=session_id,
                status='ready' if ready else 'no_path',segments=segments,
                total_length_m=round(sum(s['length_m'] for s in segments),4) if ready else None,
                blocked_edges=blocked,occupied_cells=list(evidence['occupied_cells']),
                settings=copy.deepcopy(nav.settings),stations=stations,
                station_coordinates_status='nominal_approach_nodes_not_measured_docking_poses',
                valid_for_motion=False,coverage='unobserved_is_unknown')


def mission_status(plan, evidence, live):
    if plan is None:
        return None
    used=set(e for s in plan['segments'] for e in s['edges'])
    new_blocks=sorted(used.intersection(set(evidence['blocked_edges'])-set(plan['blocked_edges'])))
    status='scan_unavailable' if not live else ('new_obstacle_replan_required' if new_blocks else 'snapshot_preview')
    return dict(plan,evidence_status=status,new_blocked_route_edges=new_blocks,valid_for_motion=False)
