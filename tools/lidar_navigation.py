"""Nominal lane graph, scan evidence and A* preview; never a motion controller."""
from __future__ import annotations
from collections import deque
import heapq
import math


BOUNDS = (0.0, .55, 1.0, 1.4, 1.85, 2.4)
DISPLAY_BOUNDARY_BAND_M = .03  # Display classification only, not a localization error bound.


def grid_id(x, y):
    if not (0 <= x < 2.4 and 0 <= y < 2.4):
        return None
    col = next(i for i in range(5) if BOUNDS[i] <= x < BOUNDS[i + 1])
    row = next(i for i in range(5) if BOUNDS[i] <= y < BOUNDS[i + 1])
    k = 4 - col
    return k * 5 + (row if k % 2 == 0 else 4 - row)


def grid_rects():
    return [dict(id=grid_id((BOUNDS[c] + BOUNDS[c+1])/2, (BOUNDS[r] + BOUNDS[r+1])/2),
                 rect=[BOUNDS[c], BOUNDS[r], BOUNDS[c+1]-BOUNDS[c], BOUNDS[r+1]-BOUNDS[r]])
            for c in range(5) for r in range(5)]


def segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = max(0, min(1, ((p[0]-a[0])*dx + (p[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)


def lane_graph():
    # Chosen approach points, NOT official dimensions, dock points or measured poses.
    v = (.35, 1.2, 2.05)
    nodes = [dict(id=r*3+c, xy=[x, y]) for r,y in enumerate(v) for c,x in enumerate(v)]
    edges = []
    for n in nodes:
        i=n['id']
        for j in ([i+1] if i%3<2 else []) + ([i+3] if i<6 else []):
            edges.append(dict(id=len(edges), a=i, b=j))
    return nodes, edges


def occupied_cell_edges(nodes, edges, occupied):
    """Axis-aligned lane edges touching an occupied cell are unavailable.

    Whole-cell exclusion matches the displayed 25-cell obstacle model; the
    independent point-clearance test still protects nearby roads outside it.
    """
    rects=[g['rect'] for g in grid_rects() if g['id'] in occupied]
    return [e['id'] for e in edges if any(
        max(min(nodes[e['a']]['xy'][0],nodes[e['b']]['xy'][0]),x)
            <=min(max(nodes[e['a']]['xy'][0],nodes[e['b']]['xy'][0]),x+w)
        and max(min(nodes[e['a']]['xy'][1],nodes[e['b']]['xy'][1]),y)
            <=min(max(nodes[e['a']]['xy'][1],nodes[e['b']]['xy'][1]),y+h)
        for x,y,w,h in rects)]


def static_blocked_edges(config, nodes, edges, radius, *, check_boundary=True):
    """Conservative disc footprint against nominal exclusions and field limits."""
    rects=[tuple(v/1000 for v in z['rect_mm'])
           for z in config.get('floor_exclusions',[])+config.get('work_zones',[])]
    blocked=[]
    def point_rect(p, rect):
        x,y,w,h=rect
        return math.hypot(max(x-p[0],0,p[0]-x-w),max(y-p[1],0,p[1]-y-h))
    for e in edges:
        a,b=nodes[e['a']]['xy'],nodes[e['b']]['xy']
        hit=check_boundary and any(min(p[0],p[1],2.4-p[0],2.4-p[1])<radius for p in (a,b))
        for rect in rects:
            x,y,w,h=rect
            # All graph edges are axis aligned, so overlapping projections intersect.
            intersects=(max(min(a[0],b[0]),x)<=min(max(a[0],b[0]),x+w)
                        and max(min(a[1],b[1]),y)<=min(max(a[1],b[1]),y+h))
            distance=0 if intersects else min(point_rect(a,rect),point_rect(b,rect),
                *(segment_distance(p,a,b) for p in ((x,y),(x+w,y),(x,y+h),(x+w,y+h))))
            hit=hit or distance<radius-1e-9
        if hit:
            blocked.append(e['id'])
    return blocked


def astar(nodes, edges, start, goal, blocked=(), turn_penalty=.2):
    """Heading is part of the state: turn costs cannot collapse to node-only g."""
    if start not in range(len(nodes)) or goal not in range(len(nodes)):
        raise ValueError('unknown route endpoint')
    if not math.isfinite(turn_penalty) or turn_penalty < 0:
        raise ValueError('invalid turn cost')
    adj={n['id']:[] for n in nodes}
    for e in edges:
        if e['id'] not in blocked:
            adj[e['a']].append((e['b'],e['id']))
            adj[e['b']].append((e['a'],e['id']))
    def h(n):
        a,b=nodes[n]['xy'],nodes[goal]['xy']
        return abs(a[0]-b[0])+abs(a[1]-b[1])
    initial=(start,None)
    queue=[(h(start),0,0,initial)]
    cost={initial:0}
    previous={}
    serial=0
    while queue:
        _,g,_,state=heapq.heappop(queue)
        if g!=cost.get(state):
            continue
        n,direction=state
        if n==goal:
            path=[n];path_edges=[]
            while state in previous:
                state,e=previous[state]
                path.append(state[0]);path_edges.append(e)
            path.reverse();path_edges.reverse()
            length=sum(math.dist(nodes[a]['xy'],nodes[b]['xy']) for a,b in zip(path,path[1:]))
            return dict(status='path',nodes=path,edges=path_edges,length_m=round(length,4),
                        cost_m=round(g,4),turn_penalty_m=turn_penalty)
        for j,e in adj[n]:
            a,b=nodes[n]['xy'],nodes[j]['xy']
            nd=(int(b[0]>a[0])-int(b[0]<a[0]),int(b[1]>a[1])-int(b[1]<a[1]))
            ng=g+math.dist(a,b)+(turn_penalty if direction is not None and nd!=direction else 0)
            ns=(j,nd)
            if ng<cost.get(ns,math.inf):
                cost[ns]=ng;previous[ns]=(state,e);serial+=1
                heapq.heappush(queue,(ng+h(j),ng,serial,ns))
    return dict(status='no_path',nodes=[],edges=[],length_m=None,cost_m=None)


def local_candidates(polar):
    """Adjacent samples only; never labels an object as black or a known cylinder."""
    points=sorted((a,math.cos(a)*r,math.sin(a)*r,r) for a,r,_ in polar)
    groups=[]
    group=[]
    for p in points:
        if group and (p[0]-group[-1][0]>.07 or math.dist(p[1:3],group[-1][1:3])>.10):
            groups.append(group);group=[]
        group.append(p)
    if group:
        groups.append(group)
    if len(groups)>1:
        a,b=groups[-1],groups[0]
        if b[0][0]+2*math.pi-a[-1][0]<.07 and math.dist(b[0][1:3],a[-1][1:3])<.10:
            groups[0]=a+b;groups.pop()
    result=[]
    for g in groups:
        if len(g)<2:
            continue
        diameter=max(math.dist(a[1:3],b[1:3]) for a in g for b in g) if len(g)<80 else math.inf
        if .015<=diameter<=.25:
            x=sum(p[1] for p in g)/len(g);y=sum(p[2] for p in g)/len(g)
            result.append(dict(x_m=round(x,4),y_m=round(y,4),range_m=round(math.hypot(x,y),4),
                               width_m=round(diameter,4),points=len(g),status='return_candidate'))
    return result


class NavigationPreview:
    """Rolling unique-scan votes; stale evidence becomes UNKNOWN, never FREE."""
    def __init__(self, config):
        self.config=config
        self.nodes,self.edges=lane_graph()
        self.history=deque(maxlen=64)
        self.last_seq=None
        self.pose_key=None
        self.settings=None
        self.reset()

    def reset(self):
        self.settings=dict(pose=None,pose_source='unknown',start_zone=None,angle_sign=1,start=8,goal=7,
                           clearance_m=.18,turn_penalty_m=.2,evidence_ttl_s=1.0,
                           far_evidence_ttl_s=2.0,self_mask_radius_m=0.0)
        self.history.clear();self.last_seq=None;self.pose_key=None

    def configure(self, command):
        s=dict(self.settings)
        if command.get('action')=='reset':
            self.reset();return
        if 'start_zone' in command:
            z=command['start_zone']
            if z is not None and (type(z) is not int or z not in (1,2)):
                raise ValueError('invalid start zone')
            s['start_zone']=z
        if 'angle_sign' in command:
            if type(command['angle_sign']) is not int or command['angle_sign'] not in (-1,1):
                raise ValueError('invalid angle sign')
            s['angle_sign']=command['angle_sign']
        if 'pose' in command:
            p=command['pose']
            source=command.get('pose_source','unknown')
            if p is not None:
                if not isinstance(p,list) or len(p)!=3 or not all(type(x) in (int,float) and math.isfinite(x) for x in p):
                    raise ValueError('invalid pose')
                if not(0<=p[0]<=2.4 and 0<=p[1]<=2.4 and -360<=p[2]<=360):
                    raise ValueError('pose outside map')
                if source not in ('estimated_start','manual_stationary'):
                    raise ValueError('pose source required')
            else:
                source='unknown'
            s['pose'],s['pose_source']=p,source
            if p is None or 'start_zone' not in command:
                s['start_zone']=None
        for k in ('start','goal'):
            if k in command:
                if type(command[k]) is not int or command[k] not in range(9):
                    raise ValueError('invalid node')
                s[k]=command[k]
        for k,low,high in [('clearance_m',.05,.5),('turn_penalty_m',0,2),('evidence_ttl_s',1,3),
                           ('far_evidence_ttl_s',1,5),('self_mask_radius_m',0,.35)]:
            if k in command:
                value=command[k]
                if type(value) not in (int,float) or not math.isfinite(value) or not low<=value<=high:
                    raise ValueError('invalid '+k)
                s[k]=value
        if any(s[k] != self.settings[k] for k in ('pose','pose_source','clearance_m','angle_sign',
                   'evidence_ttl_s','far_evidence_ttl_s','self_mask_radius_m','start_zone')):
            self.history.clear();self.last_seq=None
        self.settings=s

    def ingest(self, frame, now):
        if frame['seq']==self.last_seq:
            return
        self.last_seq=frame['seq']
        if not self.settings['pose']:
            return
        x,y,degrees=self.settings['pose']
        a=math.radians(degrees);c,s=math.cos(a),math.sin(a)
        cells=set();edge_hits=set();far_cells=set();far_edges=set();counts={}
        home=self.home_cell()
        interior_cells=set();far_interior_cells=set()
        for angle,r,_ in frame['points']:
            angle *= self.settings['angle_sign']
            lx,ly=math.cos(angle)*r,math.sin(angle)*r
            p=(x+c*lx-s*ly,y+s*lx+c*ly)
            cell=grid_id(*p)
            if cell is None:
                continue
            if cell==home and r<=self.settings['self_mask_radius_m']:
                continue
            cells.add(cell)
            counts[cell]=counts.get(cell,0)+1
            # Preserve every point for road blocking. Only the fill color distinguishes
            # evidence confined to a grid boundary from evidence farther inside.
            dx=min(abs(p[0]-b) for b in BOUNDS)
            dy=min(abs(p[1]-b) for b in BOUNDS)
            if min(dx,dy)>=DISPLAY_BOUNDARY_BAND_M:
                interior_cells.add(cell)
                if r>=1.8:
                    far_interior_cells.add(cell)
            if r>=1.8:
                far_cells.add(cell)
            for edge in self.edges:
                if segment_distance(p,self.nodes[edge['a']]['xy'],self.nodes[edge['b']]['xy'])<=self.settings['clearance_m']:
                    edge_hits.add(edge['id'])
                    if r>=1.8:
                        far_edges.add(edge['id'])
        self.history.append((now,frame['seq'],cells,edge_hits,far_cells,far_edges,counts,
                             interior_cells,far_interior_cells))

    def home_cell(self):
        if self.settings['pose'] and self.settings['start_zone'] in (1,2):
            return 4 if self.settings['start_zone']==1 else 0
        return None

    def snapshot(self, now, live):
        # A single missed scan does not clear a blocked road. Evidence expires explicitly.
        near_ttl=self.settings['evidence_ttl_s'];far_ttl=self.settings['far_evidence_ttl_s']
        recent=[h for h in self.history if 0<=now-h[0]<max(near_ttl,far_ttl)] if live else []
        cells={i:sum((i in h[2] and now-h[0]<near_ttl) or
                     (i in h[4] and now-h[0]<far_ttl) for h in recent) for i in range(25)}
        edge_votes={i:sum((i in h[3] and now-h[0]<near_ttl) or
                          (i in h[5] and now-h[0]<far_ttl) for h in recent) for i in range(12)}
        home=self.home_cell()
        occupied=[i for i,n in cells.items() if n>=3 and i!=home]
        interior_votes={i:sum((i in h[7] and now-h[0]<near_ttl) or
                             (i in h[8] and now-h[0]<far_ttl) for h in recent)
                        for i in occupied}
        boundary_only=[i for i in occupied if interior_votes[i]<3]
        point_blocked=[i for i,n in edge_votes.items() if n>=3]
        cell_blocked=occupied_cell_edges(self.nodes,self.edges,occupied)
        blocked=sorted(set(point_blocked+cell_blocked))
        static_blocked=static_blocked_edges(self.config,self.nodes,self.edges,self.settings['clearance_m'])
        route=astar(self.nodes,self.edges,self.settings['start'],self.settings['goal'],set(blocked+static_blocked),
                    self.settings['turn_penalty_m'])
        source=self.settings['pose_source']
        scope='live_manual_stationary_preview' if source=='manual_stationary' and live else (
            'estimated_pose_preview' if source=='estimated_start' and live else 'static_nominal_preview')
        # All routes are software previews; unobserved cells/edges are never called safe.
        route.update(scope=scope,valid_for_motion=False,live_obstacle_evidence=bool(recent))
        return dict(settings=self.settings,grid_rects=grid_rects(),nodes=self.nodes,edges=self.edges,
                    grid_votes=cells,edge_votes=edge_votes,occupied_cells=occupied,blocked_edges=blocked,
                    point_blocked_edges=point_blocked,cell_blocked_edges=cell_blocked,
                    boundary_only_cells=boundary_only,
                    interior_evidence_cells=[i for i in occupied if i not in boundary_only],
                    display_boundary_band_m=DISPLAY_BOUNDARY_BAND_M,
                    static_blocked_edges=static_blocked,
                    unique_scans=len(recent),window_limit=self.history.maxlen,min_scan_votes=3,
                    evidence_ttl_ms=round(self.settings['evidence_ttl_s']*1000),
                    far_evidence_ttl_ms=round(far_ttl*1000),home_cell=home,
                    corner_diagnostics={str(i):dict(hit_scans=cells[i],
                         last_hit_age_ms=round((now-max(h[0] for h in recent if i in h[2]))*1000)
                         if any(i in h[2] for h in recent) else None,
                         status='occupied' if i in occupied else 'unknown') for i in (20,24)},
                    obstacle_map_status='pose_unknown' if source=='unknown' else ('live' if live else 'scan_unavailable'),
                    pose_localization_valid=False,route=route,
                    coverage='unobserved_is_unknown',motion_enabled=False)
