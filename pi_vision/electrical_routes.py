"""Convert saved eight-stage geometry to a V1 *review artifact*, never motion.

No planner, sensor, or serial imports. Heading/speeds and chassis initial pose
must be supplied explicitly; lidar pose is never reused as a chassis pose.
"""
from copy import deepcopy
import math

from .electrical_protocol import integer, scaled, upload_payload
from .electrical_coordinates import (ELECTRICAL_FRAME, NOMINAL_FRAME,
                                     require_electrical_frame, nominal_xy_to_electrical_mm)

STATIONS = ('home', 'qr', 'material', 'processing', 'storage',
            'material', 'processing', 'storage', 'home')
WORK_HEADING_OFFSETS_DEG = {101: 90, 102: -90, 103: 90, 104: 0,
                            105: -90, 106: 90, 107: 0, 108: 0}


def wrap_deg(angle):
    return (angle + 180) % 360 - 180


def configure_work_heading_mode(route, enabled):
    """Attach local-only controller work-heading semantics; wire V1 is unchanged."""
    if type(enabled) is not bool:
        raise ValueError('work_heading_mode_enabled must be boolean')
    initial_yaw = route['initial_pose']['yaw_deg']
    active = enabled and any(WORK_HEADING_OFFSETS_DEG.get(p.get('work_id')) is not None
                             for p in route['points'])
    route['work_heading_mode_enabled'] = enabled
    route['work_heading_mode_active'] = active
    current_travel_yaw = initial_yaw
    for point in route['points']:
        point.pop('travel_yaw_deg', None)
        point.pop('expected_work_yaw_deg', None)
        if not active:
            point['travel_yaw_deg'] = point['yaw_deg']
            continue
        point['travel_yaw_deg'] = current_travel_yaw
        work_id = point.get('work_id', 0)
        if work_id:
            offset = WORK_HEADING_OFFSETS_DEG.get(work_id)
            expected = (wrap_deg(initial_yaw + offset) if offset is not None
                        else wrap_deg(point['yaw_deg']))
            point['expected_work_yaw_deg'] = expected
            current_travel_yaw = expected
    return route


def expected_work_yaw(route, point):
    active = route.get('work_heading_mode_enabled', False) and any(
        p.get('work_id') in WORK_HEADING_OFFSETS_DEG for p in route['points'])
    if active and point.get('work_id'):
        offset = WORK_HEADING_OFFSETS_DEG.get(point['work_id'])
        return wrap_deg(route['initial_pose']['yaw_deg'] + offset) if offset is not None else wrap_deg(point['yaw_deg'])
    return point['yaw_deg']


def work_pose_error(status, route, point_index, *, position_tolerance_mm=8,
                    yaw_tolerance_deg=2):
    if not 0 <= point_index < len(route['points']):
        return 'work point index outside route'
    point = route['points'][point_index]
    values = [status[k] for k in ('x_mm', 'y_mm', 'yaw_deg')]
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        return 'invalid reported work pose'
    if not point.get('work_id') or status.get('work_id') != point['work_id']:
        return 'work point identity mismatch'
    if math.hypot(status['x_mm'] - point['x_mm'], status['y_mm'] - point['y_mm']) > position_tolerance_mm:
        return 'WAIT_WORK position outside arrival tolerance'
    yaw_error = wrap_deg(status['yaw_deg'] - expected_work_yaw(route, point))
    if abs(yaw_error) > yaw_tolerance_deg:
        return 'WAIT_WORK work-heading outside arrival tolerance'
    return None


def compile_preview(plan, nodes, *, route_id, initial_pose, segment_yaws_deg,
                    move_rpm, turn_rpm, coordinate_frame, stage_limit=8,
                    work_heading_mode_enabled=True):
    # Profiles already use electrical/photo coordinates. Only nominal planner
    # nodes are transformed; never rotate the profile or a compiled route twice.
    require_electrical_frame(coordinate_frame)
    if plan.get('coordinate_frame', NOMINAL_FRAME) != NOMINAL_FRAME:
        raise ValueError('planner input must use field_nominal coordinates')
    if type(stage_limit) is not int or stage_limit not in (1, 8):
        raise ValueError('stage_limit must be 1 or 8')
    if (len(plan.get('segments', [])) != 8 or
            (stage_limit == 8 and plan.get('status') != 'ready') or
            (stage_limit == 1 and plan['segments'][0].get('status') != 'path')):
        raise ValueError('exactly eight reachable segments required')
    if plan.get('evidence_status') in ('scan_unavailable', 'new_obstacle_replan_required'):
        raise ValueError('plan evidence is invalid')
    integer(plan['settings']['start_zone'], 1, 2, 'start_zone')
    integer(route_id, 1, 0xffffffff, 'route_id')
    integer(move_rpm, 1, 120, 'move_rpm')
    integer(turn_rpm, 1, 120, 'turn_rpm')
    if len(segment_yaws_deg) != 8:
        raise ValueError('provide all eight headings explicitly')
    for yaw in segment_yaws_deg:
        scaled(yaw, 100, 180, 'segment yaw')
    coordinates = {}
    for node in nodes:
        key = integer(node['id'], 0, 65535, 'node id')
        if key in coordinates or len(node['xy']) != 2:
            raise ValueError('duplicate node or invalid XY')
        xy = node['xy']
        if any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) > 100 for v in xy):
            raise ValueError('node coordinates must be finite map metres')
        coordinates[key] = nominal_xy_to_electrical_mm(*xy)
    points, ranges = [], []
    last_node = None
    blocked = set(plan.get('blocked_edges', []))
    for i, segment in enumerate(plan['segments'][:stage_limit], 1):
        path = segment['nodes']
        if (segment['index'] != i or segment['status'] != 'path' or not path or
                segment['from_station'] != STATIONS[i-1] or segment['to_station'] != STATIONS[i] or
                len(segment['edges']) != len(path)-1 or blocked.intersection(segment['edges'])):
            raise ValueError('segment order, station, path, or blocking mismatch')
        if last_node is not None and path[0] != last_node:
            raise ValueError('segments are not contiguous')
        if any(type(n) is not int or n not in coordinates for n in path):
            raise ValueError('unknown node')
        first = len(points)
        # First path node is retained: actual initial pose need not equal the
        # nominal home approach node. Later shared endpoints are not repeated.
        targets = path if i == 1 else (path[1:] or path)
        for node in targets:
            x, y = coordinates[node]
            points.append(dict(x_mm=x, y_mm=y, yaw_deg=segment_yaws_deg[i-1],
                               move_rpm=move_rpm, turn_rpm=turn_rpm, work_id=0))
        # Unique stage IDs, including return home. They are NOT colour IDs or
        # universal grab/place operation codes; all await actual completion.
        points[-1]['work_id'] = 100 + i
        ranges.append(dict(segment=i, from_station=STATIONS[i-1], to_station=STATIONS[i],
                           first_point_index=first, last_point_index=len(points)-1, work_id=100+i))
        last_node = path[-1]
    if stage_limit == 8 and last_node != plan['segments'][0]['nodes'][0]:
        raise ValueError('eight-stage route must return to its home node')
    result = dict(route_id=route_id, initial_pose=deepcopy(initial_pose), points=points,
                  segments=ranges, start_zone=plan['settings']['start_zone'],
                  source_plan_id=plan.get('plan_id'), source_session_id=plan.get('session_id'),
                  source_seq=plan.get('source_seq'), valid_for_motion=False,
                  review_required=['actual_chassis_start', 'measured_docking_poses',
                                   'turn_then_axis_swept_path', 'motion_localization',
                                   'communication_loss_stop'],
                  coordinate_frame=ELECTRICAL_FRAME, source_coordinate_frame=NOMINAL_FRAME,
                  distance_unit='mm', angle_unit='degree')
    configure_work_heading_mode(result, work_heading_mode_enabled)
    upload_payload(result)  # Validate full payload, including the 64-point cap.
    return result


def add_qr_search(route, field_map, search_rpm):
    """Expand test stage 1, leaving its original QR gate and all later legs intact."""
    integer(search_rpm, 1, route['approach_rpm'], 'QR search rpm')
    if not route.get('work_heading_mode_active') or not route.get('footprint'):
        raise ValueError('QR search requires work-heading firmware and a measured chassis footprint')
    if route.get('qr_search') or route.get('test_mode') != 'chassis_route_dry_work':
        raise ValueError('QR search is only supported on an unmodified chassis test route')
    reference = next((entry.get('y_reference_from_top_mm')
                      for entry in field_map.get('variable_references', [])
                      if entry['id'] == 'qr_board'), None)
    if (not isinstance(reference, list) or len(reference) != 2 or
            any(type(value) is not int for value in reference) or
            not 0 < reference[0] < reference[1] < 2400):
        raise ValueError('missing or invalid QR search band in field map')
    low, high = reference  # After the 180° map transform this distance is electrical +Y.
    zone = route['start_zone']
    if zone not in (1, 2):
        raise ValueError('QR search requires start zone 1 or 2')
    near, far = (low, high) if zone == 1 else (high, low)
    route = deepcopy(route)
    first = route['segments'][0]
    end = first['last_point_index']
    original = route['points'][end]
    if original['work_id'] != 101 or not low < original['y_mm'] < high:
        raise ValueError('original QR gate must lie inside the search band')
    if wrap_deg(expected_work_yaw(route, original)) != -180:
        raise ValueError('QR search requires the existing work heading towards the left-side board')
    prior = route['points'][end-1] if end else route['initial_pose']
    # At the band ends, the nominal lane's 90° turn sweeps a rear corner
    # into the yellow area. Move the search line 50 mm toward the board,
    # then rejoin the original route on its centerline.
    scan_x = original['x_mm'] - 50
    approach = [] if prior['x_mm'] == original['x_mm'] else [dict(original, work_id=0)]
    approach.append(dict(original, work_id=0, y_mm=near, move_rpm=search_rpm))
    gates = [dict(original, x_mm=scan_x, y_mm=y, move_rpm=search_rpm) for y in (near, far)]
    rejoin = dict(original, work_id=0, x_mm=scan_x, move_rpm=search_rpm)
    replacement = [*approach, *gates, rejoin, dict(original, move_rpm=search_rpm)]
    first_gate = end + len(approach)
    route['points'][end:end+1] = replacement
    count = len(replacement) - 1
    first['last_point_index'] += count
    for segment in route['segments'][1:]:
        segment['first_point_index'] += count
        segment['last_point_index'] += count
    route['qr_search'] = dict(near_index=first_gate, far_index=first_gate+1,
                              return_index=first['last_point_index'])
    configure_work_heading_mode(route, True)
    upload_payload(route)
    return route


def segment_for_status(route, status):
    """Local stage mapping; V1 transmits point index, not segment index."""
    if status['route_id'] != route['route_id'] or status['point_count'] != len(route['points']):
        raise ValueError('status does not belong to this route')
    if status['state'] == 4 and status['point_index'] == len(route['points']):
        return dict(segment=route['segments'][-1]['segment'], route_done=True)
    for segment in route['segments']:
        if segment['first_point_index'] <= status['point_index'] <= segment['last_point_index']:
            search = route.get('qr_search') or {}
            search_gate = segment['segment'] == 1 and status['point_index'] in (
                search.get('near_index'), search.get('far_index'))
            if status['state'] == 3 and ((not search_gate and
                                          status['point_index'] != segment['last_point_index']) or
                                         status['work_id'] != segment['work_id']):
                raise ValueError('unexpected work gate')
            return dict(segment, route_done=False)
    raise ValueError('point index outside route')


def compile_chassis_test(plan, nodes, *, route_id, initial_pose, heading_deg,
                         move_rpm, turn_rpm, approach_rpm, approach_distance_mm, coordinate_frame,
                         footprint=None, stage_limit=8, stop_strategy='controller_deceleration',
                         work_heading_mode_enabled=True, test_turn_sweep_ignored_cells=(),
                         test_ignore_turn_obstacles=False):
    """Axis-path test; firmware decelerates and turns into each work stop.

    Grid edges must be axis-aligned. With work-heading mode the STM32 holds
    the previous work heading in transit and turns only after station arrival.
    split_approach retains the historical stop/restart profile for comparison.
    approach_rpm caps the short start/home connectors in the default strategy;
    it is NOT an independent terminal-speed field in the V1 wire protocol.
    """
    if stop_strategy not in ('controller_deceleration', 'split_approach'):
        raise ValueError('unknown stop strategy')
    integer(approach_rpm, 1, 120, 'approach_rpm')
    integer(approach_distance_mm, 1, 300, 'approach_distance_mm')
    if heading_deg not in (-180, -90, 0, 90, 180) or type(heading_deg) not in (int, float):
        raise ValueError('chassis test heading must be a map cardinal direction')
    if initial_pose['yaw_deg'] != heading_deg:
        raise ValueError('initial chassis heading must match the fixed test heading')
    if approach_rpm >= move_rpm:
        raise ValueError('approach_rpm must be below cruise move_rpm')
    route = compile_preview(plan, nodes, route_id=route_id, initial_pose=initial_pose,
                            segment_yaws_deg=[heading_deg] * 8,
                            move_rpm=move_rpm, turn_rpm=turn_rpm, coordinate_frame=coordinate_frame,
                            stage_limit=stage_limit, work_heading_mode_enabled=work_heading_mode_enabled)
    coordinates = {node['id']: tuple(round(v * 1000) for v in node['xy']) for node in nodes}
    previous = None
    for segment in plan['segments'][:stage_limit]:
        path = segment['nodes'] if segment['index'] == 1 else (segment['nodes'][1:] or segment['nodes'])
        for node_id in path:
            target = coordinates[node_id]
            if previous is not None and previous[0] != target[0] and previous[1] != target[1]:
                raise ValueError('route contains diagonal grid edges')
            previous = target
    # V1 stops at EVERY uploaded point, including work_id=0. Do not insert a
    # speed-change point by default: firmware already decelerates with distance.
    new_points = []
    new_segments = []
    for segment in route['segments']:
        first = len(new_points)
        original = route['points'][segment['first_point_index']:segment['last_point_index'] + 1]
        if segment['segment'] == 1:
            home = original[0]
            # Explicit X-then-Y connector from actual chassis start, rather
            # than letting firmware choose an unreviewed two-axis sweep.
            if home['x_mm'] != initial_pose['x_mm'] and home['y_mm'] != initial_pose['y_mm']:
                original.insert(0, dict(home, y_mm=round(initial_pose['y_mm']),
                                        work_id=0, move_rpm=approach_rpm))
        if segment['segment'] == 8:
            home = original[-1]
            if (home['x_mm'], home['y_mm']) != (initial_pose['x_mm'], initial_pose['y_mm']):
                original[-1] = dict(home, work_id=0)
                if home['x_mm'] != initial_pose['x_mm'] and home['y_mm'] != initial_pose['y_mm']:
                    original.append(dict(home, y_mm=round(initial_pose['y_mm']), work_id=0,
                                         move_rpm=approach_rpm))
                original.append(dict(home, x_mm=round(initial_pose['x_mm']),
                                     y_mm=round(initial_pose['y_mm']), move_rpm=approach_rpm))
        for index, point in enumerate(original):
            if (stop_strategy == 'split_approach' and
                    index == len(original) - 1 and point['work_id']):
                previous_point = new_points[-1] if new_points else route['initial_pose']
                dx = point['x_mm'] - previous_point['x_mm']
                dy = point['y_mm'] - previous_point['y_mm']
                length = abs(dx) + abs(dy)
                if length > approach_distance_mm + 6:
                    approach = dict(point, x_mm=point['x_mm'] - (approach_distance_mm if dx > 0 else -approach_distance_mm if dx < 0 else 0),
                                    y_mm=point['y_mm'] - (approach_distance_mm if dy > 0 else -approach_distance_mm if dy < 0 else 0),
                                    work_id=0, move_rpm=move_rpm)
                    new_points.append(approach)
                point = dict(point, move_rpm=approach_rpm)
            new_points.append(point)
        new_segments.append(dict(segment, first_point_index=first,
                                 last_point_index=len(new_points) - 1))
    route['points'] = new_points
    route['segments'] = new_segments
    route['test_mode'] = 'chassis_route_dry_work'
    if type(test_ignore_turn_obstacles) is not bool:
        raise ValueError('test_ignore_turn_obstacles must be boolean')
    route['test_ignore_turn_obstacles'] = test_ignore_turn_obstacles
    if (not isinstance(test_turn_sweep_ignored_cells, (list, tuple)) or
            any(type(cell) is not int or not 0 <= cell < 25 for cell in test_turn_sweep_ignored_cells)):
        raise ValueError('test_turn_sweep_ignored_cells must be grid IDs in [0,24]')
    route['test_turn_sweep_ignored_cells'] = list(dict.fromkeys(test_turn_sweep_ignored_cells))
    if stage_limit == 1:
        route['execution_scope'] = 'first_segment_only'
    route['approach_rpm'] = approach_rpm
    route['approach_distance_mm'] = approach_distance_mm
    route['stop_strategy'] = stop_strategy
    if stop_strategy == 'controller_deceleration':
        route = compact_straight_points(route)
    configure_work_heading_mode(route, work_heading_mode_enabled)
    if footprint is not None:
        from .chassis_geometry import validate_footprint
        validate_footprint(footprint)
        route['footprint'] = deepcopy(footprint)
    upload_payload(route)
    return route


def compact_straight_points(route):
    """Remove only redundant straight-through points, preserving the exact path.

    Never remove a work gate, turn, reversal, heading/speed change or segment
    endpoint. Speeds apply to the incoming leg. Equal-speed, forward-collinear
    legs have identical swept geometry after merging; no corner is cut.
    """
    result = deepcopy(route)
    points, segments = [], []
    removed = 0
    for segment in route['segments']:
        first = len(points)
        start = points[-1] if points else route['initial_pose']
        kept = []
        for point in route['points'][segment['first_point_index']:segment['last_point_index'] + 1]:
            while kept:
                a = kept[-2] if len(kept) > 1 else start
                b = kept[-1]
                dx1, dy1 = b['x_mm'] - a['x_mm'], b['y_mm'] - a['y_mm']
                dx2, dy2 = point['x_mm'] - b['x_mm'], point['y_mm'] - b['y_mm']
                straight = ((dx1 == dx2 == 0 and dy1 * dy2 > 0) or
                            (dy1 == dy2 == 0 and dx1 * dx2 > 0))
                same_motion = (a['yaw_deg'] == b['yaw_deg'] == point['yaw_deg'] and
                               all(b[k] == point[k] for k in ('move_rpm', 'turn_rpm')))
                if b['work_id'] or b.get('flags', 0) or not straight or not same_motion:
                    break
                kept.pop()
                removed += 1
            kept.append(deepcopy(point))
        points.extend(kept)
        segments.append(dict(segment, first_point_index=first, last_point_index=len(points)-1))
    result.update(points=points, segments=segments, redundant_points_removed=removed)
    upload_payload(result)
    return result


def first_segment_only(route):
    """Upload only stage 1; firmware cannot proceed into any later stage."""
    if route.get('test_mode') != 'chassis_route_dry_work' or len(route['segments']) != 8:
        raise ValueError('expected compiled eight-stage chassis test')
    result = deepcopy(route)
    result['points'] = result['points'][:result['segments'][0]['last_point_index']+1]
    result['segments'] = result['segments'][:1]
    result['execution_scope'] = 'first_segment_only'
    upload_payload(result)
    return result
