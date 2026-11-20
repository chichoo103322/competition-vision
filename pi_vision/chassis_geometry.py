"""Measured chassis outer bounds relative to its localisation reference point.

Positive forward/left follow the STM32 body convention. Four independent
extents accommodate a lidar mounting plate without moving the pose reference.
"""
import math


def validate_footprint(footprint):
    for key in ('front_mm', 'rear_mm', 'left_mm', 'right_mm', 'margin_mm'):
        value = footprint.get(key)
        minimum = 0 if key == 'margin_mm' else 1
        if type(value) not in (int, float) or not math.isfinite(value) or not minimum <= value <= 1000:
            raise ValueError(f'footprint.{key} requires a measured finite value in [{minimum}, 1000] mm')


def obstacle_radius_m(footprint):
    validate_footprint(footprint)
    return (math.hypot(max(footprint['front_mm'], footprint['rear_mm']),
                       max(footprint['left_mm'], footprint['right_mm'])) + footprint['margin_mm']) / 1000


def boundary_gaps_mm(pose, footprint, boundary):
    """Exact rectangular bounds plus margin at a fixed heading, any map frame."""
    validate_footprint(footprint)
    theta = math.radians(pose['yaw_deg'])
    c, s = math.cos(theta), math.sin(theta)
    corners = [(pose['x_mm'] + c*f - s*l, pose['y_mm'] + s*f + c*l)
               for f in (-footprint['rear_mm'], footprint['front_mm'])
               for l in (-footprint['right_mm'], footprint['left_mm'])]
    x, y, w, h = boundary
    margin = footprint['margin_mm']
    return dict(left=min(p[0] for p in corners)-x-margin,
                right=x+w-max(p[0] for p in corners)-margin,
                bottom=min(p[1] for p in corners)-y-margin,
                top=y+h-max(p[1] for p in corners)-margin)


def validate_fixed_heading_boundary(route, footprint, boundary):
    """Endpoints suffice for translations: each corner travels a straight line.

    Reject heading changes; endpoint rectangles cannot bound a turning sweep.
    """
    initial = route['initial_pose']
    for index, pose in enumerate([initial] + route['points']):
        if abs((pose['yaw_deg']-initial['yaw_deg']+180) % 360-180) > 1e-9:
            raise ValueError('rectangular boundary check requires fixed chassis heading')
        gaps = boundary_gaps_mm(pose, footprint, boundary)
        if min(gaps.values()) < -1e-6:
            raise ValueError(f'chassis footprint outside field at pose {index}: gaps_mm={gaps}')


def validate_fixed_heading_obstacles(route, footprint, rectangles):
    """Check the complete swept box for cardinal, fixed-heading translations."""
    validate_footprint(footprint)
    previous = route['initial_pose']
    for pose in route['points']:
        if abs((pose['yaw_deg']-previous['yaw_deg']+180) % 360-180) > 1e-9:
            raise ValueError('obstacle sweep requires fixed chassis heading')
        if abs(pose['yaw_deg'] % 90) > 1e-9:
            raise ValueError('obstacle sweep requires cardinal chassis heading')
        if pose['x_mm'] != previous['x_mm'] and pose['y_mm'] != previous['y_mm']:
            raise ValueError('obstacle sweep requires axis-aligned motion')
        # At cardinal headings and axis-aligned translations this swept box
        # is exact, including an asymmetric right-hand mounting plate.
        gaps = [boundary_gaps_mm(p, footprint, [0, 0, 2400, 2400]) for p in (previous, pose)]
        xmin = min(g['left'] for g in gaps)
        xmax = 2400-min(g['right'] for g in gaps)
        ymin = min(g['bottom'] for g in gaps)
        ymax = 2400-min(g['top'] for g in gaps)
        for x, y, w, h in rectangles:
            if max(xmin, x) <= min(xmax, x+w) and max(ymin, y) <= min(ymax, y+h):
                raise ValueError('measured chassis sweep intersects an occupied or excluded area')
        previous = pose


def _corners(pose, footprint, extra=0):
    theta = math.radians(pose['yaw_deg'])
    c, s = math.cos(theta), math.sin(theta)
    front = footprint['front_mm'] + footprint['margin_mm'] + extra
    rear = footprint['rear_mm'] + footprint['margin_mm'] + extra
    left = footprint['left_mm'] + footprint['margin_mm'] + extra
    right = footprint['right_mm'] + footprint['margin_mm'] + extra
    return [(pose['x_mm'] + c*f - s*l, pose['y_mm'] + s*f + c*l)
            for f, l in ((-rear, -right), (front, -right), (front, left), (-rear, left))]


def _overlaps_rect(polygon, rect):
    x, y, w, h = rect
    corners = [(x, y), (x+w, y), (x+w, y+h), (x, y+h)]
    axes = [(1, 0), (0, 1)]
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        edge = (b[0]-a[0], b[1]-a[1])
        axes.append((-edge[1], edge[0]))
    for ax, ay in axes:
        p = [px*ax + py*ay for px, py in polygon]
        r = [px*ax + py*ay for px, py in corners]
        if max(p) < min(r) or max(r) < min(p):
            return False
    return True


def _check_pose(pose, footprint, boundary, rectangles, interpolation_extra=0):
    polygon = _corners(pose, footprint, interpolation_extra)
    x, y, w, h = boundary
    if (min(p[0] for p in polygon) < x - 1e-6 or max(p[0] for p in polygon) > x+w+1e-6 or
            min(p[1] for p in polygon) < y - 1e-6 or max(p[1] for p in polygon) > y+h+1e-6):
        raise ValueError(f'chassis footprint outside field during route/turn sweep: pose={pose}')
    for rect in rectangles:
        if _overlaps_rect(polygon, rect):
            raise ValueError('measured chassis sweep intersects an occupied or excluded area: '
                             f'pose={pose}, rect_mm={rect}')


def validate_route_footprint(route, footprint, boundary, rectangles, *, turn_rectangles=None):
    """Check translations and work-station rotations with sampled expanded rectangles.

    Translation samples are at most 5 mm apart and turn samples at most 1 degree
    apart. Small extra inflation bounds the unsampled motion between samples.
    """
    validate_footprint(footprint)
    turn_rectangles = rectangles if turn_rectangles is None else turn_rectangles
    previous = dict(route['initial_pose'])
    previous_heading = previous['yaw_deg']
    _check_pose(previous, footprint, boundary, rectangles)

    def check_turn(x, y, start_heading, target_heading):
        delta = (target_heading-start_heading+180) % 360-180
        if abs(delta) < 1e-9:
            _check_pose(dict(x_mm=x, y_mm=y, yaw_deg=start_heading), footprint, boundary, rectangles)
            return
        turns = max(1, math.ceil(abs(delta)))
        radius = math.hypot(max(footprint['front_mm'], footprint['rear_mm']) + footprint['margin_mm'],
                            max(footprint['left_mm'], footprint['right_mm']) + footprint['margin_mm'])
        # Nearest angular sample differs by at most 0.5 degree; bound the
        # displacement of every corner by that arc length in every direction.
        extra = radius*math.radians(.5) + 1e-6
        # An exact half-turn may use either sign in the controller. Cover both.
        deltas = (delta, -delta) if abs(delta) == 180 else (delta,)
        for sweep in deltas:
            for i in range(turns+1):
                yaw = start_heading + sweep*i/turns
                pose = dict(x_mm=x, y_mm=y, yaw_deg=yaw)
                _check_pose(pose, footprint, boundary, turn_rectangles, interpolation_extra=extra)

    for point in route['points']:
        heading = point.get('travel_yaw_deg', point['yaw_deg'])
        if abs((heading-previous_heading+180) % 360-180) > 1e-9:
            check_turn(previous['x_mm'], previous['y_mm'], previous_heading, heading)
        dx, dy = point['x_mm']-previous['x_mm'], point['y_mm']-previous['y_mm']
        steps = max(1, math.ceil(math.hypot(dx, dy)/5))
        for i in range(steps+1):
            t = i/steps
            pose = dict(x_mm=previous['x_mm']+dx*t, y_mm=previous['y_mm']+dy*t,
                        yaw_deg=heading)
            _check_pose(pose, footprint, boundary, rectangles, interpolation_extra=3)
        if point.get('work_id'):
            target_heading = point.get('expected_work_yaw_deg', heading)
            check_turn(point['x_mm'], point['y_mm'], heading, target_heading)
            previous_heading = target_heading
        else:
            previous_heading = heading
        previous = dict(point, yaw_deg=previous_heading)
