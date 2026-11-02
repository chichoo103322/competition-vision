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
