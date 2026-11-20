import copy

import pytest

from pi_vision.chassis_geometry import (boundary_gaps_mm, obstacle_radius_m,
                                        validate_fixed_heading_boundary, validate_footprint,
                                        validate_fixed_heading_obstacles, validate_route_footprint)
from pi_vision.electrical_routes import compile_chassis_test
from tools.chassis_route_sim import synthetic_plan
from tools.electrical_route_test import validate_geometry


def base():
    # Test fixture only: long side forward, no plate. User's full outer bounds
    # are still awaiting confirmation, so this is not an executable profile.
    return dict(front_mm=143, rear_mm=143, left_mm=128.5, right_mm=128.5, margin_mm=2)


def test_rectangle_fits_zone_center_where_180mm_circle_does_not():
    gaps = boundary_gaps_mm(dict(x_mm=150, y_mm=150, yaw_deg=90), base(), [0, 0, 2400, 2400])
    assert gaps['left'] == 19.5
    assert gaps['bottom'] == pytest.approx(5)
    assert min(gaps.values()) > 0
    assert obstacle_radius_m(base()) > .18


@pytest.mark.parametrize('side,yaw,blocked', [
    ('front_mm', 90, False), ('rear_mm', 90, True),
    ('left_mm', 90, True), ('right_mm', 90, False),
    ('front_mm', -90, True), ('rear_mm', -90, False),
])
def test_plate_extension_on_correct_side_of_chassis(side, yaw, blocked):
    footprint = base()
    footprint[side] += 30
    route = dict(initial_pose=dict(x_mm=150, y_mm=150, yaw_deg=yaw), points=[])
    if blocked:
        with pytest.raises(ValueError, match='outside field'):
            validate_fixed_heading_boundary(route, footprint, [0, 0, 2400, 2400])
    else:
        validate_fixed_heading_boundary(route, footprint, [0, 0, 2400, 2400])


def test_cannot_ignore_turning_sweep_or_out_of_bounds_endpoint():
    route = dict(initial_pose=dict(x_mm=150, y_mm=150, yaw_deg=90),
                 points=[dict(x_mm=150, y_mm=150, yaw_deg=0)])
    with pytest.raises(ValueError, match='fixed chassis heading'):
        validate_fixed_heading_boundary(route, base(), [0, 0, 2400, 2400])
    route['points'] = [dict(x_mm=120, y_mm=150, yaw_deg=90)]
    with pytest.raises(ValueError, match='outside field'):
        validate_fixed_heading_boundary(route, base(), [0, 0, 2400, 2400])


@pytest.mark.parametrize('bad', [None, -1, float('nan'), True])
def test_incomplete_or_invalid_outer_bounds_rejected(bad):
    footprint = base()
    footprint['front_mm'] = bad
    with pytest.raises(ValueError):
        validate_footprint(footprint)


def test_zone_center_connector_and_obstacles_are_still_checked():
    state, profile, config = synthetic_plan(1, occupied24=False)
    profile.update(initial_pose=dict(x_mm=150, y_mm=150, yaw_deg=90), heading_deg=90)
    old = compile_chassis_test(state['mission_plan'], state['navigation']['nodes'], route_id=1, **profile)
    with pytest.raises(ValueError):
        validate_geometry(state, old, config)
    route = compile_chassis_test(state['mission_plan'], state['navigation']['nodes'],
                                 route_id=1, footprint=base(), **profile)
    validate_geometry(state, route, config)
    state['navigation']['occupied_cells'].append(4)
    with pytest.raises(ValueError, match='occupied'):
        validate_geometry(state, route, config)
    state['navigation']['occupied_cells'].clear()
    # A static exclusion adjacent to the connector still uses >=180mm radius.
    config = copy.deepcopy(config)
    config['floor_exclusions'].append(dict(rect_mm=[1900, 2070, 50, 50]))
    with pytest.raises(ValueError, match='occupied or excluded'):
        validate_geometry(state, route, config)


def test_measured_right_plate_swept_box_and_nearby_obstacle():
    footprint = dict(base(), right_mm=165.5)
    route = dict(initial_pose=dict(x_mm=150, y_mm=150, yaw_deg=90),
                 points=[dict(x_mm=350, y_mm=150, yaw_deg=90),
                         dict(x_mm=350, y_mm=1200, yaw_deg=90)])
    validate_fixed_heading_boundary(route, footprint, [0, 0, 2400, 2400])
    validate_fixed_heading_obstacles(route, footprint, [[550, 550, 450, 450]])
    with pytest.raises(ValueError, match='sweep intersects'):
        validate_fixed_heading_obstacles(route, footprint, [[510, 550, 20, 20]])


def test_work_station_turn_sweep_is_checked_against_obstacles():
    footprint = dict(front_mm=220, rear_mm=100, left_mm=50, right_mm=50, margin_mm=2)
    route = dict(initial_pose=dict(x_mm=1000, y_mm=1000, yaw_deg=0), points=[
        dict(x_mm=1000, y_mm=1000, yaw_deg=0, travel_yaw_deg=0,
             expected_work_yaw_deg=90, work_id=101)])
    validate_route_footprint(route, footprint, [0, 0, 2400, 2400], [])
    with pytest.raises(ValueError, match='sweep intersects'):
        validate_route_footprint(route, footprint, [0, 0, 2400, 2400], [[1130, 1130, 10, 10]])


def test_work_heading_translation_uses_previous_station_heading():
    footprint = dict(front_mm=200, rear_mm=80, left_mm=40, right_mm=40, margin_mm=2)
    route = dict(initial_pose=dict(x_mm=1000, y_mm=1000, yaw_deg=90), points=[
        dict(x_mm=1000, y_mm=1100, yaw_deg=90, travel_yaw_deg=90,
             expected_work_yaw_deg=180, work_id=101),
        dict(x_mm=1100, y_mm=1100, yaw_deg=90, travel_yaw_deg=180, work_id=0)])
    # The second leg is eastward while the vehicle remains facing west.
    validate_route_footprint(route, footprint, [0, 0, 2400, 2400], [])


def test_test_only_turn_override_does_not_skip_translation_collision():
    footprint = dict(front_mm=220, rear_mm=100, left_mm=50, right_mm=50, margin_mm=2)
    obstacle = [[1130, 1130, 10, 10]]
    turn = dict(initial_pose=dict(x_mm=1000, y_mm=1000, yaw_deg=0), points=[
        dict(x_mm=1000, y_mm=1000, yaw_deg=0, travel_yaw_deg=0,
             expected_work_yaw_deg=90, work_id=101)])
    validate_route_footprint(turn, footprint, [0, 0, 2400, 2400], obstacle,
                             turn_rectangles=[])
    translation = dict(initial_pose=dict(x_mm=1000, y_mm=1140, yaw_deg=0), points=[
        dict(x_mm=1000, y_mm=1140, yaw_deg=0, travel_yaw_deg=0, work_id=101,
             expected_work_yaw_deg=90)])
    with pytest.raises(ValueError, match='sweep intersects'):
        validate_route_footprint(translation, footprint, [0, 0, 2400, 2400], obstacle,
                                 turn_rectangles=[])
