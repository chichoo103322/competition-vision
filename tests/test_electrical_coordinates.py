import struct

import pytest

from pi_vision.electrical_coordinates import direction_review
from pi_vision.electrical_protocol import upload_payload as encode_upload
from pi_vision.electrical_coordinates import (ELECTRICAL_FRAME, nominal_to_electrical_pose,
                                             electrical_to_nominal_pose)


def upload_payload(route):
    return encode_upload(dict(coordinate_frame=ELECTRICAL_FRAME, **route))


@pytest.mark.parametrize('yaw, expected', [
    (0, [(500, 0), (0, 500)]),
    (90, [(0, -500), (500, 0)]),
    (180, [(-500, 0), (0, -500)]),
    (-90, [(0, 500), (-500, 0)]),
])
def test_wire_map_axes_match_electrical_contract(yaw, expected):
    for (x, y), (forward, left) in zip([(500, 0), (0, 500)], expected):
        route = dict(route_id=1, initial_pose=dict(x_mm=0, y_mm=0, yaw_deg=yaw),
                     points=[dict(x_mm=x, y_mm=y, yaw_deg=yaw)])
        payload = upload_payload(route)
        assert struct.unpack_from('<iii', payload, 8) == (0, 0, yaw * 100)
        assert struct.unpack_from('<iii', payload, 20) == (x, y, yaw * 100)
        leg = direction_review(payload)['legs'][0]
        assert (leg['forward_mm'], leg['left_mm']) == (forward, left)


def test_turn_first_and_separate_turn_waypoint_are_distinct():
    initial = dict(x_mm=0, y_mm=0, yaw_deg=0)
    end = dict(x_mm=500, y_mm=0, yaw_deg=90)
    one = direction_review(upload_payload(dict(route_id=1, initial_pose=initial, points=[end])))
    two = direction_review(upload_payload(dict(route_id=1, initial_pose=initial,
                                               points=[dict(end, yaw_deg=0), end])))
    assert one['legs'][0]['actions'] == [dict(action='逆时针转向', angle_deg=90),
                                       dict(action='右移', distance_mm=500)]
    assert [leg['actions'] for leg in two['legs']] == [
        [dict(action='前进', distance_mm=500)], [dict(action='逆时针转向', angle_deg=90)]]


def test_review_uses_quantized_wire_values_and_positive_180_tie():
    route = dict(route_id=1, initial_pose=dict(x_mm=10.4, y_mm=20.4, yaw_deg=0),
                 points=[dict(x_mm=510.4, y_mm=20.4, yaw_deg=-180)])
    review = direction_review(upload_payload(route))
    assert review['initial_pose'] == dict(x_mm=10, y_mm=20, yaw_deg=0)
    assert review['legs'][0]['actions'] == [dict(action='逆时针转向', angle_deg=180),
                                          dict(action='后退', distance_mm=500)]


@pytest.mark.parametrize('old, new', [
    ((2250, 2250, -90), (150, 150, 90)),
    ((2250, 150, 90), (150, 2250, -90)),
    ((0, 0, 0), (2400, 2400, -180)),
    ((2400, 2400, 180), (0, 0, 0)),
])
def test_full_pose_frame_conversion_and_inverse(old, new):
    original = dict(zip(('x_mm', 'y_mm', 'yaw_deg'), old))
    converted = nominal_to_electrical_pose(original)
    assert tuple(converted.values()) == new
    restored = electrical_to_nominal_pose(converted)
    assert restored['x_mm'] == old[0] and restored['y_mm'] == old[1]
    assert (restored['yaw_deg'] - old[2]) % 360 == 0
    assert tuple(original.values()) == old


@pytest.mark.parametrize('frame', [None, 'field_map', 'field_nominal', 'odom'])
def test_untagged_and_old_frames_cannot_be_encoded(frame):
    route = dict(route_id=1, coordinate_frame=frame,
                 initial_pose=dict(x_mm=0, y_mm=0, yaw_deg=90),
                 points=[dict(x_mm=0, y_mm=500, yaw_deg=90)])
    with pytest.raises(ValueError, match='coordinate_frame'):
        encode_upload(route)


def test_zone1_compiled_wire_goes_forward_to_qr_in_new_frame():
    from tools.chassis_route_sim import synthetic_plan
    from pi_vision.electrical_routes import compile_chassis_test
    from tools.electrical_route_test import validate_geometry
    state, profile, field_map = synthetic_plan(1, occupied24=False)
    profile['initial_pose']['yaw_deg'] = profile['heading_deg'] = 90
    route = compile_chassis_test(state['mission_plan'], state['navigation']['nodes'], route_id=9, **profile)
    validate_geometry(state, route, field_map)
    payload = encode_upload(route)
    assert struct.unpack_from('<iii', payload, 8) == (350, 350, 9000)
    review = direction_review(payload)
    first_stage_end = route['segments'][0]['last_point_index']
    first_stage = review['legs'][:first_stage_end+1]
    assert first_stage[-1]['target'] == dict(x_mm=350, y_mm=1200, yaw_deg=90)
    assert sum(p['forward_mm'] for p in first_stage) == 850
    assert all(p['left_mm'] == 0 and p['forward_mm'] >= 0 for p in first_stage)
    assert route['points'][-1]['x_mm'] == 350 and route['points'][-1]['y_mm'] == 350
    # Obstacle evidence is still in the original map; its start cell must block.
    state['navigation']['occupied_cells'].append(4)
    with pytest.raises(ValueError, match='occupied'):
        validate_geometry(state, route, field_map)
