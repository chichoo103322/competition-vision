"""Verify Pi wire directions against supplied production C and simulated wheels.

No hardware access. Expectations are explicit body movements, independent of
the firmware's map-to-body implementation and of the Python direction review.
"""
import argparse
import json
from pathlib import Path
import tempfile

from pi_vision.electrical_coordinates import direction_review, ELECTRICAL_FRAME
from pi_vision.electrical_link import ElectricalClient
from pi_vision.electrical_protocol import upload_payload
from tools.chassis_route_sim import build_library, PlantTransport


def cases():
    # yaw, +X body displacement, +Y body displacement
    table = [(0, (500, 0), (0, 500)), (90, (0, -500), (500, 0)),
             (180, (-500, 0), (0, -500)), (-90, (0, 500), (-500, 0))]
    for yaw, x_body, y_body in table:
        for axis, vector, body in [('X', (500, 0), x_body), ('Y', (0, 500), y_body)]:
            for sign in (1, -1):
                yield dict(name=f'yaw{yaw}_{sign:+d}{axis}', yaw=yaw,
                           targets=[(sign*vector[0], sign*vector[1], yaw)],
                           expected=(sign*body[0], sign*body[1], 0),
                           phases=[2 if body[0] else 3])
    yield dict(name='turn_before_move', yaw=0, targets=[(500, 0, 90)],
               expected=(0, -500, 90), phases=[1, 3])
    yield dict(name='split_move_then_turn', yaw=0, targets=[(500, 0, 0), (500, 0, 90)],
               expected=(500, 0, 90), phases=[2, 1])


def check_case(firmware, folder, case):
    lib = build_library(firmware, folder)
    if lib.sim_metric(11) != -1:
        raise AssertionError('firmware imu_yaw_sign must remain -1')
    transport = PlantTransport(lib, 'none')
    client = ElectricalClient(transport, clock=transport.clock)
    route = dict(route_id=1, coordinate_frame=ELECTRICAL_FRAME,
                 initial_pose=dict(x_mm=1000, y_mm=1000, yaw_deg=case['yaw']),
                 valid_for_motion=True, points=[
                     dict(x_mm=1000+x, y_mm=1000+y, yaw_deg=yaw, move_rpm=40,
                          turn_rpm=20, work_id=i+1)
                     for i, (x, y, yaw) in enumerate(case['targets'])])
    client.upload(route)
    client.start(1)
    phases, phase_remaining, released = [], [], set()
    while transport.clock() < 90:
        s = client.status()
        if s['state'] in (5, 6):
            raise AssertionError(f"controller stopped/fault: {s}")
        if s['phase'] in (1, 2, 3) and (not phases or phases[-1] != s['phase']):
            phases.append(s['phase'])
            phase_remaining.append(lib.sim_metric(7))
        if s['state'] == 3 and s['point_index'] not in released:
            target = route['points'][s['point_index']]
            if abs(s['x_mm']-target['x_mm']) > 8 or abs(s['y_mm']-target['y_mm']) > 8:
                raise AssertionError('reported map position incorrect')
            if abs((s['yaw_deg']-target['yaw_deg']+180) % 360-180) > 2:
                raise AssertionError('reported map heading incorrect')
            released.add(s['point_index'])
            client.work_done(1, s['point_index'], success=True)
        if s['state'] == 4:
            break
        transport.sleep(.02)
    else:
        raise AssertionError('direction case timed out')
    actual = [lib.sim_metric(k) for k in (2, 3, 10)]
    if any(abs(a-b) > tol for a, b, tol in zip(actual, case['expected'], (8, 8, 2))):
        raise AssertionError(f"body displacement/yaw {actual}, expected {case['expected']}")
    prefix = len(case['phases'])
    # Rotation settles within a tolerance, not at mathematical exact 90 deg.
    # route.c may then correct a small residual on the other translation axis.
    residual_ok = all(p in (2, 3) and abs(mm) <= 15
                      for p, mm in zip(phases[prefix:], phase_remaining[prefix:]))
    if phases[:prefix] != case['phases'] or not residual_ok or len(released) != len(route['points']):
        raise AssertionError(f"incorrect movement order: {phases}, expected {case['phases']}")
    return dict(name=case['name'], result='passed', expected_body=case['expected'],
                observed_body=actual, phases=phases, phase_remaining_mm=phase_remaining,
                review=direction_review(upload_payload(route)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('firmware', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    results = []
    with tempfile.TemporaryDirectory(prefix='electrical-directions-') as directory:
        for index, case in enumerate(cases()):
            folder = Path(directory) / str(index)
            folder.mkdir()
            results.append(check_case(args.firmware.resolve(), folder, case))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dict(software_simulation_only=True, cases=results),
                                      ensure_ascii=False, indent=2)+'\n')
    print(f'{len(results)} direction cases passed; production C with simulated UART/CAN/wheels only')


if __name__ == '__main__':
    main()
