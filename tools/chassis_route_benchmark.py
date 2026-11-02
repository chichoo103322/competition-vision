"""Offline A/B replay of a saved plan against the supplied STM32 C controller.

No device/serial access. Wheel/IMU feedback is simulated; times are NOT real
vehicle predictions. Compare the saved route with compact routes at fixed RPM.
"""
import argparse
import ctypes as C
import json
from pathlib import Path
import tempfile

from pi_vision.electrical_routes import compile_chassis_test
from tools.chassis_route_sim import build_library, PlantTransport, ObservedClient
from tools.electrical_route_test import run_closed_loop, validate_geometry, validate_profile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('firmware', type=Path)
    parser.add_argument('--prepared', required=True, type=Path)
    parser.add_argument('--profile', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--cruise-rpm', type=int, nargs='+', default=[100, 120])
    parser.add_argument('--fault', choices=('none', 'imu', 'stall', 'stop'), default='none')
    args = parser.parse_args()
    prepared = json.loads(args.prepared.read_text())
    state = prepared['state']
    profile = json.loads(args.profile.read_text())
    config = json.loads((Path(__file__).resolve().parents[1] / 'config/field_map_national_2027.json').read_text())
    cases = [('saved', prepared['route'])]
    for rpm in args.cruise_rpm:
        updated = dict(profile, move_rpm=rpm, stop_strategy='controller_deceleration')
        validate_profile(updated)
        route = compile_chassis_test(state['mission_plan'], state['navigation']['nodes'],
                                     route_id=prepared['route']['route_id'], **updated)
        cases.append((f'compact_{rpm}', route))
    reports = []
    for name, route in cases:
        validate_geometry(state, route, config)
        with tempfile.TemporaryDirectory(prefix='route-benchmark-') as directory:
            lib = build_library(args.firmware.resolve(), Path(directory))
            transport = PlantTransport(lib, args.fault)
            client = ObservedClient(transport, clock=transport.clock)
            lib.sim_select.argtypes = [C.c_uint32]
            lib.sim_select.restype = C.c_uint32
            if lib.sim_select(route['start_zone']) != 0:
                raise RuntimeError('simulated area selection rejected')
            selected = client.wait_start_area(5)
            result = run_closed_loop(client, route, station_dwell_s=2, poll_s=.2,
                                     timeout_s=600, log=[], clock=transport.clock,
                                     sleep=transport.sleep, start_request_id=selected['request_id'])
            expected = (result['result'] == 'done' and transport.slow_seen and
                        result['test_stations_completed'] == len(route['segments'])) if args.fault == 'none' else (
                        result['result'] == 'failed' and result['controller_stop_confirmed'])
            report = dict(name=name, points=len(route['points']), result=result['result'],
                          simulated_seconds=transport.clock(),
                          closed_loop_subactions=lib.sim_metric(5),
                          deceleration_observed=transport.slow_seen,
                          work_stops_verified=len({m['point_index'] for m in transport.wait_metrics}),
                          passed=expected, route=route, details=result)
            reports.append(report)
            print(json.dumps({k: v for k, v in report.items() if k not in ('route', 'details')},
                             ensure_ascii=False), flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dict(software_simulation_only=True, fault=args.fault,
        source_prepared=str(args.prepared), reports=reports), ensure_ascii=False, indent=2)+'\n')
    if not all(r['passed'] for r in reports):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
