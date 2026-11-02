"""Compile/run actual STM32 control sources with a simulated wheel/IMU plant.

No serial devices are opened. Downloaded firmware files are only read.
"""
import argparse
import ctypes as C
import json
import math
from pathlib import Path
import re
import subprocess
import tempfile

from pi_vision.electrical_link import ElectricalClient
from pi_vision.electrical_coordinates import ELECTRICAL_FRAME, nominal_xy_to_electrical_mm
from pi_vision.electrical_routes import compile_chassis_test
from tools.electrical_route_test import run_closed_loop, validate_geometry
from tools.lidar_navigation import NavigationPreview
from tools.lidar_mission import build_mission_plan, mission_status

ROOT = Path(__file__).resolve().parents[1]


def synthetic_plan(zone=2, occupied24=True):
    config = json.loads((ROOT / 'config/field_map_national_2027.json').read_text())
    nav = NavigationPreview(config)
    pose = [2.05, 2.05 if zone == 1 else .35, 0]
    nav.configure(dict(start_zone=zone, pose=pose, pose_source='manual_stationary'))
    points = []
    if occupied24:
        dx, dy = .30 - pose[0], 2.10 - pose[1]
        points = [[math.atan2(dy, dx), math.hypot(dx, dy), 20]]
    for seq in range(1, 4):
        nav.ingest(dict(seq=seq, points=points), seq / 10)
    evidence = nav.snapshot(.31, True)
    plan = build_mission_plan(nav, evidence, 3, 'synthetic-scans-only')
    state = dict(valid=True, age_ms=10, session_id='synthetic-scans-only', frame=dict(seq=3),
                 navigation=evidence, mission_plan=mission_status(plan, evidence, True))
    x, y = nominal_xy_to_electrical_mm(*pose[:2])
    profile = dict(coordinate_frame=ELECTRICAL_FRAME, initial_pose=dict(x_mm=x, y_mm=y, yaw_deg=0),
                   heading_deg=0, move_rpm=40, turn_rpm=20, approach_rpm=12, approach_distance_mm=120)
    return state, profile, config


def build_library(firmware, directory):
    sources = ['Core/Src/route.c', 'Core/Src/chassis_heading.c', 'Core/Src/chassis_motion.c',
               'Core/Src/chassis_debug.c', 'imu/imu.c']
    harness = (firmware / 'tests/heading/harness.c').read_text()
    harness = harness.replace('__declspec(dllexport)', '')
    harness = re.sub(r'void \*mem(?:set|cpy)\([^\n]*\)\n\{[^}]*\}\n', '', harness)
    old = 'if(debug) Chassis_DebugTask(); else Chassis_Task();'
    if old not in harness:
        raise ValueError('supplied firmware harness changed; review adapter')
    harness = '#include "route.h"\n' + harness.replace(old,
              'if(debug==2) Route_Task(); else if(debug) Chassis_DebugTask(); else Chassis_Task();')
    harness += (ROOT / 'tools/chassis_route_host.c').read_text()
    (directory / 'harness.c').write_text(harness)
    library = directory / 'chassis.so'
    has_area = 'Route_RequestStartArea' in (firmware / 'Core/Inc/route.h').read_text()
    subprocess.run(['clang', '-shared', '-fPIC', '-std=c99', '-U__weak', '-D__weak=',
                    *(['-DSIM_START_AREA'] if has_area else []),
                    '-include', str(firmware / 'tests/heading/mock.h'),
                    '-I' + str(firmware / 'Core/Inc'), '-I' + str(firmware / 'imu'),
                    '-I' + str(firmware / 'can_zdt'), *(str(firmware / s) for s in sources),
                    str(directory / 'harness.c'), '-o', str(library)], check=True)
    lib = C.CDLL(str(library))
    lib.sim_step.argtypes = [C.c_uint32]
    lib.sim_feed.argtypes = [C.c_void_p, C.c_uint16]
    lib.sim_read.argtypes = [C.c_void_p]
    lib.sim_read.restype = C.c_uint32
    lib.sim_tick.restype = C.c_uint32
    lib.sim_metric.argtypes = [C.c_uint32]
    lib.sim_metric.restype = C.c_float
    lib.sim_fault.argtypes = [C.c_uint32]
    lib.sim_init()
    return lib


class PlantTransport:
    def __init__(self, lib, fault):
        self.lib, self.fault = lib, fault
        self.buffer = bytearray()
        self.wait_metrics = []
        self.approaches = []
        self.injected = False
        self.peak = 0.0
        self.slow_seen = False

    def clock(self):
        return self.lib.sim_tick() / 1000

    def step(self, ms):
        # Sample actual production controller speed and its remaining distance.
        for _ in range(ms):
            self.lib.sim_step(1)
            rpm = abs(self.lib.sim_metric(6))
            self.peak = max(self.peak, rpm)
            if self.peak > 8 and 0 < abs(self.lib.sim_metric(7)) < 30 and 0.5 < rpm < 8:
                self.slow_seen = True
            if self.fault != 'none' and self.clock() >= 2 and not self.injected:
                self.injected = True
                self.lib.sim_fault({'imu': 1, 'stall': 2, 'stop': 3}[self.fault])

    def sleep(self, seconds):
        self.step(max(1, round(seconds*1000)))

    def write(self, packet):
        self.lib.sim_feed(packet, len(packet))
        self.step(1)
        return len(packet)

    def read(self, size):
        self.step(5)
        buf = C.create_string_buffer(65536)
        n = self.lib.sim_read(buf)
        self.buffer.extend(buf.raw[:n])
        data = bytes(self.buffer[:min(size, 19)])
        del self.buffer[:len(data)]
        return data


class ObservedClient(ElectricalClient):
    def status(self):
        status = super().status()
        if status['state'] == 3:
            metrics = dict(point_index=status['point_index'], fastest_wheel_rpm=self.transport.lib.sim_metric(0),
                           settled_ms=self.transport.lib.sim_metric(1))
            self.transport.wait_metrics.append(metrics)
            if metrics['fastest_wheel_rpm'] > 1.01 or metrics['settled_ms'] < 250:
                raise RuntimeError('simulated wheels not settled at WAIT_WORK')
        return status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('firmware', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--zone', type=int, choices=(1, 2), default=2)
    parser.add_argument('--heading', type=int, choices=(-180, -90, 0, 90, 180), default=0)
    parser.add_argument('--fault', choices=('none', 'imu', 'stall', 'stop'), default='none')
    parser.add_argument('--key-mode', action='store_true', help='exercise new START_AREA/ACK/token upload path')
    args = parser.parse_args()
    state, profile, config = synthetic_plan(args.zone)
    profile['initial_pose']['yaw_deg'] = profile['heading_deg'] = args.heading
    route = compile_chassis_test(state['mission_plan'], state['navigation']['nodes'], route_id=1, **profile)
    validate_geometry(state, route, config)
    with tempfile.TemporaryDirectory(prefix='chassis-route-sim-') as folder:
        lib = build_library(args.firmware.resolve(), Path(folder))
        transport = PlantTransport(lib, args.fault)
        client = ObservedClient(transport, clock=transport.clock)
        selected = None
        if args.key_mode:
            lib.sim_select.argtypes = [C.c_uint32]
            lib.sim_select.restype = C.c_uint32
            if lib.sim_select(args.zone) != 0:
                raise RuntimeError('simulated key selection rejected')
            selected, route = client.plan_after_key(
                lambda signal, check: compile_chassis_test(state['mission_plan'], state['navigation']['nodes'],
                                                          route_id=1, **profile))
        result = run_closed_loop(client, route, station_dwell_s=2, poll_s=.2, timeout_s=600,
                                 log=[], clock=transport.clock, sleep=transport.sleep,
                                 start_request_id=None if selected is None else selected['request_id'])
        result.update(software_simulation_only=True, route=route,
                      source='production route/heading/motion/IMU, simulated UART/CAN/wheels',
                      synthetic_radar_occupied=state['navigation']['occupied_cells'],
                      simulated_seconds=transport.clock(), wait_metrics=transport.wait_metrics,
                      peak_wheel_command_rpm=lib.sim_metric(4),
                      closed_loop_subactions=lib.sim_metric(5), deceleration_observed=transport.slow_seen,
                      net_body_displacement_mm=[lib.sim_metric(2), lib.sim_metric(3)])
        result['start_area_signal'] = selected
        if args.fault == 'none' and (result['result'] != 'done' or not transport.slow_seen):
            result['simulation_assertion_failed'] = True
        if args.fault != 'none' and (result['result'] != 'failed' or not result['controller_stop_confirmed']):
            result['simulation_assertion_failed'] = True
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('route', 'log', 'wait_metrics')}, ensure_ascii=False))
    if result.get('simulation_assertion_failed'):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
