"""One stationary material approach/alignment/lower trial; claw remains externally controlled."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time
from urllib.request import Request, urlopen

import numpy as np

from pi_vision.electrical_link import ElectricalClient
from tools.arm_calibrate import StationaryCalibration


def corrected_target(arm, pixel, reference, matrix, radius0):
    error = np.asarray(pixel, dtype=float)-reference
    radial, tangent = np.asarray(matrix, dtype=float)@error
    radius = radius0+arm['reach_deg']*113.1/360
    distance = math.hypot(radial, tangent)
    if not math.isfinite(distance) or radius <= 0 or radius+radial <= 0:
        raise ValueError('invalid correction geometry')
    scale = min(1., 4/max(distance, 1e-9))
    # One-degree maximum correction per stopped observation.
    lo, hi = 0., scale
    for _ in range(32):
        mid = (lo+hi)/2
        if abs(math.degrees(math.atan2(tangent*mid, radius+radial*mid))) <= 1:
            lo = mid
        else:
            hi = mid
    radial, tangent = radial*lo, tangent*lo
    yaw = arm['yaw_deg']/3+math.degrees(math.atan2(tangent, radius+radial))
    reach = math.hypot(radius+radial, tangent)-radius0
    if not -3 <= yaw <= 3 or not -8 <= reach <= 0:
        raise RuntimeError('first approach trial exceeds yaw +/-3deg or reach -8..0mm')
    return yaw, reach, math.hypot(radial, tangent)


class PickHeightTrial(StationaryCalibration):
    def wait_arm(self, state, timeout=45):
        # Same identity checks as calibration, with PICK's LOWERING transition.
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            arm = self.client.arm_status()
            if tuple(arm[k] for k in ('route_id', 'point_index', 'work_id')) != self.context:
                raise RuntimeError('arm context changed')
            if not arm['standalone_test_mode'] or not arm['origins_valid'] or arm['fault']:
                raise RuntimeError('arm mode, origins or fault invalid: '+str(arm))
            if arm['command_id'] != self.command:
                raise RuntimeError('arm command replaced')
            if arm['state_name'] == state:
                self.record('arm_arrived', arm=arm)
                return arm
            if arm['state_name'] not in ('PREPARING', 'ALIGNING', 'LOWERING'):
                raise RuntimeError('unexpected state: '+arm['state_name'])
            time.sleep(.12)
        raise TimeoutError('motion did not finish')

    def run_trial(self, test_id, mapping):
        arm = self.client.arm_status()
        if (arm['state_name'] not in ('READY', 'READY_LEAVE') or not arm['origins_valid']
                or not arm['standalone_test_mode'] or arm['fault']):
            raise RuntimeError('standalone READY/READY_LEAVE with valid origin required')
        if any(mapping['reference'].get(k) != self.reference.get(k)
               for k in ('camera_source', 'hsv_profile_sha256', 'resolution', 'origin_radius_mm')):
            raise RuntimeError('mapping/reference identity mismatch')
        self.context = (test_id, 0, 102)
        req = Request('http://127.0.0.1:8090/material-target', data=b'{"color_id":2}',
                      headers={'Content-Type':'application/json'})
        with urlopen(req, timeout=3):
            pass
        self.started = True
        self.client.arm_test_begin(test_id, 102)
        arm = self.wait_arm('WAIT_VISION')
        total = 0.
        aligned = None
        self.record('waiting_yellow')
        for index in range(9):
            observation = self.observe(time.monotonic()+.3, timeout=20, wait_for_target=True)
            error = np.asarray(observation['pixel'])-self.reference['reference_pixel']
            self.record('visual_check', pixel=observation['pixel'], error_px=error.tolist())
            if max(abs(error)) <= 3 and aligned is not None:
                # Fresh continuous observations only, then lower using the firmware's configured stroke.
                self.command += 1
                self.client.arm_pick(*self.context, self.command, aligned)
                lower = self.wait_arm('WAIT_GRIP')
                moved = (arm['z_deg']-lower['z_deg'])/9
                if not 9.8 <= moved <= 10.2:
                    raise RuntimeError('firmware lowered by unexpected distance: '+str(moved))
                with urlopen('http://127.0.0.1:8090/frame.jpg', timeout=3) as response:
                    (self.output/'at_grasp_height.jpg').write_bytes(response.read())
                report = dict(result='at_grasp_height', lowered_mm=moved,
                    pixel_before_lower=observation['pixel'], error_before_lower_px=error.tolist(),
                    status=lower, physical_grasp_completed=False, gripper_command_sent=False)
                (self.output/'result.json').write_text(json.dumps(report,indent=2))
                self.record('holding_at_grasp_height', **report)
                return report
            if index == 8:
                raise RuntimeError('alignment corrections exhausted')
            if max(abs(error)) <= 3:
                yaw, reach, distance = arm['yaw_deg']/3, min(0., arm['reach_deg']*113.1/360), 0.
            else:
                yaw, reach, distance = corrected_target(arm, observation['pixel'],
                    self.reference['reference_pixel'], mapping['matrix'], self.reference['origin_radius_mm'])
            total += distance
            if total > 16:
                raise RuntimeError('approach cumulative correction budget exhausted')
            self.command += 1
            self.record('align_request', yaw_deg=yaw, reach_mm=reach, total_mm=total)
            self.client.arm_align(*self.context,self.command,0,yaw*3,reach*360/113.1)
            arm = self.wait_arm('ALIGNED')
            if arm['aligned_command_id'] != self.command or abs(arm['z_deg']/9) > .1:
                raise RuntimeError('alignment identity/observation height invalid')
            aligned = self.command
        raise RuntimeError('no verified alignment')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--reference', required=True, type=Path)
    parser.add_argument('--mapping', required=True, type=Path)
    parser.add_argument('--test-id', required=True, type=int)
    args = parser.parse_args()
    args.output.mkdir()
    reference=json.loads(args.reference.read_text())
    mapping=json.loads(args.mapping.read_text())
    import serial
    with (Path(tempfile.gettempdir())/f'robot-preview-start-{os.getuid()}.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        subprocess.run(['systemctl','--user','stop','chassis-button'],check=True)
        with serial.Serial(args.port,115200,timeout=.05,write_timeout=1,exclusive=True) as port:
            run=PickHeightTrial(ElectricalClient(port),args.output,reference)
            try:
                run.run_trial(args.test_id,mapping)
            except BaseException as exc:
                run.record('failed',error=str(exc))
                if run.started:
                    try:
                        run.client.stop(0)
                        run.record('after_stop',arm=run.client.arm_status())
                    except Exception as stop_error:
                        run.record('stop_unconfirmed',error=str(stop_error))
                raise


if __name__=='__main__':
    main()
