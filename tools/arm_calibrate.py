"""Stationary arm/camera calibration through one UART owner and shared camera state."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import time
import subprocess
import tempfile
from urllib.request import Request, urlopen

import numpy as np

from pi_vision.electrical_link import ElectricalClient


def fit_correction(displacements_mm, pixel_changes):
    motion = np.asarray(displacements_mm, dtype=float)
    pixels = np.asarray(pixel_changes, dtype=float)
    if (motion.shape != pixels.shape or motion.ndim != 2 or motion.shape[1] != 2
            or len(motion) < 3 or not np.isfinite(motion).all() or not np.isfinite(pixels).all()
            or np.linalg.matrix_rank(motion) != 2):
        raise ValueError('independent finite radial/yaw samples required')
    jacobian = np.linalg.lstsq(motion, pixels, rcond=None)[0].T
    if not np.isfinite(jacobian).all() or np.linalg.cond(jacobian) > 20:
        raise ValueError('camera-to-motion mapping singular or poorly conditioned')
    residual = np.linalg.norm(pixels-motion@jacobian.T, axis=1)
    if max(residual) > 2:
        raise ValueError('movement/pixel samples disagree by more than 2 pixels')
    return -np.linalg.inv(jacobian), jacobian, residual


def relative_displacement(pose, baseline, origin_radius):
    def point(value):
        radius = origin_radius+value['reach_mm']
        angle = math.radians(value['yaw_deg'])
        return np.array([radius*math.cos(angle), radius*math.sin(angle)])
    return point(pose)-point(baseline)


class StationaryCalibration:
    def __init__(self, client, output, reference):
        self.client, self.output, self.reference = client, output, reference
        self.context = None
        self.command = 1
        self.camera_session = None
        self.samples = []
        self.started = False

    def record(self, event, **data):
        item = dict(event=event, unix_s=time.time(), **data)
        with (self.output/'events.jsonl').open('a') as f:
            f.write(json.dumps(item)+'\n')
        print(json.dumps(item), flush=True)

    def snapshot(self):
        with urlopen('http://127.0.0.1:8090/state', timeout=3) as r:
            return json.load(r)

    def wait_arm(self, state, timeout=45):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            arm = self.client.arm_status()
            if self.context and tuple(arm[k] for k in ('route_id', 'point_index', 'work_id')) != self.context:
                raise RuntimeError('arm test context changed')
            if not arm['standalone_test_mode'] or not arm['origins_valid'] or arm['fault']:
                raise RuntimeError('arm mode, origins or motor fault invalid: '+str(arm))
            if arm['command_id'] != self.command:
                raise RuntimeError('another command replaced this test')
            if arm['state_name'] == state:
                self.record('arm_arrived', arm=arm)
                return arm
            if arm['state_name'] not in ('PREPARING', 'ALIGNING', 'PARKING'):
                raise RuntimeError('unexpected arm state: '+arm['state_name'])
            time.sleep(.15)
        raise TimeoutError('arm arrival timed out')

    def observe(self, floor, timeout=8, wait_for_target=False):
        deadline = time.monotonic()+timeout
        points, records, seen = [], [], set()
        while time.monotonic() < deadline:
            state = self.snapshot()
            capture, obs = state['capture'], state['material_observation']
            now = time.monotonic()
            if (state.get('camera_source') != self.reference['camera_source']
                    or state.get('hsv_profile_sha256') != self.reference['hsv_profile_sha256']
                    or [state['preview']['source_width'], state['preview']['source_height']] != self.reference['resolution']
                    or state.get('material_backend') != 'HSVMaterialDetector'
                    or state.get('material_search_color') != 2
                    or state.get('material_selection') != 'unique_color_in_search_window'
                    or not capture['valid']
                    or not 0 <= now-capture['acquired_monotonic_s'] < 1):
                raise RuntimeError('camera identity, acquisition or requested yellow color changed')
            session = state['session_id']
            if self.camera_session is not None and self.camera_session != session:
                raise RuntimeError('camera restarted')
            self.camera_session = session
            if (obs['valid'] and obs['acquired_monotonic_s'] > floor
                    and 0 <= now-obs['acquired_monotonic_s'] < 1
                    and obs['source_sequence'] not in seen):
                seen.add(obs['source_sequence'])
                materials = state['materials']
                if (len(materials) != 1 or materials[0]['color_id'] != 2
                        or materials[0]['point_kind'] != 'top_face_ellipse_center'):
                    if wait_for_target:
                        points, records = [], []
                        time.sleep(.12)
                        continue
                    raise RuntimeError('stationary yellow top face missing or ambiguous')
                m = materials[0]
                points.append([m['center_x'], m['center_y']])
                records.append(dict(sequence=obs['source_sequence'], acquired=obs['acquired_monotonic_s']))
                if len(points) >= 7 and records[-1]['acquired']-records[0]['acquired'] >= .6:
                    if np.ptp(points, axis=0).max() > 3:
                        raise RuntimeError('material/turntable is moving or top-face position is unstable')
                    point = np.median(points, axis=0).tolist()
                    return dict(pixel=point, points=points, source_frames=records)
            time.sleep(.12)
        raise TimeoutError('insufficient new stable top-face frames')

    def align(self, yaw, reach):
        # Calibration stays at origin height and within the user's small trial.
        if not -1 <= yaw <= 1 or not -4.1 <= reach <= 0:
            raise ValueError('calibration sample exceeds one degree / four millimetres')
        self.command += 1
        self.client.arm_align(*self.context, self.command, 0, yaw*3, reach*360/113.1)
        arm = self.wait_arm('ALIGNED')
        if arm['aligned_command_id'] != self.command:
            raise RuntimeError('aligned reference mismatch')
        actual = dict(z_mm=arm['z_deg']/9, yaw_deg=arm['yaw_deg']/3,
                      reach_mm=arm['reach_deg']*113.1/360)
        if (abs(actual['z_mm']) > .1 or abs(actual['yaw_deg']-yaw) > .2
                or abs(actual['reach_mm']-reach) > .2):
            raise RuntimeError('actual pose differs from requested calibration point')
        observation = self.observe(time.monotonic()+.3)
        sample = dict(requested=dict(yaw_deg=yaw, reach_mm=reach), actual=actual, **observation)
        self.samples.append(sample)
        self.record('sample', **sample)
        return sample

    def run(self, test_id):
        arm = self.client.arm_status()
        if (not arm['standalone_test_mode'] or not arm['origins_valid'] or arm['fault']
                or arm['state_name'] not in ('READY', 'READY_LEAVE')):
            raise RuntimeError('record manual origins with the physical key before calibration: '+str(arm))
        self.context = (test_id, 0, 102)
        self.record('initial', arm=arm)
        req = Request('http://127.0.0.1:8090/material-target',
                      data=b'{"color_id":2}', headers={'Content-Type': 'application/json'})
        with urlopen(req, timeout=3):
            pass
        self.started = True
        self.client.arm_test_begin(test_id, 102)
        self.wait_arm('WAIT_VISION')
        baseline = self.align(0, 0)
        motions, pixels = [], []
        for yaw, reach in [(0, -4), (1, 0), (-1, 0)]:
            sample = self.align(yaw, reach)
            motions.append(relative_displacement(sample['actual'], baseline['actual'],
                                                self.reference['origin_radius_mm']).tolist())
            pixels.append((np.asarray(sample['pixel'])-baseline['pixel']).tolist())
            returned = self.align(0, 0)
            if np.linalg.norm(np.asarray(returned['pixel'])-baseline['pixel']) > 3:
                raise RuntimeError('target drift or mechanical repeatability exceeds 3 pixels')
        correction, jacobian, residual = fit_correction(motions, pixels)
        holdout = self.align(.5, -2)
        displacement = relative_displacement(holdout['actual'], baseline['actual'],
                                             self.reference['origin_radius_mm'])
        predicted = np.asarray(baseline['pixel'])+jacobian@displacement
        holdout_error = float(np.linalg.norm(predicted-holdout['pixel']))
        if holdout_error > 3:
            raise RuntimeError('independent holdout image error exceeds 3 pixels')
        self.align(0, 0)
        report = dict(calibration_id='raw_top_20261119_'+str(test_id),
            matrix=correction.tolist(), jacobian=jacobian.tolist(), fit_residual_px=residual.tolist(),
            holdout_error_px=holdout_error, baseline=baseline, samples=self.samples,
            reference=self.reference, scope='local static sampling at origin height, yaw +/-1deg, reach -4..0mm')
        (self.output/'mapping.json').write_text(json.dumps(report, indent=2))
        self.record('mapping_validated', matrix=report['matrix'], holdout_error_px=holdout_error)
        self.command += 1
        self.client.arm_finish(*self.context, self.command)
        self.wait_arm('READY_LEAVE')
        self.record('finished')
        return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--test-id', required=True, type=int)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('use a new output directory')
    args.output.mkdir(parents=True)
    reference = json.loads(args.reference.read_text())
    import serial
    lock_path = Path(tempfile.gettempdir())/f'robot-preview-start-{os.getuid()}.lock'
    with lock_path.open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        subprocess.run(['systemctl', '--user', 'stop', 'chassis-button'], check=True)
        with serial.Serial(args.port, 115200, timeout=.05, write_timeout=1, exclusive=True) as port:
            run = StationaryCalibration(ElectricalClient(port), args.output, reference)
            try:
                run.run(args.test_id)
            except BaseException as exc:
                run.record('failed', error=str(exc))
                if run.started:
                    try:
                        run.client.stop(0)
                        arm = run.client.arm_status()
                        if arm['state_name'] == 'FAULT' and not arm['origins_valid']:
                            run.record('fault_latched_after_stop_request', arm=arm)
                        elif arm['state_name'] != 'STOPPED' or arm['origins_valid']:
                            raise RuntimeError('stop/origin invalidation not confirmed')
                        else:
                            run.record('stop_confirmed', arm=arm)
                    except Exception as stop_error:
                        run.record('stop_unconfirmed', error=str(stop_error))
                raise


if __name__ == '__main__':
    main()
