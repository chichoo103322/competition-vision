"""独立三色实机试验；必须显式模拟未实现的夹爪/料盘，不代表真实抓取。"""
import argparse
from collections import deque
from dataclasses import asdict, dataclass, replace
import fcntl
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError

import numpy as np

from pi_vision.electrical_link import ElectricalClient
from pi_vision.arm_visual_servo import AdaptivePixelMap, observation_dynamics
from tools.turntable_work import COLORS


def parse_order(value):
    if (len(value) != 3 or any(c not in '123456' for c in value)
            or len(set(value)) != 3):
        raise ValueError('本次试验须指定三个不同的业务颜色编号1..6')
    return [int(c) for c in value]


@dataclass(frozen=True)
class TrialTuning:
    tolerance_px: float = 8.
    confirm_frames: int = 3
    confirm_span_s: float = .2
    settle_s: float = .1
    max_step_mm: float = 3.5
    max_yaw_step_deg: float = .8
    max_retraction_mm: float = 60.
    max_adjustments: int = 32
    max_total_correction_mm: float = 90.
    alignment_budget_s: float = 180.

    def __post_init__(self):
        # Stay inside the firmware's ALIGN micro-motion corridor: 1deg/4mm,
        # front 60mm of travel. Raw feedback/actuator arrival checks stay strict.
        ranges = dict(tolerance_px=(0,20), confirm_span_s=(.2,2), settle_s=(0,2),
            max_step_mm=(0,3.8), max_yaw_step_deg=(0,.9),
            max_retraction_mm=(0,60), max_total_correction_mm=(0,120),
            alignment_budget_s=(0,600))
        for name,(lo,hi) in ranges.items():
            value = getattr(self,name)
            if (isinstance(value,bool) or not isinstance(value,(int,float))
                    or not math.isfinite(value) or not lo <= value <= hi
                    or (lo==0 and value==0 and name!='settle_s')):
                raise ValueError('invalid trial tuning: '+name)
        if type(self.confirm_frames) is not int or not 3 <= self.confirm_frames <= 15:
            raise ValueError('at least three distinct source frames required')
        if type(self.max_adjustments) is not int or not 1 <= self.max_adjustments <= 64:
            raise ValueError('invalid maximum adjustment count')

    def accepts(self, error):
        error = np.asarray(error, dtype=float)
        return bool(error.shape==(2,) and np.isfinite(error).all()
                    and np.max(np.abs(error)) <= self.tolerance_px)


class ObservationUnavailable(TimeoutError):
    """No usable visual observation yet; stay at photo height and reacquire."""


class TargetOutsideWorkspace(RuntimeError):
    """The turntable target is not currently reachable; do not chase past limits."""


def correction_target(arm, pixel, reference, matrix, radius0, *, tuning=None):
    tuning = tuning or TrialTuning()
    radial, tangent = np.asarray(matrix, dtype=float) @ (np.asarray(pixel)-reference)
    radius = radius0+arm['reach_cdeg']*113.1/36000
    if not np.isfinite([radial, tangent, radius]).all() or radius <= 0:
        raise ValueError('nonfinite correction or invalid arm radius')
    scale = min(1., tuning.max_step_mm/max(math.hypot(radial, tangent), 1e-9))
    for _ in range(30):
        dr, dt = radial*scale, tangent*scale
        if radius+dr > 0 and abs(math.degrees(math.atan2(dt, radius+dr))) <= tuning.max_yaw_step_deg:
            break
        scale *= .9
    else:
        raise RuntimeError('cannot bound correction')
    yaw = arm['yaw_cdeg']/300+math.degrees(math.atan2(dt, radius+dr))
    reach = math.hypot(radius+dr, dt)-radius0
    if not -6 <= yaw <= 6 or not -tuning.max_retraction_mm <= reach <= 0:
        raise TargetOutsideWorkspace(f'target outside trial yaw +/-6deg / reach -{tuning.max_retraction_mm:g}..0mm')
    target = [0, round(yaw*300), round(reach*36000/113.1)]
    if (abs(target[1]-arm['yaw_cdeg']) > tuning.max_yaw_step_deg*300+1
            or abs(target[2]-arm['reach_cdeg'])*113.1/36000 > tuning.max_step_mm+.01):
        raise RuntimeError('rounded correction exceeds step limit')
    return target, math.hypot(dr, dt)


class ArmTrial:
    """Shared command correlation and measured arrival checks for stand-alone arm trials."""

    def __init__(self, client, output, tuning, *, clock=time.monotonic, sleep=time.sleep):
        self.client, self.output, self.tuning = client, Path(output), tuning
        self.clock, self.sleep = clock, sleep
        self.command, self.context, self.started = 1, None, False
        self.color = None
        self.results = []
        self.last_motion = dict(elapsed_s=0., full_avoidance=False)

    def record(self, event, **data):
        row = dict(event=event, unix_s=time.time(), **data)
        with (self.output/'events.jsonl').open('a') as f:
            f.write(json.dumps(row)+'\n')
        print(json.dumps(row, ensure_ascii=False), flush=True)

    def checked_arm(self, expected_command=None):
        a = self.client.arm_status()
        if (not a['origins_valid'] or not a['standalone_test_mode'] or a['fault']
                or (self.context and tuple(a[k] for k in ('route_id','point_index','work_id')) != self.context)
                or a['command_id'] != (self.command if expected_command is None else expected_command)):
            raise RuntimeError('arm origin/mode/fault/context/command invalid: '+str(a))
        return a

    @staticmethod
    def envelope(a):
        if (not -100.4 <= a['z_cdeg']/900 <= .2
                or not -60.6 <= a['reach_cdeg']*113.1/36000 <= .4
                or abs(a['yaw_cdeg']/300) > 6.3):
            raise RuntimeError('arm left trial stroke/clearance envelope')

    def wait_arm(self, wanted, transitional, *, target=None, vertical_from=None,
                 alternate_z_target=None, timeout=40):
        start = self.clock()
        deadline = start+timeout
        full_avoidance = False
        while self.clock() < deadline:
            a = self.checked_arm()
            self.envelope(a)
            self.record('feedback', color_id=self.color, arm=a)
            if wanted=='ALIGNED' and a['state_name']=='ALIGNING' and a['active_axis']==0:
                full_avoidance = True
            if (wanted=='ALIGNED' and target is not None and
                    abs(a['reach_cdeg']-target[2])*113.1/36000>self.tuning.max_step_mm+1):
                full_avoidance = True
            if vertical_from is not None:
                if alternate_z_target is not None:
                    # Firmware may use the aligned Z target rather than a
                    # small encoder residual at the top physical limit.
                    if a['target_z_cdeg'] == alternate_z_target and alternate_z_target != target[0]:
                        self.record('vertical_reference_selected', z_target=alternate_z_target,
                                    previous_expected_z=target[0])
                        target = [alternate_z_target, *target[1:]]
                    alternate_z_target = None
                if (a['target_z_cdeg'] != target[0]
                        or a['aligned_command_id'] != vertical_from['aligned_command_id']
                        or abs(a['yaw_cdeg']-vertical_from['yaw_cdeg'])/300 > .2
                        or abs(a['reach_cdeg']-vertical_from['reach_cdeg'])*113.1/36000 > .3
                        or a['target_yaw_cdeg'] != vertical_from['target_yaw_cdeg']
                        or a['target_reach_cdeg'] != vertical_from['target_reach_cdeg']):
                    raise RuntimeError('expected Z-only motion changed reference or horizontal target')
                low, high = sorted([vertical_from['z_cdeg'], target[0]])
                if not low-180 <= a['z_cdeg'] <= high+180:
                    raise RuntimeError('Z feedback overshot requested stroke')
            if a['state_name'] == wanted:
                if target is not None and (abs(a['z_cdeg']-target[0])/900 > .2
                        or abs(a['yaw_cdeg']-target[1])/300 > .2
                        or abs(a['reach_cdeg']-target[2])*113.1/36000 > .3):
                    raise RuntimeError('arrival differs from requested pose')
                if wanted == 'ALIGNED' and a['aligned_command_id'] != self.command:
                    raise RuntimeError('ALIGNED reference mismatch')
                self.last_motion = dict(elapsed_s=self.clock()-start,full_avoidance=full_avoidance)
                self.record('arrived', state=wanted, **self.last_motion, arm=a)
                return a
            if a['state_name'] not in transitional:
                raise RuntimeError('unexpected state '+a['state_name'])
            self.sleep(.12)
        raise TimeoutError('motion timeout waiting '+wanted)

    def align_to(self, target):
        self.command += 1
        self.record('align_request', color_id=self.color, command=self.command, target_cdeg=target)
        self.client.arm_align(*self.context, self.command, *(v/100 for v in target))
        return self.wait_arm('ALIGNED', ('ALIGNING',), target=target)

    def lower_and_lift(self, aligned, *, release=False):
        aligned_id = aligned['aligned_command_id']
        if aligned_id != self.command:
            raise RuntimeError('cannot PICK without latest verified alignment')
        self.command += 1
        self.record('pick_request', color_id=self.color, reference_id=aligned_id, command=self.command)
        self.client.arm_pick(*self.context, self.command, aligned_id)
        down = [aligned['z_cdeg']-90000, aligned['yaw_cdeg'], aligned['reach_cdeg']]
        alternate_down = None
        if abs(aligned['target_z_cdeg']-aligned['z_cdeg']) <= 180:
            alternate_down = aligned['target_z_cdeg']-90000
        lower = self.wait_arm('WAIT_GRIP', ('LOWERING',), target=down, vertical_from=aligned,
                              alternate_z_target=alternate_down)
        lift_z = lower['target_z_cdeg'] + 90000
        action = 'release' if release else 'grip'
        completion = {'physical_place_completed' if release else 'physical_grasp_completed': False}
        self.record('simulated_'+action, color_id=self.color, simulated=True,
                    servo_command_sent=False, **completion)
        self.sleep(.5)
        lower = self.checked_arm()
        if lower['state_name'] != 'WAIT_GRIP' or lower['aligned_command_id'] != aligned_id:
            raise RuntimeError('cannot simulate completion outside current WAIT_GRIP')
        self.command += 1
        self.client.arm_grip_done(*self.context, self.command, aligned_id)
        self.record('simulated_'+action+'_done_sent', color_id=self.color,
                    reference_id=aligned_id, **completion)
        lifted = self.wait_arm('PICKED', ('LIFTING',),
            target=[lift_z, aligned['yaw_cdeg'], aligned['reach_cdeg']], vertical_from=lower)
        return lower, lifted


class ThreeColorTrial(ArmTrial):
    def __init__(self, client, reference, mapping, snapshot, select_color, output, *,
                 simulate_servos=False, clock=time.monotonic, sleep=time.sleep,
                 save_frame=None, tuning=None):
        if simulate_servos is not True:
            raise ValueError('explicit --simulate-servos required; real servos are unimplemented')
        for k in ('camera_source', 'hsv_profile_sha256', 'resolution', 'origin_radius_mm'):
            if reference.get(k) != mapping['reference'].get(k):
                raise ValueError('mapping/reference identity mismatch: '+k)
        matrix = np.asarray(mapping['matrix'], dtype=float)
        if matrix.shape != (2, 2) or not np.isfinite(matrix).all():
            raise ValueError('finite 2x2 mapping required')
        super().__init__(client, output, tuning or TrialTuning(), clock=clock, sleep=sleep)
        self.reference, self.mapping = reference, mapping
        self.snapshot, self.select_color = snapshot, select_color
        self.save_frame = save_frame
        self.camera_session = self.capture_epoch = self.generation = None
        self.last_sequence, self.last_acquired = 0, float('-inf')

    def camera_identity(self, state):
        now = self.clock()
        capture = state.get('capture', {})
        stamp = capture.get('acquired_monotonic_s')
        if (state.get('camera_source') != self.reference['camera_source']
                or state.get('hsv_profile_sha256') != self.reference['hsv_profile_sha256']
                or [state['preview']['source_width'], state['preview']['source_height']] != self.reference['resolution']
                or state.get('material_backend') != 'HSVMaterialDetector'
                or state.get('material_selection') != 'unique_color_in_search_window'
                or state.get('material_search_color') != self.color):
            raise RuntimeError('dedicated HSV camera/profile/target identity changed')
        session, epoch = state.get('session_id'), capture.get('epoch')
        if not session or (self.camera_session is not None and
                (session != self.camera_session or epoch != self.capture_epoch)):
            raise RuntimeError('camera restarted or capture epoch changed')
        if (not capture.get('valid') or not isinstance(stamp,(int,float))
                or not 0<=now-stamp<1):
            raise ObservationUnavailable('fresh camera capture not yet available')
        self.camera_session, self.capture_epoch = session, epoch

    def set_color(self, color):
        self.color = color
        while True:
            self.checked_arm()
            try:
                self.select_color(color)
                self.color_floor = self.clock()
                state = self.snapshot()
                self.camera_identity(state)
                break
            except HTTPError as e:
                if e.code<500:
                    raise
                self.record('waiting_camera',color_id=color,reason=str(e),motion_sent=False)
            except (ObservationUnavailable,URLError,OSError,json.JSONDecodeError) as e:
                self.record('waiting_camera',color_id=color,reason=str(e),motion_sent=False)
            self.sleep(.3)
        self.generation = state['qr_session']['generation']
        self.record('waiting_color', color_id=color, color_name=COLORS[color],
                    generation=self.generation)

    def observe(self, floor, timeout=180):
        floor = max(floor, self.color_floor, self.last_acquired)
        deadline, last_status = self.clock()+timeout, float('-inf')
        count = self.tuning.confirm_frames
        points, frames, tracks = (deque(maxlen=count) for _ in range(3))
        while self.clock() < deadline:
            if self.clock()-last_status > .5:
                a = self.checked_arm()
                if a['state_name'] not in ('WAIT_VISION', 'ALIGNED') or abs(a['z_cdeg'])/900 > .15:
                    raise RuntimeError('arm no longer stopped at observation height')
                last_status = self.clock()
            try:
                state = self.snapshot()
                self.camera_identity(state)
            except HTTPError as e:
                if e.code<500:
                    raise
                points.clear(); frames.clear(); tracks.clear()
                self.sleep(.1)
                continue
            except (ObservationUnavailable,URLError,OSError,json.JSONDecodeError):
                points.clear(); frames.clear(); tracks.clear()
                self.sleep(.1)
                continue
            obs = state['material_observation']
            seq, stamp = obs.get('source_sequence'), obs.get('acquired_monotonic_s')
            if (not obs.get('valid') or obs.get('generation') != self.generation
                    or obs.get('capture_epoch') != self.capture_epoch
                    or not isinstance(stamp, (int, float)) or not 0 <= self.clock()-stamp < 1):
                points.clear(); frames.clear(); tracks.clear()
                self.sleep(.1)
                continue
            if seq <= self.last_sequence or stamp <= floor:
                self.sleep(.1)
                continue
            self.last_sequence, self.last_acquired = seq, stamp
            materials = state.get('materials', [])
            if (len(materials) != 1 or materials[0].get('color_id') != self.color
                    or materials[0].get('point_kind') != 'top_face_ellipse_center'):
                points.clear(); frames.clear(); tracks.clear()
            else:
                if frames and stamp-frames[-1]['acquired'] > .7:
                    points.clear(); frames.clear(); tracks.clear()
                track = materials[0].get('track_id')
                if tracks and track!=tracks[-1]:
                    points.clear(); frames.clear(); tracks.clear()
                point = [materials[0]['center_x'], materials[0]['center_y']]
                if not np.isfinite(point).all():
                    raise RuntimeError('nonfinite top face')
                points.append(point)
                frames.append(dict(sequence=seq, acquired=stamp))
                tracks.append(track)
                if (len(points) == count
                        and stamp-frames[0]['acquired'] >= self.tuning.confirm_span_s-1e-9
                        and np.ptp(points, axis=0).max() <= 3):
                    return dict(pixel=np.median(points, axis=0).tolist(), track_id=track,
                                points=list(points), source_frames=list(frames),
                                **observation_dynamics(list(points),list(frames)))
            self.sleep(.1)
        raise ObservationUnavailable('requested color has no unique stable fresh top face: '+COLORS[self.color])

    def wait_reachable_target(self, timeout=180):
        """转盘目标未到可达位置时只等待，不把外围/前伸越界候选当作对齐失败。"""
        deadline = self.clock()+timeout
        while self.clock() < deadline:
            try:
                observation = self.observe(self.clock()+self.tuning.settle_s,
                    timeout=min(8, deadline-self.clock()))
            except ObservationUnavailable:
                continue
            a = self.checked_arm()
            error = np.asarray(observation['pixel'])-self.reference['reference_pixel']
            radial, tangent = np.asarray(self.mapping['matrix']) @ error
            radius = self.reference['origin_radius_mm']+a['reach_cdeg']*113.1/36000
            full_yaw = a['yaw_cdeg']/300+math.degrees(math.atan2(tangent, radius+radial))
            full_reach = math.hypot(radius+radial, tangent)-self.reference['origin_radius_mm']
            if (self.tuning.accepts(error) or (radius+radial > 0
                    and -6 <= full_yaw <= 6 and -self.tuning.max_retraction_mm <= full_reach <= 0)):
                return observation
            self.record('waiting_reachable_target', color_id=self.color,
                pixel=observation['pixel'], estimated_yaw_deg=full_yaw,
                estimated_reach_mm=full_reach, motion_sent=False)
        raise ObservationUnavailable('requested color did not enter reachable approach region: '+COLORS[self.color])

    def align_color(self):
        servo = AdaptivePixelMap(self.mapping['matrix'])
        anchor, stagnant, total, adjustments = None, 0, 0., 0
        need_acquire, active_since, hold_reason = True, None, None
        while True:
            try:
                observation = (self.wait_reachable_target(timeout=8) if need_acquire else
                    self.observe(self.clock()+self.tuning.settle_s,timeout=8))
            except ObservationUnavailable as e:
                self.record('waiting_target',color_id=self.color,reason=str(e),motion_sent=False)
                servo.recover()
                anchor, stagnant, need_acquire = None, 0, True
                continue
            need_acquire = False
            error = np.asarray(observation['pixel'])-self.reference['reference_pixel']
            self.record('visual_check', color_id=self.color, observation=observation,
                        error_px=error.tolist())
            a = self.checked_arm()
            if a['state_name'] not in ('ALIGNED','WAIT_VISION'):
                raise RuntimeError('arm left observation state')
            self.envelope(a)
            if self.tuning.accepts(error) and a['state_name'] == 'ALIGNED':
                if self.save_frame:
                    self.save_frame(self.output/f'{self.color}_aligned.jpg')
                return a, observation, error.tolist()
            if anchor is not None:
                before_obs, before_arm, before_error = anchor
                progress = float(np.linalg.norm(before_error)-np.linalg.norm(error))
                stagnant = stagnant+1 if progress<.5 else 0
                identity_changed = before_obs.get('track_id')!=observation.get('track_id')
                if progress < -3 or stagnant>=2 or identity_changed:
                    servo.recover()
                    self.record('visual_reacquire',color_id=self.color,progress_px=progress,
                        reason='target_changed_or_error_growth' if progress < -3 or identity_changed else 'no_progress',
                        next_step_gain=servo.step_gain,previous_motion=self.last_motion,
                        motion_sent=False)
                    anchor, stagnant, need_acquire = None, 0, True
                    continue
                if progress>.5:
                    servo.progress()
                update = servo.update(before_obs,observation,before_arm,a,
                    self.reference['origin_radius_mm'],self.last_motion)
                self.record('mapping_update',color_id=self.color,**update)
            if self.tuning.accepts(error):
                target, distance = [0, a['yaw_cdeg'], min(0, a['reach_cdeg'])], 0.
            else:
                exhausted = (adjustments>=self.tuning.max_adjustments or
                    total>=self.tuning.max_total_correction_mm or (active_since is not None
                    and self.clock()-active_since>=self.tuning.alignment_budget_s))
                if exhausted:
                    if hold_reason!='motion_budget':
                        self.record('holding_for_target',color_id=self.color,reason='motion_budget',
                            adjustments=adjustments,total_correction_mm=total,
                            physical_grasp_completed=False,motion_sent=False)
                    hold_reason='motion_budget'
                    anchor=None
                    continue
                step_tuning = replace(self.tuning,max_step_mm=self.tuning.max_step_mm*servo.step_gain,
                    max_yaw_step_deg=self.tuning.max_yaw_step_deg*servo.step_gain)
                pixel = servo.predicted_pixel(observation,self.clock(),max_lead_px=self.tuning.tolerance_px/2)
                try:
                    target, distance = correction_target(a,pixel,
                        self.reference['reference_pixel'],servo.correction_matrix,
                        self.reference['origin_radius_mm'],tuning=step_tuning)
                except TargetOutsideWorkspace as e:
                    self.record('waiting_reachable_target',color_id=self.color,
                        pixel=observation['pixel'],reason=str(e),motion_sent=False)
                    anchor, need_acquire = None, True
                    continue
                if total+distance>self.tuning.max_total_correction_mm:
                    total=self.tuning.max_total_correction_mm
                    anchor=None
                    continue
            if active_since is None:
                active_since = self.clock()
            self.record('servo_correction',color_id=self.color,predicted_pixel=(
                observation['pixel'] if distance==0 else pixel.tolist()),step_gain=servo.step_gain)
            anchor = (observation,a,error.copy())
            total += distance
            adjustments += 1
            self.align_to(target)

    def cycle(self):
        aligned, obs, error = self.align_color()
        lower, lifted = self.lower_and_lift(aligned)
        self.align_to([0, 0, 0])
        self.record('simulated_tray_transfer', color_id=self.color, simulated=True,
                    physical_place_completed=False, servo_command_sent=False)
        result = dict(color_id=self.color, color_name=COLORS[self.color],
            aligned_pixel=obs['pixel'], error_px=error,
            lowered_mm=(aligned['z_cdeg']-lower['z_cdeg'])/900,
            lifted_z_cdeg=lifted['z_cdeg'], simulated_grip=True, simulated_tray=True,
            physical_grasp_completed=False, physical_place_completed=False, returned_to_origin=True)
        self.results.append(result)
        self.record('color_cycle_completed', **result)

    def run(self, order, test_id):
        order = parse_order(''.join(map(str, order)))
        a = self.client.arm_status()
        chassis = self.client.status()
        if (a['state_name'] != 'READY' or not a['origins_valid'] or not a['standalone_test_mode']
                or a['fault'] or any(abs(a[k]) > 50 for k in ('z_cdeg','yaw_cdeg','reach_cdeg'))
                or chassis['state_name'] != 'EMPTY' or chassis['fault']):
            raise RuntimeError('fresh manual origin / standalone READY / EMPTY chassis required')
        self.context = (test_id, 0, 102)
        self.started = True
        self.record('sequence_started', order=order, order_source='user_confirmed_test_order',
                    simulated_servos=True, physical_grasp_completed=False, tuning=asdict(self.tuning))
        self.client.arm_test_begin(test_id, 102)
        self.wait_arm('WAIT_VISION', ('PREPARING',), target=[0, 0, 0])
        for color in order:
            self.set_color(color)
            self.cycle()
        result = dict(result='three_color_simulation_completed', order=order, colors=self.results,
            final_arm=self.checked_arm(), held_at='photo_origin_ALIGNED',
            physical_grasp_completed=False, physical_place_completed=False, simulated_servos=True,
            tuning=asdict(self.tuning))
        (self.output/'result.json').write_text(json.dumps(result, indent=2))
        self.record('sequence_completed', **result)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--order', required=True, help='operator-confirmed three-color order, e.g. 231')
    parser.add_argument('--reference', required=True, type=Path)
    parser.add_argument('--mapping', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--simulate-servos', action='store_true')
    parser.add_argument('--tolerance-px',type=float,default=8,
        help='grasp acceptance per image axis in original pixels (default: 8)')
    parser.add_argument('--confirm-frames',type=int,default=3)
    parser.add_argument('--confirm-span-s',type=float,default=.2)
    parser.add_argument('--settle-s',type=float,default=.1)
    parser.add_argument('--max-step-mm',type=float,default=3.5)
    parser.add_argument('--max-yaw-step-deg',type=float,default=.8)
    parser.add_argument('--max-retraction-mm',type=float,default=60,
        help='0..60mm, bounded by the existing horizontal micro-motion corridor')
    parser.add_argument('--alignment-budget-s',type=float,default=180,
        help='active alignment budget; after exhaustion hold and await target, never skip a color')
    args = parser.parse_args()
    order = parse_order(args.order)
    if not args.simulate_servos:
        parser.error('--simulate-servos is required; real grip/tray controls are unimplemented')
    tuning = TrialTuning(**{name:getattr(args,name) for name in (
        'tolerance_px','confirm_frames','confirm_span_s','settle_s',
        'max_step_mm','max_yaw_step_deg','max_retraction_mm','alignment_budget_s')})
    args.output.mkdir(parents=True, exist_ok=False)
    reference, mapping = (json.loads(p.read_text()) for p in (args.reference, args.mapping))
    def snapshot():
        with urlopen('http://127.0.0.1:8090/state', timeout=3) as r:
            return json.load(r)
    def select_color(color):
        req = Request('http://127.0.0.1:8090/material-target',
            data=json.dumps(dict(color_id=color)).encode(), headers={'Content-Type':'application/json'})
        with urlopen(req, timeout=3):
            pass
    def save_frame(path):
        with urlopen('http://127.0.0.1:8090/frame.jpg', timeout=3) as r:
            path.write_bytes(r.read())
    import serial
    with (Path(tempfile.gettempdir())/f'robot-preview-start-{os.getuid()}.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        subprocess.run(['systemctl','--user','stop','chassis-button.service'], check=True)
        with serial.Serial(args.port, 115200, timeout=.05, write_timeout=1, exclusive=True) as port:
            trial = ThreeColorTrial(ElectricalClient(port), reference, mapping, snapshot, select_color,
                args.output, simulate_servos=True, save_frame=save_frame, tuning=tuning)
            try:
                deadline = time.monotonic()+600
                last_state = None
                while time.monotonic() < deadline:
                    try:
                        a = trial.client.arm_status()
                    except (TimeoutError, OSError) as e:
                        trial.record('waiting_controller', error=str(e))
                        time.sleep(.5)
                        continue
                    if a['state_name'] != last_state:
                        trial.record('waiting_manual_key', arm=a)
                        last_state = a['state_name']
                    if a['state_name'] == 'READY' and a['origins_valid'] and not a['fault']:
                        trial.run(order, int(time.time()))
                        break
                    time.sleep(.3)
                else:
                    raise TimeoutError('manual-key wait expired; no sequence started')
            except BaseException as e:
                trial.record('failed', error=str(e), completed_colors=trial.results)
                if trial.started:
                    try:
                        trial.client.stop(0)
                        trial.record('after_stop', arm=trial.client.arm_status())
                    except Exception as stop_error:
                        trial.record('stop_unconfirmed', error=str(stop_error))
                raise


if __name__ == '__main__':
    main()
