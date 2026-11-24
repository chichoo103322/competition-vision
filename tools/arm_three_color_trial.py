"""独立原料区三色测试，支持旧版模拟和 STM32 v7 自动入盘。"""
import argparse
from collections import deque
from dataclasses import asdict, dataclass, replace
import fcntl
from http.client import HTTPException
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

from pi_vision.observation import confirmation_window_ready
from pi_vision.electrical_link import ElectricalClient
from pi_vision.arm_visual_servo import AdaptivePixelMap, observation_dynamics, local_motion_mm
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
        ranges = dict(tolerance_px=(0,32), confirm_span_s=(.2,2), settle_s=(0,2),
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
    # Along this correction ray, |s*dt| <= tan(limit)*(radius+s*dr).
    # Stop exactly at the first yaw/length limit instead of shrinking by 10% steps.
    yaw_slope = math.tan(math.radians(tuning.max_yaw_step_deg))
    denominator = abs(tangent) - radial * yaw_slope
    if denominator > 0:
        scale = min(scale, radius * yaw_slope / denominator)
    dr, dt = radial * scale, tangent * scale
    if radius + dr <= 0:
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


def tray_grasp_radius(axes, cap):
    """Allow an offset up to half the inscribed top-face radius, in source pixels."""
    if not isinstance(axes, (tuple, list)) or len(axes) != 2 or not all(
            isinstance(x, (int, float)) and math.isfinite(x) and x > 0 for x in axes):
        return min(cap, 8.)
    return min(cap, min(axes) * .25)


TRAY_PICK_MAX_YAW_DEG = 12.
TRAY_GRASP_JITTER_PX = 12


def tray_approach(arm, error, reference_radius, matrix, tuning, grasp_radius):
    """Find a reachable photo-height pose and check its predicted residual in pixels."""
    matrix = np.asarray(matrix)
    radial, tangent = matrix @ error
    reach = arm['reach_cdeg'] * 113.1 / 36000
    yaw = arm['yaw_cdeg'] / 300
    radius = reference_radius + reach
    if radius + radial <= 0:
        return None
    desired_yaw = yaw + math.degrees(math.atan2(tangent, radius + radial))
    desired_reach = math.hypot(radius + radial, tangent) - reference_radius
    target_yaw = round(np.clip(desired_yaw, -6, TRAY_PICK_MAX_YAW_DEG) * 300)
    target_reach = max(math.ceil(-tuning.max_retraction_mm * 36000 / 113.1),
                       min(0, round(desired_reach * 36000 / 113.1)))
    end_radius = reference_radius + target_reach * 113.1 / 36000
    angle = math.radians(target_yaw / 300 - yaw)
    displacement = np.array([end_radius * math.cos(angle) - radius, end_radius * math.sin(angle)])
    remaining = error - np.linalg.solve(matrix, displacement)
    residual_px = np.linalg.norm(remaining)
    if residual_px > grasp_radius:
        # A target just past the centre-reach boundary can still benefit from
        # one horizontal approach. Re-observe there; this never authorizes PICK.
        if (residual_px > grasp_radius + 12 or np.linalg.norm(error) - residual_px < 12
                or np.linalg.norm(displacement) < 2.):
            return None
    return [0, target_yaw, target_reach], float(np.linalg.norm(displacement))


def tracking_approach(arm, error, radius, matrix, tuning, grasp_radius):
    plan = tray_approach(arm, error, radius, matrix, tuning, grasp_radius)
    # Distant lateral targets in the field move farther in the image per
    # actuator step than the origin calibration predicts. Permit a bounded
    # approach; only subsequent measured pixels can authorize a descent.
    if plan is None and abs(error[0]) > 2 * abs(error[1]):
        plan = tray_approach(arm, error, radius, .66 * np.asarray(matrix), tuning, grasp_radius)
    return plan


def micro_start_ok(arm):
    return -50 <= arm['z_cdeg'] <= 0 and TRAY_PARK[2] <= arm['reach_cdeg'] <= 0


def measured_view_target(arm, pixel, reference, mapping, tuning, visited=()):
    # Encoder residuals at the photo origin can be slightly positive.
    if not (-50 <= arm['z_cdeg'] <= 50 and TRAY_PARK[2] <= arm['reach_cdeg'] <= 95):
        return None
    error = np.asarray(pixel) - reference['reference_pixel']
    best_error = np.linalg.norm(error) - 3
    best = None
    for sample in mapping.get('samples', ()):
        actual, requested = sample.get('actual', {}), sample.get('requested', {})
        if len(sample.get('source_frames', ())) < 3:
            continue
        yaw, reach = requested.get('yaw_deg'), requested.get('reach_mm')
        values = (yaw, reach, actual.get('yaw_deg'), actual.get('reach_mm'), actual.get('z_mm'))
        if not all(isinstance(v, (float,int)) and math.isfinite(v) for v in values):
            continue
        if (abs(actual['z_mm']) > .15 or abs(yaw-actual['yaw_deg']) > .2
                or abs(reach-actual['reach_mm']) > .3 or not -6 <= yaw <= TRAY_PICK_MAX_YAW_DEG
                or not -tuning.max_retraction_mm <= reach <= 0):
            continue
        target = [0, round(yaw*300), round(reach*36000/113.1)]
        if (tuple(target) in visited
                or abs(target[1]-arm['yaw_cdeg']) > tuning.max_yaw_step_deg*300
                or abs(target[2]-arm['reach_cdeg'])*113.1/36000 > tuning.max_step_mm):
            continue
        end = dict(yaw_cdeg=target[1], reach_cdeg=target[2])
        delta = local_motion_mm(arm,end,reference['origin_radius_mm'])
        remaining = np.linalg.norm(error-np.linalg.solve(mapping['matrix'], delta))
        if remaining < best_error:
            best_error, best = remaining, (target, float(np.linalg.norm(delta)))
    return best


def measured_edge_view(arm, pixel, reference, mapping, tuning, visited):
    width, height = reference['resolution']
    lower = np.array([.38 * width, .31 * height])
    upper = np.array([.62 * width, .69 * height])
    point = np.asarray(pixel)
    desired = point - np.clip(point, lower, upper)
    initial_residual = np.linalg.norm(desired)
    if initial_residual < 18:
        return None
    # These full ALIGN targets reached ALIGNED with fresh top faces in the
    # 16:39 and 17:22 raw-station runs. Units are motor centidegrees.
    poses = ((0, -328, -5379), (0, -380, -6499),
             (0, -1205, -5377), (0, 1677, -6743))
    best = None
    best_score = (math.inf, math.inf)
    for target in poses:
        _, yaw, reach = target
        if (target in visited
                or -reach * 113.1 / 36000 > tuning.max_retraction_mm):
            continue
        delta = local_motion_mm(arm, dict(yaw_cdeg=yaw, reach_cdeg=reach),
                                reference['origin_radius_mm'])
        shift = np.linalg.solve(mapping['matrix'], delta)
        if abs(shift[0]) > 24 and shift[0] * (point[0] - width / 2) < 0:
            continue
        predicted = point - shift
        remaining = np.linalg.norm(predicted - np.clip(predicted, lower, upper))
        if remaining > initial_residual - 12:
            continue
        distance = float(np.linalg.norm(delta))
        score = (max(0., remaining - 8), distance)
        if score < best_score:
            best_score, best = score, (list(target), distance)
    return best


class ViewpointNeeded(Exception):
    """Unique colour is visible, but the rim has not been localized."""
    def __init__(self, pixel):
        self.pixel = pixel


def tray_correction(arm, error, reference_radius, matrix, tuning, grasp_radius, *, allow_coarse):
    plan = tracking_approach(arm, error, reference_radius, matrix, tuning, grasp_radius)
    if plan is None:
        raise TargetOutsideWorkspace('grasp window outside the reachable workspace')
    target, distance = plan
    dyaw = target[1] - arm['yaw_cdeg']
    dreach = target[2] - arm['reach_cdeg']
    steps = max(distance / tuning.max_step_mm,
                abs(dreach) * 113.1 / 36000 / tuning.max_step_mm,
                abs(dyaw) / (300 * tuning.max_yaw_step_deg))
    # Full ALIGN invokes the firmware clearance path. Use it only when getting
    # inside the window would otherwise take several move-and-observe cycles.
    fraction_to_window = max(0., 1 - grasp_radius / max(1., np.linalg.norm(error)))
    # Median encoder move and post-arrival observation times in the five
    # recorded runs, in seconds (7 short moves, 9 clearance moves, 16 views).
    short_s, clearance_s, view_s = .74, 4.68, .37
    # In the recorded two-step micro correction (15:52), the image
    # closed by roughly 20-27 px per step instead of the nominal ~16.
    moves = max(1, math.ceil(.78 * steps * fraction_to_window))
    short_plan_s = moves * (short_s + view_s)
    # Even a short ALIGN can invoke full clearance if an encoder is just
    # outside the firmware micro corridor at its initial pose.
    if not micro_start_ok(arm):
        short_plan_s += clearance_s - short_s
    damped_steps = .66 * steps
    coarse = (allow_coarse and moves > 1 and damped_steps > 1
              and short_plan_s > clearance_s + 2*view_s)
    # Both field runs repeatedly crossed the centre after a full-size first
    # ALIGN. Use a damped first move; later adjustments use fresh measurements.
    fraction = .66 if coarse else 1 / max(1., steps)
    if fraction < 1:
        target = [0, round(arm['yaw_cdeg'] + dyaw * fraction),
                  round(arm['reach_cdeg'] + dreach * fraction)]
        target[1] = int(np.clip(target[1], -1800, 300 * TRAY_PICK_MAX_YAW_DEG))
        target[2] = int(np.clip(target[2], math.ceil(-tuning.max_retraction_mm * 36000 / 113.1), 0))
        distance *= fraction
    return target, distance, coarse


# Motor centidegrees from car_code1 7 arm_config.h and confirmed field settings.
TRAY_PARK = (0, 0, -19098)
TRAY_REACH_TARGETS = (-31830, -36605)  # 100 mm and 115 mm
# STM32 reports 120° on the current car and 150° in the supplied v7 source.
TRAY_YAW_TARGETS = (36000, 45000)
TRAY_YAW_APPROACH_FEEDBACK_MARGIN = 180
TRAY_REACH_APPROACH_FEEDBACK_MARGIN = 180
PHOTO_RETURN_FORWARD_MARGIN_MM = 0.6
ARRIVAL_TOLERANCES = (180, 60, 95)
ARM_AXES = ('z', 'yaw', 'reach')
PICK_MOTOR_CDEG_PER_MM = 900
GRASP_LIFT_CLEAR_MM = 5
DEFAULT_PICK_DROP_MM = 100
REAL_TRAY_PICK_DROPS_MM = (50, 80, DEFAULT_PICK_DROP_MM)


def checked_arrival(arm, target):
    if any(abs(arm[f'{axis}_cdeg'] - value) > tolerance
           for axis, value, tolerance in zip(ARM_AXES, target, ARRIVAL_TOLERANCES)):
        raise RuntimeError('arrival differs from requested pose')


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
    def envelope(a, max_yaw_deg=6., *, max_forward_mm=.4):
        if (not -100.4 <= a['z_cdeg']/900 <= .2
                or not -60.6 <= a['reach_cdeg']*113.1/36000 <= max_forward_mm
                or not -6.3 <= a['yaw_cdeg']/300 <= max_yaw_deg + .3):
            raise RuntimeError('arm left trial stroke/clearance envelope')

    def wait_arm(self, wanted, transitional, *, target=None, vertical_from=None,
                 pick_drop_options=None, photo_return=False, timeout=40):
        start = self.clock()
        deadline = start+timeout
        full_avoidance = False
        minimum_reach = None
        selected_pick_drop_mm = None
        while self.clock() < deadline:
            a = self.checked_arm()
            self.record('feedback', color_id=self.color, arm=a)
            if (photo_return and a['state_name'] == 'ALIGNING' and a['active_axis'] == 2
                    and all(a[f'target_{axis}_cdeg'] == 0 for axis in ARM_AXES)):
                self.envelope(a, max_forward_mm=PHOTO_RETURN_FORWARD_MARGIN_MM)
            else:
                self.envelope(a)
            if wanted == 'ALIGNED':
                minimum_reach = a['reach_cdeg'] if minimum_reach is None else min(minimum_reach, a['reach_cdeg'])
            if wanted=='ALIGNED' and a['state_name']=='ALIGNING' and a['active_axis']==0:
                full_avoidance = True
            if (wanted=='ALIGNED' and target is not None and
                    abs(a['reach_cdeg']-target[2])*113.1/36000>self.tuning.max_step_mm+1):
                full_avoidance = True
            if vertical_from is not None:
                if pick_drop_options is not None and selected_pick_drop_mm is None:
                    if a['state_name'] not in (*transitional, wanted):
                        raise RuntimeError('unexpected state '+a['state_name'])
                    commanded_z = a['target_z_cdeg']
                    expected_lift_refs = [vertical_from['z_cdeg']]
                    if abs(vertical_from['target_z_cdeg'] - vertical_from['z_cdeg']) <= 180:
                        expected_lift_refs.append(vertical_from['target_z_cdeg'])
                    # The firmware snapshots the Z encoder on PICK. Small
                    # encoder changes between reports must not change the
                    # selected stroke profile or the absolute travel envelope.
                    tolerance = 50 if vertical_from.get('tray_enabled') else 0
                    for mm in pick_drop_options:
                        lift_ref = commanded_z + mm * PICK_MOTOR_CDEG_PER_MM
                        if any(abs(lift_ref - ref) <= tolerance for ref in expected_lift_refs):
                            selected_pick_drop_mm = mm
                            break
                    if selected_pick_drop_mm is None:
                        raise RuntimeError(f'Z-only target mismatch: controller reports {commanded_z} cdeg, '
                                           f'allowed pick strokes {pick_drop_options} mm')
                    target = [commanded_z, *target[1:]]
                    self.record('pick_stroke_confirmed', color_id=self.color, command=self.command,
                                drop_mm=selected_pick_drop_mm, target_z_cdeg=commanded_z)
                if a['target_z_cdeg'] != target[0]:
                    raise RuntimeError(f'Z-only target mismatch: expected {target[0]} cdeg, '
                                       f'controller reports {a["target_z_cdeg"]} cdeg')
                if a['aligned_command_id'] != vertical_from['aligned_command_id']:
                    raise RuntimeError('Z-only motion changed alignment reference')
                if (abs(a['yaw_cdeg']-vertical_from['yaw_cdeg'])/300 > .2
                        or abs(a['reach_cdeg']-vertical_from['reach_cdeg'])*113.1/36000 > .3
                        or a['target_yaw_cdeg'] != vertical_from['target_yaw_cdeg']
                        or a['target_reach_cdeg'] != vertical_from['target_reach_cdeg']):
                    raise RuntimeError('Z-only motion changed horizontal target or feedback')
                low, high = sorted([vertical_from['z_cdeg'], target[0]])
                if not low-180 <= a['z_cdeg'] <= high+180:
                    raise RuntimeError('Z feedback overshot requested stroke')
            if a['state_name'] == wanted:
                if target is not None:
                    checked_arrival(a, target)
                if wanted == 'ALIGNED' and a['aligned_command_id'] != self.command:
                    raise RuntimeError('ALIGNED reference mismatch')
                self.last_motion = dict(elapsed_s=self.clock()-start,full_avoidance=full_avoidance)
                trace = {} if minimum_reach is None else dict(min_reach_mm=round(minimum_reach*113.1/36000, 2))
                self.record('arrived', state=wanted, **self.last_motion, arm=a, **trace)
                if selected_pick_drop_mm is not None:
                    return dict(a, pick_drop_mm=selected_pick_drop_mm)
                return a
            if a['state_name'] not in transitional:
                raise RuntimeError('unexpected state '+a['state_name'])
            self.sleep(.12)
        raise TimeoutError('motion timeout waiting '+wanted)

    def align_to(self, target, *, before=None, photo_return=False):
        self.command += 1
        details = {}
        if before is not None:
            reasons = []
            if not micro_start_ok(before):
                reasons.append('start_outside_micro_corridor')
            if abs(target[1]-before['yaw_cdeg']) > 300:
                reasons.append('yaw_step_over_1deg')
            if abs(target[2]-before['reach_cdeg']) > 1273:
                reasons.append('reach_step_over_4mm')
            details = dict(start_cdeg=[before[k+'_cdeg'] for k in ARM_AXES],
                           predicted_clearance_reasons=reasons)
        self.record('align_request', color_id=self.color, command=self.command, target_cdeg=target, **details)
        self.client.arm_align(*self.context, self.command, *(v/100 for v in target))
        return self.wait_arm('ALIGNED', ('ALIGNING',), target=target, photo_return=photo_return)

    def lower_for_pick(self, aligned, *, drop_mm=DEFAULT_PICK_DROP_MM):
        aligned_id = aligned['aligned_command_id']
        if aligned_id != self.command:
            raise RuntimeError('cannot PICK without latest verified alignment')
        self.command += 1
        self.record('pick_request', color_id=self.color, reference_id=aligned_id,
                    command=self.command, expected_drop_mm=drop_mm)
        self.client.arm_pick(*self.context, self.command, aligned_id)
        allowed = REAL_TRAY_PICK_DROPS_MM if drop_mm is None else (drop_mm,)
        down = [aligned['z_cdeg'] - allowed[0] * PICK_MOTOR_CDEG_PER_MM,
                aligned['yaw_cdeg'], aligned['reach_cdeg']]
        return self.wait_arm('WAIT_GRIP', ('LOWERING',), target=down, vertical_from=aligned,
                             pick_drop_options=allowed)

    def lower_and_lift(self, aligned, *, release=False):
        if aligned.get('tray_enabled'):
            raise RuntimeError('simulated grip requires firmware with tray mode disabled')
        lower = self.lower_for_pick(aligned)
        aligned_id = aligned['aligned_command_id']
        lift_z = lower['target_z_cdeg'] + DEFAULT_PICK_DROP_MM * PICK_MOTOR_CDEG_PER_MM
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
                 simulate_servos=False, real_tray=False, pick_drop_mm=None,
                 clock=time.monotonic, sleep=time.sleep, save_frame=None, save_failure=None, tuning=None, micro_only=False):
        if (simulate_servos is True) == (real_tray is True):
            raise ValueError('choose exactly one of --simulate-servos and --real-tray')
        for k in ('camera_source', 'hsv_profile_sha256', 'resolution', 'origin_radius_mm'):
            if reference.get(k) != mapping['reference'].get(k):
                raise ValueError('mapping/reference identity mismatch: '+k)
        matrix = np.asarray(mapping['matrix'], dtype=float)
        if matrix.shape != (2, 2) or not np.isfinite(matrix).all():
            raise ValueError('finite 2x2 mapping required')
        defaults = TrialTuning(tolerance_px=32., settle_s=0.) if real_tray else TrialTuning()
        super().__init__(client, output, tuning or defaults, clock=clock, sleep=sleep)
        self.real_tray = real_tray is True
        allowed_drops = REAL_TRAY_PICK_DROPS_MM if self.real_tray else (DEFAULT_PICK_DROP_MM,)
        if pick_drop_mm is None and not self.real_tray:
            pick_drop_mm = DEFAULT_PICK_DROP_MM
        if pick_drop_mm is not None and (type(pick_drop_mm) is not int or pick_drop_mm not in allowed_drops):
            raise ValueError(f'pick stroke must match a supported firmware build: {allowed_drops} mm')
        self.pick_drop_mm = pick_drop_mm
        self.reference, self.mapping = reference, mapping
        self.snapshot, self.select_color = snapshot, select_color
        self.save_frame = save_frame
        self.save_failure = save_failure
        self.micro_only = micro_only
        self.camera_session = self.capture_epoch = self.generation = None
        self.last_sequence, self.last_acquired = 0, float('-inf')

    def wait_for_manual_ready(self, timeout=600):
        """Keep every report received by a poll, including short ZEROING events."""
        deadline = self.clock() + timeout
        last_state = None
        saw_origin_start = False
        first_poll = True

        while self.clock() < deadline:
            generation = self.client.arm_generation
            try:
                current = self.client.arm_status()
            except (TimeoutError, OSError) as exc:
                self.record('waiting_controller', error=str(exc))
                saw_origin_start = False
                first_poll = True
                self.sleep(.5)
                continue

            self.check_firmware_mode(current)
            reports = [current]
            if self.real_tray and not first_poll:
                received = self.client.arm_generation - generation
                reports = list(self.client.arm_events)[-received:]
            for arm in reports:
                state = arm['state_name']
                if self.real_tray:
                    self.check_firmware_mode(arm)
                    self.require_tray_count(arm, 0)
                    free_context = not any(arm[k] for k in ('route_id', 'point_index', 'work_id'))
                    if state == 'NEED_ORIGIN' and free_context and not arm['origins_valid']:
                        saw_origin_start = True
                    elif state == 'ZEROING' and free_context:
                        # The unsolicited event can precede the firmware update of opcode.
                        if not first_poll:
                            saw_origin_start = True
                    elif state != 'READY':
                        saw_origin_start = False
                if state != last_state:
                    self.record('waiting_manual_key', arm=arm)
                    last_state = state

            ready = current['state_name'] == 'READY' and current['origins_valid'] and not current['fault']
            manual_origin = (saw_origin_start and current['opcode'] == 1 and current['command_id'] == 0
                             and not any(current[k] for k in ('route_id', 'point_index', 'work_id')))
            if ready and (not self.real_tray or manual_origin):
                return current
            if self.real_tray and ready and first_poll:
                self.record('awaiting_new_key', reason='existing READY does not start a real tray trial')
            if self.real_tray and ready:
                saw_origin_start = False
            first_poll = False
            self.sleep(.08 if self.real_tray else .3)
        raise TimeoutError('manual-key wait expired; no sequence started')

    def envelope(self, arm, *, max_forward_mm=.4):
        super().envelope(arm, TRAY_PICK_MAX_YAW_DEG if self.real_tray else 6.,
                         max_forward_mm=max_forward_mm)

    def check_firmware_mode(self, arm):
        if not arm['standalone_test_mode']:
            raise RuntimeError('independent arm test firmware is required')
        if bool(arm.get('tray_enabled')) != self.real_tray:
            raise RuntimeError('firmware tray flag does not match --real-tray / --simulate-servos')
        if self.real_tray:
            if arm.get('tray_uncertain'):
                raise RuntimeError('STM32 reports uncertain tray contents; inspect and clear before retry')
            if not arm.get('grip_done_required'):
                raise RuntimeError('STM32 firmware does not advertise the GRIP_DONE handoff')

    def checked_arm(self, expected_command=None):
        arm = super().checked_arm(expected_command)
        self.check_firmware_mode(arm)
        return arm

    @staticmethod
    def require_tray_count(arm, expected):
        if arm.get('tray_count') != expected:
            raise RuntimeError(f'STM32 tray count {arm.get("tray_count")} differs from expected {expected}')

    @staticmethod
    def check_tray_transfer_pose(arm, *, tray_reach_target=TRAY_REACH_TARGETS[0],
                                 allow_single_peak=False):
        _, yaw_margin, reach_margin = ARRIVAL_TOLERANCES
        yaw_limit = max(TRAY_YAW_TARGETS)
        reported_reach = arm['target_reach_cdeg']
        if reported_reach in TRAY_REACH_TARGETS:
            tray_reach_target = reported_reach
            yaw_limit = arm['target_yaw_cdeg']
            if yaw_limit not in TRAY_YAW_TARGETS or arm['target_z_cdeg'] != TRAY_PARK[0]:
                raise RuntimeError('unexpected v7 tray approach target')
            # Extra feedback room applies only on the moving axis and only
            # with the other horizontal axis already at the safe pose.
            if (arm['active_axis'] == 1 and
                    abs(arm['reach_cdeg'] - TRAY_PARK[2]) <= reach_margin):
                yaw_margin = TRAY_YAW_APPROACH_FEEDBACK_MARGIN
            if arm['active_axis'] == 2 and abs(arm['yaw_cdeg'] - yaw_limit) <= yaw_margin:
                reach_margin = TRAY_REACH_APPROACH_FEEDBACK_MARGIN
                if allow_single_peak:
                    # One extra encoder sample up to 20 cdeg (~0.063 mm).
                    reach_margin += 20
        elif arm['target_yaw_cdeg'] in TRAY_YAW_TARGETS or reported_reach < TRAY_REACH_TARGETS[0]:
            raise RuntimeError(f'unexpected v7 tray approach target: reach={reported_reach}')
        reach_min = tray_reach_target - reach_margin
        if (not -180 <= arm['z_cdeg'] <= 180
                or not -1890 <= arm['yaw_cdeg'] <= yaw_limit + yaw_margin
                or not reach_min <= arm['reach_cdeg'] <= 127):
            raise RuntimeError(
                f'arm left the v7 tray-transfer envelope: '
                f'axis={arm["active_axis"]}, z={arm["z_cdeg"]}, yaw={arm["yaw_cdeg"]}, '
                f'reach={arm["reach_cdeg"]}, reach_min={reach_min}')

    def load_tray(self, aligned):
        count_before = len(self.results)
        self.require_tray_count(aligned, count_before)
        pick_started = self.clock()
        lower = self.lower_for_pick(aligned, drop_mm=self.pick_drop_mm)
        self.require_tray_count(lower, count_before)
        aligned_id = aligned['aligned_command_id']
        lift_z = lower['target_z_cdeg'] + lower['pick_drop_mm'] * PICK_MOTOR_CDEG_PER_MM
        self.command += 1
        self.client.arm_grip_done(*self.context, self.command, aligned_id)
        self.record('lift_and_load_requested', color_id=self.color,
                    reference_id=aligned_id, expected_tray_count=count_before + 1)
        deadline = self.clock() + 90
        loading_seen = False
        lift_clear_s = None
        tray_reach_target = None
        previous_tray_pose = None
        count_seen = count_before
        while self.clock() < deadline:
            arm = self.checked_arm()
            state = arm['state_name']
            self.record('tray_feedback', color_id=self.color, arm=arm)
            if arm['aligned_command_id'] != aligned_id:
                raise RuntimeError('tray transfer alignment reference changed')
            if (lift_clear_s is None and state in ('LIFTING', 'TRAY_LOADING', 'PICKED')
                    and arm['z_cdeg'] - lower['z_cdeg'] >= GRASP_LIFT_CLEAR_MM * PICK_MOTOR_CDEG_PER_MM):
                lift_clear_s = self.clock()
                self.record('grasp_lift_feedback', color_id=self.color,
                            pick_to_lift_s=round(lift_clear_s - pick_started, 3),
                            first_target_to_lift_s=round(lift_clear_s - self.first_target_seen, 3)
                            if self.first_target_seen is not None else None,
                            lift_mm=round((arm['z_cdeg'] - lower['z_cdeg']) / PICK_MOTOR_CDEG_PER_MM, 2))
            if state == 'LIFTING' and not loading_seen:
                self.require_tray_count(arm, count_before)
                self.envelope(arm)
                low, high = sorted([lower['z_cdeg'], lift_z])
                if (not low-180 <= arm['z_cdeg'] <= high+180 or arm['target_z_cdeg'] != lift_z
                        or abs(arm['yaw_cdeg']-lower['yaw_cdeg']) > ARRIVAL_TOLERANCES[1]
                        or abs(arm['reach_cdeg']-lower['reach_cdeg']) > ARRIVAL_TOLERANCES[2]
                        or any(arm[f'target_{axis}_cdeg'] != lower[f'target_{axis}_cdeg']
                               for axis in ('yaw', 'reach'))):
                    raise RuntimeError('initial lift changed horizontal pose or left the Z stroke')
            elif state == 'TRAY_LOADING':
                loading_seen = True
                reported_reach = arm['target_reach_cdeg']
                if reported_reach in TRAY_REACH_TARGETS:
                    if tray_reach_target is not None and reported_reach != tray_reach_target:
                        raise RuntimeError('v7 tray reach target changed during transfer')
                    tray_reach_target = reported_reach
                # The return command targets park while the arm can still be at the tray.
                reach_limit = tray_reach_target or TRAY_REACH_TARGETS[0]
                nominal_min = reach_limit - TRAY_REACH_APPROACH_FEEDBACK_MARGIN
                # A second low sample or a larger excursion fails immediately.
                first_crossing = (
                    arm['active_axis'] == 2 and reported_reach == tray_reach_target
                    and previous_tray_pose is not None and previous_tray_pose['active_axis'] == 2
                    and previous_tray_pose['target_reach_cdeg'] == reported_reach
                    and previous_tray_pose['target_yaw_cdeg'] == arm['target_yaw_cdeg']
                    and previous_tray_pose['reach_cdeg'] >= nominal_min
                    and abs(previous_tray_pose['yaw_cdeg'] - arm['target_yaw_cdeg']) <= ARRIVAL_TOLERANCES[1]
                )
                self.check_tray_transfer_pose(arm, tray_reach_target=reach_limit,
                                              allow_single_peak=first_crossing)
                if arm['reach_cdeg'] < nominal_min:
                    self.record('tray_reach_recheck', previous_reach_cdeg=previous_tray_pose['reach_cdeg'],
                                reach_cdeg=arm['reach_cdeg'])
                previous_tray_pose = arm
                count = arm['tray_count']
                if not count_seen <= count <= count_before + 1:
                    raise RuntimeError('tray count regressed or incremented more than once')
                count_seen = count
            elif state == 'PICKED':
                self.require_tray_count(arm, count_before + 1)
                checked_arrival(arm, TRAY_PARK)
                return lower, arm
            else:
                raise RuntimeError('unexpected state during tray transfer: '+state)
            self.sleep(.12)
        raise TimeoutError('STM32 did not confirm PICKED after v7 tray transfer')

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
        self.color_started = self.clock()
        self.first_reachable = None
        self.first_usable = None
        self.first_target_seen = None
        self.view_targets = set()
        self.last_unreachable_view_hint = None
        self.last_usable_state = None
        self.failure_samples = 0
        self.next_failure_sample = self.clock()
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

    def sample_failed_observation(self, state, reason):
        if (self.save_failure is None or self.failure_samples >= 4
                or self.clock() < self.next_failure_sample):
            return
        obs = state['material_observation']
        key = f"{self.camera_session}-{self.generation}-{self.capture_epoch}-{obs['source_sequence']}"
        name = f"wait_{self.color}_{obs['source_sequence']}.png"
        self.next_failure_sample = self.clock() + 2.
        try:
            self.save_failure(key, self.output / name)
        except (OSError, ValueError, HTTPException) as exc:
            self.record('wait_sample_unavailable', color_id=self.color, reason=str(exc))
            return
        self.failure_samples += 1
        self.record('wait_sample', color_id=self.color, image=name, reason=reason,
                    sample_key=key, acquired_monotonic_s=obs['acquired_monotonic_s'],
                    search_window=state.get('material_target_window'), materials=state.get('materials'),
                    view_hint=state.get('material_view_hint'))

    def observe(self, floor, timeout=180):
        floor = max(floor, self.color_floor, self.last_acquired)
        deadline, last_status = self.clock()+timeout, float('-inf')
        reason = 'no fresh source frame'
        view_points = deque()
        unconfirmed_since = self.clock()
        failed_frames = 0
        count = self.tuning.confirm_frames
        samples = deque()
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
                reason = 'fresh camera or material observation unavailable'
                failed_frames = 0
                view_points.clear()
                samples.clear()
                self.sleep(.1)
                continue
            except (ObservationUnavailable,URLError,OSError,json.JSONDecodeError):
                reason = 'fresh camera or material observation unavailable'
                failed_frames = 0
                view_points.clear()
                samples.clear()
                self.sleep(.1)
                continue
            obs = state['material_observation']
            seq, stamp = obs.get('source_sequence'), obs.get('acquired_monotonic_s')
            if (not obs.get('valid') or obs.get('generation') != self.generation
                    or obs.get('capture_epoch') != self.capture_epoch
                    or not isinstance(stamp, (int, float)) or not 0 <= self.clock()-stamp < 1):
                reason = 'fresh camera or material observation unavailable'
                failed_frames = 0
                view_points.clear()
                samples.clear()
                self.sleep(.1)
                continue
            if seq <= self.last_sequence or stamp <= max(floor, self.last_acquired):
                self.sleep(.05)
                continue
            self.last_sequence, self.last_acquired = seq, stamp
            materials = state.get('materials', [])
            if (len(materials) != 1 or materials[0].get('color_id') != self.color
                    or materials[0].get('point_kind') != 'top_face_ellipse_center'):
                reason = state.get('material_localization_status') or 'no unique requested top face'
                samples.clear()
                failed_frames += 1
                if failed_frames >= 3:
                    self.sample_failed_observation(state, reason)
                hint = state.get('material_view_hint') or {}
                xy = hint.get('pixel') or []
                if (self.real_tray and len(self.view_targets) < 3 and not materials
                        and reason in ('top_face_not_confirmed','clipped_region_no_complete_rim')
                        and hint.get('color_id') == self.color and hint.get('source_sequence') == seq
                        and len(xy) == 2 and np.isfinite(xy).all()):
                    if view_points and stamp-view_points[-1][0] > .5:
                        view_points.clear()
                    view_points.append((stamp,xy))
                    if confirmation_window_ready(view_points, count, self.tuning.confirm_span_s):
                        points = [item[1] for item in view_points]
                        if np.ptp(points,axis=0).max() <= 5:
                            pixel = np.median(points, axis=0)
                            previous = self.last_unreachable_view_hint
                            if previous is None or np.linalg.norm(pixel - previous) >= 12:
                                if self.first_target_seen is None:
                                    self.first_target_seen = view_points[0][0]
                                raise ViewpointNeeded(pixel.tolist())
                else:
                    view_points.clear()
            else:
                view_points.clear()
                failed_frames = 0
                if samples and stamp - samples[-1][0] > .7:
                    samples.clear()
                track = materials[0].get('track_id')
                reason = 'waiting for consistent fresh top-face positions'
                point = [materials[0]['center_x'], materials[0]['center_y']]
                if not np.isfinite(point).all():
                    raise RuntimeError('nonfinite top face')
                radius = tray_grasp_radius(materials[0].get('top_face_axes'), self.tuning.tolerance_px)
                if samples and track != samples[-1][3]:
                    _, previous, _, previous_track, previous_radius = samples[-1]
                    reference = np.asarray(self.reference['reference_pixel'])
                    nearby_in_grasp = (
                        self.real_tray and track is not None and previous_track is not None
                        and np.linalg.norm(point - reference) <= radius
                        and np.linalg.norm(previous - reference) <= previous_radius
                        and np.linalg.norm(np.subtract(point, previous)) <= TRAY_GRASP_JITTER_PX
                    )
                    if not nearby_in_grasp:
                        samples.clear()
                samples.append((stamp, point, seq, track, radius))
                if confirmation_window_ready(samples, count, self.tuning.confirm_span_s):
                    points = [sample[1] for sample in samples]
                    radius = min(s[4] for s in samples)
                    errors = np.asarray(points) - self.reference['reference_pixel']
                    in_grasp = bool(np.all(np.linalg.norm(errors, axis=1) <= radius))
                    jitter = TRAY_GRASP_JITTER_PX if self.real_tray and in_grasp and track is not None else 3
                    frames = [dict(sequence=s[2], acquired=s[0]) for s in samples]
                    dynamics = observation_dynamics(points, frames)
                    spread = np.ptp(points, axis=0).max()
                    # A slowly rotating, uniquely tracked piece need not stand
                    # still for an approach. Descent still needs measured frames
                    # inside the grasp window, with the tighter jitter check.
                    tracking = (self.real_tray and not in_grasp and track is not None
                                and spread <= 12 and dynamics['velocity_reliable']
                                and np.linalg.norm(dynamics['velocity_px_s']) <= 30)
                    if spread <= jitter or tracking:
                        if self.real_tray and self.first_target_seen is None:
                            self.first_target_seen = samples[0][0]
                        # The actuator needs seconds to reach the piece. A
                        # steady traversal of the grasp window cannot authorize it.
                        if self.real_tray and in_grasp and dynamics['velocity_reliable']:
                            reason = 'target moving inside grasp window'
                            self.sleep(.05)
                            continue
                        self.last_usable_state = state
                        return dict(pixel=np.median(points, axis=0).tolist(), track_id=track,
                                    points=points, source_frames=frames, grasp_radius_px=radius,
                                    in_grasp_window=in_grasp, **dynamics)
                if self.clock() - unconfirmed_since >= max(1., self.tuning.confirm_span_s + .3):
                    self.sample_failed_observation(state, reason)
            self.sleep(.05)
        raise ObservationUnavailable(f'{COLORS[self.color]}: {reason}')

    def wait_reachable_target(self, timeout=180):
        """转盘目标未到可达位置时只等待，不把外围/前伸越界候选当作对齐失败。"""
        floor = self.clock()+self.tuning.settle_s
        deadline = self.clock()+timeout
        reason = 'requested color did not enter reachable approach region: '+COLORS[self.color]
        while deadline - self.clock() >= self.tuning.confirm_span_s:
            try:
                observation = self.observe(floor,
                    timeout=min(8, deadline-self.clock()))
            except ObservationUnavailable as exc:
                reason = str(exc)
                continue
            if self.real_tray and self.first_usable is None:
                self.first_usable = self.clock()
                self.record('first_usable_top_face', color_id=self.color, pixel=observation['pixel'],
                            color_to_first_usable_s=round(self.first_usable-self.color_started, 3))
            a = self.checked_arm()
            error = np.asarray(observation['pixel'])-self.reference['reference_pixel']
            radial, tangent = np.asarray(self.mapping['matrix']) @ error
            radius = self.reference['origin_radius_mm']+a['reach_cdeg']*113.1/36000
            full_yaw = a['yaw_cdeg']/300+math.degrees(math.atan2(tangent, radius+radial))
            full_reach = math.hypot(radius+radial, tangent)-self.reference['origin_radius_mm']
            if self.real_tray:
                if observation['in_grasp_window'] or tracking_approach(a, error, self.reference['origin_radius_mm'],
                        self.mapping['matrix'], self.tuning, observation['grasp_radius_px']) is not None:
                    return observation
            elif (self.tuning.accepts(error) or (radius+radial > 0
                    and -6 <= full_yaw <= 6 and -self.tuning.max_retraction_mm <= full_reach <= 0)):
                return observation
            reason = (f'{COLORS[self.color]}: top face outside reachable grasp window '
                      f'(yaw {full_yaw:.2f} deg, reach {full_reach:.1f} mm)')
            if self.real_tray and self.last_usable_state is not None:
                self.sample_failed_observation(self.last_usable_state, reason)
            self.record('waiting_reachable_target', color_id=self.color,
                pixel=observation['pixel'], estimated_yaw_deg=full_yaw,
                estimated_reach_mm=full_reach, motion_sent=False)
        raise ObservationUnavailable(reason)

    def align_color(self):
        servo = AdaptivePixelMap(self.mapping['matrix'])
        anchor, stagnant, total, adjustments = None, 0, 0., 0
        need_acquire, active_since, hold_reason = True, None, None
        coarse_used = False
        observation_timeout = 2 if self.real_tray else 8
        while True:
            try:
                observation = (self.wait_reachable_target(timeout=observation_timeout) if need_acquire else
                    self.observe(self.clock()+self.tuning.settle_s,timeout=observation_timeout))
            except ViewpointNeeded as hint:
                a = self.checked_arm()
                self.envelope(a)
                if a['state_name'] not in ('WAIT_VISION', 'ALIGNED'):
                    raise RuntimeError('arm left observation state')
                plan = None
                if not self.micro_only:
                    plan = measured_edge_view(a, hint.pixel, self.reference, self.mapping,
                                              self.tuning, self.view_targets)
                coarse_view = plan is not None
                if plan is None:
                    plan = measured_view_target(a, hint.pixel, self.reference, self.mapping,
                                                self.tuning, visited=self.view_targets)
                if (plan and adjustments < self.tuning.max_adjustments
                        and total+plan[1] <= self.tuning.max_total_correction_mm
                        and (active_since is None or self.clock()-active_since < self.tuning.alignment_budget_s)):
                    if active_since is None:
                        active_since = self.clock()
                    target, distance = plan
                    self.view_targets.add(tuple(target))
                    self.last_unreachable_view_hint = None
                    total += distance
                    adjustments += 1
                    self.record('rim_view_change', color_id=self.color, target_cdeg=target,
                                selection_pixel=hint.pixel, combined_move=coarse_view)
                    self.align_to(target, before=a)
                else:
                    self.last_unreachable_view_hint = hint.pixel
                    self.record('waiting_for_complete_rim', color_id=self.color, motion_sent=False,
                                selection_pixel=hint.pixel, reason='no_available_measured_view')
                anchor, stagnant, need_acquire = None, 0, True
                continue
            except ObservationUnavailable as e:
                self.record('waiting_target',color_id=self.color,reason=str(e),motion_sent=False)
                if anchor is not None or not self.real_tray:
                    servo.recover()
                anchor, stagnant, need_acquire = None, 0, True
                continue
            need_acquire = False
            if self.real_tray and self.first_reachable is None:
                self.first_reachable = self.clock()
                self.record('first_reachable', color_id=self.color,
                            color_to_first_reachable_s=round(self.first_reachable-self.color_started, 3))
            error = np.asarray(observation['pixel'])-self.reference['reference_pixel']
            self.record('visual_check', color_id=self.color, observation=observation,
                        error_px=error.tolist())
            a = self.checked_arm()
            if a['state_name'] not in ('ALIGNED','WAIT_VISION'):
                raise RuntimeError('arm left observation state')
            self.envelope(a)
            accepted = observation['in_grasp_window'] if self.real_tray else self.tuning.accepts(error)
            if accepted and a['state_name'] == 'ALIGNED':
                if self.save_frame:
                    self.save_frame(self.output/f'{self.color}_aligned.jpg')
                return a, observation, error.tolist()
            if self.real_tray and anchor is not None and anchor[0]['track_id'] != observation['track_id']:
                # The post-motion window already confirmed a unique target on fresh frames.
                # Discard the old track's motion model and use the new observation directly.
                servo = AdaptivePixelMap(self.mapping['matrix'])
                anchor, stagnant = None, 0
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
            if accepted:
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
                    coarse = False
                    if self.real_tray:
                        target, distance, coarse = tray_correction(a, pixel - self.reference['reference_pixel'],
                            self.reference['origin_radius_mm'], servo.correction_matrix, step_tuning,
                            observation['grasp_radius_px'], allow_coarse=not coarse_used and not self.micro_only)
                    else:
                        target, distance = correction_target(a, pixel,
                            self.reference['reference_pixel'], servo.correction_matrix,
                            self.reference['origin_radius_mm'], tuning=step_tuning)
                except TargetOutsideWorkspace as e:
                    self.record('waiting_reachable_target',color_id=self.color,
                        pixel=observation['pixel'],reason=str(e),motion_sent=False)
                    anchor, need_acquire = None, True
                    continue
                if total+distance>self.tuning.max_total_correction_mm:
                    total=self.tuning.max_total_correction_mm
                    anchor=None
                    continue
                if coarse:
                    coarse_used = True
                    self.record('coarse_approach', color_id=self.color, target_cdeg=target)
            if active_since is None:
                active_since = self.clock()
            self.record('servo_correction',color_id=self.color,predicted_pixel=(
                observation['pixel'] if distance==0 else pixel.tolist()),step_gain=servo.step_gain)
            anchor = (observation,a,error.copy())
            total += distance
            adjustments += 1
            self.align_to(target, before=a)

    def cycle(self):
        aligned, obs, error = self.align_color()
        result = dict(color_id=self.color, color_name=COLORS[self.color],
                      aligned_pixel=obs['pixel'], error_px=error,
                      physical_grasp_completed=False, physical_place_completed=False)
        if self.real_tray:
            now = self.clock()
            result.update(color_to_first_usable_s=round(self.first_usable-self.color_started, 3),
                          usable_to_pick_ready_s=round(now-self.first_usable, 3),
                          color_to_first_reachable_s=round(self.first_reachable-self.color_started, 3),
                          color_to_pick_ready_s=round(now - self.color_started, 3),
                          reachable_to_pick_ready_s=round(now - self.first_reachable, 3),
                          grasp_radius_px=obs['grasp_radius_px'])
            self.record('pick_ready', color_id=self.color, error_px=error,
                        color_to_first_usable_s=result['color_to_first_usable_s'],
                        usable_to_pick_ready_s=result['usable_to_pick_ready_s'],
                        color_to_pick_ready_s=result['color_to_pick_ready_s'],
                        reachable_to_pick_ready_s=result['reachable_to_pick_ready_s'],
                        grasp_radius_px=obs['grasp_radius_px'])
            lower, parked = self.load_tray(aligned)
            returned_to_origin = len(self.results) < 2
            if returned_to_origin:
                photo = self.align_to([0, 0, 0], before=parked, photo_return=True)
                self.require_tray_count(photo, parked['tray_count'])
            result.update(controller_tray_load_completed=True, tray_count=parked['tray_count'],
                          firmware_pick_drop_mm=lower['pick_drop_mm'],
                          simulated_grip=False, simulated_tray=False,
                          returned_to_origin=returned_to_origin)
        else:
            lower, lifted = self.lower_and_lift(aligned)
            self.align_to([0, 0, 0])
            self.record('simulated_tray_transfer', color_id=self.color, simulated=True,
                        physical_place_completed=False, servo_command_sent=False)
            result.update(lifted_z_cdeg=lifted['z_cdeg'], simulated_grip=True,
                          simulated_tray=True, returned_to_origin=True)
        result['lowered_mm'] = (aligned['z_cdeg']-lower['z_cdeg'])/900
        self.results.append(result)
        self.record('color_cycle_completed', **result)

    def run(self, order, test_id):
        order = parse_order(''.join(map(str, order)))
        a = self.client.arm_status()
        self.check_firmware_mode(a)
        if self.real_tray:
            self.require_tray_count(a, 0)
        chassis = self.client.status()
        if (a['state_name'] != 'READY' or not a['origins_valid'] or not a['standalone_test_mode']
                or a['fault'] or any(abs(a[k]) > 50 for k in ('z_cdeg','yaw_cdeg','reach_cdeg'))
                or chassis['state_name'] != 'EMPTY' or chassis['fault']):
            raise RuntimeError('fresh manual origin / standalone READY / EMPTY chassis required')
        self.context = (test_id, 0, 102)
        self.started = True
        self.record('sequence_started', order=order, order_source='user_confirmed_test_order',
                    simulated_servos=not self.real_tray, physical_grasp_completed=False,
                    expected_pick_drop_mm=self.pick_drop_mm, tuning=asdict(self.tuning))
        self.client.arm_test_begin(test_id, 102)
        self.wait_arm('WAIT_VISION', ('PREPARING',), target=[0, 0, 0])
        for color in order:
            self.set_color(color)
            self.cycle()
        if self.real_tray:
            self.command += 1
            self.client.arm_finish(*self.context, self.command)
            final_arm = self.wait_arm('READY_LEAVE', ('PARKING',), target=TRAY_PARK)
            self.require_tray_count(final_arm, 3)
            completion, held_at = 'three_color_tray_sequence_completed', 'transport_READY_LEAVE'
        else:
            final_arm = self.checked_arm()
            completion, held_at = 'three_color_simulation_completed', 'photo_origin_ALIGNED'
        result = dict(result=completion, order=order, colors=self.results, final_arm=final_arm,
            held_at=held_at, physical_grasp_completed=False, physical_place_completed=False,
            simulated_servos=not self.real_tray, expected_pick_drop_mm=self.pick_drop_mm,
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
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--simulate-servos', action='store_true')
    mode.add_argument('--real-tray', action='store_true',
                      help='v7 auto-claw and tray; requires an empty tray and a new manual KEY')
    parser.add_argument('--pick-drop-mm', type=int, choices=REAL_TRAY_PICK_DROPS_MM,
                        help='optional fixed stroke; real tray checks the STM32 report automatically by default')
    parser.add_argument('--tolerance-px',type=float,
        help='real tray: top-face offset cap (default 32 px); simulation: per-axis tolerance (default 8 px)')
    parser.add_argument('--confirm-frames',type=int,default=3)
    parser.add_argument('--confirm-span-s',type=float,default=.2)
    parser.add_argument('--settle-s',type=float)
    parser.add_argument('--micro-only', action='store_true',
                        help='bound visual correction commands to micro steps, including far targets')
    parser.add_argument('--save-aligned-frame', action='store_true', help='save a debug JPEG before each pick')
    parser.add_argument('--max-step-mm',type=float,default=3.5)
    parser.add_argument('--max-yaw-step-deg',type=float,default=.8)
    parser.add_argument('--max-retraction-mm',type=float,default=60,
        help='0..60mm, bounded by the existing horizontal micro-motion corridor')
    parser.add_argument('--alignment-budget-s',type=float,default=180,
        help='active alignment budget; after exhaustion hold and await target, never skip a color')
    args = parser.parse_args()
    order = parse_order(args.order)
    if not args.real_tray and args.pick_drop_mm not in (None, DEFAULT_PICK_DROP_MM):
        parser.error('simulation mode requires a 100 mm stroke')
    if args.tolerance_px is None:
        args.tolerance_px = 32. if args.real_tray else 8.
    if args.settle_s is None:
        args.settle_s = 0. if args.real_tray else .1
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
    def save_failure(key, path):
        with urlopen(f'http://127.0.0.1:8090/material-sample/{key}.png', timeout=.8) as reply:
            data = reply.read()
        if not data.startswith(b'\x89PNG\r\n\x1a\n'):
            raise ValueError('material sample is not a PNG')
        path.write_bytes(data)
    import serial
    with (Path(tempfile.gettempdir())/f'robot-preview-start-{os.getuid()}.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        subprocess.run(['systemctl','--user','stop','chassis-button.service'], check=True)
        with serial.Serial(args.port, 115200, timeout=.05, write_timeout=1, exclusive=True) as port:
            trial = ThreeColorTrial(ElectricalClient(port), reference, mapping, snapshot, select_color,
                args.output, simulate_servos=args.simulate_servos, real_tray=args.real_tray,
                pick_drop_mm=args.pick_drop_mm, tuning=tuning, micro_only=args.micro_only,
                save_failure=save_failure if args.real_tray else None,
                save_frame=save_frame if args.save_aligned_frame or args.simulate_servos else None)
            try:
                trial.wait_for_manual_ready()
                trial.run(order, int(time.time()))
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
