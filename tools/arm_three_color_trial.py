"""独立原料区三色测试，支持旧版模拟和 STM32 v7 自动入盘。"""
import argparse
from collections import deque
from dataclasses import asdict, dataclass, replace
from enum import IntEnum
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
from pi_vision.electrical_link import ArmStatusMissing, ElectricalClient, Rejected
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
    wide_tray_yaw: bool = False
    follow_controller_limits: bool = False

    def __post_init__(self):
        # Stay inside the firmware's ALIGN micro-motion corridor: 1deg/4mm,
        # front 60mm of travel. Raw feedback/actuator arrival checks stay strict.
        min_span = 0. if self.follow_controller_limits else .2
        min_frames = 1 if self.follow_controller_limits else 3
        max_tol = 120. if self.follow_controller_limits else 32.
        ranges = dict(tolerance_px=(0,max_tol), confirm_span_s=(min_span,2), settle_s=(0,2),
            max_step_mm=(0,3.8), max_yaw_step_deg=(0,.9),
            max_retraction_mm=(0,60), max_total_correction_mm=(0,120),
            alignment_budget_s=(0,600))
        for name,(lo,hi) in ranges.items():
            value = getattr(self,name)
            if (isinstance(value,bool) or not isinstance(value,(int,float))
                    or not math.isfinite(value) or not lo <= value <= hi
                    or (lo==0 and value==0 and name not in ('settle_s', 'confirm_span_s'))):
                raise ValueError('invalid trial tuning: '+name)
        if type(self.confirm_frames) is not int or not min_frames <= self.confirm_frames <= 15:
            raise ValueError('at least three distinct source frames required')
        if type(self.max_adjustments) is not int or not 1 <= self.max_adjustments <= 64:
            raise ValueError('invalid maximum adjustment count')

    def accepts(self, error):
        error = np.asarray(error, dtype=float)
        return bool(error.shape==(2,) and np.isfinite(error).all()
                    and np.max(np.abs(error)) <= self.tolerance_px)

    @property
    def tray_yaw_limits(self):
        return (-12., 22.) if self.wide_tray_yaw else (-6., 12.)


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
    ratio = .45 if cap > 32 else .25
    return min(cap, min(axes) * ratio)


TRAY_GRASP_JITTER_PX = 12
TRAY_STILL_SPEED_PX_S = 5.
TRAY_FAST_STILL_SPEED_PX_S = 20.
TRAY_MAX_TRACKING_SPEED_PX_S = 25.
RIM_HINT_STATUSES = ('top_face_not_confirmed', 'clipped_region_no_complete_rim')


def tray_photo_target(arm, radial, tangent, reference_radius, tuning):
    """Convert a local displacement in mm to a bounded photo-height ALIGN target."""
    reach = arm['reach_cdeg'] * 113.1 / 36000
    yaw = arm['yaw_cdeg'] / 300
    radius = reference_radius + reach
    if not np.isfinite([radius, radial, tangent]).all() or radius <= 0 or radius + radial <= 0:
        return None
    desired_yaw = yaw + math.degrees(math.atan2(tangent, radius + radial))
    desired_reach = math.hypot(radius + radial, tangent) - reference_radius
    if tuning.follow_controller_limits:
        return 0, round(desired_yaw * 300), round(desired_reach * 36000 / 113.1)
    target_yaw = round(np.clip(desired_yaw, *tuning.tray_yaw_limits) * 300)
    target_reach = max(math.ceil(-tuning.max_retraction_mm * 36000 / 113.1),
                       min(0, round(desired_reach * 36000 / 113.1)))
    return 0, target_yaw, target_reach


def tray_approach(arm, error, reference_radius, matrix, tuning, grasp_radius):
    """Find a reachable photo-height pose and check its predicted residual in pixels."""
    matrix = np.asarray(matrix)
    radial, tangent = matrix @ error
    target = tray_photo_target(arm, radial, tangent, reference_radius, tuning)
    if target is None:
        return None
    end = dict(yaw_cdeg=target[1], reach_cdeg=target[2])
    displacement = local_motion_mm(arm, end, reference_radius)
    distance = float(np.linalg.norm(displacement))
    remaining = error - np.linalg.solve(matrix, displacement)
    residual_px = np.linalg.norm(remaining)
    if residual_px > grasp_radius:
        # A target just past the centre-reach boundary can still benefit from
        # one horizontal approach. Re-observe there; this never authorizes PICK.
        if (residual_px > grasp_radius + 12 or np.linalg.norm(error) - residual_px < 12
                or distance < 2.):
            return None
    return list(target), distance


def tracking_approach(arm, error, radius, matrix, tuning, grasp_radius):
    plan = tray_approach(arm, error, radius, matrix, tuning, grasp_radius)
    # Distant lateral and lower-edge targets can move farther in the image per
    # actuator step than the origin calibration predicts. Permit a bounded
    # approach; only subsequent measured pixels can authorize a descent.
    if plan is None and (abs(error[0]) > 2 * abs(error[1]) or np.linalg.norm(error) > 120):
        plan = tray_approach(arm, error, radius, .66 * np.asarray(matrix), tuning, grasp_radius)
    return plan


def micro_start_ok(arm):
    # This predicts the STM32 micro corridor, including its actual zero limits.
    return -50 <= arm['z_cdeg'] <= 0 and TRAY_PARK[2] <= arm['reach_cdeg'] <= 0


def calibrated_view_poses(mapping):
    for sample in mapping.get('samples', ()):
        actual, requested = sample.get('actual', {}), sample.get('requested', {})
        if len(sample.get('source_frames', ())) < 3:
            continue
        yaw, reach = requested.get('yaw_deg'), requested.get('reach_mm')
        values = (yaw, reach, actual.get('yaw_deg'), actual.get('reach_mm'), actual.get('z_mm'))
        if not all(isinstance(v, (float,int)) and math.isfinite(v) for v in values):
            continue
        if (abs(actual['z_mm']) > .15 or abs(yaw-actual['yaw_deg']) > .2
                or abs(reach-actual['reach_mm']) > .3):
            continue
        yield 0, round(yaw*300), round(reach*36000/113.1)


def usable_view_pose(target, tuning, visited):
    _, yaw, reach = target
    not_visited = not any(abs(yaw - previous[1]) <= ARRIVAL_TOLERANCES[1]
                          and abs(reach - previous[2]) <= ARRIVAL_TOLERANCES[2]
                          for previous in visited)
    if tuning.follow_controller_limits:
        return not_visited
    min_yaw, max_yaw = tuning.tray_yaw_limits
    return (min_yaw * 300 <= yaw <= max_yaw * 300
            and math.ceil(-tuning.max_retraction_mm * 36000 / 113.1) <= reach <= 0
            and not_visited)


def view_start_ok(arm):
    # An observed start may be just outside the legal command range.
    return (abs(arm['z_cdeg']) <= PHOTO_VIEW_MAX_Z_CDEG
            and TRAY_PARK[2] <= arm['reach_cdeg'] <= TRAY_TRANSFER_HARD_FORWARD_CDEG)


def measured_view_target(arm, pixel, reference, mapping, tuning, visited=()):
    if not tuning.follow_controller_limits and not view_start_ok(arm):
        return None
    error = np.asarray(pixel) - reference['reference_pixel']
    best_error = np.linalg.norm(error) - 3
    best = None
    for target in calibrated_view_poses(mapping):
        if (not usable_view_pose(target, tuning, visited)
                or abs(target[1]-arm['yaw_cdeg']) > tuning.max_yaw_step_deg*300
                or abs(target[2]-arm['reach_cdeg'])*113.1/36000 > tuning.max_step_mm):
            continue
        end = dict(yaw_cdeg=target[1], reach_cdeg=target[2])
        delta = local_motion_mm(arm,end,reference['origin_radius_mm'])
        remaining = np.linalg.norm(error-np.linalg.solve(mapping['matrix'], delta))
        if remaining < best_error:
            best_error, best = remaining, (list(target), float(np.linalg.norm(delta)))
    return best


def edge_view_target(arm, pixel, reference, mapping, tuning, visited):
    """Use the partial rim hint to move the camera toward a clearer view."""
    if not tuning.follow_controller_limits and not view_start_ok(arm):
        return None
    width, height = reference['resolution']
    lower = np.array([.38 * width, .31 * height])
    upper = np.array([.62 * width, .69 * height])
    point = np.asarray(pixel)
    desired = point - np.clip(point, lower, upper)
    initial_residual = np.linalg.norm(desired)
    if initial_residual < 18:
        return None
    matrix = np.asarray(mapping['matrix'])
    reference_radius = reference['origin_radius_mm']
    # Aim for the nearest point of the inner window. Try a damped move first
    # because the origin calibration can overestimate motion near an edge.
    for gain in (.8, 1.):
        radial, tangent = matrix @ (gain * desired)
        target = tray_photo_target(arm, radial, tangent, reference_radius, tuning)
        if target is None or not usable_view_pose(target, tuning, visited):
            continue
        end = dict(yaw_cdeg=target[1], reach_cdeg=target[2])
        delta = local_motion_mm(arm, end, reference_radius)
        distance = float(np.linalg.norm(delta))
        if distance < 2.:
            continue
        predicted = point - np.linalg.solve(matrix, delta)
        remaining = np.linalg.norm(predicted - np.clip(predicted, lower, upper))
        if remaining <= initial_residual - 12:
            return list(target), distance
    return None


class ViewpointNeeded(Exception):
    """Unique colour is visible, but the rim has not been localized."""
    def __init__(self, pixel):
        self.pixel = pixel


def tray_correction(arm, error, reference_radius, matrix, tuning, grasp_radius, *, allow_coarse):
    plan = tracking_approach(arm, error, reference_radius, matrix, tuning, grasp_radius)
    if plan is None:
        raise TargetOutsideWorkspace('grasp window outside the reachable workspace')
    target, distance = plan
    if tuning.follow_controller_limits and allow_coarse:
        # STM32 原料区支持扇区内直接一步到位，不再切成 0.8°/3.5mm 碎步。
        return target, distance, False
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
        if not tuning.follow_controller_limits:
            min_yaw, max_yaw = tuning.tray_yaw_limits
            target[1] = int(np.clip(target[1], 300 * min_yaw, 300 * max_yaw))
            target[2] = int(np.clip(target[2], math.ceil(-tuning.max_retraction_mm * 36000 / 113.1), 0))
        distance *= fraction
    return target, distance, coarse


# Motor centidegrees from the controller headers and reported field settings.
TRAY_PARK = (0, 0, -19098)
# Safe tray approach range: yaw 110..165 deg, reach -140..-90 mm.
# Accepts field tuning (e.g. 120/142/147/150 deg and 100/115/125/130 mm) without stopping.
TRAY_YAW_RANGE_CDEG = (33000, 49500)
TRAY_REACH_RANGE_CDEG = (-44562, -28647)


def is_tray_approach_target(yaw_cdeg, reach_cdeg):
    return (TRAY_YAW_RANGE_CDEG[0] <= yaw_cdeg <= TRAY_YAW_RANGE_CDEG[1]
            and TRAY_REACH_RANGE_CDEG[0] <= reach_cdeg <= TRAY_REACH_RANGE_CDEG[1])


def is_tray_park_target(yaw_cdeg, reach_cdeg):
    return (abs(yaw_cdeg - TRAY_PARK[1]) <= ARRIVAL_TOLERANCES[1]
            and abs(reach_cdeg - TRAY_PARK[2]) <= ARRIVAL_TOLERANCES[2])
TRAY_YAW_APPROACH_FEEDBACK_MARGIN = 180
TRAY_REACH_APPROACH_FEEDBACK_MARGIN = 180
PHOTO_RETURN_FORWARD_MARGIN_MM = 0.6
SOFT_FORWARD_MARGIN_MM = .4
HARD_FORWARD_MARGIN_MM = 4.
HARD_RETRACTION_MM = 64.
HARD_Z_MARGIN_MM = 3.
HARD_YAW_MARGIN_DEG = 2.5
HARD_ENVELOPE_FRAMES = 3
TRAY_TRANSFER_SOFT_FORWARD_CDEG = 127
TRAY_TRANSFER_HARD_FORWARD_CDEG = 955
TRAY_TRANSFER_HARD_REACH_MARGIN_CDEG = 1273
PHOTO_VIEW_MAX_Z_CDEG = 450
SETTLE_CONFIRM_S = .6
ARM_STATUS_RECOVERY_S = 2.
ARRIVAL_TOLERANCES = (180, 60, 95)
ARRIVAL_HARD_TOLERANCES = (2700, 750, 1273)
# Relative limits for a Z-only move are tighter than the global travel envelope.
VERTICAL_HARD_TOLERANCES = (1800, 450, 636)
STARTUP_ORIGIN_TOLERANCES = (450, 180, 300)
ARM_AXES = ('z', 'yaw', 'reach')
PICK_MOTOR_CDEG_PER_MM = 900
GRASP_LIFT_CLEAR_MM = 5
DEFAULT_PICK_DROP_MM = 100
DEFAULT_ALIGN_PICK_DROP_CDEG = 59450
REAL_TRAY_PICK_DROPS_MM = (50, 80, DEFAULT_PICK_DROP_MM)
# 45950 cdeg was reported on the car. 77450 (+35 mm) and 90950 (+50 mm) extend that stroke.
REAL_TRAY_PICK_DROPS_CDEG = (45000, 45950, 59450, 72000, 77450, 90000, 90950)


class FeedbackLevel(IntEnum):
    OK = 0
    SOFT = 1
    HARD = 2


class FeedbackDebounce:
    """Track one motion phase using distinct STM32 reports."""

    def __init__(self):
        self.hard_frames = 0
        self.last_tick = None
        self.warned = False

    def check(self, trial, arm, level, *, phase, terminal=False):
        tick = arm.get('tick')
        if tick is not None and self.last_tick is not None and not terminal:
            if not 0 < (tick - self.last_tick) % (1 << 32) < (1 << 31):
                return False
        self.last_tick = tick
        self.hard_frames = self.hard_frames + 1 if level == FeedbackLevel.HARD else 0
        if level == FeedbackLevel.HARD and (terminal or self.hard_frames >= HARD_ENVELOPE_FRAMES):
            raise RuntimeError(f'arm left hard trial envelope in {phase}: '
                               f'z={arm["z_cdeg"]}, yaw={arm["yaw_cdeg"]}, '
                               f'reach={arm["reach_cdeg"]} cdeg')
        if level != FeedbackLevel.OK and not self.warned:
            trial.record('envelope_warning', color_id=trial.color, arm=arm, phase=phase,
                         severity=level.name.lower())
            self.warned = True
        return level != FeedbackLevel.HARD


def arrival_drift(arm, target):
    return tuple(abs(arm[f'{axis}_cdeg'] - value) for axis, value in zip(ARM_AXES, target))


def checked_arrival(arm, target):
    if any(drift > tolerance for drift, tolerance in zip(arrival_drift(arm, target), ARRIVAL_TOLERANCES)):
        raise RuntimeError('arrival differs from requested pose')


def pose_feedback_level(drift, hard_tolerances=VERTICAL_HARD_TOLERANCES):
    if any(value > maximum for value, maximum in zip(drift, hard_tolerances)):
        return FeedbackLevel.HARD
    if any(value > maximum for value, maximum in zip(drift, ARRIVAL_TOLERANCES)):
        return FeedbackLevel.SOFT
    return FeedbackLevel.OK


def vertical_feedback_drift(arm, before, target_z):
    low, high = sorted((before['z_cdeg'], target_z))
    return (max(low - arm['z_cdeg'], arm['z_cdeg'] - high, 0),
            abs(arm['yaw_cdeg'] - before['yaw_cdeg']),
            abs(arm['reach_cdeg'] - before['reach_cdeg']))


def z_only_target_feedback(arm, before, target_z):
    # A near report can settle on the next poll. Keep the expected target
    # locked so that the near report cannot authorize the next command.
    z_allowance, yaw_allowance, reach_allowance = (50, 60, 95) if before.get('tray_enabled') else (0, 0, 0)
    z_error = abs(arm['target_z_cdeg'] - target_z)
    yaw_error = abs(arm['target_yaw_cdeg'] - before['target_yaw_cdeg'])
    reach_error = abs(arm['target_reach_cdeg'] - before['target_reach_cdeg'])
    if z_error > z_allowance:
        raise RuntimeError(f'Z-only target mismatch: expected {target_z} cdeg, '
                           f'controller reports {arm["target_z_cdeg"]} cdeg')
    if yaw_error > yaw_allowance or reach_error > reach_allowance:
        raise RuntimeError('Z-only motion changed horizontal target')
    return FeedbackLevel.SOFT if z_error or yaw_error or reach_error else FeedbackLevel.OK


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
    def envelope(a, max_yaw_deg=6., *, min_yaw_deg=-6., max_forward_mm=.4):
        if (not -100.4 <= a['z_cdeg']/900 <= .2
                or not -60.6 <= a['reach_cdeg']*113.1/36000 <= max_forward_mm
                or not min_yaw_deg - .3 <= a['yaw_cdeg']/300 <= max_yaw_deg + .3):
            raise RuntimeError('arm left trial stroke/clearance envelope')

    def motion_feedback_level(self, arm, *, lower_z_cdeg=None, photo_return=False):
        self.envelope(arm)
        return FeedbackLevel.OK

    def arrival_feedback_level(self, arm, target):
        checked_arrival(arm, target)
        return FeedbackLevel.OK

    def vertical_feedback_level(self, arm, before, target_z):
        z_drift, yaw_drift, reach_drift = vertical_feedback_drift(arm, before, target_z)
        if yaw_drift > ARRIVAL_TOLERANCES[1] or reach_drift > ARRIVAL_TOLERANCES[2]:
            raise RuntimeError('Z-only motion changed horizontal feedback')
        if z_drift > ARRIVAL_TOLERANCES[0]:
            raise RuntimeError('Z feedback overshot requested stroke')
        return FeedbackLevel.OK

    def wait_arm(self, wanted, transitional, *, target=None, vertical_from=None,
                 pick_drop_cdeg_options=None, photo_return=False, timeout=40):
        start = self.clock()
        deadline = start+timeout
        full_avoidance = False
        minimum_reach = None
        selected_pick_drop_cdeg = None
        trust_controller = bool(getattr(self, 'real_tray', False))
        feedback = FeedbackDebounce()
        settling = False
        settling_since = None
        while settling or self.clock() < deadline:
            a = self.checked_arm()
            self.record('feedback', color_id=self.color, arm=a)
            state = a['state_name']
            terminal = state == wanted
            if not terminal and state not in transitional:
                raise RuntimeError('unexpected state '+state)
            if not terminal and self.clock() >= deadline:
                break
            target_level = FeedbackLevel.OK
            if photo_return and not trust_controller:
                commanded_z, yaw, reach = target
                yaw_error = abs(a['target_yaw_cdeg'] - yaw)
                reach_error = abs(a['target_reach_cdeg'] - reach)
                if (abs(a['target_z_cdeg'] - commanded_z) > PHOTO_VIEW_MAX_Z_CDEG
                        or yaw_error > ARRIVAL_TOLERANCES[1] or reach_error > ARRIVAL_TOLERANCES[2]):
                    raise RuntimeError('photo return changed requested pose')
                if yaw_error or reach_error:
                    target_level = FeedbackLevel.SOFT
            in_photo_return = (photo_return and state == 'ALIGNING' and a['active_axis'] == 2
                               and a['target_yaw_cdeg'] == a['target_reach_cdeg'] == 0)
            vertical_level = FeedbackLevel.OK
            if wanted == 'ALIGNED':
                minimum_reach = a['reach_cdeg'] if minimum_reach is None else min(minimum_reach, a['reach_cdeg'])
            if wanted=='ALIGNED' and a['state_name']=='ALIGNING' and a['active_axis']==0:
                full_avoidance = True
            if (wanted=='ALIGNED' and target is not None and
                    abs(a['reach_cdeg']-target[2])*113.1/36000>self.tuning.max_step_mm+1):
                full_avoidance = True
            if vertical_from is not None:
                if pick_drop_cdeg_options is not None and selected_pick_drop_cdeg is None:
                    commanded_z = a['target_z_cdeg']
                    if trust_controller:
                        # Follow whatever pick stroke STM32 is configured with; never stop on stroke tuning.
                        selected_pick_drop_cdeg = max(0, vertical_from['z_cdeg'] - commanded_z)
                    else:
                        expected_lift_refs = [vertical_from['z_cdeg']]
                        if abs(vertical_from['target_z_cdeg'] - vertical_from['z_cdeg']) <= 180:
                            expected_lift_refs.append(vertical_from['target_z_cdeg'])
                        for drop_cdeg in pick_drop_cdeg_options:
                            lift_ref = commanded_z + drop_cdeg
                            if any(lift_ref == ref for ref in expected_lift_refs):
                                selected_pick_drop_cdeg = drop_cdeg
                                break
                        if selected_pick_drop_cdeg is None:
                            raise RuntimeError(f'Z-only target mismatch: controller reports {commanded_z} cdeg, '
                                               f'allowed pick strokes {pick_drop_cdeg_options} cdeg')
                    target = [commanded_z, *target[1:]]
                    self.record('pick_stroke_confirmed', color_id=self.color, command=self.command,
                                drop_cdeg=selected_pick_drop_cdeg,
                                drop_mm=selected_pick_drop_cdeg / PICK_MOTOR_CDEG_PER_MM,
                                target_z_cdeg=commanded_z)
                if a['aligned_command_id'] != vertical_from['aligned_command_id']:
                    raise RuntimeError('Z-only motion changed alignment reference')
                if not trust_controller:
                    target_level = z_only_target_feedback(a, vertical_from, target[0])
                    vertical_level = self.vertical_feedback_level(a, vertical_from, target[0])
            if terminal and wanted == 'ALIGNED' and a['aligned_command_id'] != self.command:
                raise RuntimeError('ALIGNED reference mismatch')
            if trust_controller:
                if terminal:
                    self.last_motion = dict(elapsed_s=self.clock()-start, full_avoidance=full_avoidance)
                    trace = {} if minimum_reach is None else dict(min_reach_mm=round(minimum_reach*113.1/36000, 2))
                    self.record('arrived', state=wanted, **self.last_motion, arm=a, **trace)
                    if selected_pick_drop_cdeg is not None:
                        return dict(a, pick_drop_cdeg=selected_pick_drop_cdeg,
                                    pick_drop_mm=selected_pick_drop_cdeg / PICK_MOTOR_CDEG_PER_MM)
                    return a
                self.sleep(.02)
                continue
            lower_z = min(target[0], vertical_from['z_cdeg']) if vertical_from is not None else None
            absolute_level = self.motion_feedback_level(a, lower_z_cdeg=lower_z, photo_return=in_photo_return)
            arrival_level = (self.arrival_feedback_level(a, target)
                             if terminal and target is not None else FeedbackLevel.OK)
            pose_level = max(target_level, vertical_level, arrival_level)
            phase = f'{state} arrival' if terminal else 'photo_return' if in_photo_return else state
            fresh_pose = feedback.check(self, a, max(absolute_level, pose_level), phase=phase)
            soft_settled = (
                terminal and fresh_pose and pose_level == FeedbackLevel.SOFT
                and settling_since is not None and self.clock() - settling_since >= SETTLE_CONFIRM_S
            )
            if terminal and (not fresh_pose or (pose_level != FeedbackLevel.OK and not soft_settled)):
                if not settling:
                    settling_since = self.clock()
                    self.record('waiting_pose_confirmation', color_id=self.color, phase=state,
                                target_cdeg=target, arm=a)
                settling = True
            elif terminal:
                if soft_settled:
                    self.record('arrival_soft_accepted', color_id=self.color, phase=state,
                                target_cdeg=target, arm=a)
                self.last_motion = dict(elapsed_s=self.clock()-start,full_avoidance=full_avoidance)
                trace = {} if minimum_reach is None else dict(min_reach_mm=round(minimum_reach*113.1/36000, 2))
                self.record('arrived', state=wanted, **self.last_motion, arm=a, **trace)
                if selected_pick_drop_cdeg is not None:
                    return dict(a, pick_drop_cdeg=selected_pick_drop_cdeg,
                                pick_drop_mm=selected_pick_drop_cdeg / PICK_MOTOR_CDEG_PER_MM)
                return a
            else:
                settling = False
                settling_since = None
            self.sleep(.12)
        raise TimeoutError('motion timeout waiting '+wanted)

    def align_to(self, target, *, before=None, photo_return=False, preselect_color=None):
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
        try:
            self.client.arm_align(*self.context, self.command, *(v/100 for v in target))
        except Rejected:
            self.command -= 1
            raise
        if preselect_color is not None and getattr(self, 'real_tray', False):
            # 回拍照原点途中提前切好下一个颜色，视觉同步预热，到点即抓。
            try:
                self.select_color(preselect_color)
                self._preselected_color = preselect_color
                self._preselect_floor = self.clock()
                self.record('preselect_color', next_color_id=preselect_color)
            except (HTTPError, URLError, OSError):
                self._preselected_color = None
        return self.wait_arm('ALIGNED', ('ALIGNING',), target=target, photo_return=photo_return)

    def lower_for_pick(self, aligned, *, drop_mm=DEFAULT_PICK_DROP_MM):
        aligned_id = aligned['aligned_command_id']
        if aligned_id != self.command:
            raise RuntimeError('cannot PICK without latest verified alignment')
        self.command += 1
        self.record('pick_request', color_id=self.color, reference_id=aligned_id,
                    command=self.command, expected_drop_mm=drop_mm)
        self.client.arm_pick(*self.context, self.command, aligned_id)
        allowed = REAL_TRAY_PICK_DROPS_CDEG if drop_mm is None else (drop_mm * PICK_MOTOR_CDEG_PER_MM,)
        down = [aligned['z_cdeg'] - allowed[0],
                aligned['yaw_cdeg'], aligned['reach_cdeg']]
        return self.wait_arm('WAIT_GRIP', ('LOWERING',), target=down, vertical_from=aligned,
                             pick_drop_cdeg_options=allowed)

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
                 clock=time.monotonic, sleep=time.sleep, save_frame=None, save_failure=None, tuning=None, micro_only=False,
                 empty_claw_check=False, max_empty_claw_retries=3,
                 mid_drop_check_mm=60., grasp_check_radius_px=640., grasp_close_wait_s=.55,
                 grasp_check_frames=3, grasp_check_min_hits=2, grasp_check_timeout_s=2.5,
                 direct_pick_after_align=False, patch_snapshot=None,
                 align_pick=False, align_pick_drop_cdeg=None):
        if (simulate_servos is True) == (real_tray is True):
            raise ValueError('choose exactly one of --simulate-servos and --real-tray')
        for k in ('camera_source', 'hsv_profile_sha256', 'resolution', 'origin_radius_mm'):
            if reference.get(k) != mapping['reference'].get(k):
                raise ValueError('mapping/reference identity mismatch: '+k)
        matrix = np.asarray(mapping['matrix'], dtype=float)
        if matrix.shape != (2, 2) or not np.isfinite(matrix).all():
            raise ValueError('finite 2x2 mapping required')
        defaults = (TrialTuning(tolerance_px=32., settle_s=0., follow_controller_limits=True)
                    if real_tray else TrialTuning())
        super().__init__(client, output, tuning or defaults, clock=clock, sleep=sleep)
        self.real_tray = real_tray is True
        if self.tuning.wide_tray_yaw and not self.real_tray:
            raise ValueError('wide yaw requires real tray mode')
        if empty_claw_check and not self.real_tray:
            raise ValueError('empty-claw check requires real tray mode')
        if align_pick and not self.real_tray:
            raise ValueError('align-pick requires real tray mode')
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
        self.empty_claw_check = bool(empty_claw_check)
        self.max_empty_claw_retries = int(max_empty_claw_retries)
        self.mid_drop_check_mm = float(mid_drop_check_mm)
        self.grasp_check_radius_px = float(grasp_check_radius_px)
        self.patch_snapshot = patch_snapshot
        if empty_claw_check and not 1 <= self.grasp_check_radius_px <= max(reference['resolution']):
            raise ValueError('empty-claw check requires a valid radius')
        self.grasp_close_wait_s = float(grasp_close_wait_s)
        self.grasp_check_frames = int(grasp_check_frames)
        self.grasp_check_min_hits = int(grasp_check_min_hits)
        self.grasp_check_timeout_s = float(grasp_check_timeout_s)
        self.direct_pick_after_align = bool(direct_pick_after_align)
        self.align_pick = bool(align_pick)
        self.align_pick_drop_cdeg = (int(align_pick_drop_cdeg) if align_pick_drop_cdeg is not None
                                     else (pick_drop_mm * PICK_MOTOR_CDEG_PER_MM if pick_drop_mm is not None
                                           else DEFAULT_ALIGN_PICK_DROP_CDEG))
        if self.align_pick_drop_cdeg <= 0:
            raise ValueError('align_pick_drop_cdeg must be positive')
        self._pending_align_pick = None
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

    def envelope(self, arm):
        if not self.real_tray:
            super().envelope(arm)

    def motion_feedback_level(self, arm, *, lower_z_cdeg=None, photo_return=False):
        if not self.real_tray:
            super().envelope(arm)
        return FeedbackLevel.OK

    def arrival_feedback_level(self, arm, target):
        if not self.real_tray:
            return super().arrival_feedback_level(arm, target)
        return FeedbackLevel.OK

    def vertical_feedback_level(self, arm, before, target_z):
        if not self.real_tray:
            return super().vertical_feedback_level(arm, before, target_z)
        return FeedbackLevel.OK

    def align_to(self, target, *, before=None, photo_return=False, preselect_color=None):
        if (self.real_tray and not self.tuning.follow_controller_limits
                and (target[0] != 0 or not usable_view_pose(target, self.tuning, ()))):
            raise RuntimeError('ALIGN target outside the photo-height command limits')
        return super().align_to(target, before=before, photo_return=photo_return,
                                preselect_color=preselect_color)

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
        recovery_deadline = None
        while True:
            try:
                arm = super().checked_arm(expected_command)
            except ArmStatusMissing as exc:
                if not self.real_tray:
                    raise
                now = self.clock()
                if recovery_deadline is None:
                    recovery_deadline = now + ARM_STATUS_RECOVERY_S
                    self.record('waiting_arm_status', color_id=self.color, command=self.command,
                                error=str(exc), motion_sent=False)
                elif now >= recovery_deadline:
                    raise
                self.sleep(.12)
                continue
            self.check_firmware_mode(arm)
            if recovery_deadline is not None:
                self.record('arm_status_recovered', color_id=self.color, command=self.command, arm=arm)
            return arm

    @staticmethod
    def require_tray_count(arm, expected):
        if arm.get('tray_count') != expected:
            raise RuntimeError(f'STM32 tray count {arm.get("tray_count")} differs from expected {expected}')

    @staticmethod
    def check_tray_transfer_pose(arm, *, tray_approach_target=None,
                                 allow_single_peak=False, min_yaw_deg=-6.):
        _, yaw_margin, reach_margin = ARRIVAL_TOLERANCES
        yaw_limit, tray_reach_target = tray_approach_target or (TRAY_YAW_RANGE_CDEG[1], TRAY_REACH_RANGE_CDEG[0])
        reported_yaw = arm['target_yaw_cdeg']
        reported_reach = arm['target_reach_cdeg']
        if is_tray_approach_target(reported_yaw, reported_reach):
            if abs(arm['target_z_cdeg'] - TRAY_PARK[0]) > PHOTO_VIEW_MAX_Z_CDEG:
                raise RuntimeError('unexpected v7 tray approach target')
            tray_reach_target = reported_reach
            yaw_limit = reported_yaw
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
        elif not is_tray_park_target(reported_yaw, reported_reach):
            raise RuntimeError(f'unexpected v7 tray approach target: yaw={reported_yaw}, reach={reported_reach}')
        hard_z = HARD_Z_MARGIN_MM * PICK_MOTOR_CDEG_PER_MM
        hard_yaw_min = (min_yaw_deg - HARD_YAW_MARGIN_DEG) * 300
        hard_yaw_max = yaw_limit + HARD_YAW_MARGIN_DEG * 300
        hard_reach_min = tray_reach_target - TRAY_TRANSFER_HARD_REACH_MARGIN_CDEG
        if (not -hard_z <= arm['z_cdeg'] <= hard_z
                or not hard_yaw_min <= arm['yaw_cdeg'] <= hard_yaw_max
                or arm['reach_cdeg'] < hard_reach_min
                or arm['reach_cdeg'] > TRAY_TRANSFER_HARD_FORWARD_CDEG):
            return FeedbackLevel.HARD
        soft_yaw_min = min_yaw_deg * 300 - 90
        reach_min = tray_reach_target - reach_margin
        if (not -180 <= arm['z_cdeg'] <= 180
                or not soft_yaw_min <= arm['yaw_cdeg'] <= yaw_limit + yaw_margin
                or arm['reach_cdeg'] < reach_min
                or arm['reach_cdeg'] > TRAY_TRANSFER_SOFT_FORWARD_CDEG):
            return FeedbackLevel.SOFT
        return FeedbackLevel.OK

    def check_object_present_fast(self, z_cdeg, *, stage='mid_drop_6cm', timeout_s=.20):
        """One positive raw frame passes immediately. Require two different frames for a miss."""
        started = self.clock()
        floor = max(started, self.last_acquired)
        samples = []
        outcome = 'unconfirmed'
        radius = round(self.grasp_check_radius_px)
        if self.patch_snapshot is not None:
            deadline = started + timeout_s
            x, y = (round(v) for v in self.reference['reference_pixel'])
            min_area = max(16, min(250, round(.0025 * math.pi * radius**2)))
            mode = 'grasp_patch'
            while self.clock() < deadline:
                try:
                    sample = self.patch_snapshot(x, y, radius, max(.05, deadline-self.clock()))
                except (HTTPException, OSError, ValueError):
                    sample = None
                if sample:
                    if sample.get('source') == 'frame_jpg':
                        mode = 'frame_jpg_any_color'
                        deadline = max(deadline, started + max(timeout_s, .55))
                        frame_id = sample.get('frame_id', 0)
                        last_fid = getattr(self, '_last_frame_patch_id', 0)
                        if frame_id > last_fid and sample['acquired'] > started - .25:
                            self._last_frame_patch_id = frame_id
                            samples.append(sample)
                            if sample['area_px'] >= min_area:
                                outcome = 'present'
                                break
                            if len(samples) == 2:
                                outcome = 'absent'
                                break
                    else:
                        if (sample['session_id'], sample['generation'], sample['capture_epoch'], sample['color_id']) != (
                                self.camera_session, self.generation, self.capture_epoch, self.color):
                            raise RuntimeError('grasp patch camera or selected color changed')
                        if (sample['sequence'] > self.last_sequence and floor < sample['acquired'] <= self.clock() <= deadline):
                            self.last_sequence, self.last_acquired = sample['sequence'], sample['acquired']
                            floor = sample['acquired']
                            samples.append(sample)
                            if sample['area_px'] >= min_area:
                                outcome = 'present'
                                break
                            if len(samples) == 2:
                                outcome = 'absent'
                                break
                self.sleep(min(.01, max(0., deadline-self.clock())))
        else:
            # 视觉服务尚未升级 /grasp-patch 时回退到 /state：只要画面识别到目标颜色即判定未空爪，不做中心距离拦截。
            deadline = started + max(timeout_s, .50)
            min_area = 0
            mode = 'state_color_fallback'
            while self.clock() < deadline:
                try:
                    state = self.snapshot()
                    self.camera_identity(state)
                except (HTTPException, ObservationUnavailable, URLError, OSError, json.JSONDecodeError):
                    self.sleep(min(.02, max(0., deadline-self.clock())))
                    continue
                obs = state.get('material_observation') or {}
                seq, stamp = obs.get('source_sequence'), obs.get('acquired_monotonic_s')
                if (obs.get('valid') and obs.get('generation') == self.generation
                        and obs.get('capture_epoch') == self.capture_epoch
                        and isinstance(stamp, (int, float))
                        and seq is not None and seq > self.last_sequence
                        and floor < stamp <= self.clock() <= deadline):
                    self.last_sequence, self.last_acquired = seq, stamp
                    floor = stamp
                    materials = state.get('materials') or []
                    hint = state.get('material_view_hint') or {}
                    has_material = bool(materials)
                    has_hint = bool(hint.get('color_id')) and hint.get('source_sequence') == seq
                    hit = has_material or has_hint
                    samples.append(dict(
                        sequence=seq, acquired=stamp, hit=hit,
                        source='material' if has_material else 'view_hint' if has_hint else 'none',
                        status=state.get('material_localization_status'),
                    ))
                    if hit:
                        outcome = 'present'
                        break
                    if len(samples) == 2:
                        outcome = 'absent'
                        break
                self.sleep(min(.02, max(0., deadline-self.clock())))
        event = {'present': 'target_color_present', 'absent': 'empty_claw_detected',
                 'unconfirmed': 'grasp_check_unconfirmed'}[outcome]
        self.record(event, color_id=self.color, stage=stage, mode=mode, samples=samples,
                    min_area_px=min_area, radius_px=radius, z_cdeg=z_cdeg,
                    elapsed_s=round(self.clock() - started, 3))
        return outcome == 'present'

    def lower_with_mid_drop_check(self, aligned, *, timeout=40):
        """下降中用局部色块复核目标，错过检查深度时在 WAIT_GRIP 补查。"""
        aligned_id = aligned['aligned_command_id']
        if aligned_id != self.command:
            raise RuntimeError('cannot PICK without latest verified alignment')
        self.command += 1
        self.record('pick_request', color_id=self.color, reference_id=aligned_id,
                    command=self.command, expected_drop_mm=self.pick_drop_mm)
        self.client.arm_pick(*self.context, self.command, aligned_id)
        start = self.clock()
        deadline = start + timeout
        check_drop_cdeg = self.mid_drop_check_mm * PICK_MOTOR_CDEG_PER_MM
        checked = False
        grasped = True
        selected_pick_drop_cdeg = None
        while self.clock() < deadline:
            a = self.checked_arm()
            self.record('feedback', color_id=self.color, arm=a)
            state = a['state_name']
            if state not in ('LOWERING', 'WAIT_GRIP'):
                raise RuntimeError('unexpected state ' + state)
            if selected_pick_drop_cdeg is None:
                selected_pick_drop_cdeg = max(0, aligned['z_cdeg'] - a['target_z_cdeg'])
                self.record('pick_stroke_confirmed', color_id=self.color, command=self.command,
                            drop_cdeg=selected_pick_drop_cdeg,
                            drop_mm=selected_pick_drop_cdeg / PICK_MOTOR_CDEG_PER_MM,
                            target_z_cdeg=a['target_z_cdeg'])
            if a['aligned_command_id'] != aligned_id:
                raise RuntimeError('Z-only motion changed alignment reference')
            descended = aligned['z_cdeg'] - a['z_cdeg']
            if not checked and state == 'LOWERING' and descended >= check_drop_cdeg:
                checked = True
                grasped = self.check_object_present_fast(a['z_cdeg'], stage='mid_drop_6cm')
                if not grasped:
                    return dict(a, pick_drop_cdeg=selected_pick_drop_cdeg,
                                pick_drop_mm=selected_pick_drop_cdeg / PICK_MOTOR_CDEG_PER_MM), False
            if state == 'WAIT_GRIP':
                if not checked:
                    checked = True
                    grasped = self.check_object_present_fast(a['z_cdeg'], stage='wait_grip_fallback')
                self.last_motion = dict(elapsed_s=self.clock() - start, full_avoidance=False)
                self.record('arrived', state='WAIT_GRIP', **self.last_motion, arm=a)
                return dict(a, pick_drop_cdeg=selected_pick_drop_cdeg,
                            pick_drop_mm=selected_pick_drop_cdeg / PICK_MOTOR_CDEG_PER_MM), grasped
            self.sleep(.02)
        raise TimeoutError('motion timeout waiting WAIT_GRIP')

    def abort_empty_grasp(self, aligned, lower):
        """空爪时发 GRIP_ABORT(opcode=9)：松爪/跳过闭爪、抬回拍照高度、回原点，不入盘不加计数。"""
        count_before = len(self.results)
        aligned_id = aligned['aligned_command_id']
        self.command += 1
        self.client.arm_grip_abort(*self.context, self.command, aligned_id)
        self.record('grip_abort_requested', color_id=self.color, command=self.command,
                    reference_id=aligned_id, expected_tray_count=count_before,
                    aborted_from_state=lower.get('state_name'))
        lifted = self.wait_arm('WAIT_VISION', ('LOWERING', 'LIFTING'), timeout=40)
        self.require_tray_count(lifted, count_before)
        photo = self.align_to([0, 0, 0], before=lifted, photo_return=True)
        self.require_tray_count(photo, count_before)
        self.record('empty_claw_aborted', color_id=self.color, arm=photo, tray_count=count_before)
        return photo

    def align_pick_with_check(self, target, *, before, timeout=40):
        """发 CMD_ALIGN_PICK(opcode=10, auto_load=False)：三轴联动对准并下压，到 6 cm 复核颜色，落底 WAIT_GRIP 后决定入盘或中止。"""
        self.command += 1
        self.record('align_pick_request', color_id=self.color, command=self.command,
                    target_cdeg=target, auto_load=False)
        try:
            self.client.arm_align_pick(*self.context, self.command, *(v / 100 for v in target), auto_load=False)
        except Rejected:
            self.command -= 1
            raise
        start = self.clock()
        deadline = start + timeout
        check_drop_cdeg = self.mid_drop_check_mm * PICK_MOTOR_CDEG_PER_MM
        checked = False
        grasped = True
        while self.clock() < deadline:
            a = self.checked_arm()
            self.record('feedback', color_id=self.color, arm=a)
            state = a['state_name']
            if state not in ('ALIGNING', 'WAIT_GRIP'):
                raise RuntimeError('unexpected state ' + state)
            descended = before['z_cdeg'] - a['z_cdeg']
            if not checked and state == 'ALIGNING' and descended >= check_drop_cdeg:
                checked = True
                # 电控 CMD_GRIP_ABORT 仅在 LOWERING/WAIT_GRIP 受理，这里先完成快速拍帧判定，等落底 WAIT_GRIP 立刻发中止。
                grasped = self.check_object_present_fast(a['z_cdeg'], stage='mid_drop_6cm')
            if state == 'WAIT_GRIP':
                if not checked:
                    checked = True
                    grasped = self.check_object_present_fast(a['z_cdeg'], stage='wait_grip_fallback')
                drop_cdeg = max(0, before['z_cdeg'] - a['target_z_cdeg'])
                self.last_motion = dict(elapsed_s=self.clock() - start, full_avoidance=False)
                self.record('arrived', state='WAIT_GRIP', **self.last_motion, arm=a)
                return dict(a, pick_drop_cdeg=drop_cdeg,
                            pick_drop_mm=drop_cdeg / PICK_MOTOR_CDEG_PER_MM), grasped
            self.sleep(.02)
        raise TimeoutError('motion timeout waiting WAIT_GRIP')

    def align_pick_and_load(self, target, *, before, timeout=90):
        """发 CMD_ALIGN_PICK(opcode=10, auto_load=True)：三轴联动对准下压、闭爪、抬升回缩、入盘一条指令到底。"""
        count_before = len(self.results)
        self.require_tray_count(before, count_before)
        pick_started = self.clock()
        self.command += 1
        self.record('align_pick_request', color_id=self.color, command=self.command,
                    target_cdeg=target, auto_load=True)
        try:
            self.client.arm_align_pick(*self.context, self.command, *(v / 100 for v in target), auto_load=True)
        except Rejected:
            self.command -= 1
            raise
        deadline = pick_started + timeout
        loading_seen = False
        lift_clear_s = None
        count_seen = count_before
        lowest_z = target[0]
        while self.clock() < deadline:
            arm = self.checked_arm()
            state = arm['state_name']
            self.record('tray_feedback', color_id=self.color, arm=arm)
            if state in ('ALIGNING', 'WAIT_GRIP'):
                lowest_z = min(lowest_z, arm['z_cdeg'])
                self.require_tray_count(arm, count_before)
            elif state == 'LIFTING':
                if (lift_clear_s is None
                        and arm['z_cdeg'] - lowest_z >= GRASP_LIFT_CLEAR_MM * PICK_MOTOR_CDEG_PER_MM):
                    lift_clear_s = self.clock()
                    self.record('grasp_lift_feedback', color_id=self.color,
                                pick_to_lift_s=round(lift_clear_s - pick_started, 3),
                                first_target_to_lift_s=round(lift_clear_s - self.first_target_seen, 3)
                                if self.first_target_seen is not None else None,
                                lift_mm=round((arm['z_cdeg'] - lowest_z) / PICK_MOTOR_CDEG_PER_MM, 2))
                if not loading_seen:
                    self.require_tray_count(arm, count_before)
            elif state == 'TRAY_LOADING':
                loading_seen = True
                count = arm['tray_count']
                if not count_seen <= count <= count_before + 1:
                    raise RuntimeError('tray count regressed or incremented more than once')
                count_seen = count
            elif state == 'PICKED':
                self.require_tray_count(arm, count_before + 1)
                drop_cdeg = max(0, before['z_cdeg'] - target[0])
                lower = dict(z_cdeg=lowest_z, pick_drop_cdeg=drop_cdeg,
                             pick_drop_mm=drop_cdeg / PICK_MOTOR_CDEG_PER_MM)
                return lower, arm
            else:
                raise RuntimeError('unexpected state during align_pick tray transfer: ' + state)
            self.sleep(.02)
        raise TimeoutError('STM32 did not confirm PICKED after ALIGN_PICK')

    def finish_tray_load(self, aligned, lower, pick_started):
        count_before = len(self.results)
        self.require_tray_count(lower, count_before)
        aligned_id = aligned['aligned_command_id']
        self.command += 1
        self.client.arm_grip_done(*self.context, self.command, aligned_id)
        self.record('lift_and_load_requested', color_id=self.color,
                    reference_id=aligned_id, expected_tray_count=count_before + 1)
        deadline = self.clock() + 90
        loading_seen = False
        lift_clear_s = None
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
            elif state == 'TRAY_LOADING':
                loading_seen = True
                count = arm['tray_count']
                if not count_seen <= count <= count_before + 1:
                    raise RuntimeError('tray count regressed or incremented more than once')
                count_seen = count
            elif state == 'PICKED':
                self.require_tray_count(arm, count_before + 1)
                return lower, arm
            else:
                raise RuntimeError('unexpected state during tray transfer: '+state)
            self.sleep(.02)
        raise TimeoutError('STM32 did not confirm PICKED after v7 tray transfer')

    def load_tray(self, aligned):
        count_before = len(self.results)
        self.require_tray_count(aligned, count_before)
        pick_started = self.clock()
        lower = self.lower_for_pick(aligned, drop_mm=self.pick_drop_mm)
        return self.finish_tray_load(aligned, lower, pick_started)

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
        preselected = getattr(self, '_preselected_color', None) == color
        self._preselected_color = None
        while True:
            self.checked_arm()
            try:
                if preselected:
                    self.color_floor = getattr(self, '_preselect_floor', self.clock())
                    preselected = False
                else:
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

    def observe(self, floor, timeout=180, *, required_frames=None, required_span_s=None):
        floor = max(floor, self.color_floor, self.last_acquired)
        deadline, last_status = self.clock()+timeout, float('-inf')
        reason = 'no fresh source frame'
        view_points = deque()
        unconfirmed_since = self.clock()
        failed_frames = 0
        count = self.tuning.confirm_frames if required_frames is None else required_frames
        span_s = self.tuning.confirm_span_s if required_span_s is None else required_span_s
        status_period = 2.0 if self.real_tray else .5
        poll_sleep = .02 if self.real_tray else .05
        samples = deque()
        while self.clock() < deadline:
            if self.clock()-last_status > status_period:
                a = self.checked_arm()
                if a['state_name'] not in ('WAIT_VISION', 'ALIGNED') or (not self.real_tray and abs(a['z_cdeg'])/900 > .15):
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
                self.sleep(.05 if self.real_tray else .1)
                continue
            except (ObservationUnavailable,URLError,OSError,json.JSONDecodeError):
                reason = 'fresh camera or material observation unavailable'
                failed_frames = 0
                view_points.clear()
                samples.clear()
                self.sleep(.05 if self.real_tray else .1)
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
                self.sleep(.05 if self.real_tray else .1)
                continue
            if seq <= self.last_sequence or stamp <= max(floor, self.last_acquired):
                self.sleep(poll_sleep)
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
                if (self.real_tray and len(self.view_targets) < 4 and not materials
                        and reason in RIM_HINT_STATUSES
                        and hint.get('color_id') == self.color and hint.get('source_sequence') == seq
                        and len(xy) == 2 and np.isfinite(xy).all()):
                    if view_points and stamp-view_points[-1][0] > .5:
                        view_points.clear()
                    view_points.append((stamp, xy, seq))
                    if (count == 1 and len(view_points) >= 1) or confirmation_window_ready(view_points, count, span_s):
                        points = [item[1] for item in view_points]
                        max_ptp = TRAY_GRASP_JITTER_PX if self.direct_pick_after_align else 5
                        if np.ptp(points, axis=0).max() <= max_ptp:
                            pixel = np.median(points, axis=0)
                            if self.direct_pick_after_align:
                                # 大夹爪直接用唯一色块质心兜底一步到位，省去 1~2 秒额外换视角重拍。
                                if self.first_target_seen is None:
                                    self.first_target_seen = view_points[0][0]
                                frames = [dict(sequence=item[2], acquired=item[0]) for item in view_points]
                                dynamics = observation_dynamics(points, frames)
                                speed = float(np.linalg.norm(dynamics['velocity_px_s']))
                                if speed <= TRAY_MAX_TRACKING_SPEED_PX_S:
                                    radius = min(self.tuning.tolerance_px, 45.)
                                    errors = np.asarray(points) - self.reference['reference_pixel']
                                    in_grasp = bool(np.all(np.linalg.norm(errors, axis=1) <= radius))
                                    self.last_usable_state = state
                                    return dict(pixel=pixel.tolist(), track_id=None,
                                                points=points, source_frames=frames, grasp_radius_px=radius,
                                                in_grasp_window=in_grasp, centroid_fallback=True, **dynamics)
                            else:
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
                max_gap = .75 if self.real_tray else .7
                if samples and stamp - samples[-1][0] > max_gap:
                    samples.clear()
                track = materials[0].get('track_id')
                reason = 'waiting for consistent fresh top-face positions'
                point = [materials[0]['center_x'], materials[0]['center_y']]
                if not np.isfinite(point).all():
                    raise RuntimeError('nonfinite top face')
                radius = tray_grasp_radius(materials[0].get('top_face_axes'), self.tuning.tolerance_px)
                if samples and track != samples[-1][3]:
                    _, previous, _, previous_track, _ = samples[-1]
                    nearby_same_color = (
                        self.real_tray and track is not None and previous_track is not None
                        and np.linalg.norm(np.subtract(point, previous)) <= TRAY_GRASP_JITTER_PX
                    )
                    if not nearby_same_color:
                        samples.clear()
                samples.append((stamp, point, seq, track, radius))
                if (count == 1 and len(samples) >= 1) or confirmation_window_ready(samples, count, span_s):
                    points = [sample[1] for sample in samples]
                    radius = min(s[4] for s in samples)
                    errors = np.asarray(points) - self.reference['reference_pixel']
                    in_grasp = bool(np.all(np.linalg.norm(errors, axis=1) <= radius))
                    track_changed = any(sample[3] != track for sample in samples)
                    jitter = (TRAY_GRASP_JITTER_PX
                              if self.real_tray and track is not None and (in_grasp or track_changed)
                              else 3)
                    frames = [dict(sequence=s[2], acquired=s[0]) for s in samples]
                    dynamics = observation_dynamics(points, frames)
                    if track_changed:
                        dynamics['stationary_for_mapping'] = False
                    speed = float(np.linalg.norm(dynamics['velocity_px_s']))
                    spread = np.ptp(points, axis=0).max()
                    # A slowly rotating, uniquely tracked piece need not stand
                    # still for an approach. Descent still needs measured frames
                    # inside the grasp window, with the tighter jitter check.
                    tracking = (self.real_tray and not in_grasp and track is not None
                                and spread <= TRAY_GRASP_JITTER_PX and dynamics['velocity_reliable'])
                    if spread <= jitter or tracking:
                        if self.real_tray and self.first_target_seen is None:
                            self.first_target_seen = samples[0][0]
                        # The actuator needs seconds to reach the piece. A
                        # steady traversal of the grasp window cannot authorize it.
                        still_limit = (TRAY_FAST_STILL_SPEED_PX_S
                                       if self.direct_pick_after_align else TRAY_STILL_SPEED_PX_S)
                        if self.real_tray and in_grasp and speed > still_limit:
                            reason = 'target moving inside grasp window'
                            self.sleep(poll_sleep)
                            continue
                        if self.real_tray and speed > TRAY_MAX_TRACKING_SPEED_PX_S:
                            reason = f'target_moving:{speed:.1f}px/s'
                            self.sleep(poll_sleep)
                            continue
                        self.last_usable_state = state
                        return dict(pixel=np.median(points, axis=0).tolist(), track_id=track,
                                    points=points, source_frames=frames, grasp_radius_px=radius,
                                    in_grasp_window=in_grasp, **dynamics)
                if self.clock() - unconfirmed_since >= max(1., span_s + .3):
                    self.sample_failed_observation(state, reason)
            self.sleep(poll_sleep)
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
        follow_controller = self.real_tray and self.tuning.follow_controller_limits
        observation_timeout = 2 if self.real_tray else 8
        while True:
            try:
                observation = (self.wait_reachable_target(timeout=observation_timeout) if need_acquire else
                    self.observe(self.clock()+self.tuning.settle_s, timeout=observation_timeout,
                                 **({'required_frames': 1, 'required_span_s': 0.} if follow_controller else {})))
            except ViewpointNeeded as hint:
                a = self.checked_arm()
                self.envelope(a)
                if a['state_name'] not in ('WAIT_VISION', 'ALIGNED'):
                    raise RuntimeError('arm left observation state')
                plan = None
                if not self.micro_only:
                    plan = edge_view_target(a, hint.pixel, self.reference, self.mapping,
                                            self.tuning, self.view_targets)
                computed_view = plan is not None
                if plan is None:
                    plan = measured_view_target(a, hint.pixel, self.reference, self.mapping,
                                                self.tuning, visited=self.view_targets)
                within_budget = follow_controller or (
                    adjustments < self.tuning.max_adjustments
                    and total + plan[1] <= self.tuning.max_total_correction_mm
                    and (active_since is None or self.clock() - active_since < self.tuning.alignment_budget_s)
                ) if plan else False
                if plan and within_budget:
                    if active_since is None:
                        active_since = self.clock()
                    target, distance = plan
                    self.view_targets.add(tuple(target))
                    self.last_unreachable_view_hint = None
                    total += distance
                    adjustments += 1
                    self.record('rim_view_change', color_id=self.color, target_cdeg=target,
                                selection_pixel=hint.pixel, combined_move=computed_view,
                                view_source='computed' if computed_view else 'calibration_sample')
                    try:
                        self.align_to(target, before=a)
                    except Rejected as exc:
                        self.record('controller_rejected_align', color_id=self.color,
                                    target_cdeg=target, error=str(exc), motion_sent=False)
                else:
                    self.last_unreachable_view_hint = hint.pixel
                    self.record('waiting_for_complete_rim', color_id=self.color, motion_sent=False,
                                selection_pixel=hint.pixel, reason='no_available_viewpoint')
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
                if self.real_tray and self.align_pick:
                    self._pending_align_pick = ([-self.align_pick_drop_cdeg, a['yaw_cdeg'], a['reach_cdeg']], a)
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
                target = [0, 0, 0] if self.real_tray else [0, a['yaw_cdeg'], min(0, a['reach_cdeg'])]
                distance = 0.
            else:
                exhausted = False if follow_controller else (
                    adjustments>=self.tuning.max_adjustments or
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
                if not follow_controller and total+distance>self.tuning.max_total_correction_mm:
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
            if (self.real_tray and self.align_pick and self.direct_pick_after_align
                    and (accepted or not self.micro_only)):
                # 三轴联动模式：不先单独发 ALIGN，直接把 [z_pick, yaw, reach] 交给 CMD_ALIGN_PICK(opcode=10) 一步到位。
                self._pending_align_pick = ([-self.align_pick_drop_cdeg, target[1], target[2]], a)
                if self.save_frame:
                    self.save_frame(self.output/f'{self.color}_aligned.jpg')
                return a, observation, (error.tolist() if accepted else [0., 0.])
            anchor = (observation,a,error.copy())
            total += distance
            adjustments += 1
            try:
                arrived = self.align_to(target, before=a)
            except Rejected as exc:
                self.record('controller_rejected_align', color_id=self.color,
                            target_cdeg=target, error=str(exc), motion_sent=False)
                if a['reach_cdeg'] < TRAY_PARK[2] and target[1] != a['yaw_cdeg']:
                    # 电控在 reach < -60mm 时拒绝带 yaw 的 ALIGN；先单动 reach 回到安全区。
                    safe_reach = [0, a['yaw_cdeg'], max(TRAY_PARK[2], min(0, target[2]))]
                    try:
                        self.align_to(safe_reach, before=a)
                    except Rejected as recovery_exc:
                        self.record('controller_rejected_align', color_id=self.color,
                                    target_cdeg=safe_reach, error=str(recovery_exc), motion_sent=False)
                anchor, stagnant, need_acquire = None, 0, True
                continue
            self.last_unreachable_view_hint = None
            if (self.real_tray and self.direct_pick_after_align
                    and (accepted or not self.micro_only)):
                # 大夹爪一步到位后不再二次拍照确认，直接返回触发 PICK。
                if self.save_frame:
                    self.save_frame(self.output/f'{self.color}_aligned.jpg')
                return arrived, observation, (error.tolist() if accepted else [0., 0.])

    def cycle(self, next_color=None):
        if self.real_tray:
            empty_claw_retries = 0
            while True:
                self._pending_align_pick = None
                aligned, obs, error = self.align_color()
                now = self.clock()
                result = dict(color_id=self.color, color_name=COLORS[self.color],
                              aligned_pixel=obs['pixel'], error_px=error,
                              physical_grasp_completed=False, physical_place_completed=False,
                              color_to_first_usable_s=round(self.first_usable-self.color_started, 3),
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
                if self._pending_align_pick is not None:
                    pick_target, before_arm = self._pending_align_pick
                    self._pending_align_pick = None
                    count_before = len(self.results)
                    self.require_tray_count(before_arm, count_before)
                    pick_started = self.clock()
                    try:
                        if self.empty_claw_check:
                            lower, grasped = self.align_pick_with_check(pick_target, before=before_arm)
                        else:
                            lower, parked = self.align_pick_and_load(pick_target, before=before_arm)
                            grasped = True
                    except Rejected as exc:
                        self.record('controller_rejected_align_pick', color_id=self.color,
                                    target_cdeg=pick_target, error=str(exc), motion_sent=False)
                        self.view_targets = set()
                        self.last_unreachable_view_hint = None
                        continue
                    if self.empty_claw_check:
                        if not grasped:
                            empty_claw_retries += 1
                            self.abort_empty_grasp(lower, lower)
                            if empty_claw_retries > self.max_empty_claw_retries:
                                raise RuntimeError(f'target color unconfirmed after {self.max_empty_claw_retries} retries')
                            self.view_targets = set()
                            self.last_unreachable_view_hint = None
                            continue
                        lower, parked = self.finish_tray_load(lower, lower, pick_started)
                elif self.empty_claw_check:
                    count_before = len(self.results)
                    self.require_tray_count(aligned, count_before)
                    pick_started = self.clock()
                    lower, grasped = self.lower_with_mid_drop_check(aligned)
                    if not grasped:
                        empty_claw_retries += 1
                        self.abort_empty_grasp(aligned, lower)
                        if empty_claw_retries > self.max_empty_claw_retries:
                            raise RuntimeError(f'target color unconfirmed after {self.max_empty_claw_retries} retries')
                        self.view_targets = set()
                        self.last_unreachable_view_hint = None
                        continue
                    lower, parked = self.finish_tray_load(aligned, lower, pick_started)
                else:
                    lower, parked = self.load_tray(aligned)
                returned_to_origin = len(self.results) < 2
                if returned_to_origin:
                    photo = self.align_to([0, 0, 0], before=parked, photo_return=True,
                                          preselect_color=next_color)
                    self.require_tray_count(photo, parked['tray_count'])
                result.update(controller_tray_load_completed=True, tray_count=parked['tray_count'],
                              firmware_pick_drop_mm=lower['pick_drop_mm'],
                              simulated_grip=False, simulated_tray=False,
                              returned_to_origin=returned_to_origin,
                              empty_claw_retries=empty_claw_retries)
                break
        else:
            aligned, obs, error = self.align_color()
            result = dict(color_id=self.color, color_name=COLORS[self.color],
                          aligned_pixel=obs['pixel'], error_px=error,
                          physical_grasp_completed=False, physical_place_completed=False)
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
                or a['fault']
                or (not self.real_tray and any(abs(a[k]) > 50 for k in ('z_cdeg','yaw_cdeg','reach_cdeg')))
                or chassis['state_name'] != 'EMPTY' or chassis['fault']):
            raise RuntimeError('fresh manual origin / standalone READY / EMPTY chassis required')
        self.context = (test_id, 0, 102)
        self.started = True
        self.record('sequence_started', order=order, order_source='user_confirmed_test_order',
                    simulated_servos=not self.real_tray, physical_grasp_completed=False,
                    expected_pick_drop_mm=self.pick_drop_mm, tuning=asdict(self.tuning))
        self.client.arm_test_begin(test_id, 102)
        self.wait_arm('WAIT_VISION', ('PREPARING',), target=[0, 0, 0])
        for index, color in enumerate(order):
            self.set_color(color)
            next_color = order[index + 1] if index + 1 < len(order) else None
            self.cycle(next_color=next_color)
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


def run_trial(trial, order):
    try:
        trial.wait_for_manual_ready()
        return trial.run(order, int(time.time()))
    except BaseException as exc:
        trial.record('failed', error=str(exc), completed_colors=trial.results, stop_sent=False)
        if trial.started and (not trial.real_tray or isinstance(exc, KeyboardInterrupt)):
            try:
                ack = trial.client.stop(0)
                trial.record('stop_acknowledged', controller=ack)
                trial.record('after_stop', arm=trial.client.arm_status())
            except Exception as stop_error:
                trial.record('stop_unconfirmed', error=str(stop_error))
        raise


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
    parser.add_argument('--confirm-frames',type=int,default=None)
    parser.add_argument('--confirm-span-s',type=float,default=None)
    parser.add_argument('--settle-s',type=float)
    parser.add_argument('--micro-only', action='store_true',
                        help='bound visual correction commands to micro steps, including far targets')
    parser.add_argument('--wide-tray-yaw', action='store_true',
                        help='real tray: allow -12..+22deg after checking physical clearance')
    parser.add_argument('--no-direct-pick', action='store_true',
                        help='real tray: re-observe after one-shot ALIGN instead of picking immediately')
    parser.add_argument('--no-align-pick', action='store_true',
                        help='real tray: disable simultaneous 3-axis CMD_ALIGN_PICK (opcode=10) and use legacy ALIGN+PICK')
    parser.add_argument('--align-pick-drop-cdeg', type=int, default=None,
                        help=f'real tray: Z drop in motor cdeg for CMD_ALIGN_PICK (default {DEFAULT_ALIGN_PICK_DROP_CDEG}, matching ARM_PICK_DROP_CDEG)')
    parser.add_argument('--empty-claw-check', action='store_true',
                        help='real tray: check a local HSV patch during descent, retry on GRIP_ABORT')
    parser.add_argument('--max-empty-claw-retries', type=int, default=3,
                        help='max empty-claw retries per color when --empty-claw-check is enabled')
    parser.add_argument('--mid-drop-check-mm', type=float, default=60.,
                        help='descent depth in mm (~6 cm) at which to snap 1 frame for mid-drop empty-claw check')
    parser.add_argument('--grasp-check-radius-px', type=float, default=450.,
                        help='radius in source pixels for any-color HSV presence during descent (default 450)')
    parser.add_argument('--grasp-close-wait-s', type=float, default=.55,
                        help='seconds to wait after WAIT_GRIP before sampling frames for empty-claw check')
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
    if args.wide_tray_yaw and not args.real_tray:
        parser.error('--wide-tray-yaw requires --real-tray')
    if args.empty_claw_check and not args.real_tray:
        parser.error('--empty-claw-check requires --real-tray')
    if args.tolerance_px is None:
        args.tolerance_px = 60. if args.real_tray else 8.
    if args.confirm_frames is None:
        args.confirm_frames = 2 if args.real_tray else 3
    if args.confirm_span_s is None:
        args.confirm_span_s = 0. if args.real_tray else .2
    if args.settle_s is None:
        args.settle_s = 0. if args.real_tray else .1
    tuning = TrialTuning(follow_controller_limits=args.real_tray, **{name:getattr(args,name) for name in (
        'tolerance_px','confirm_frames','confirm_span_s','settle_s',
        'max_step_mm','max_yaw_step_deg','max_retraction_mm','alignment_budget_s','wide_tray_yaw')})
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
    def patch_snapshot(x, y, radius, timeout):
        with urlopen(f'http://127.0.0.1:8090/grasp-patch/{x}/{y}/{radius}/any', timeout=timeout) as reply:
            return json.load(reply)
    def frame_any_color_snapshot(x, y, radius, timeout):
        import cv2
        with urlopen('http://127.0.0.1:8090/frame.jpg', timeout=timeout) as reply:
            frame_id = int(reply.headers.get('X-Frame-Id', '0'))
            age_ms = float(reply.headers.get('X-Capture-Age-Ms', '0'))
            data = reply.read()
        img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is None or img.size == 0:
            return None
        src_w, src_h = reference['resolution']
        h, w = img.shape[:2]
        sx, sy = w / src_w, h / src_h
        cx = max(0, min(w - 1, round(x * sx)))
        cy = max(0, min(h - 1, round(y * sy)))
        r = max(1, min(max(h, w), round(radius * min(sx, sy))))
        left, top = max(0, cx - r), max(0, cy - r)
        crop = img[top:cy + r + 1, left:cx + r + 1]
        hsv = cv2.cvtColor(cv2.GaussianBlur(crop, (5, 5), 0), cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 50, 35), (179, 255, 255))
        circle = np.zeros_like(mask)
        cv2.circle(circle, (cx - left, cy - top), r, 255, -1)
        mask &= circle
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        _, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        area_px = round(int(stats[1:, cv2.CC_STAT_AREA].max(initial=0)) / (sx * sy))
        return dict(source='frame_jpg', frame_id=frame_id,
                    acquired=time.monotonic() - age_ms / 1000.0,
                    any_color=True, area_px=area_px)
    active_patch_snapshot = None
    if args.empty_claw_check:
        state_caps = snapshot()
        active_patch_snapshot = (patch_snapshot if state_caps.get('grasp_any_color_patch_supported')
                                 else frame_any_color_snapshot)
    with (Path(tempfile.gettempdir())/f'robot-preview-start-{os.getuid()}.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        subprocess.run(['systemctl','--user','stop','chassis-button.service'], check=True)
        with serial.Serial(args.port, 115200, timeout=.05, write_timeout=1, exclusive=True) as port:
            trial = ThreeColorTrial(ElectricalClient(port), reference, mapping, snapshot, select_color,
                args.output, simulate_servos=args.simulate_servos, real_tray=args.real_tray,
                pick_drop_mm=args.pick_drop_mm, tuning=tuning, micro_only=args.micro_only,
                empty_claw_check=args.empty_claw_check,
                patch_snapshot=active_patch_snapshot,
                max_empty_claw_retries=args.max_empty_claw_retries,
                mid_drop_check_mm=args.mid_drop_check_mm,
                grasp_check_radius_px=args.grasp_check_radius_px,
                grasp_close_wait_s=args.grasp_close_wait_s,
                direct_pick_after_align=args.real_tray and not args.no_direct_pick,
                align_pick=(args.real_tray and not args.no_align_pick
                            and not args.no_direct_pick and not args.micro_only),
                align_pick_drop_cdeg=args.align_pick_drop_cdeg,
                save_failure=save_failure if args.real_tray else None,
                save_frame=save_frame if args.save_aligned_frame or args.simulate_servos else None)
            run_trial(trial, order)


if __name__ == '__main__':
    main()
