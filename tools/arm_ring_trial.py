"""独立数字圆环测试。视觉验收与 KEY 启动的云台联调使用同一检测器。"""
import argparse
from collections import deque
from contextlib import ExitStack
from dataclasses import asdict
import fcntl
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time
from types import SimpleNamespace

import cv2
import numpy as np

from pi_vision.camera import Camera
from pi_vision.config import load_config
from pi_vision.electrical_link import ElectricalClient
from pi_vision.placement_rings import PlacementRings
from pi_vision.stream import FrameStreamer
from pi_vision.task_confirmation import parse_competition_task
from tools.arm_three_color_trial import ArmTrial, TrialTuning


ORIGIN = [0, 0, 0]
REACH_MM_PER_CDEG = 113.1 / 36000
FIXED_DROP_MM = 100


def placement_order(positions, task_code, station):
    if positions is not None:
        if station is not None or len(positions) != 3 or set(positions) != set('123'):
            raise ValueError('--positions 须为 1/2/3 的排列，不同时指定 --station')
        return [int(n) for n in positions]
    task = parse_competition_task(task_code)
    if station not in ('batch1', 'batch2-processing'):
        raise ValueError('完整任务码须指定 --station batch1 或 batch2-processing')
    return [int(n) for n in (task.group2 if station == 'batch1' else task.group4)]


def project(matrix, pixel):
    point = matrix @ np.r_[pixel, 1.]
    if not np.isfinite(point).all() or abs(point[2]) < 1e-8:
        raise ValueError('地面映射在此点无效')
    return point[:2] / point[2]


class FloorCalibration:
    """Photo-height pixels to ground offsets from the claw's vertical projection."""

    def __init__(self, config, camera, *, lower, confirmed_downstroke_mm=None):
        if (config.get('camera_source') != camera.source
                or config.get('resolution') != [camera.width, camera.height]
                or config.get('camera_flip') != camera.flip
                or config.get('camera_undistort') is not camera.undistort
                or config.get('photo_z_cdeg') != 0):
            raise ValueError('放置区标定与相机、分辨率或拍照高度不匹配')
        try:
            self.radius = float(config['origin_radius_mm'])
            self.max_yaw = float(config['max_abs_yaw_deg'])
            self.max_reach = float(config['max_retraction_mm'])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError('请填写实测的原点半径、最大旋转角和最大回缩距离') from exc
        if not (0 < self.radius <= 500 and 0 < self.max_yaw <= 90
                and 0 < self.max_reach <= min(120, self.radius-10)):
            raise ValueError('原点半径或实测工作范围无效')
        self.limit_mm = float(config.get('validation_limit_mm', 1))
        if not 1 <= self.limit_mm <= 3:
            raise ValueError('地面验算门槛须在 1 至 3 mm')
        confirmed_for_trial = confirmed_downstroke_mm == FIXED_DROP_MM
        if lower and self.limit_mm > 1 and not confirmed_for_trial:
            raise ValueError('放宽验算的标定仅供水平对照试验')
        if lower and not confirmed_for_trial and config.get('clear_downstroke_mm') != FIXED_DROP_MM:
            raise ValueError('下降需要确认本场地三个位置的 100 mm 下降行程')
        source, ground = self.points(config['fit_points'], camera, 4)
        check_source, check_ground = self.points(config['check_points'], camera, 1)
        if np.min(np.linalg.norm(check_source[:, None]-source, axis=2)) < 10:
            raise ValueError('验收点必须独立于拟合点，像素距离至少 10')
        self.hull = cv2.convexHull(source.astype(np.float32))
        if cv2.contourArea(self.hull) < 1000:
            raise ValueError('拟合点需要覆盖盘面，不能共线或聚在一处')
        if config.get('calibration_source') == 'encoder_and_ring_motion':
            self.matrix = np.asarray(config.get('projective_matrix'), dtype=float)
        else:
            self.matrix, _ = cv2.findHomography(source, ground, method=0)
        if self.matrix is None or self.matrix.shape != (3, 3) or not np.isfinite(self.matrix).all():
            raise ValueError('无法拟合有效的地面投影')
        # No projection pole may cross the calibrated patch.
        denominators = np.c_[source, np.ones(len(source))] @ self.matrix[2]
        if not (min(denominators) > 1e-8 or max(denominators) < -1e-8):
            raise ValueError('地面映射在标定范围内不连续')
        reference = project(np.linalg.inv(self.matrix), [0, 0])
        self.offset(reference)
        for pixels, offsets in ((source, ground), (check_source, check_ground)):
            mapped = np.array([self.offset(p) for p in pixels])
            if np.linalg.norm(mapped-offsets, axis=1).max() > self.limit_mm:
                raise ValueError(f'地面拟合或独立验收误差超过 {self.limit_mm:g} mm')

    @staticmethod
    def points(rows, camera, minimum):
        values = np.asarray(rows, dtype=float)
        if (values.ndim != 2 or values.shape[1] != 4 or len(values) < minimum
                or not np.isfinite(values).all()):
            raise ValueError('标定点格式为 [原图x, 原图y, 径向mm, 切向mm]')
        source = values[:, :2]
        if (np.any(source < 0) or np.any(source >= [camera.width, camera.height])
                or len(np.unique(source, axis=0)) != len(source)):
            raise ValueError('标定像素超出原图或重复')
        return source, values[:, 2:]

    def offset(self, pixel, *, coarse=False):
        distance = cv2.pointPolygonTest(self.hull, tuple(map(float, pixel)), True)
        mapped = project(self.matrix, pixel)
        if distance < -.01:
            if not coarse:
                raise ValueError('目标超出已标定地面区域')
            vertices = self.hull.reshape(-1, 2)
            ends = np.roll(vertices, -1, axis=0)
            edges = ends-vertices
            fraction = np.clip(np.sum((np.asarray(pixel)-vertices)*edges, axis=1) /
                               np.sum(edges*edges, axis=1), 0, 1)
            nearest = vertices+fraction[:, None]*edges
            boundary = nearest[np.argmin(np.linalg.norm(nearest-pixel, axis=1))]
            if np.linalg.norm(mapped-project(self.matrix, boundary)) > 2*self.limit_mm:
                raise ValueError('目标超出已标定地面区域及粗定位误差范围')
        return mapped

    def world_point(self, horizontal, offset):
        yaw, reach = horizontal
        angle = math.radians(yaw/300)
        radial, tangent = offset
        rotation = np.array([[math.cos(angle), -math.sin(angle)],
                             [math.sin(angle), math.cos(angle)]])
        return rotation @ [self.radius+reach*REACH_MM_PER_CDEG+radial, tangent]

    def target(self, point, *, local_correction=False):
        yaw = math.degrees(math.atan2(point[1], point[0]))
        reach = np.linalg.norm(point)-self.radius
        if local_correction:
            # A short correction near an end stop can still improve yaw.
            # Never command outside the stroke, and confirm the residual
            # after arrival before accepting alignment or lowering.
            if -self.max_reach-3.4 <= reach <= 3.4:
                reach = min(0., max(-self.max_reach, reach))
        if not (-self.max_yaw <= yaw <= self.max_yaw and -self.max_reach <= reach <= 0):
            raise ValueError(f'目标超出已验证行程，旋转 {yaw:.2f}°、伸缩 {reach:.2f} mm')
        return [0, round(yaw*300), round(reach/REACH_MM_PER_CDEG)]


class RingObserver:
    def __init__(self, source, size, *, clock=time.monotonic, sleep=time.sleep):
        self.source, self.size, self.clock, self.sleep = source, size, clock, sleep
        self.identity = None
        self.sequence = -1
        self.acquired_at = float('-inf')
        self.span_s = None
        self.jitter_px = {}

    def observe(self, wanted, *, all_rings=False, floor, health=lambda: None, timeout=25,
                calibration_anchors=None):
        samples = deque(maxlen=4)
        deadline = self.clock()+timeout
        next_health = self.clock()
        last_state, jitter = [], None
        while self.clock() < deadline:
            if self.clock() >= next_health:
                health()
                next_health = self.clock()+.4
            state = self.source()
            capture = state['capture']
            meta = state['ring_observation']
            if capture['valid']:
                identity = state['session_id'], capture['epoch']
                size = [state['preview']['source_width'], state['preview']['source_height']]
                if size != self.size or self.identity not in (None, identity):
                    raise RuntimeError('放置测试的相机分辨率或采集会话发生变化')
                self.identity = identity
            seq, stamp = meta.get('source_sequence'), meta.get('acquired_monotonic_s')
            if (not capture['valid'] or not meta['valid']
                    or meta['capture_epoch'] != capture['epoch']
                    or stamp is None or not 0 <= self.clock()-stamp < .8):
                samples.clear()
            elif stamp > floor and seq is not None and seq > self.sequence:
                self.sequence = seq
                rings = state['rings']
                last_state = [(r['position_id'], round(r['digit_score'], 3),
                               round(r['center_x'], 2), round(r['center_y'], 2)) for r in rings]
                ids = [r['position_id'] for r in rings if r['position_id'] is not None]
                unique = (len(ids) == len(set(ids)) and
                          (not all_rings or len(rings) == len(ids) == 3))
                centers = {r['position_id']: [r['center_x'], r['center_y']] for r in rings
                           if r['point_kind'] == 'ring_circle_center'}
                if calibration_anchors is not None:
                    centers = match_calibration_rings(rings, calibration_anchors)
                    unique = bool(centers)
                if (unique and wanted <= set(centers)
                        and all(np.isfinite(centers[n]).all() for n in wanted)):
                    if samples and stamp-samples[-1][0] > .8:
                        samples.clear()
                    samples.append((stamp, {n: centers[n] for n in wanted}))
                    while samples and stamp-samples[0][0] > 2.5:
                        samples.popleft()
                    if len(samples) >= 4 and stamp-samples[0][0] >= .25:
                        points = {n: np.array([p[n] for _, p in samples]) for n in wanted}
                        jitter = {n: float(np.ptp(path, axis=0).max()) for n, path in points.items()}
                        if max(jitter.values()) <= 2:
                            self.acquired_at = stamp
                            self.span_s = stamp-samples[0][0]
                            self.jitter_px = jitter
                            return {n: np.median(path, axis=0) for n, path in points.items()}
                else:
                    samples.clear()
            self.sleep(.04)
        raise TimeoutError(f'没有连续读到稳定环心 {sorted(wanted)}，'
                           f'末帧数字/分数/坐标 {last_state}，最近确认抖动 {jitter}')


def match_calibration_rings(rings, anchors):
    """Carry verified IDs across a bounded motion. Never infer IDs from layout."""
    rings = [r for r in rings if r['point_kind'] == 'ring_circle_center']
    if len(rings) < 3:
        return {}
    ids = sorted(anchors)
    previous = np.asarray([anchors[n] for n in ids])
    points = np.asarray([[r['center_x'], r['center_y']] for r in rings])
    if not np.isfinite(points).all():
        return {}
    spacing = np.linalg.norm(previous[:, None]-previous, axis=2)
    gate = min(140, .43*spacing[np.triu_indices(3, k=1)].min())
    distances = np.linalg.norm(previous[:, None]-points, axis=2)
    within = distances <= gate
    if not np.all(within.sum(axis=1) == 1):
        return {}
    matches = within.argmax(axis=1)
    for i, j in enumerate(matches):
        if rings[j]['position_id'] not in (None, ids[i]):
            return {}
    matched = set(matches)
    if any(r['position_id'] in anchors for j, r in enumerate(rings) if j not in matched):
        return {}
    return {n: points[j] for n, j in zip(ids, matches)}


def tick_after(value, previous):
    return 0 < (value-previous) % (1 << 32) < (1 << 31)


class RingTrial(ArmTrial):
    def __init__(self, client, output, calibration, observer, *, lower, simulate_grip=False,
                 reuse_ready_origin=False, **timing):
        super().__init__(client, output, TrialTuning(tolerance_px=2), **timing)
        self.calibration, self.observer, self.lower = calibration, observer, lower
        self.simulate_grip = simulate_grip
        self.reuse_ready_origin = reuse_ready_origin

    def envelope(self, a):
        if (not -100.4 <= a['z_cdeg']/900 <= .2
                or not -max(self.calibration.max_reach, 60)-.6
                       <= a['reach_cdeg']*REACH_MM_PER_CDEG <= .4
                or abs(a['yaw_cdeg']/300) > self.calibration.max_yaw+.3):
            raise RuntimeError('云台反馈超出本次测试行程')

    def wait_key(self, timeout=600):
        self.record('waiting_for_controller')
        deadline = self.clock()+timeout
        baseline = None
        saw_zeroing = False
        while self.clock() < deadline:
            try:
                arm = self.client.arm_status()
            except TimeoutError:
                baseline, saw_zeroing = None, False
                self.sleep(.15)
                continue
            if not arm['standalone_test_mode']:
                raise RuntimeError('当前电控是整车固件，需切换到独立云台测试固件')
            if baseline is None or not tick_after(arm['tick'], baseline):
                baseline, saw_zeroing = arm['tick'], False
                self.record('press_key', message='请把云台摆到标定对应原点，再短按并松开 KEY0 或 KEY1')
            saw_zeroing |= any(a['state_name'] == 'ZEROING' and a['command_id'] == 0
                              and a['standalone_test_mode'] and tick_after(a['tick'], baseline)
                              for a in self.client.arm_events)
            if (saw_zeroing and arm['state_name'] == 'READY' and arm['origins_valid']
                    and not arm['fault'] and arm['command_id'] == 0):
                self.origin_health()
                return
            self.sleep(.08)
        raise TimeoutError('未收到新 KEY 对应的记录原点动作')

    def origin_health(self):
        arm = self.client.arm_status()
        self.envelope(arm)
        if (arm['state_name'] != 'READY' or not arm['origins_valid']
                or not arm['standalone_test_mode'] or arm['fault'] or arm['command_id'] != 0
                or any(abs(arm[k]) > 50 for k in ('z_cdeg', 'yaw_cdeg', 'reach_cdeg'))):
            raise RuntimeError('新原点未就绪或云台已离开原点')
        chassis = self.client.status()
        if chassis['state_name'] != 'EMPTY' or chassis['fault'] or chassis['chassis_fault']:
            raise RuntimeError('底盘未处于无故障 EMPTY 状态')

    def observation_health(self):
        arm = self.checked_arm()
        self.envelope(arm)
        if arm['state_name'] not in ('WAIT_VISION', 'ALIGNED') or abs(arm['z_cdeg']) > 135:
            raise RuntimeError('云台未停在标定拍照高度')
        return arm

    def cycle(self, position, expected_world, target, *, hold=False):
        self.record('approach_ring', position=position, target_cdeg=target)
        self.align_to(target)
        local_world = None
        for attempt in range(7):
            points = self.observer.observe({position}, floor=self.clock()+.15,
                                           health=self.observation_health, timeout=15)
            arm = self.observation_health()
            if self.clock()-self.observer.acquired_at >= .8:
                raise RuntimeError('圆环中心观测已过期')
            pixel = points[position]
            offset = self.calibration.offset(pixel)
            world = self.calibration.world_point([arm['yaw_cdeg'], arm['reach_cdeg']], offset)
            if local_world is None:
                # The first close view and origin view each carry calibration
                # error. Subsequent corrections only compare to the fixed
                # local anchor and still require a freshly recognized ID.
                distance = float(np.linalg.norm(world-expected_world))
                if distance > max(6, 3*self.calibration.limit_mm):
                    raise RuntimeError(f'新观测与原点识别的位置相差 {distance:.2f} mm')
                local_world = world
            else:
                distance = float(np.linalg.norm(world-local_world))
                if distance > 6:
                    raise RuntimeError(f'微调画面中的环心偏离首次近距离观测 {distance:.2f} mm')
            self.record('ring_visual_check', position=position, aligned_pixel=pixel.tolist(),
                        residual_mm=offset.tolist(), correction=attempt,
                        association_distance_mm=distance)
            if np.linalg.norm(offset) <= self.calibration.limit_mm:
                break
            if attempt == 6:
                raise RuntimeError(f'圆心对齐未达到 {self.calibration.limit_mm:g} mm 视觉容差')
            # Keep local correction in firmware's small-motion range.
            limited = offset*min(1., 3/np.linalg.norm(offset))
            delta = self.calibration.world_point([arm['yaw_cdeg'], arm['reach_cdeg']], limited)
            correction = self.calibration.target(delta, local_correction=True)
            if abs(correction[1]-arm['yaw_cdeg']) > 270:
                correction = [0, arm['yaw_cdeg']+int(math.copysign(270, correction[1]-arm['yaw_cdeg'])),
                              correction[2]]
            if correction == target:
                raise RuntimeError('纠偏目标未变化，停止重复发送')
            target = correction
            self.align_to(target)
        lowered = lifted_z_cdeg = None
        if self.lower:
            self.record('ring_vertical_cycle', position=position, drop_mm=FIXED_DROP_MM,
                        simulated_action='grip' if self.simulate_grip else 'release')
            lower, lifted = self.lower_and_lift(arm, release=not self.simulate_grip)
            lowered = (arm['z_cdeg']-lower['z_cdeg'])/900
            lifted_z_cdeg = lifted['z_cdeg']
        if not hold and (target != ORIGIN or self.lower):
            self.align_to(ORIGIN)
        result = dict(position=position, aligned_pixel=pixel.tolist(), residual_mm=offset.tolist(),
                      lowered_mm=lowered, lifted_z_cdeg=lifted_z_cdeg,
                      returned_to_origin=not hold or target == ORIGIN,
                      held_for_inspection=hold, tolerance_mm=self.calibration.limit_mm,
                      simulated_release=self.lower and not self.simulate_grip,
                      simulated_grip=self.lower and self.simulate_grip,
                      physical_grasp_completed=False, physical_place_completed=False)
        self.results.append(result)
        self.record('ring_cycle_completed', **result)

    def run(self, order, test_id):
        if self.reuse_ready_origin:
            self.origin_health()
            self.record('using_ready_origin')
        else:
            self.wait_key()
        points = self.observer.observe({1, 2, 3}, all_rings=True, floor=self.clock(),
                                       health=self.origin_health)
        world = {n: self.calibration.world_point([0, 0], self.calibration.offset(points[n], coarse=True))
                 for n in order}
        targets = {n: self.calibration.target(world[n]) for n in order}
        self.record('all_rings_confirmed', order=order, centers={n: p.tolist() for n, p in points.items()},
                    approach_targets_cdeg=targets)
        self.origin_health()
        if self.clock()-self.observer.acquired_at >= .8:
            raise RuntimeError('原点圆环观测已过期')
        self.context = test_id, 0, 103
        self.started = True
        self.client.arm_test_begin(test_id, 103)
        self.wait_arm('WAIT_VISION', ('PREPARING',), target=ORIGIN)
        for position in order:
            self.cycle(position, world[position], targets[position])
        return dict(order=order, positions=self.results, physical_place_completed=False,
                    final_arm=self.checked_arm())


def ensure_camera_free(source):
    path = Path(source).resolve(strict=True)
    probe = subprocess.run(['fuser', str(path)], capture_output=True, text=True)
    if probe.returncode != 1 or probe.stderr.strip():
        raise RuntimeError(f'无法确认相机空闲，请检查 {path} 的占用')


def load_floor_calibration(path, camera, *, lower, confirmed_downstroke_mm=None):
    try:
        data = json.loads(path.read_text())
    except FileNotFoundError as exc:
        raise ValueError(
            f'缺少放置区地面标定 {path}。请根据 config/arm_ring_floor.example.json 填入实测数据。'
            '暂时可用 --vision-only 检查圆环编号和环心。') from exc
    except (OSError, UnicodeError, ValueError) as exc:
        raise ValueError(f'无法读取地面标定 {path}：{exc}') from exc
    if not isinstance(data, dict):
        raise ValueError('放置区地面标定须为 JSON 对象')
    try:
        return FloorCalibration(data, camera, lower=lower,
                                confirmed_downstroke_mm=confirmed_downstroke_mm)
    except (KeyError, TypeError, ValueError, np.linalg.LinAlgError, cv2.error) as exc:
        raise ValueError(f'放置区地面标定 {path} 无效：{exc}') from exc


def run_live(args, order, config, calibration):
    size = [config.width, config.height]
    camera = Camera(config)
    pipeline = SimpleNamespace(camera=camera, qr_camera=camera, material_detector=None,
                               ring_detector=PlacementRings(size))
    source = FrameStreamer(camera, pipeline, enable_materials=False, enable_qr=False,
                           preview_enabled=True, preview_width=config.width, draw_overlay=False,
                           detect_interval=.12)
    observer = RingObserver(source.get_state, size)
    with (Path(tempfile.gettempdir())/f'robot-preview-start-{os.getuid()}.lock').open('a') as lock, ExitStack() as restore:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.auto_calibrate or args.take_devices:
            for service in ('chassis-button.service', 'material-vision.service'):
                if subprocess.run(['systemctl', '--user', 'is-active', '--quiet', service],
                                  check=False).returncode == 0:
                    restore.callback(subprocess.run, ['systemctl', '--user', 'start', service], check=True)
                    subprocess.run(['systemctl', '--user', 'stop', service], check=True)
        if not args.vision_only:
            status = subprocess.run(['systemctl', '--user', 'is-active', 'chassis-button.service'],
                                    capture_output=True, text=True)
            if status.stdout.strip() not in ('inactive', 'failed'):
                raise RuntimeError('请在停车后先停止底盘按键服务，再执行独立圆环试验')
        ensure_camera_free(config.source)
        source.start()
        try:
            if args.vision_only:
                try:
                    wanted = {args.inspect_position} if args.inspect_position else {1, 2, 3}
                    points = observer.observe(wanted, all_rings=not args.inspect_position,
                                              floor=time.monotonic())
                finally:
                    jpeg = source.get_jpeg()
                    if jpeg:
                        (args.output/'camera.jpg').write_bytes(jpeg)
                return dict(order=order, centers_px={n: p.tolist() for n, p in points.items()},
                            confirmation_frames=4, confirmation_span_s=observer.span_s,
                            jitter_px=observer.jitter_px, physical_place_completed=False, motion_sent=False)
            import serial
            with serial.Serial(args.port, 115200, timeout=.02, write_timeout=1, exclusive=True) as port:
                client = ElectricalClient(port)
                if args.auto_calibrate:
                    from tools.arm_ring_calibrate import RingCalibrationTrial
                    trial = RingCalibrationTrial(client, args.output, observer, config, args.calibration,
                                                 trial_limit=args.trial_tolerance_mm, reference_capture=args.reference_capture)
                else:
                    trial = RingTrial(client, args.output, calibration, observer,
                                      lower=args.lower, simulate_grip=args.simulate_grip,
                                      reuse_ready_origin=args.reuse_ready_origin)
                try:
                    return trial.run(order, int(time.time()))
                except BaseException as exc:
                    trial.record('failed', reason=str(exc), completed_positions=trial.results)
                    if trial.started:
                        try:
                            trial.client.stop(0)
                        except Exception as stop_error:
                            trial.record('stop_unconfirmed', reason=str(stop_error))
                    raise
        finally:
            source.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    ordering = parser.add_mutually_exclusive_group()
    ordering.add_argument('--positions', help='放置顺序，例如 231')
    ordering.add_argument('--task-code', help='完整任务码 CCC+PPP+CCC+PPP')
    ordering.add_argument('--inspect-position', type=int, choices=(1, 2, 3),
                          help='只检查一个圆环，可用于视觉验证或自动标定试验')
    parser.add_argument('--station', choices=('batch1', 'batch2-processing'))
    parser.add_argument('--output', type=Path)
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument('--image', type=Path, help='只回放图片，不做连续帧确认或控制')
    inputs.add_argument('--camera-config', type=Path)
    parser.add_argument('--vision-only', action='store_true', help='只确认三个编号和环心，不打开串口')
    parser.add_argument('--port')
    parser.add_argument('--calibration', type=Path)
    parser.add_argument('--auto-calibrate', action='store_true',
                        help='水平采样地面标定，验证后保存到 --calibration 并测试水平对齐')
    parser.add_argument('--take-devices', action='store_true',
                        help='停车后使用：试验期间独占串口和相机，退出后恢复原服务')
    parser.add_argument('--reuse-ready-origin', action='store_true',
                        help='仅在已按新 KEY 且未移动的预检重试中复用有效 READY 原点')
    parser.add_argument('--trial-tolerance-mm', type=float, choices=(1., 3.), default=1.,
                        help='自动标定严格验算失败时，可显式使用 3 mm 做水平对照')
    parser.add_argument('--reference-samples', type=Path,
                        help='同一相机和云台上一次已验收的标定日志，补足地面覆盖')
    parser.add_argument('--check-calibration', action='store_true',
                        help='只检查地面标定文件，无需停止服务或连接主控')
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--simulate-release', action='store_true',
                        help='对齐后执行固件固定下降和抬升，模拟松爪')
    action.add_argument('--simulate-grip', action='store_true',
                        help='对齐后实际下降和抬升，中间只模拟夹取')
    parser.add_argument('--confirmed-downstroke-mm', type=int, choices=(FIXED_DROP_MM,),
                        help='操作者实测确认本轮三处均可下降的固件行程，单位 mm')
    args = parser.parse_args()
    args.lower = args.simulate_release or args.simulate_grip
    if args.reuse_ready_origin and (args.image or args.vision_only or args.check_calibration or args.auto_calibrate):
        parser.error('--reuse-ready-origin 仅用于地面标定完成后的独立运动试验')
    if args.confirmed_downstroke_mm is not None and not args.lower:
        parser.error('--confirmed-downstroke-mm 适用于夹爪模拟试验')
    if args.take_devices and (args.image or args.vision_only or args.check_calibration):
        parser.error('--take-devices 用于独立云台运动试验')
    if args.inspect_position and (args.station or not (args.auto_calibrate or args.vision_only)):
        parser.error('--inspect-position 用于单环视觉验证或自动标定试验')
    if args.reference_samples and not args.auto_calibrate:
        parser.error('--reference-samples 仅用于自动标定')
    if args.trial_tolerance_mm != 1 and not args.auto_calibrate:
        parser.error('--trial-tolerance-mm 仅用于自动标定')
    if args.auto_calibrate and (args.image or args.vision_only or args.check_calibration or args.lower):
        parser.error('--auto-calibrate 仅用于独立水平标定和对齐')
    if args.check_calibration:
        if (not args.camera_config or not args.calibration or args.vision_only or args.port
                or args.positions or args.task_code or args.station or args.output):
            parser.error('--check-calibration 须指定 --camera-config、--calibration，可加夹爪模拟选项')
    elif args.output is None or not (args.positions or args.task_code or args.inspect_position):
        parser.error('试验须指定 --output 和放置点顺序')
    elif args.image or args.vision_only:
        if args.lower or args.port or args.calibration:
            parser.error('视觉检查不使用串口、标定文件或下降选项')
    elif not args.port or not args.calibration:
        parser.error('运动试验须指定 --port 和实测地面 --calibration')
    try:
        config = load_config(args.camera_config).camera if args.camera_config else None
        if args.auto_calibrate and args.calibration.exists():
            raise ValueError(f'标定文件已存在，不自动覆盖：{args.calibration}')
        calibration = (load_floor_calibration(args.calibration, config, lower=args.lower,
                                              confirmed_downstroke_mm=args.confirmed_downstroke_mm)
                       if args.calibration and not args.auto_calibrate else None)
        if args.check_calibration:
            print('放置区地面标定文件检查通过')
            return
        args.reference_capture = None
        if args.reference_samples:
            with args.reference_samples.open() as log:
                rows = [json.loads(line) for line in log]
            records = [r for r in rows if r.get('event') == 'floor_calibration_capture_complete']
            if len(records) != 1 or not any(r.get('event') == 'floor_calibration_saved' for r in rows):
                raise ValueError('参考日志须包含一组通过验算并保存的采样')
            args.reference_capture = records[0]
        order = ([args.inspect_position] if args.inspect_position else
                 placement_order(args.positions, args.task_code, args.station))
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    args.output.mkdir(parents=True, exist_ok=False)
    if args.image:
        frame = cv2.imread(str(args.image))
        if frame is None:
            parser.error('无法读取回放图片')
        result = dict(order=order, candidates=[asdict(r) for r in
                      PlacementRings(frame.shape[1::-1]).detect(frame)], motion_sent=False)
    else:
        result = run_live(args, order, config, calibration)
    text = json.dumps(result, ensure_ascii=False, indent=2)
    (args.output/'result.json').write_text(text+'\n')
    print(text)


if __name__ == '__main__':
    main()
