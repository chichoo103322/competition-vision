"""Collect encoder and numbered-ring samples at the origin height."""
import json
import math
from types import SimpleNamespace

from pi_vision.floor_from_motion import fit_floor
from tools.arm_ring_trial import FloorCalibration, ORIGIN, REACH_MM_PER_CDEG, RingTrial


FIRMWARE_YAW_LIMIT_DEG = 90
MAX_RETRACTION_MM = 60
RADIUS_MM = 230


class RingCalibrationTrial(RingTrial):
    def __init__(self, client, output, observer, camera, calibration_path, *, trial_limit, reference_capture=None):
        scope = SimpleNamespace(max_yaw=FIRMWARE_YAW_LIMIT_DEG, max_reach=MAX_RETRACTION_MM)
        super().__init__(client, output, scope, observer, lower=False)
        self.camera = camera
        self.calibration_path = calibration_path
        self.trial_limit = trial_limit
        self.reference_capture = reference_capture
        self.pose = ORIGIN
        self.anchors = None

    def move(self, yaw_deg, reach_mm):
        target = [0, round(yaw_deg*300), round(reach_mm/REACH_MM_PER_CDEG)]
        count = max(1, math.ceil(abs(target[1]-self.pose[1])/240),
                    math.ceil(abs(target[2]-self.pose[2])*REACH_MM_PER_CDEG/3))
        previous = self.pose
        for i in range(1, count+1):
            step = [round(a+(b-a)*i/count) for a, b in zip(previous, target)]
            self.align_to(step)
            self.pose = step

    def sample(self, yaw_deg, reach_mm):
        self.move(yaw_deg, reach_mm)
        points = self.observer.observe({1, 2, 3}, floor=self.clock()+.15,
                                       health=self.observation_health, timeout=18,
                                       calibration_anchors=self.anchors)
        self.anchors = points
        arm = self.observation_health()
        if abs(arm['z_cdeg']) > 135:
            raise RuntimeError('标定采样离开原点高度')
        sample = dict(yaw_cdeg=arm['yaw_cdeg'], reach_cdeg=arm['reach_cdeg'],
                      centers={n: p.tolist() for n, p in points.items()},
                      jitter_px=self.observer.jitter_px)
        self.record('floor_calibration_sample', **sample)
        return sample

    def run(self, order, test_id):
        self.wait_key()
        self.observer.observe({1, 2, 3}, floor=self.clock(),
                              health=self.origin_health, timeout=18)
        self.origin_health()
        self.context, self.started = (test_id, 0, 103), True
        self.client.arm_test_begin(test_id, 103)
        self.wait_arm('WAIT_VISION', ('PREPARING',), target=ORIGIN)
        # Coherent path with only small yaw/reach commands at Z=0.
        base = self.sample(0, 0)
        fit = [base, self.sample(0, -12), self.sample(0, -30),
               self.sample(5.4, -30), self.sample(5.4, -5),
               self.sample(-5.4, -5), self.sample(-5.4, -30)]
        check = [self.sample(-2.7, -20), self.sample(2.7, -18), self.sample(0, 0)]
        self.record('floor_calibration_capture_complete', fitting=fit, validation=check)
        camera = self.camera
        size = [camera.width, camera.height]
        rows, checks, training, holdout, matrix = fit_floor(
            fit, check, size, RADIUS_MM, limit_mm=self.trial_limit,
            reference_capture=self.reference_capture)
        limit = 1 if max(training, holdout) <= 1 else self.trial_limit
        self.record('floor_fit', training_mm=training, holdout_mm=holdout,
                    independent_points=len(checks), acceptance_limit_mm=limit)
        config = dict(camera_source=camera.source, resolution=size, camera_flip=camera.flip,
                      camera_undistort=camera.undistort, photo_z_cdeg=0,
                      origin_radius_mm=RADIUS_MM, max_abs_yaw_deg=FIRMWARE_YAW_LIMIT_DEG,
                      max_retraction_mm=MAX_RETRACTION_MM, clear_downstroke_mm=None,
                      fit_points=rows, check_points=checks, calibration_source='encoder_and_ring_motion',
                      training_residual_mm=training, validation_residual_mm=holdout,
                      validation_limit_mm=limit, projective_matrix=matrix.tolist())
        self.calibration = FloorCalibration(config, camera, lower=False)
        self.calibration_path.parent.mkdir(parents=True, exist_ok=True)
        encoded = json.dumps(config, indent=2)+'\n'
        with self.calibration_path.open('x') as file:
            file.write(encoded)
        self.record('floor_calibration_saved', path=str(self.calibration_path))
        origin, targets, blocked = {}, {}, {}
        for n in order:
            try:
                latest = check[-1]
                origin[n] = self.calibration.world_point([latest['yaw_cdeg'], latest['reach_cdeg']],
                    self.calibration.offset(latest['centers'][n]))
                targets[n] = self.calibration.target(origin[n])
            except ValueError as exc:
                blocked[n] = str(exc)
        if blocked:
            self.record('horizontal_trial_unreachable', blocked=blocked,
                        plan_world_mm={n: p.tolist() for n, p in origin.items()})
        reachable = [n for n in order if n in targets]
        if blocked and self.trial_limit == 1:
            reachable = []
        for position in reachable:
            self.cycle(position, origin[position], targets[position],
                       hold=self.trial_limit > 1 and position == reachable[-1])
        return dict(order=order, calibration_saved=True, positions=self.results,
                    blocked_positions=blocked, validation_residual_mm=holdout,
                    validation_limit_mm=limit, physical_place_completed=False,
                    final_arm=self.checked_arm())
