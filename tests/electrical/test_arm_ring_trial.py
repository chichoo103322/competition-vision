from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from tools.arm_ring_trial import (
    FloorCalibration, RingObserver, RingTrial, match_calibration_rings, placement_order,
)
from pi_vision.placement_rings import PlacementRings
from test_arm_three_color_trial import Plant


def test_numbered_concentric_fit_independently_of_layout():
    image = np.full((720, 1280, 3), 238, np.uint8)
    layout = {3: (250, 344), 1: (640, 360), 2: (1030, 376)}
    for n, (x, y) in layout.items():
        for radius in (45, 55, 65, 75, 85):
            for angle in range(0, 360, 45):
                cv2.ellipse(image, (x, y), (radius, radius), 0, angle, angle+32, (0, 0, 0), 2)
        (w, h), _ = cv2.getTextSize(str(n), cv2.FONT_HERSHEY_SIMPLEX, 1.4, 2)
        cv2.putText(image, str(n), (x-w//2, y+h//2), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 0), 2)
    points = {r.position_id: (r.center_x, r.center_y)
              for r in PlacementRings((1280, 720)).detect(image)}
    assert set(points) == set(layout)
    assert all(np.linalg.norm(np.subtract(points[n], layout[n])) < 1.5 for n in layout)


def test_digit_at_rotated_claw_view():
    patch = np.full((155, 155), 240, np.uint8)
    cv2.putText(patch, '3', (57, 97), cv2.FONT_HERSHEY_SIMPLEX, 1.55, 20, 3)
    transform = cv2.getRotationMatrix2D((77, 77), 43, 1.)
    patch = cv2.warpAffine(patch, transform, (155, 155), borderValue=240)
    frame = np.full((360, 640, 3), 240, np.uint8)
    frame[103:258, 243:398] = cv2.cvtColor(patch, cv2.COLOR_GRAY2BGR)
    for radius in (54, 65, 75, 85):
        cv2.circle(frame, (320, 180), radius, (20, 20, 20), 2)
    found = [r for r in PlacementRings((640, 360)).detect(frame) if r.position_id == 3]
    assert len(found) == 1
    assert np.linalg.norm([found[0].center_x-320, found[0].center_y-180]) < 1.5


@pytest.fixture
def calibration_data():
    camera = SimpleNamespace(source='/dev/camera', width=1280, height=720, flip=0, undistort=False)
    # Synthetic camera frame: x moves tangentially, y moves radially.
    def point(x, y):
        return [x, y, (y-360)/5, (x-640)/5]
    config = dict(camera_source=camera.source, resolution=[1280, 720], camera_flip=0,
                  camera_undistort=False, photo_z_cdeg=0, origin_radius_mm=230,
                  max_abs_yaw_deg=30, max_retraction_mm=60, clear_downstroke_mm=100,
                  fit_points=[point(x, y) for x in (80,1200) for y in (80,650)],
                  check_points=[point(380,240), point(850,380)])
    return camera, config


def test_task_group_and_independent_ground_checks(calibration_data):
    assert placement_order(None, '156+123+516+231', 'batch2-processing') == [2, 3, 1]
    with pytest.raises(ValueError):
        placement_order('112', None, None)
    camera, config = calibration_data
    config['check_points'][0][2] += 3
    with pytest.raises(ValueError, match='1 mm'):
        FloorCalibration(config, camera, lower=True)


def test_relaxed_floor_calibration_is_for_horizontal_inspection(calibration_data):
    camera, config = calibration_data
    config['check_points'][0][2] += 2.5
    config['validation_limit_mm'] = 3
    assert FloorCalibration(config, camera, lower=False).limit_mm == 3
    with pytest.raises(ValueError, match='仅供水平'):
        FloorCalibration(config, camera, lower=True)
    assert FloorCalibration(config, camera, lower=True, confirmed_downstroke_mm=100).limit_mm == 3


def test_coarse_only_boundary_margin(calibration_data):
    camera, config = calibration_data
    calibration = FloorCalibration(config, camera, lower=False)
    assert np.isfinite(calibration.offset([640, 653], coarse=True)).all()
    with pytest.raises(ValueError, match='超出已标定'):
        calibration.offset([640, 653])
    with pytest.raises(ValueError, match='超出已标定'):
        calibration.offset([640, 665], coarse=True)


def test_local_correction_at_forward_stop_still_rotates(calibration_data):
    camera, config = calibration_data
    calibration = FloorCalibration(config, camera, lower=False)
    angle = np.deg2rad(-25)
    point = 230.17*np.array([np.cos(angle), np.sin(angle)])
    with pytest.raises(ValueError, match='超出已验证'):
        calibration.target(point)
    assert calibration.target(point, local_correction=True) == [0, -7500, 0]
    with pytest.raises(ValueError, match='超出已验证'):
        calibration.target(1.1*point, local_correction=True)


class RingPlant(Plant):
    def __init__(self):
        super().__init__()
        self.keys = False
        self.arm_events = []

    def status(self):
        return dict(state_name='EMPTY', fault=0, chassis_fault=0)

    def arm_status(self):
        if self.now >= 100.2 and not self.keys:
            self.arm_events.append(dict(self.arm, state_name='ZEROING', tick=100150))
            self.keys = True
        return dict(self.arm, tick=round(self.now*1000))


@pytest.mark.parametrize('is_lower', [False, True])
def test_three_ring_trial_checks_reachable_starts_on_fresh_key_and_returns(
        tmp_path, calibration_data, is_lower):
    plant = RingPlant()
    camera, config = calibration_data
    calibration = FloorCalibration(config, camera, lower=is_lower)
    points = {1: np.array([425., 175.]), 2: np.array([640., 225.]), 3: np.array([860., 175.])}
    world = {n: calibration.world_point([0, 0], calibration.offset(p)) for n,p in points.items()}

    class Observation:
        acquired_at = 0.

        def observe(self, wanted, *, floor, health, **_):
            health()
            assert plant.keys
            plant.now = max(floor+.25, plant.now)
            self.acquired_at = plant.now
            if len(wanted) == 3:
                assert plant.calls == []
                return points
            angle = np.deg2rad(plant.arm['yaw_cdeg']/300)
            c, s = np.cos(angle), np.sin(angle)
            rotation_back = np.array([[c, s], [-s, c]])
            return {n: (rotation_back @ world[n] - calibration.world_point(
                [0, plant.arm['reach_cdeg']], [0, 0]))[::-1]*5+[640,360] for n in wanted}

    result = RingTrial(plant, tmp_path, calibration, Observation(), lower=is_lower,
                       simulate_grip=is_lower,
                       clock=lambda:plant.now, sleep=plant.sleep).run([2, 3, 1], 1234)
    assert [item['position'] for item in result['positions']] == [2,3,1]
    assert sum(c[0]=='pick' for c in plant.calls) == (3 if is_lower else 0)
    assert sum(c[0]=='simulated_grip_done' for c in plant.calls) == (3 if is_lower else 0)
    assert all(item['simulated_grip'] == is_lower and not item['physical_grasp_completed']
               for item in result['positions'])
    assert sum(c[0]=='align' and c[-3:]==(0,0,0) for c in plant.calls) == 3
    assert all(np.linalg.norm(item['residual_mm']) <= 1 for item in result['positions'])


def test_observer_rejects_duplicate_id_and_old_frame():
    now = [30.]
    calls = [0]

    def snapshot():
        calls[0] += 1
        seq = calls[0] if calls[0] <= 5 else 6
        ids = [1, 1, 3] if calls[0] <= 5 else [1, 2, 3]
        return dict(
            session_id='test', capture=dict(valid=True, epoch=0),
            preview=dict(source_width=1280, source_height=720),
            ring_observation=dict(valid=True, capture_epoch=0, source_sequence=seq,
                                  acquired_monotonic_s=now[0]),
            rings=[dict(position_id=n, center_x=200*n, center_y=350, digit_score=.9,
                        point_kind='ring_circle_center') for n in ids])

    observer = RingObserver(snapshot, [1280,720], clock=lambda:now[0],
                            sleep=lambda duration: now.__setitem__(0, now[0]+duration))
    with pytest.raises(TimeoutError):
        observer.observe({1,2,3}, all_rings=True, floor=now[0], timeout=1)


@pytest.mark.parametrize('jitter', [0, 4])
def test_observer_uses_full_stable_interval_at_25_hz(jitter):
    now = [0.]
    seq = [0]

    def snapshot():
        seq[0] += 1
        x = 200 + jitter * (seq[0] % 2)
        return dict(session_id='test', capture=dict(valid=True, epoch=0),
                    preview=dict(source_width=1280, source_height=720),
                    ring_observation=dict(valid=True, capture_epoch=0, source_sequence=seq[0],
                                          acquired_monotonic_s=now[0]),
                    rings=[dict(position_id=1, center_x=x, center_y=350, digit_score=.9,
                                point_kind='ring_circle_center')])

    observer = RingObserver(snapshot, [1280, 720], clock=lambda: now[0],
                            sleep=lambda dt: now.__setitem__(0, now[0]+dt))
    if jitter:
        with pytest.raises(TimeoutError):
            observer.observe({1}, floor=0, timeout=.5)
    else:
        assert observer.observe({1}, floor=0, timeout=.5)[1].tolist() == [200, 350]
        assert observer.confirmation_frames >= 7 and observer.span_s >= .25


def test_calibration_only_tracks_previously_verified_unreadable_ring():
    anchors = {3: [300, 400], 2: [640, 400], 1: [980, 400]}
    rings = [dict(position_id=n, center_x=x, center_y=440, point_kind='ring_circle_center')
             for n, x in [(1, 920), (None, 580), (3, 240)]]
    rings.append(dict(position_id=None, center_x=800, center_y=670, point_kind='ring_circle_center'))
    assert match_calibration_rings(rings, anchors)[2].tolist() == [580, 440]
    rings[3].update(center_x=650, center_y=400)
    assert match_calibration_rings(rings, anchors) == {}
    rings[3].update(center_x=800, center_y=670)
    rings[1]['position_id'] = 3
    assert match_calibration_rings(rings, anchors) == {}
    rings[1]['position_id'] = None
    rings[1]['center_x'] = 475
    assert match_calibration_rings(rings, anchors) == {}
