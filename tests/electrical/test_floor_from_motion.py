import math

import numpy as np
import pytest

from pi_vision.floor_from_motion import fit_floor


def test_floor_fit_uses_encoder_poses_and_independent_holdout():
    projection = np.array([[.005, -.35, 220], [.35, .004, -227],
                           [.000035, .00008, 1.]])
    inverse = np.linalg.inv(projection)
    rings = np.array([[203, -100], [210, 5], [214, 105]])

    def sample(yaw, reach, world=rings):
        theta = math.radians(yaw)
        c, s = math.cos(theta), math.sin(theta)
        local = world @ np.array([[c, -s], [s, c]]) - [230+reach, 0]
        rays = np.c_[local, np.ones(3)] @ inverse.T
        points = rays[:, :2]/rays[:, 2:]
        return dict(yaw_cdeg=yaw*300, reach_cdeg=reach*36000/113.1,
                    centers=dict(zip((1, 2, 3), points)))

    fitting = [sample(*pose) for pose in [
        (0, 0), (0, -12), (0, -30), (5.4, -30), (5.4, -5), (-5.4, -5), (-5.4, -30)]]
    check = [sample(*pose) for pose in [(-2.7, -20), (2.7, -18), (0, 0)]]
    _, checks, training_error, holdout_error, _ = fit_floor(fitting, check, [1280, 720], 230)
    assert checks and max(training_error, holdout_error) < .001

    shifted = rings - [35, 12]
    prior = dict(fitting=[sample(0, 0, shifted), sample(0, -25, shifted),
                          sample(5.4, -22, shifted), sample(-5.4, -4, shifted)],
                 validation=[sample(2.7, -16, shifted)])
    _, _, train, independent, _ = fit_floor(
        fitting, check, [1280, 720], 230, reference_capture=prior)
    assert max(train, independent) < .001

    check[0]['centers'][2] += [8, 0]
    with pytest.raises(ValueError, match='独立验算误差'):
        fit_floor(fitting, check, [1280, 720], 230)


def test_translation_only_does_not_determine_absolute_claw_position():
    samples = [dict(yaw_cdeg=0, reach_cdeg=-3175*i,
                    centers={n: [300*n, 450-25*i] for n in (1, 2, 3)}) for i in range(5)]
    with pytest.raises(ValueError, match='退化'):
        fit_floor(samples, samples, [1280, 720], 230)
