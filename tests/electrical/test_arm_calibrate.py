import numpy as np
import pytest
from tools.arm_calibrate import fit_correction


def test_measured_mapping_produces_opposite_pixel_motion_for_alignment():
    motion = np.array([[-4, 0], [-.04, 4], [-.04, -4]])
    jacobian = np.array([[.4, -9], [9, .2]])
    pixels = motion@jacobian.T
    correction, fitted, residual = fit_correction(motion, pixels)
    assert np.allclose(fitted, jacobian)
    assert np.allclose(jacobian@correction@np.array([30, -15]), [-30, 15])
    assert max(residual) < 1e-10


@pytest.mark.parametrize('motion,pixels', [
    ([[0, 0], [0, 0], [0, 0]], [[1, 2], [2, 3], [3, 4]]),
    ([[-4, 0], [0, 4], [0, -4]], [[1, 0], [2, 0], [-2, 0]]),
    ([[-4, 0], [0, 4], [0, -4]], [[0, 40], [40, 0], [-40, 20]]),
])
def test_singular_or_inconsistent_calibration_is_rejected(motion, pixels):
    with pytest.raises(ValueError):
        fit_correction(motion, pixels)
