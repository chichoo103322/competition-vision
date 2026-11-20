"""Estimate a metric plane from fixed features and measured planar arm motion."""
import math

import numpy as np


def sample_arrays(samples, radius):
    pixels, rotations, origins = [], [], []
    for sample in samples:
        theta = math.radians(sample['yaw_cdeg']/300)
        c, s = math.cos(theta), math.sin(theta)
        rotation = np.array([[c, -s], [s, c]])
        radial = radius+sample['reach_cdeg']*113.1/36000
        centers = {int(n): point for n, point in sample['centers'].items()}
        pixels.append([centers[n] for n in (1, 2, 3)])
        rotations.append(rotation)
        origins.append(rotation @ [radial, 0.])
    return np.asarray(pixels), np.asarray(rotations), np.asarray(origins)


def fit_floor(samples, holdouts, size, radius, *, limit_mm=1, reference_capture=None):
    if not 1 <= limit_mm <= 3:
        raise ValueError('地面验算门槛须在 1 至 3 mm')
    scale = max(size)/2
    image_center = np.asarray(size)/2
    captures = [(samples, holdouts)]
    if reference_capture is not None:
        captures.append((reference_capture['fitting'], reference_capture['validation']))
    fitting = [sample_arrays(fit, radius) for fit, _ in captures]
    trials = [sample_arrays(check, radius) for _, check in captures]
    normalized = [(pixels-image_center)/scale for pixels, _, _ in fitting]
    homogeneous = [np.concatenate((p, np.ones((*p.shape[:2], 1))), axis=2) for p in normalized]
    rhs = np.concatenate([
        np.repeat((origins[0]-origins[1:])[:, None, :], 3, axis=1).ravel()
        for _, _, origins in fitting
    ])

    def solve(slope):
        if np.max(np.abs(slope)) > .35:
            return None
        equations = []
        # The camera mapping is shared. Each parking position has its own
        # ground coordinates, so only poses within a capture are compared.
        for p, h, (_, rotations, _) in zip(normalized, homogeneous, fitting):
            rays = h/(1+p@slope)[..., None]
            coefficients = np.einsum('jik,jnl->jnikl', rotations, rays).reshape(-1, 3, 2, 6)
            equations.append((coefficients[1:]-coefficients[0]).reshape(-1, 6))
        design = np.concatenate(equations)
        values, _, rank, singular = np.linalg.lstsq(design, rhs, rcond=None)
        if rank != 6 or singular[0] > singular[-1]*200:
            return None
        residual = design@values-rhs
        return residual, values.reshape(2, 3)

    slope = np.zeros(2)
    initial = solve(slope)
    if initial is None:
        raise ValueError('环心和云台采样退化，无法独立估计地面映射')
    for _ in range(12):
        error, values = solve(slope)
        d = .0005
        neighbors = [solve(slope+step) for step in (np.array([d, 0]), np.array([0, d]))]
        if any(value is None for value in neighbors):
            break
        jacobian = np.column_stack([(value[0]-error)/d for value in neighbors])
        step = np.linalg.lstsq(jacobian, -error, rcond=None)[0]
        improved = False
        for fraction in (1., .5, .25, .125):
            candidate = slope+fraction*step
            fitted = solve(candidate)
            if fitted is not None and np.linalg.norm(fitted[0]) < np.linalg.norm(error)-1e-7:
                slope = candidate
                improved = True
                break
        if not improved:
            break
    error, values = solve(slope)
    matrix = np.vstack((values, [*slope, 1.]))
    transform = np.eye(3)
    transform[:2, :2] /= scale
    transform[:2, 2] = -image_center/scale
    matrix = matrix@transform

    def to_world(points, rot, origin):
        camera = np.concatenate((points, np.ones((*points.shape[:2], 1))), axis=2)@matrix.T
        return np.einsum('jik,jnk->jni', rot, camera[:, :, :2]/camera[:, :, 2:])+origin[:, None, :]

    def rows(points, rots, bases, world):
        offsets = np.einsum('jki,jnk->jni', rots, world - bases[:, None, :])
        return np.concatenate((points, offsets), axis=2).reshape(-1, 4)

    fit_rows, check_rows = [], []
    train_error = holdout_error = 0.
    for fit, trial in zip(fitting, trials):
        trained = to_world(*fit)
        world = np.mean(trained, axis=0)
        predicted = to_world(*trial)
        train_error = max(train_error, float(np.linalg.norm(trained-world, axis=2).max()))
        holdout_error = max(holdout_error, float(np.linalg.norm(predicted-world, axis=2).max()))
        fit_rows.append(rows(*fit, world))
        check_rows.append(rows(*trial, world))
    if max(train_error, holdout_error) > limit_mm:
        raise ValueError(f'地面拟合误差 {train_error:.2f} mm，独立验算误差 {holdout_error:.2f} mm，门槛 {limit_mm:g} mm')
    fit_points = np.concatenate(fit_rows)
    check_points = np.concatenate(check_rows)
    independent = np.min(np.linalg.norm(check_points[:, None, :2]-fit_points[:, :2], axis=2), axis=1) >= 10
    return fit_points.tolist(), check_points[independent].tolist(), train_error, holdout_error, matrix
