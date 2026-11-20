"""Bounded runtime visual-servo model; measured calibration files are never rewritten."""
import math

import numpy as np


def local_motion_mm(before, after, origin_radius_mm):
    """Encoder displacement in the previous observation's radial/tangent frame."""
    def point(a):
        radius = origin_radius_mm+a['reach_cdeg']*113.1/36000
        yaw = math.radians(a['yaw_cdeg']/300)
        return np.array([radius*math.cos(yaw), radius*math.sin(yaw)])
    angle = math.radians(before['yaw_cdeg']/300)
    rotation = np.array([[math.cos(angle),math.sin(angle)],
                         [-math.sin(angle),math.cos(angle)]])
    return rotation @ (point(after)-point(before))


def observation_dynamics(points, frames):
    """Estimate motion only from same-pose fresh frames, not before/after arm moves."""
    p = np.asarray(points, dtype=float)
    t = np.asarray([f['acquired'] for f in frames], dtype=float)
    result = dict(velocity_px_s=[0.,0.], velocity_reliable=False,
                  stationary_for_mapping=False, pixel_time_s=float(np.median(t)))
    if len(p)<3 or p.shape!=(len(t),2) or not np.isfinite(p).all() or not np.isfinite(t).all():
        return result
    if np.any(np.diff(t)<=0) or t[-1]-t[0]<.2-1e-9:
        return result
    result['stationary_for_mapping'] = bool(np.ptp(p,axis=0).max()<=1)
    centered = t-t.mean()
    velocity = centered @ (p-p.mean(axis=0)) / (centered @ centered)
    residual = np.max(np.linalg.norm(p-p.mean(axis=0)-centered[:,None]*velocity,axis=1))
    segments = np.diff(p,axis=0)/np.diff(t)[:,None]
    speeds = np.linalg.norm(segments,axis=1)
    moving = (np.linalg.norm(velocity)>1.5 and np.min(speeds)>1
              and np.max(speeds)/np.min(speeds)<=2.5
              and np.all(segments @ velocity > 0) and residual<=1.)
    if moving:
        result.update(velocity_px_s=velocity.tolist(),velocity_reliable=True,
                      stationary_for_mapping=False)
    return result


class AdaptivePixelMap:
    """Conservative secant updates from short, stable, identified observations."""
    def __init__(self, correction_matrix):
        c = np.asarray(correction_matrix,dtype=float)
        if c.shape!=(2,2) or not np.isfinite(c).all() or np.linalg.cond(c)>20:
            raise ValueError('well-conditioned finite camera correction matrix required')
        self.base_jacobian = -np.linalg.inv(c)
        self.jacobian = self.base_jacobian.copy()
        self.step_gain = 1.
        self.motion_latency_s = .65
        self.bad_observations = 0

    @property
    def correction_matrix(self):
        return -np.linalg.inv(self.jacobian)

    def recover(self):
        self.step_gain = max(.4,self.step_gain*.7)
        self.bad_observations += 1
        if self.bad_observations>=3:
            self.jacobian = self.base_jacobian.copy()

    def progress(self):
        self.step_gain = min(1.,self.step_gain+.15)
        self.bad_observations = 0

    def predicted_pixel(self, observation, now, max_lead_px=4.):
        pixel = np.asarray(observation['pixel'],dtype=float)
        if not observation.get('velocity_reliable'):
            return pixel
        age = now-observation['pixel_time_s']
        if not 0<=age<1:
            return pixel
        lead = np.asarray(observation['velocity_px_s'])*min(.8,age+self.motion_latency_s)
        lead *= min(1.,max_lead_px/max(np.linalg.norm(lead),1e-9))
        return pixel+lead

    def update(self, before_obs, after_obs, before_arm, after_arm, origin_radius_mm,
               motion):
        duration = motion.get('elapsed_s',0.)
        if motion.get('full_avoidance') or not 0<duration<=2.:
            return dict(updated=False,reason='slow_or_clearance_motion')
        self.motion_latency_s = .8*self.motion_latency_s+.2*duration
        if (not before_obs.get('stationary_for_mapping')
                or not after_obs.get('stationary_for_mapping')
                or before_obs.get('track_id') is None
                or before_obs.get('track_id')!=after_obs.get('track_id')):
            return dict(updated=False,reason='moving_or_unidentified_target')
        dt = after_obs['pixel_time_s']-before_obs['pixel_time_s']
        if not 0<dt<=3:
            return dict(updated=False,reason='observation_gap')
        delta = local_motion_mm(before_arm,after_arm,origin_radius_mm)
        distance = np.linalg.norm(delta)
        if not .5<=distance<=4.2:
            return dict(updated=False,reason='insufficient_or_large_motion')
        pixels = np.asarray(after_obs['pixel'])-before_obs['pixel']
        expected = self.jacobian @ delta
        residual = pixels-expected
        if np.linalg.norm(residual)>max(4.,.35*np.linalg.norm(expected)):
            return dict(updated=False,reason='target_motion_or_model_outlier',
                        residual_px=residual.tolist())
        candidate = self.jacobian+.25*np.outer(residual,delta)/(delta @ delta)
        relative = candidate @ np.linalg.inv(self.base_jacobian)
        singular = np.linalg.svd(relative,compute_uv=False)
        if (np.linalg.det(candidate)*np.linalg.det(self.base_jacobian)<=0
                or np.linalg.cond(candidate)>20 or singular.min()<.65 or singular.max()>1.5):
            return dict(updated=False,reason='calibration_update_bound')
        self.jacobian = candidate
        return dict(updated=True,reason='trusted_encoder_secant',
                    correction_matrix=self.correction_matrix.tolist(),residual_px=residual.tolist())
