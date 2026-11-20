"""相机随夹爪运动的局部极坐标对齐；没有未标定默认值，不控制夹爪。"""
from dataclasses import dataclass
import math


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


@dataclass(frozen=True)
class MotorPose:
    z_deg: float
    yaw_deg: float
    reach_deg: float


@dataclass(frozen=True)
class MechanismPose:
    z_mm: float
    yaw_deg: float
    reach_mm: float


class ArmCalibration:
    """矩阵把像素误差换成随机构转动的径向/切向毫米修正，不是底盘X/Y。"""
    def __init__(self, config):
        if config.get('measured') is not True:
            raise ValueError('measured arm/camera calibration required')
        self.data = config
        for name in ('calibration_id', 'plane_id', 'camera_source', 'hsv_profile_sha256'):
            if not isinstance(config.get(name), str) or not config[name]:
                raise ValueError('missing calibration identity: ' + name)
        if not config['camera_source'].startswith('/dev/v4l/by-id/'):
            raise ValueError('material camera requires a stable device path')
        self.signs = config['motor_signs']
        if set(self.signs) != {'z', 'yaw', 'reach'} or any(type(s) is not int or s not in (-1, 1) for s in self.signs.values()):
            raise ValueError('all three motor signs must be measured +/-1')
        self.radius0 = config['origin_radius_mm']
        if not number(self.radius0) or self.radius0 <= 0:
            raise ValueError('measure pivot-to-grasp radius at manual origin')
        self.width, self.height = config['resolution']
        if any(type(v) is not int or v <= 0 for v in (self.width, self.height)):
            raise ValueError('invalid calibration resolution')
        self.reference = config['reference_pixel']
        if (len(self.reference) != 2 or not all(number(v) for v in self.reference) or
                not 0 <= self.reference[0] < self.width or not 0 <= self.reference[1] < self.height):
            raise ValueError('invalid calibrated claw reference pixel')
        self.matrix = config['correction_radial_tangent_mm_per_pixel']
        if len(self.matrix) != 2 or any(len(r) != 2 or not all(number(v) for v in r) for r in self.matrix):
            raise ValueError('invalid radial/tangent correction matrix')
        if abs(self.matrix[0][0]*self.matrix[1][1]-self.matrix[0][1]*self.matrix[1][0]) < 1e-12:
            raise ValueError('singular correction matrix')
        self.ranges = config['measured_pose_ranges']
        self.limits = config['motor_limits_deg']
        for mapping in (self.ranges, self.limits):
            if set(mapping) != {'z', 'yaw', 'reach'}:
                raise ValueError('all measured ranges required')
            for bounds in mapping.values():
                if len(bounds) != 2 or not all(number(v) for v in bounds) or bounds[0] > bounds[1]:
                    raise ValueError('invalid measured range')

    def motor_pose(self, pose):
        values = (pose.z_mm, pose.yaw_deg, pose.reach_mm)
        for name, value in zip(('z', 'yaw', 'reach'), values):
            if not number(value) or not self.ranges[name][0] <= value <= self.ranges[name][1]:
                raise ValueError('pose outside measured calibration range: ' + name)
        motor = MotorPose(pose.z_mm*360/40*self.signs['z'],
                          pose.yaw_deg*3*self.signs['yaw'],
                          pose.reach_mm*360/113.1*self.signs['reach'])
        for name, value in zip(('z', 'yaw', 'reach'), (motor.z_deg, motor.yaw_deg, motor.reach_deg)):
            if not self.limits[name][0] <= value <= self.limits[name][1]:
                raise ValueError('motor pose exceeds verified limit: ' + name)
        return motor

    def mechanism_pose(self, arm):
        pose = MechanismPose(arm['z_deg']/9*self.signs['z'],
                             arm['yaw_deg']/3*self.signs['yaw'],
                             arm['reach_deg']*113.1/360*self.signs['reach'])
        self.motor_pose(pose)
        return pose

    def correct(self, pose, error, max_step_mm, max_yaw_step_deg):
        radial, tangent = [sum(row[i]*error[i] for i in (0, 1)) for row in self.matrix]
        length = math.hypot(radial, tangent)
        if not number(length) or length == 0:
            raise ValueError('invalid image correction')
        scale = min(1, max_step_mm/length)
        radial, tangent = radial*scale, tangent*scale
        radius = self.radius0 + pose.reach_mm
        if radius <= 0 or radius + radial <= 0:
            raise ValueError('invalid arm radius')
        # Limit along the same local correction vector, retaining radial geometry.
        angle = math.degrees(math.atan2(tangent, radius+radial))
        if abs(angle) > max_yaw_step_deg:
            lo, hi = 0., 1.
            for _ in range(32):
                mid = (lo+hi)/2
                if abs(math.degrees(math.atan2(tangent*mid, radius+radial*mid))) <= max_yaw_step_deg:
                    lo = mid
                else:
                    hi = mid
            radial, tangent = radial*lo, tangent*lo
        delta_yaw = math.degrees(math.atan2(tangent, radius+radial))
        target = MechanismPose(pose.z_mm, pose.yaw_deg+delta_yaw,
                               math.hypot(radius+radial, tangent)-self.radius0)
        self.motor_pose(target)
        return target, math.hypot(radial, tangent)
