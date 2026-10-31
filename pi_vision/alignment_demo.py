"""Software-only alignment plant; fabricated calibration never drives hardware."""

import argparse
import json
from .alignment import AlignmentCalibration, AlignmentController


def simulation_calibration():
    return AlignmentCalibration(calibration_id='SIMULATION_ONLY', camera_id='sim-camera',
        pose_id='sim-observation-pose', plane_id='sim-plane',
        point_kind='contour_bounding_box_center', hsv_profile_sha256='simulation-profile',
        width=1280, height=720, reference_pixel=(640., 360.),
        correction_matrix_mm_per_pixel=((.2, 0.), (0., .2)), simulation_only=True)


class AlignmentPlant:
    """Replies only to simulation=true. Pixel motion is an ideal synthetic model."""
    def __init__(self, failure=None):
        self.failure = failure
        self.vision = self.motion = None
        self.error = [80., -40.]
        self.sequence = self.adjustments = 0

    def command(self, channel, command):
        if command.get('simulation') is not True:
            raise ValueError('simulation plant refuses real commands')
        if channel == 'vision':
            self.vision = None if command['operation'] == 'stop' else command
        elif channel == 'motion':
            self.motion = command
        else:
            raise ValueError('unknown channel')

    @staticmethod
    def reply(command, status, data):
        return {k: command[k] for k in ('schema_version', 'simulation', 'run_id', 'command_id')} | {
            'status': status, 'data': data}

    def update(self, now):
        out = []
        if self.motion is not None:
            command, self.motion = self.motion, None
            op, p = command['operation'], command['params']
            if (self.failure == 'stop_timeout' and op == 'stop' or
                    self.failure in ('motion_timeout', 'stop_timeout') and op == 'adjust_xy'):
                return out
            if self.failure == 'motion_failed' and op == 'adjust_xy':
                out.append(('motion', self.reply(command, 'failed', {'reason': 'simulated_failure'})))
            else:
                if op == 'adjust_xy':
                    self.error[0] -= p['dx_mm']/.2
                    self.error[1] -= p['dy_mm']/.2
                    self.adjustments += 1
                out.append(('motion', self.reply(command, 'done', dict(stopped=True,
                    adjusted=op == 'adjust_xy', pose_id=p.get('pose_id'),
                    calibration_id=p.get('calibration_id')))))
        if self.vision is not None:
            self.sequence += 1
            c = simulation_calibration()
            lost = self.failure == 'target_lost' and self.adjustments > 0
            target = dict(color_id=self.vision['params']['color_id'], u=640.+self.error[0],
                          v=360.+self.error[1], point_kind=c.point_kind)
            data = dict(status='no_target' if lost else 'ok', valid=True,
                        acquired_unix_s=now, source_sequence=self.sequence,
                        source_session_id='sim-session', camera_id=c.camera_id,
                        camera_role='material', coordinate_frame='camera_image', units='pixels',
                        width=c.width, height=c.height, hsv_profile_sha256=c.hsv_profile_sha256,
                        zone='raw', targets=[] if lost else [target])
            out.append(('vision', self.reply(self.vision, 'observation', data)))
        return out


def simulate(failure=None):
    now = [1000.]
    engine = AlignmentController(simulation_calibration(), simulation=True,
                                 clock=lambda: now[0], wall_clock=lambda: now[0])
    plant = AlignmentPlant(failure)
    engine.start(1)
    trace = []
    for _ in range(500):
        engine.tick()
        for item in engine.drain_commands():
            trace.append(item)
            plant.command(item['channel'], item['command'])
        for channel, reply in plant.update(now[0]):
            engine.feedback(channel, reply)
        if engine.phase in ('aligned', 'fault', 'cancelled'):
            break
        now[0] += .1
    expected = 'fault' if failure else 'aligned'
    return dict(evidence='software_alignment_simulation_only',
                ok=engine.phase == expected and engine.stop_confirmed == (failure != 'stop_timeout'),
                state=engine.snapshot(), trace=trace)


def main():
    parser = argparse.ArgumentParser(description='局部对准纯软件模拟，不打开设备')
    parser.add_argument('--failure', choices=['target_lost', 'motion_failed', 'motion_timeout', 'stop_timeout'])
    args = parser.parse_args()
    report = simulate(args.failure)
    print(json.dumps(report, ensure_ascii=False, allow_nan=False))
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
