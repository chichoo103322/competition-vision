"""路线测试站点：只驻留并确认云台允许离站，不识别、对齐或抓放。"""
import math
import time


def timed_station_test(config):
    settings = config.get('test_station_work')
    if settings is None:
        return False
    if not isinstance(settings, dict) or settings.get('mode') != 'timed_dwell':
        raise ValueError('test_station_work must explicitly select timed_dwell')
    return True


class TimedStationWork:
    mode = 'timed_dwell'

    def __init__(self, client, settings, publish=lambda **_: None, *, clock=time.monotonic):
        self.client, self.clock, self.publish = client, clock, publish
        self.dwell_s = settings.get('dwell_s', 2.)
        self.motion_timeout_s = settings.get('motion_timeout_s', 45.)
        for value in (self.dwell_s, self.motion_timeout_s):
            if isinstance(value, bool) or not isinstance(value, (int,float)) or not math.isfinite(value) or value<=0:
                raise ValueError('positive finite test dwell and parking timeout required')
        self.results = []
        self.context = None

    def preflight(self):
        a = self.client.arm_status()
        if (a['state_name']!='READY' or not a['origins_valid'] or a['fault']
                or a.get('standalone_test_mode')):
            raise RuntimeError('路线测试需要整车模式、有效人工原点和READY云台；不自动清零')

    def start(self, status, qr_result=None):
        if status['state']!=3 or status['phase']!=4 or not 102<=status['work_id']<=107:
            raise RuntimeError('timed test station requires stopped WAIT_WORK/phase4')
        self.context = tuple(status[k] for k in ('route_id','point_index','work_id'))
        self.started_at = self.clock()
        self.command = None
        self.done = None
        self.publish(event='TEST_STATION_DWELL', work_id=status['work_id'],
            dwell_s=self.dwell_s, simulated_station_work=True,
            physical_grasp_completed=False, physical_place_completed=False)

    def poll(self, status):
        if (tuple(status[k] for k in ('route_id','point_index','work_id'))!=self.context
                or status['state']!=3 or status['phase']!=4):
            raise RuntimeError('route left test station before release')
        if self.done is not None:
            return self.done
        if self.clock()-self.started_at>self.motion_timeout_s+self.dwell_s:
            raise TimeoutError('test station parking/arrival feedback timeout')
        a = self.client.arm_status()
        if (tuple(a[k] for k in ('route_id','point_index','work_id'))!=self.context
                or not a['origins_valid'] or a['fault'] or a.get('standalone_test_mode')):
            raise RuntimeError('test station arm context/origin/fault/mode invalid: '+str(a))
        if self.command is not None and a['command_id']!=self.command:
            raise RuntimeError('test station FINISH was replaced')
        if a['state_name']=='READY_LEAVE':
            if self.clock()-self.started_at<self.dwell_s:
                return None
            self.done = dict(mode=self.mode, **dict(zip(
                ('route_id','point_index','work_id'),self.context)),
                simulated_station_work=True, arm_parked=True,
                visual_alignment_confirmed=False, physical_grasp_completed=False,
                physical_place_completed=False, dwell_s=self.dwell_s, arm=a)
            self.results.append(self.done)
            return self.done
        if self.command is None and a['state_name']=='WAIT_VISION':
            self.command = a['command_id']+1
            self.client.arm_finish(*self.context,self.command)
            self.publish(event='TEST_STATION_FINISH',work_id=status['work_id'],
                command_id=self.command, simulated_station_work=True)
        elif not (self.command is None and a['state_name']=='PREPARING'
                  or self.command is not None and a['state_name']=='PARKING'):
            raise RuntimeError('unexpected arm state at timed test station: '+a['state_name'])
        return None

    def close(self):
        # Route runner owns STOP on errors/cancellation. Never release a station here.
        pass
