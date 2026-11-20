import pytest

from tools.route_station_dwell import TimedStationWork, timed_station_test
from tools.chassis_button_service import make_test_station_work


class Arm:
    def __init__(self):
        self.a=dict(route_id=7,point_index=3,work_id=102,command_id=0,
            state_name='WAIT_VISION',origins_valid=True,fault=0,standalone_test_mode=False)
        self.finishes=[]

    def arm_status(self):return dict(self.a)

    def arm_finish(self,*args):
        self.finishes.append(args)
        self.a.update(command_id=args[-1],state_name='PARKING')


def status():return dict(route_id=7,point_index=3,work_id=102,state=3,phase=4)


def test_finish_and_real_parking_required_before_timed_release():
    arm=Arm();now=[0.]
    work=TimedStationWork(arm,dict(dwell_s=2),clock=lambda:now[0])
    work.start(status())
    assert work.poll(status()) is None
    assert arm.finishes==[(7,3,102,1)]
    now[0]=3
    assert work.poll(status()) is None  # dwell alone cannot release while PARKING
    arm.a['state_name']='READY_LEAVE'
    result=work.poll(status())
    assert result['mode']=='timed_dwell' and result['arm_parked']
    assert result['visual_alignment_confirmed'] is False
    assert not result['physical_grasp_completed'] and not result['physical_place_completed']
    assert len(arm.finishes)==1


def test_parking_early_still_waits_two_seconds():
    arm=Arm();now=[0.]
    work=TimedStationWork(arm,dict(dwell_s=2),clock=lambda:now[0])
    work.start(status());work.poll(status())
    arm.a['state_name']='READY_LEAVE';now[0]=1
    assert work.poll(status()) is None
    now[0]=2
    assert work.poll(status())['arm_parked']


@pytest.mark.parametrize('change',[dict(fault=2),dict(origins_valid=False),
    dict(route_id=8),dict(state_name='WAIT_GRIP'),dict(standalone_test_mode=True)])
def test_fault_or_wrong_context_never_finishes_or_releases(change):
    arm=Arm();arm.a.update(change)
    work=TimedStationWork(arm,{},clock=lambda:0)
    work.start(status())
    with pytest.raises(RuntimeError):work.poll(status())
    assert not arm.finishes and not work.results


def test_explicit_timed_mode_ignores_visual_config_without_mutating_it():
    config=dict(test_station_work=dict(mode='timed_dwell',dwell_s=2),
        turntable_work=dict(enabled=True,alignment=None),arm_alignment=None)
    work=make_test_station_work(Arm(),config,lambda **_:None)
    assert isinstance(work,TimedStationWork)
    assert config['turntable_work']['enabled']  # selection does not erase saved visual settings
    with pytest.raises(ValueError):make_test_station_work(Arm(),config,lambda **_:None,object())
    assert not timed_station_test({})
    with pytest.raises(ValueError):timed_station_test(dict(test_station_work=dict(mode='unknown')))


def test_preflight_queries_but_never_zeroes_or_moves():
    arm=Arm();arm.a['state_name']='READY'
    work=TimedStationWork(arm,{})
    work.preflight()
    arm.a['standalone_test_mode']=True
    with pytest.raises(RuntimeError):work.preflight()
    assert not arm.finishes
