import copy
from dataclasses import replace
import json

import numpy as np
import pytest

from tools.arm_three_color_trial import (
    ArmTrial, FeedbackLevel, ThreeColorTrial, TrialTuning, ObservationUnavailable,
    correction_target, edge_view_target, micro_start_ok, parse_order, tracking_approach,
    tray_approach, tray_correction, view_start_ok)


class Plant:
    def __init__(self):
        self.now = 100.
        self.seq = self.generation = 0
        self.color = 2
        self.calls = []
        self.visual_problem = None
        self.fail_lower = False
        self.initial_errors = {}
        self.arm = dict(route_id=0, point_index=0, work_id=0, command_id=0,
            aligned_command_id=0, state_name='READY', origins_valid=True,
            standalone_test_mode=True, fault=0, z_cdeg=0, yaw_cdeg=0, reach_cdeg=0,
            target_z_cdeg=0, target_yaw_cdeg=0, target_reach_cdeg=0)
        self.reference = dict(camera_source='/dev/v4l/by-id/test-camera',
            hsv_profile_sha256='TEST', resolution=[1280,720], origin_radius_mm=230,
            reference_pixel=[648,359])
        self.mapping = dict(reference=self.reference, matrix=[[0,-.22],[.22,0]])

    def sleep(self, seconds):
        self.now += seconds

    def status(self):
        return dict(state_name='EMPTY', fault=0)

    def arm_status(self):
        return dict(self.arm)

    def arm_test_begin(self, test_id, work):
        self.calls.append(('begin',test_id,work))
        self.arm.update(route_id=test_id, work_id=work, command_id=1, state_name='WAIT_VISION')

    def arm_align(self, route, point, work, cmd, z, yaw, reach):
        self.calls.append(('align', self.color, cmd, z,yaw,reach))
        self.arm.update(command_id=cmd, aligned_command_id=cmd, state_name='ALIGNED',
            z_cdeg=round(z*100), yaw_cdeg=round(yaw*100), reach_cdeg=round(reach*100),
            target_z_cdeg=round(z*100), target_yaw_cdeg=round(yaw*100), target_reach_cdeg=round(reach*100))

    def arm_pick(self, route, point, work, cmd, ref):
        self.calls.append(('pick',self.color,cmd,ref))
        assert ref == self.arm['aligned_command_id']
        self.arm.update(command_id=cmd,state_name='WAIT_GRIP',z_cdeg=-90000,target_z_cdeg=-90000)
        if self.fail_lower:
            self.arm.update(state_name='FAULT', fault=2, origins_valid=False)

    def arm_grip_done(self, route, point, work, cmd, ref):
        self.calls.append(('simulated_grip_done',self.color,cmd,ref))
        assert self.arm['state_name']=='WAIT_GRIP' and ref==self.arm['aligned_command_id']
        self.arm.update(command_id=cmd,state_name='PICKED',z_cdeg=0,target_z_cdeg=0)

    def select_color(self, color):
        # The next color may only be selected after returning to all three zero targets.
        assert all(self.arm[k]==0 for k in ('z_cdeg','yaw_cdeg','reach_cdeg'))
        self.calls.append(('select',color))
        self.color = color
        self.generation += 1

    def snapshot(self):
        self.seq += 1
        acquired = self.now-.01
        obs = dict(valid=True, generation=self.generation, capture_epoch=0,
                   acquired_monotonic_s=acquired, source_sequence=self.seq)
        dx,dy = self.initial_errors.get(self.color,(0,0))
        m = dict(color_id=self.color,track_id=1,
            center_x=round(648+dx-18*self.arm['yaw_cdeg']/300),
            center_y=round(359+dy+4.5*self.arm['reach_cdeg']*113.1/36000),
            point_kind='top_face_ellipse_center')
        state = dict(camera_source=self.reference['camera_source'],hsv_profile_sha256='TEST',
            preview=dict(source_width=1280,source_height=720),material_backend='HSVMaterialDetector',
            material_selection='unique_color_in_search_window',material_search_color=self.color,
            capture=dict(valid=True,acquired_monotonic_s=acquired,epoch=0),session_id='test-session',
            qr_session=dict(generation=self.generation), material_observation=obs,materials=[m])
        if self.visual_problem=='old_generation': obs['generation']-=1
        if self.visual_problem=='duplicate': obs['source_sequence']=1
        if self.visual_problem=='stale': obs['acquired_monotonic_s']=self.now-3
        if self.visual_problem=='wrong_color': m['color_id']=6
        if self.visual_problem=='side': m['point_kind']='contour_area_centroid'
        if self.visual_problem=='ambiguous': state['materials'].append(dict(m))
        return state


def trial(plant, tmp_path, **kwargs):
    return ThreeColorTrial(plant,plant.reference,plant.mapping,plant.snapshot,plant.select_color,
        tmp_path,simulate_servos=True,clock=lambda:plant.now,sleep=plant.sleep,**kwargs)


def test_pick_accepts_firmware_zero_reference_for_small_encoder_residual(tmp_path):
    plant = Plant()
    plant.arm.update(route_id=9, work_id=103, state_name='ALIGNED',
                     command_id=2, aligned_command_id=2, z_cdeg=1, target_z_cdeg=0)
    motion = ArmTrial(plant, tmp_path, TrialTuning(),
                      clock=lambda:plant.now, sleep=plant.sleep)
    motion.context = (9, 0, 103)
    motion.command = 2
    lower, upper = motion.lower_and_lift(plant.arm_status())
    assert lower['target_z_cdeg'] == -90000 and upper['target_z_cdeg'] == 0


def test_full_231_sequence_preserves_order_return_origin_and_simulation_flags(tmp_path):
    p = Plant()
    result = trial(p,tmp_path).run([2,3,1],999)
    assert [c[1] for c in p.calls if c[0]=='select']==[2,3,1]
    assert [c[1] for c in p.calls if c[0]=='pick']==[2,3,1]
    assert [c[1] for c in p.calls if c[0]=='simulated_grip_done']==[2,3,1]
    assert len(result['colors'])==3
    assert all(c['lowered_mm']==100 and c['returned_to_origin'] for c in result['colors'])
    assert all(not c['physical_grasp_completed'] and not c['physical_place_completed'] for c in result['colors'])
    assert result['final_arm']['state_name']=='ALIGNED'
    assert all(result['final_arm'][k]==0 for k in ('z_cdeg','yaw_cdeg','reach_cdeg'))
    assert result['simulated_servos'] and not result['physical_grasp_completed']


def test_no_real_grip_simulation_without_explicit_mode(tmp_path):
    p=Plant()
    with pytest.raises(ValueError,match='simulate-servos'):
        ThreeColorTrial(p,p.reference,p.mapping,p.snapshot,p.select_color,tmp_path)
    assert not p.calls


def test_three_colors_each_correct_then_reconfirm_before_lowering(tmp_path):
    p=Plant(); p.initial_errors={2:(22,21),3:(-20,18),1:(10,12)}
    result=trial(p,tmp_path).run([2,3,1],999)
    assert all(max(abs(v) for v in c['error_px'])<=8 for c in result['colors'])
    assert len([c for c in p.calls if c[0]=='align'])>6
    assert [c[1] for c in p.calls if c[0]=='pick']==[2,3,1]


def test_forward_unreachable_turntable_candidate_waits_then_accepts_without_moving(tmp_path):
    p=Plant();p.initial_errors={3:(-54,-26)}
    t=trial(p,tmp_path)
    t.context=(999,0,102)
    p.arm_test_begin(999,102)
    t.set_color(3)
    base_sleep=t.sleep
    def advance(seconds):
        base_sleep(seconds)
        if p.now>102:
            p.initial_errors[3]=(-20,18)
    t.sleep=advance
    obs=t.wait_reachable_target(timeout=8)
    assert obs['pixel']==[628,377]
    assert p.now>102
    assert not any(c[0] in ('align','pick') for c in p.calls)


def test_unreachable_target_timeout_never_issues_a_correction(tmp_path):
    p=Plant();p.initial_errors={3:(-54,-26)}
    t=trial(p,tmp_path)
    t.context=(999,0,102)
    p.arm_test_begin(999,102)
    t.set_color(3)
    with pytest.raises(TimeoutError,match='reachable'):
        t.wait_reachable_target(timeout=3)
    assert not any(c[0] in ('align','pick') for c in p.calls)


@pytest.mark.parametrize('problem',['old_generation','duplicate','stale','wrong_color','side','ambiguous'])
def test_bad_color_observation_never_allows_pick(problem,tmp_path):
    p=Plant(); p.visual_problem=problem
    t=trial(p,tmp_path)
    t.context=(999,0,102)
    p.arm_test_begin(999,102)
    t.set_color(2)
    with pytest.raises(TimeoutError):
        t.observe(t.clock(),timeout=2)
    assert not any(c[0]=='pick' for c in p.calls)


def test_fault_at_lowering_never_sends_simulated_grip_done_or_next_color(tmp_path):
    p=Plant();p.fail_lower=True
    with pytest.raises(RuntimeError,match='fault'):
        trial(p,tmp_path).run([2,3,1],999)
    assert [c[1] for c in p.calls if c[0]=='select']==[2]
    assert not any(c[0]=='simulated_grip_done' for c in p.calls)


def test_unreachable_forward_or_far_reach_is_rejected():
    p=Plant()
    with pytest.raises(RuntimeError,match='outside'):
        correction_target(p.arm,[648,330],[648,359],p.mapping['matrix'],230)
    a=copy.copy(p.arm);a['reach_cdeg']=-19000
    with pytest.raises(RuntimeError,match='outside'):
        correction_target(a,[648,430],[648,359],p.mapping['matrix'],230)


def test_recorded_red_four_pixel_error_accepts_without_extra_motion(tmp_path):
    # Replay the final run5 reach and (1,4)px residual, rather than relabeling it as failure.
    p=Plant();p.initial_errors={1:(1,114)}
    t=trial(p,tmp_path)
    t.context=(999,0,102)
    p.arm_test_begin(999,102)
    t.set_color(1)
    t.command=2
    p.arm.update(command_id=2,aligned_command_id=2,state_name='ALIGNED',reach_cdeg=-7759)
    a,obs,error=t.align_color()
    assert error==[1,4] and a['aligned_command_id']==2
    assert not any(c[0]=='align' for c in p.calls)


def test_retraction_beyond_old_25mm_limit_is_valid_but_60mm_limit_remains():
    p=Plant();a=dict(p.arm,reach_cdeg=-7759)
    target,_=correction_target(a,[649,363],[648,359],p.mapping['matrix'],230)
    assert -60 <= target[2]*113.1/36000 < -25
    with pytest.raises(RuntimeError,match='outside'):
        correction_target(a,[649,363],[648,359],p.mapping['matrix'],230,
                          tuning=TrialTuning(max_retraction_mm=25))
    a['reach_cdeg']=-19000
    with pytest.raises(RuntimeError,match='outside'):
        correction_target(a,[648,430],[648,359],p.mapping['matrix'],230)


def test_fast_tuning_uses_fewer_corrections_and_less_confirmation_time(tmp_path):
    strict=TrialTuning(tolerance_px=3,confirm_frames=7,confirm_span_s=.6,settle_s=.25,
        max_step_mm=2.5,max_yaw_step_deg=.6,max_retraction_mm=25)
    outcomes=[]
    for name,tuning in [('strict',strict),('fast',TrialTuning())]:
        out=tmp_path/name;out.mkdir()
        p=Plant();p.initial_errors={2:(35,60)}
        t=trial(p,out,tuning=tuning)
        t.context=(999,0,102)
        p.arm_test_begin(999,102)
        t.set_color(2)
        _,_,error=t.align_color()
        assert tuning.accepts(error)
        outcomes.append((len([c for c in p.calls if c[0]=='align']),p.now-100))
    assert outcomes[1][0] < outcomes[0][0]
    assert outcomes[1][1] < outcomes[0][1]


def test_fast_confirmation_still_requires_three_distinct_new_frames(tmp_path):
    p=Plant();t=trial(p,tmp_path)
    t.context=(999,0,102)
    p.arm_test_begin(999,102)
    t.set_color(2)
    floor=p.now
    obs=t.observe(floor,timeout=2)
    stamps=[f['acquired'] for f in obs['source_frames']]
    # At 20 Hz, all intermediate frames must participate in the jitter check.
    assert len(stamps) >= 5
    assert len({f['sequence'] for f in obs['source_frames']}) == len(stamps)
    assert min(stamps)>floor and stamps[-1]-stamps[0]>=.2-1e-9


class ScriptedTrial(ThreeColorTrial):
    def __init__(self,*args,pixels,**kwargs):
        super().__init__(*args,**kwargs)
        self.pixels=iter(pixels)

    def observe(self,floor,timeout=180):
        self.sleep(.5)
        point=next(self.pixels)
        return dict(pixel=list(point),track_id=1,pixel_time_s=self.clock()-.1,
                    stationary_for_mapping=True,velocity_reliable=False)


def scripted(plant,tmp_path,pixels,**kwargs):
    t=ScriptedTrial(plant,plant.reference,plant.mapping,plant.snapshot,plant.select_color,
        tmp_path,simulate_servos=True,clock=lambda:plant.now,sleep=plant.sleep,pixels=pixels,**kwargs)
    t.context=(999,0,102)
    plant.arm_test_begin(999,102)
    t.set_color(3)
    return t


def test_recorded_blue_error_growth_reacquires_then_continues_without_pick_or_stop(tmp_path):
    # First three samples are the actual failing trace; later samples simulate target recovery.
    p=Plant()
    t=scripted(p,tmp_path,[[723,437],[711,426],[721,429],[721,429],[648,359]])
    a,_,error=t.align_color()
    assert error==[0,0] and a['state_name']=='ALIGNED'
    text=(tmp_path/'events.jsonl').read_text()
    assert 'visual_reacquire' in text
    assert 'pixel error increased after correction' not in text
    assert len([c for c in p.calls if c[0]=='align'])==3
    assert not any(c[0] in ('pick','simulated_grip_done') for c in p.calls)


def test_runtime_mapping_uses_only_real_short_motion_samples_and_keeps_source_unchanged(tmp_path):
    class ShortMotionPlant(Plant):
        polls=0
        moving=False
        def arm_align(self,*args):
            super().arm_align(*args)
            self.moving=True;self.polls=0
        def arm_status(self):
            a=super().arm_status()
            if self.moving:
                self.polls+=1
                if self.polls==1:a.update(state_name='ALIGNING',active_axis=1)
                else:self.moving=False
            return a
    p=ShortMotionPlant();p.initial_errors={2:(35,60)}
    p.mapping['matrix']=[[0,-.27],[.27,0]]
    saved=copy.deepcopy(p.mapping)
    t=trial(p,tmp_path)
    t.context=(999,0,102);p.arm_test_begin(999,102);t.set_color(2)
    _,_,error=t.align_color()
    assert t.tuning.accepts(error)
    assert '"updated": true' in (tmp_path/'events.jsonl').read_text()
    assert p.mapping==saved


def test_stalled_target_reobserves_instead_of_ending_the_color(tmp_path):
    p=Plant()
    t=scripted(p,tmp_path,[[668,379]]*3+[[648,359]])
    _,_,error=t.align_color()
    assert error==[0,0]
    assert 'no_progress' in (tmp_path/'events.jsonl').read_text()


def test_budget_exhaustion_holds_without_more_corrections_until_target_arrives(tmp_path):
    p=Plant()
    t=scripted(p,tmp_path,[[683,419],[673,399],[648,359]],
               tuning=TrialTuning(max_adjustments=1))
    _,_,error=t.align_color()
    assert error==[0,0]
    assert len([c for c in p.calls if c[0]=='align'])==1
    assert 'holding_for_target' in (tmp_path/'events.jsonl').read_text()


def test_temporary_visual_loss_preserves_the_color_and_waits_for_fresh_target(tmp_path):
    p=Plant()
    t=trial(p,tmp_path)
    t.context=(999,0,102)
    p.arm_test_begin(999,102)
    t.set_color(3)
    original=t.observe
    def temporarily_missing(floor,timeout=180):
        if p.now<109:
            p.sleep(2)
            raise ObservationUnavailable('target temporarily hidden')
        return original(floor,timeout)
    t.observe=temporarily_missing
    _,_,error=t.align_color()
    assert error==[0,0]
    assert 'waiting_target' in (tmp_path/'events.jsonl').read_text()
    assert [c[1] for c in p.calls if c[0]=='select']==[3]


def test_mcu_communication_timeout_is_never_misclassified_as_visual_retry(tmp_path):
    p=Plant();t=trial(p,tmp_path)
    t.context=(999,0,102);p.arm_test_begin(999,102);t.set_color(3)
    def offline():raise TimeoutError('MCU offline')
    p.arm_status=offline
    with pytest.raises(TimeoutError,match='MCU offline'):t.align_color()
    assert not any(c[0] in ('align','pick') for c in p.calls)


def test_camera_capture_interruption_waits_but_identity_change_still_fails(tmp_path):
    p=Plant();t=trial(p,tmp_path)
    t.context=(999,0,102);p.arm_test_begin(999,102);t.set_color(3)
    base=p.snapshot
    def interrupted():
        state=base()
        if p.now<101:state['capture']['valid']=False
        return state
    t.snapshot=interrupted
    obs=t.observe(p.now,timeout=3)
    assert obs['pixel']==[648,359] and p.now>=101
    def replaced():
        state=base();state['camera_source']='another-camera';return state
    t.snapshot=replaced
    with pytest.raises(RuntimeError,match='identity'):t.observe(p.now,timeout=1)


@pytest.mark.parametrize('kwargs',[
    dict(tolerance_px=float('nan')),dict(tolerance_px=0),dict(confirm_frames=2),
    dict(confirm_span_s=.1),dict(max_step_mm=4),dict(max_yaw_step_deg=1),
    dict(max_retraction_mm=61),dict(max_retraction_mm=-1),dict(settle_s=-.1)])
def test_tuning_cannot_bypass_frame_confirmation_or_firmware_micro_corridor(kwargs):
    with pytest.raises(ValueError):TrialTuning(**kwargs)


@pytest.mark.parametrize('bad',['','22','222','2311','230','abc'])
def test_invalid_operator_color_order_rejected(bad):
    with pytest.raises(ValueError):parse_order(bad)


@pytest.mark.parametrize('fault', [
    None, 'skipped_count', 'uncertain', 'wrong_park', 'wrong_mode', 'nonempty',
    'wrong_drop', 'wrong_horizontal', 'changed_drop', 'fixed_wrong_drop',
    'one_peak', 'repeated_peak', 'hard_peak', 'changed_tray_target',
    'tray_forward_rebound', 'tray_forward_repeat',
])
@pytest.mark.parametrize('drop_cdeg', [45000, 45950, 72000, 90000, 90950])
@pytest.mark.parametrize('tray_reach', [-31830, -36605], ids=['tray_100mm', 'tray_115mm'])
def test_v7_three_colors_require_confirmed_load_and_end_ready_leave(
        tmp_path, fault, drop_cdeg, tray_reach):
    fixed_drop_mm = round(drop_cdeg / 900) if drop_cdeg not in (45950, 90950) else 50

    class TrayPlant(Plant):
        def __init__(self):
            super().__init__()
            self.arm.update(tray_enabled=fault != 'wrong_mode', tray_uncertain=False,
                            tray_count=int(fault == 'nonempty'), grip_done_required=True)
            self.transfer_steps = []

        def arm_pick(self, route, index, work, command, reference):
            super().arm_pick(route, index, work, command, reference)
            target_z = -drop_cdeg
            if fault == 'wrong_drop':
                target_z = -63000
            elif fault == 'fixed_wrong_drop':
                target_z = -45950 if fixed_drop_mm == 50 else -45000
            self.arm.update(z_cdeg=target_z, target_z_cdeg=target_z)
            if fault == 'changed_drop':
                alternate_z = -90000 if drop_cdeg == 72000 else -72000
                self.arm.update(state_name='LOWERING', z_cdeg=-2)
                self.transfer_steps = [
                    dict(), dict(state_name='WAIT_GRIP', z_cdeg=alternate_z, target_z_cdeg=alternate_z)
                ]
            elif target_z in (-45950, -90950):
                self.arm.update(state_name='LOWERING', z_cdeg=0)
                self.transfer_steps = [
                    dict(), dict(z_cdeg=target_z - 250),
                    dict(state_name='WAIT_GRIP', z_cdeg=target_z)
                ]
            if fault == 'wrong_horizontal':
                self.arm['target_yaw_cdeg'] += 1

        def arm_grip_done(self, route, index, work, command, reference):
            assert self.arm['state_name'] == 'WAIT_GRIP'
            assert reference == self.arm['aligned_command_id']
            self.calls.append(('lift', self.color))
            count = self.arm['tray_count']
            self.arm.update(command_id=command, state_name='LIFTING', target_z_cdeg=0)
            tray_yaw = 36000 if drop_cdeg in (45000, 45950, 72000) and tray_reach == -31830 else 45000
            self.transfer_steps = [
                dict(state_name='LIFTING', z_cdeg=-(drop_cdeg // 2)),
                dict(state_name='TRAY_LOADING', active_axis=1, z_cdeg=-1, yaw_cdeg=tray_yaw + 83,
                     reach_cdeg=-19095, target_yaw_cdeg=tray_yaw, target_reach_cdeg=tray_reach),
                dict(state_name='TRAY_LOADING', active_axis=2, z_cdeg=-1,
                     reach_cdeg=tray_reach - 96, yaw_cdeg=tray_yaw + 28),
                dict(state_name='TRAY_LOADING', active_axis=255, z_cdeg=0,
                     reach_cdeg=tray_reach, yaw_cdeg=tray_yaw),
                dict(state_name='TRAY_LOADING', tray_count=count + (2 if fault == 'skipped_count' else 1),
                     tray_uncertain=fault == 'uncertain'),
                dict(active_axis=2, target_yaw_cdeg=0, target_reach_cdeg=-19098,
                     reach_cdeg=tray_reach - 50),
                dict(state_name='PICKED', yaw_cdeg=0, reach_cdeg=-15000 if fault == 'wrong_park' else -19098,
                     target_yaw_cdeg=0, target_reach_cdeg=-19098),
            ]

            if fault in ('one_peak', 'repeated_peak', 'hard_peak'):
                peak = dict(reach_cdeg=tray_reach - (201 if fault == 'hard_peak' else 188))
                self.transfer_steps[3:3] = [peak] * (2 if fault == 'repeated_peak' else 1)
            if fault == 'changed_tray_target':
                self.transfer_steps[2]['target_reach_cdeg'] = -31830 if tray_reach == -36605 else -36605
            if fault in ('tray_forward_rebound', 'tray_forward_repeat'):
                rebound = [dict(reach_cdeg=1000)] * 3 if fault == 'tray_forward_repeat' else [dict(reach_cdeg=203)]
                self.transfer_steps[4:4] = [*rebound, dict(reach_cdeg=tray_reach)]

        def arm_status(self):
            if self.transfer_steps:
                self.arm.update(self.transfer_steps.pop(0))
            return super().arm_status()

        def arm_finish(self, route, index, work, command):
            assert self.arm['state_name'] == 'PICKED' and self.arm['tray_count'] == 3
            self.calls.append(('finish', self.color))
            self.arm.update(command_id=command, state_name='READY_LEAVE')

    plant = TrayPlant()
    runner = ThreeColorTrial(plant, plant.reference, plant.mapping, plant.snapshot,
                             plant.select_color, tmp_path, real_tray=True,
                             pick_drop_mm=fixed_drop_mm if fault == 'fixed_wrong_drop' else None,
                             clock=lambda: plant.now, sleep=plant.sleep)
    if fault and fault not in ('one_peak', 'tray_forward_rebound'):
        with pytest.raises(RuntimeError, match={
            'repeated_peak': 'tray-transfer envelope', 'hard_peak': 'tray-transfer envelope',
            'skipped_count': 'tray count', 'uncertain': 'uncertain tray', 'wrong_park': 'arrival',
            'wrong_mode': 'tray flag', 'nonempty': 'tray count',
            'wrong_drop': 'Z-only target mismatch', 'wrong_horizontal': 'horizontal target',
            'changed_drop': 'Z-only target mismatch', 'fixed_wrong_drop': 'Z-only target mismatch',
            'changed_tray_target': 'tray reach target changed',
            'tray_forward_repeat': 'hard trial envelope in TRAY_LOADING',
        }[fault]):
            runner.run([2, 3, 1], 200)
        assert not runner.results and (not plant.calls if fault in ('wrong_mode', 'nonempty')
                                       else [entry for entry in plant.calls if entry[0] == 'select'] == [('select', 2)])
        if fault in ('wrong_drop', 'wrong_horizontal', 'changed_drop', 'fixed_wrong_drop'):
            assert not any(entry[0] == 'lift' for entry in plant.calls)
    else:
        result = runner.run([2, 3, 1], 200)
        assert [c['tray_count'] for c in result['colors']] == [1, 2, 3]
        assert [c['lowered_mm'] for c in result['colors']] == [drop_cdeg / 900] * 3
        assert [c['firmware_pick_drop_mm'] for c in result['colors']] == [drop_cdeg / 900] * 3
        assert [c['returned_to_origin'] for c in result['colors']] == [True, True, False]
        assert result['simulated_servos'] is False and result['physical_grasp_completed'] is False
        assert result['final_arm']['state_name'] == 'READY_LEAVE'
        assert plant.calls[-1] == ('finish', 1)
        if fault == 'tray_forward_rebound':
            events = [json.loads(line) for line in (tmp_path/'events.jsonl').read_text().splitlines()]
            assert any(e['event'] == 'envelope_warning' and e['phase'] == 'TRAY_LOADING' for e in events)


@pytest.mark.parametrize('case', ['reported_transition', 'batched_zero', 'stale_ready'])
def test_v7_key_wait_handles_short_zeroing_without_accepting_old_ready(tmp_path, case):
    from collections import deque

    class KeyPlant(Plant):
        def __init__(self):
            super().__init__()
            self.arm.update(tray_enabled=True, tray_uncertain=False, tray_count=0,
                            grip_done_required=True, opcode=1)
            ready = dict(self.arm)
            need_origin = dict(ready, state_name='NEED_ORIGIN', origins_valid=False, opcode=0)
            zeroing = dict(ready, state_name='ZEROING', origins_valid=False, opcode=0)
            self.batches = {
                'reported_transition': [[need_origin], [ready]],
                'batched_zero': [[ready], [zeroing, ready]],
                'stale_ready': [[zeroing, ready], [ready]],
            }[case]
            self.arm_generation = 0
            self.arm_events = deque(maxlen=256)

        def arm_status(self):
            batch = self.batches.pop(0) if self.batches else [dict(self.arm)]
            self.arm_generation += len(batch)
            self.arm_events.extend(batch)
            return batch[-1]

    plant = KeyPlant()
    runner = ThreeColorTrial(plant, plant.reference, plant.mapping, plant.snapshot,
                             plant.select_color, tmp_path, real_tray=True,
                             clock=lambda: plant.now, sleep=plant.sleep)
    if case == 'stale_ready':
        with pytest.raises(TimeoutError, match='manual-key wait expired'):
            runner.wait_for_manual_ready(timeout=.5)
    else:
        assert runner.wait_for_manual_ready(timeout=.5)['state_name'] == 'READY'
    assert plant.calls == []


@pytest.mark.parametrize('changed,allowed', [
    (dict(yaw_cdeg=45180), True), (dict(yaw_cdeg=45181), False),
    (dict(active_axis=2), False), (dict(target_yaw_cdeg=0), False),
    (dict(target_reach_cdeg=-37000), False),
    (dict(reach_cdeg=-31830), False),
])
def test_tray_yaw_feedback_margin_only_applies_to_outbound_at_clearance(changed, allowed):
    feedback = dict(active_axis=1, z_cdeg=-3, yaw_cdeg=45085, reach_cdeg=-19100,
                    target_z_cdeg=0, target_yaw_cdeg=45000, target_reach_cdeg=-31830)
    feedback.update(changed)
    if allowed:
        ThreeColorTrial.check_tray_transfer_pose(feedback)
    else:
        with pytest.raises(RuntimeError, match='tray-transfer envelope|unexpected v7 tray approach target'):
            ThreeColorTrial.check_tray_transfer_pose(feedback)

@pytest.mark.parametrize('changed,allowed', [
    ({}, True), (dict(reach_cdeg=-32010), True), (dict(reach_cdeg=-32011), False),
    (dict(active_axis=1), False), (dict(yaw_cdeg=35939), False),
    (dict(target_reach_cdeg=-19098), False), (dict(target_yaw_cdeg=42000), False),
])
def test_tray_reach_feedback_margin_requires_correct_axis_and_settled_yaw(changed, allowed):
    feedback = dict(active_axis=2, z_cdeg=-1, yaw_cdeg=36028, reach_cdeg=-31926,
                    target_z_cdeg=0, target_yaw_cdeg=36000, target_reach_cdeg=-31830)
    feedback.update(changed)
    if allowed:
        ThreeColorTrial.check_tray_transfer_pose(feedback)
    else:
        with pytest.raises(RuntimeError, match='tray-transfer envelope|unexpected v7 tray approach target'):
            ThreeColorTrial.check_tray_transfer_pose(feedback)


@pytest.mark.parametrize('photo_return, readings, axis, reported_target, final_reach, error', [
    (True, [138], 2, {}, 35, None),
    (True, [-19093, 135, 203, 190], 2, {'target_z_cdeg': -1}, 35, None),
    (True, [203] * 6, 2, {}, 35, None),
    (True, [1500, 1500, 200, 1500, 1500], 2, {}, 35, None),
    (True, [(1, 1500), (1, 1500), (1, 1500)], 2, {}, 35, None),
    (True, [1500, 1550, 1600], 2, {}, 35, 'hard trial envelope'),
    (True, [203], 1, {}, 35, None),
    (True, [203], 2, {'target_reach_cdeg': -100}, 35, 'photo return changed requested pose'),
    (True, [203], 2, {'target_yaw_cdeg': 1}, 35, 'photo return changed requested pose'),
    (True, [203], 2, {'target_z_cdeg': -136}, 35, 'photo return changed requested pose'),
    (False, [203], 2, {}, 35, None),
    (True, [203], 2, {'target_z_cdeg': -1}, 100, 'arrival differs'),
    (True, [203], 2, {}, 1500, 'hard trial envelope'),
])
def test_photo_return_allows_only_bounded_transient_forward_feedback(
        tmp_path, photo_return, readings, axis, reported_target, final_reach, error):
    plant = Plant()
    plant.arm.update(state_name='PICKED', route_id=200, work_id=102, command_id=4,
                     reach_cdeg=-19098, tray_enabled=True, tray_uncertain=False,
                     tray_count=1, grip_done_required=True)
    runner = ThreeColorTrial(plant, plant.reference, plant.mapping, plant.snapshot,
                             plant.select_color, tmp_path, real_tray=True,
                             clock=lambda: plant.now, sleep=plant.sleep)
    runner.context = (200, 0, 102)
    runner.command = 4
    parked = plant.arm_status()
    z_feedback = -1 if reported_target.get('target_z_cdeg') == -1 else 0
    feedback = iter([
        *(dict(state_name='ALIGNING', active_axis=axis, z_cdeg=z_feedback, reach_cdeg=reading,
               tick=tick, aligned_command_id=0, **reported_target)
          for tick, reading in (value if isinstance(value, tuple) else (i, value)
                                for i, value in enumerate(readings, 1))),
        dict(state_name='ALIGNED', active_axis=255, z_cdeg=z_feedback,
             reach_cdeg=final_reach, **reported_target),
    ])
    plant.arm_status = lambda: dict(plant.arm, **next(feedback))

    if error:
        with pytest.raises(RuntimeError, match=error):
            runner.align_to([0, 0, 0], before=parked, photo_return=photo_return)
    else:
        arrived = runner.align_to([0, 0, 0], before=parked, photo_return=photo_return)
        assert arrived['reach_cdeg'] == final_reach and arrived['tray_count'] == 1
        events = [json.loads(line) for line in (tmp_path/'events.jsonl').read_text().splitlines()]
        assert sum(e['event'] == 'envelope_warning' for e in events) == int(readings != [138])


def test_photo_view_accepts_static_residual_without_calling_it_firmware_micro_motion():
    arm = dict(z_cdeg=40, yaw_cdeg=0, reach_cdeg=203)
    assert view_start_ok(arm)
    assert not micro_start_ok(arm)
    assert not view_start_ok(dict(arm, reach_cdeg=956))
    assert not view_start_ok(dict(arm, z_cdeg=-136))


@pytest.mark.parametrize('target', [[0, 3601, 0], [0, 0, 1], [0, 0, -19099], [-10, 0, 0]])
def test_relaxed_feedback_cannot_relax_outgoing_photo_pose(tmp_path, target):
    plant = Plant()
    t = real_photo_trial(plant, tmp_path, (0, 0))
    with pytest.raises(RuntimeError, match='photo-height command limits'):
        t.align_to(target)
    assert not any(call[0] == 'align' for call in plant.calls)


def real_photo_trial(plant, tmp_path, error, axes=(210, 220)):
    plant.initial_errors[2] = error
    plant.arm.update(tray_enabled=True, tray_uncertain=False, grip_done_required=True)

    def snapshot():
        state = plant.snapshot()
        # Recorded source cadence: one processed frame about every 0.2 s.
        seq = int(plant.now * 5)
        state['material_observation'].update(source_sequence=seq, acquired_monotonic_s=seq / 5)
        for item in state['materials']:
            item['top_face_axes'] = axes
        return state

    t = ThreeColorTrial(plant, plant.reference, plant.mapping, snapshot, plant.select_color,
                        tmp_path, real_tray=True, clock=lambda:plant.now, sleep=plant.sleep)
    plant.arm_test_begin(999, 102)
    t.context = (999, 0, 102)
    t.set_color(2)
    return t


@pytest.mark.parametrize('error, expected_moves', [((15, 26), 0), ((24, 35), 1), ((47, 78), 4)])
def test_real_tray_grasp_window_and_distant_approach(tmp_path, error, expected_moves):
    p = Plant()
    t = real_photo_trial(p, tmp_path, error)
    p.arm.update(state_name='ALIGNED', aligned_command_id=1)
    original_align = p.arm_align
    def timed_align(*args):
        original_align(*args)
        p.sleep(.65)
    p.arm_align = timed_align
    arm, obs, _ = t.align_color()
    assert len([c for c in p.calls if c[0]=='align']) == expected_moves
    assert obs['in_grasp_window'] and obs['grasp_radius_px'] == 32
    assert all(f['acquired'] > 100 + .65 * expected_moves for f in obs['source_frames'])
    if error != (47, 78):
        assert p.now - 100 <= 2
    assert arm['aligned_command_id'] == t.command


@pytest.mark.parametrize('axes,offset,radius,accepted', [
    ((40,90), (10,0), 10, True), ((40,90), (8,8), 10, False),
    (None, (9,0), 8, False), ((200,200), (25,25), 32, False),
])
def test_real_grasp_radius_depends_on_face_and_radial_offset(tmp_path,axes,offset,radius,accepted):
    t = real_photo_trial(Plant(), tmp_path, offset, axes)
    obs = t.observe(t.clock(), timeout=2)
    assert obs['grasp_radius_px'] == radius
    assert obs['in_grasp_window'] is accepted


def test_logged_blue_edge_pose_requires_reachable_part_of_face(tmp_path):
    p = Plant()
    t = real_photo_trial(p, tmp_path, (123,84))
    assert t.wait_reachable_target(timeout=2)['pixel'] == [771,443]
    pose, _ = tray_approach(p.arm, (123,84), 230, p.mapping['matrix'], t.tuning, 32)
    assert 1800 < pose[1] < 3600 and -19098 <= pose[2] <= 0
    assert tray_approach(p.arm, (275,84), 230, p.mapping['matrix'], t.tuning, 32) is None
    assert not any(c[0] in ('align','pick') for c in p.calls)


def test_real_grasp_checks_each_fresh_frame_at_edge(tmp_path):
    p = Plant()
    t = real_photo_trial(p, tmp_path, (31,0))
    original = t.snapshot
    def crossing_window():
        state = original()
        if state['material_observation']['source_sequence'] % 3 == 0:
            state['materials'][0]['center_x'] += 2
        return state
    t.snapshot = crossing_window
    obs = t.observe(p.now, timeout=2)
    assert obs['pixel'] == [679,359]
    assert not obs['in_grasp_window']


def test_real_tray_disc_edge_yellow_approaches_then_rechecks(tmp_path):
    plant = Plant()
    t = real_photo_trial(plant, tmp_path, (180,139))
    arm, obs, _ = t.align_color()
    moves = [c for c in plant.calls if c[0] == 'align']
    assert moves and 1800 < moves[0][4] * 100 <= 3600
    assert t.first_reachable < obs['source_frames'][0]['acquired']
    assert obs['in_grasp_window'] and arm['state_name'] == 'ALIGNED'
    assert t.motion_feedback_level(dict(arm, yaw_cdeg=3700)) == FeedbackLevel.SOFT
    with pytest.raises(RuntimeError, match='hard trial envelope'):
        t.envelope(dict(arm, yaw_cdeg=4351))


def test_missing_yellow_rim_explains_wait_without_motion(tmp_path):
    plant = Plant()
    t = real_photo_trial(plant, tmp_path, (180,139))
    snapshot = t.snapshot
    def no_rim():
        state = snapshot()
        return dict(state, materials=[], material_localization_status='top_face_not_confirmed')
    t.snapshot = no_rim
    with pytest.raises(ObservationUnavailable, match='top_face_not_confirmed'):
        t.wait_reachable_target(timeout=1)
    assert not any(c[0] in ('align','pick') for c in plant.calls)


def test_consistently_moving_disc_target_can_approach_but_cannot_pick(tmp_path):
    p = Plant()
    t = real_photo_trial(p, tmp_path, (50, 20))
    snapshot = t.snapshot
    def sliding():
        state = snapshot()
        seq = state['material_observation']['source_sequence']
        state['materials'][0]['center_x'] += 3 * (seq - 500)
        return state
    t.snapshot = sliding
    obs = t.wait_reachable_target(timeout=2)
    assert obs['velocity_reliable'] and not obs['in_grasp_window']
    assert not any(c[0] == 'pick' for c in p.calls)


def test_disc_target_traversing_grasp_window_waits_for_still_frames(tmp_path):
    p = Plant()
    t = real_photo_trial(p, tmp_path, (0, 0))
    snapshot = t.snapshot

    def sliding():
        state = snapshot()
        seq = state['material_observation']['source_sequence']
        state['materials'][0]['center_x'] += 2 * (seq - 500)
        return state

    t.snapshot = sliding
    with pytest.raises(ObservationUnavailable, match='target moving inside grasp window'):
        t.observe(p.now, timeout=1)


def test_disc_target_with_reliable_low_speed_can_align_for_pick(tmp_path):
    plant = Plant()
    t = real_photo_trial(plant, tmp_path, (0, 0))
    plant.arm.update(state_name='ALIGNED', aligned_command_id=1)
    snapshot = t.snapshot

    def barely_moving():
        state = snapshot()
        seq = state['material_observation']['source_sequence']
        state['materials'][0]['center_x'] += .5 * (seq - 500)
        return state

    t.snapshot = barely_moving
    _, obs, _ = t.align_color()
    assert obs['in_grasp_window'] and obs['velocity_reliable']
    assert np.linalg.norm(obs['velocity_px_s']) == pytest.approx(2.5)
    assert not any(c[0] in ('align', 'pick') for c in plant.calls)


def test_grasp_window_edge_tracks_nearby_flicker_outside_grasp_radius(tmp_path):
    plant = Plant()
    t = real_photo_trial(plant, tmp_path, (85, 100))
    snapshot = t.snapshot

    def flicker():
        state = snapshot()
        seq = state['material_observation']['source_sequence']
        state['materials'][0]['center_x'] += (0, 7, 2)[seq % 3]
        state['materials'][0]['track_id'] = seq
        return state

    t.snapshot = flicker
    obs = t.wait_reachable_target(timeout=1)
    assert not obs['in_grasp_window']
    assert {point[0] for point in obs['points']} == {733, 735, 740}
    assert len({frame['sequence'] for frame in obs['source_frames']}) == 3


def test_grasp_window_accepts_small_flicker_and_fresh_tracking_ids(tmp_path):
    plant = Plant()
    t = real_photo_trial(plant, tmp_path, (0, 0))
    snapshot = t.snapshot

    def flicker():
        state = snapshot()
        seq = state['material_observation']['source_sequence']
        state['materials'][0].update(
            center_x=652 + (10 if seq % 3 == 0 else 0), center_y=368, track_id=seq)
        return state

    t.snapshot = flicker
    obs = t.observe(plant.now, timeout=1)
    assert obs['in_grasp_window'] and not obs['velocity_reliable']
    assert len({frame['sequence'] for frame in obs['source_frames']}) >= 3
    assert {x for x, _ in obs['points']} == {652, 662}


def test_blue_just_forward_can_approach_sideways_but_cannot_chase_past_boundary():
    matrix = [[.000165,-.222586],[.220358,-.012505]]
    arm = Plant().arm
    plan = tray_approach(arm, np.array([68,-31]), 230, matrix, TrialTuning(), 32)
    assert plan and plan[0][1] > 0 and plan[0][2] == 0
    assert tray_approach(dict(arm, yaw_cdeg=plan[0][1]), np.array([0,-36]),
                         230, matrix, TrialTuning(), 32) is None


def test_wait_before_any_motion_does_not_shrink_first_tray_step(tmp_path):
    import json
    plant = Plant()
    t = real_photo_trial(plant, tmp_path, (24,35))
    get_target = t.wait_reachable_target
    def late_target(timeout):
        if plant.now < 103:
            plant.sleep(timeout)
            raise ObservationUnavailable('requested_color_not_seen')
        return get_target(timeout)
    t.wait_reachable_target = late_target
    t.align_color()
    events = [json.loads(line) for line in (tmp_path/'events.jsonl').read_text().splitlines()]
    assert sum(e['event'] == 'waiting_target' for e in events) == 2
    assert next(e['step_gain'] for e in events if e['event']=='servo_correction') == 1.


def test_failure_samples_are_bounded_unique_and_capture_errors_allow_recovery(tmp_path):
    plant = Plant()
    t = real_photo_trial(plant, tmp_path, (0,0))
    take = t.snapshot
    def missing():
        state = take()
        if plant.now < 113:
            state.update(materials=[], material_localization_status='top_face_not_confirmed')
        return state
    t.snapshot = missing
    captured = []
    def save(key, path):
        captured.append((key, path.name))
        if len(captured) == 1:
            raise OSError('frame expired')
    t.save_failure = save
    arm, obs, _ = t.align_color()
    assert arm['state_name'] == 'ALIGNED' and obs['in_grasp_window']
    assert len(captured) == len(set(captured)) == 5
    assert len({x[1] for x in captured}) == 5 and t.failure_samples == 4


def test_timed_planner_uses_short_moves_until_clearance_is_cheaper():
    p = Plant()
    def choose(error, z=0):
        return tray_correction(dict(p.arm,z_cdeg=z), np.array(error), 230, p.mapping['matrix'],
                               TrialTuning(), 32, allow_coarse=True)
    assert not choose((48,75))[2]
    assert choose((48,75), 2)[2]
    assert choose((48,168))[2]


def test_edge_view_computes_retraction_from_pixel_and_current_pose():
    plant = Plant()
    tuning = TrialTuning()
    shallow, _ = edge_view_target(
        plant.arm, [678, 571], plant.reference, plant.mapping, tuning, visited=set())
    deep, distance = edge_view_target(
        plant.arm, [678, 660], plant.reference, plant.mapping, tuning, visited=set())
    assert shallow[:2] == deep[:2] == [0, 0]
    assert deep[2] < shallow[2] < 0
    assert distance == pytest.approx(.8 * .22 * (660 - .69 * 720), abs=.002)
    shifted, _ = edge_view_target(
        dict(plant.arm, reach_cdeg=-3200, yaw_cdeg=500), [678, 660],
        plant.reference, plant.mapping, tuning, visited=set())
    assert shifted == [0, 500, deep[2] - 3200]
    stronger, _ = edge_view_target(
        plant.arm, [678, 660], plant.reference, plant.mapping, tuning,
        visited={(0, 10, deep[2] + 30)})
    assert stronger[2] < deep[2]
    assert edge_view_target(
        dict(plant.arm, z_cdeg=-136), [678, 660], plant.reference, plant.mapping,
        tuning, visited=set()) is None


def test_edge_view_obeys_limits_and_waits_when_pixels_cannot_improve():
    plant = Plant()
    tuning = TrialTuning(max_retraction_mm=20)
    point = [1035, 699]
    target, _ = edge_view_target(
        plant.arm, point, plant.reference, plant.mapping, tuning, visited=set())
    assert target == [0, 3600, -6366]
    at_limit = dict(plant.arm, yaw_cdeg=target[1], reach_cdeg=target[2])
    assert edge_view_target(
        at_limit, point, plant.reference, plant.mapping, tuning, visited=set()) is None
    assert edge_view_target(
        plant.arm, [648, 359], plant.reference, plant.mapping, tuning, visited=set()) is None


@pytest.mark.parametrize('offset', [(233, 226), (-170, 180)])
def test_wide_tray_limits_allow_asymmetric_approach_after_opt_in(tmp_path, offset):
    plant = Plant()
    t = real_photo_trial(plant, tmp_path, offset)
    error = np.asarray(offset)
    matrix = [[.000165, -.222586], [.220358, -.012505]]
    assert tray_approach(plant.arm, error, 230, matrix, t.tuning, 18.) is None
    t.tuning = replace(t.tuning, wide_tray_yaw=True)
    target, _ = tray_approach(plant.arm, error, 230, matrix, t.tuning, 18.)
    yaw = target[1] / 300
    assert -12 <= yaw < -6 if offset[0] < 0 else 12 < yaw <= 22
    t.envelope(dict(plant.arm, yaw_cdeg=target[1], reach_cdeg=target[2]))
    for outside in (-3691, 6691):
        assert t.motion_feedback_level(dict(plant.arm, yaw_cdeg=outside)) == FeedbackLevel.SOFT
    for outside in (-4400, 7400):
        with pytest.raises(RuntimeError, match='hard trial envelope'):
            t.envelope(dict(plant.arm, yaw_cdeg=outside))
    step, _, coarse = tray_correction(
        plant.arm, error, 230, matrix, t.tuning, 18., allow_coarse=False)
    assert not coarse and abs(step[1]) <= 240 and -1115 <= step[2] <= 0


def test_far_lower_top_face_can_use_damped_approach():
    plant = Plant()
    error = np.array([0, 330])
    tuning = TrialTuning()
    assert tray_approach(plant.arm, error, 230, plant.mapping['matrix'], tuning, 18.) is None
    target, _ = tracking_approach(plant.arm, error, 230, plant.mapping['matrix'], tuning, 18.)
    assert -19098 <= target[2] < -12000


@pytest.mark.parametrize('edge_hint, micro_only, view_count', [
    (None, False, 1),
    (None, False, 2),
    ([586, 571], False, 1),
    ([586, 571], False, 2),
    ([678, 660], False, 1),
    ([391, 580], False, 1),
    ([912, 575], False, 1),
    ([586, 571], True, 1),
])
def test_partial_rim_only_moves_to_observation_height_then_rechecks(
        tmp_path, edge_hint, micro_only, view_count):
    import json
    p = Plant()
    t = real_photo_trial(p, tmp_path, (0,0))
    t.micro_only = micro_only
    p.arm.update(z_cdeg=1, yaw_cdeg=-50, reach_cdeg=36)
    t.mapping['samples'] = [dict(requested=dict(yaw_deg=.5, reach_mm=-2.),
                                 actual=dict(yaw_deg=.47, reach_mm=-1.88, z_mm=0),
                                 source_frames=[{}, {}, {}]),
                            dict(requested=dict(yaw_deg=0, reach_mm=-4.),
                                 actual=dict(yaw_deg=0, reach_mm=-3.88, z_mm=0),
                                 source_frames=[{}, {}, {}])]
    original = t.snapshot
    moves = 0
    last_move_at = 0
    def partial():
        assert p.now < 105, 'timed out waiting for the measured view change'
        state = original()
        if moves < view_count:
            seq = state['material_observation']['source_sequence']
            pixel = [750, 500] if moves == 0 else [665, 490]
            if edge_hint is not None:
                pixel = edge_hint
            state.update(materials=[], material_localization_status='clipped_region_no_complete_rim',
                         material_view_hint=dict(color_id=2, source_sequence=seq, pixel=pixel))
        else:
            state['materials'][0].update(center_x=648, center_y=359)
        return state
    t.snapshot = partial
    original_align = p.arm_align
    def move(*args):
        nonlocal moves, last_move_at
        original_align(*args)
        moves += 1
        last_move_at = p.now
    p.arm_align = move
    _, obs, _ = t.align_color()
    moves = [cmd for cmd in p.calls if cmd[0]=='align']
    assert len(moves) == len(t.view_targets) == view_count
    assert all(cmd[3] == 0 and -190.98 <= cmd[5] < 0 for cmd in moves)
    events = [json.loads(line) for line in (tmp_path/'events.jsonl').read_text().splitlines()]
    observed_sources = [e['view_source'] for e in events if e['event'] == 'rim_view_change']
    if edge_hint is None or micro_only:
        assert [cmd[4:] for cmd in moves] == [(1.5, -6.37), (0, -12.73)][:view_count]
        assert observed_sources == ['calibration_sample'] * view_count
    else:
        assert observed_sources == ['computed'] * view_count
    assert obs['in_grasp_window']
    assert min(f['acquired'] for f in obs['source_frames']) > last_move_at
    assert not any(cmd[0]=='pick' for cmd in p.calls)


def test_missing_rim_after_usable_top_face_can_request_another_view(tmp_path):
    plant = Plant()
    t = real_photo_trial(plant, tmp_path, (0, 0))
    assert t.wait_reachable_target(timeout=1)['in_grasp_window']
    first_usable = t.first_usable
    snapshot = t.snapshot

    def partial_until_move():
        state = snapshot()
        if not any(c[0] == 'align' for c in plant.calls):
            seq = state['material_observation']['source_sequence']
            state.update(materials=[], material_localization_status='top_face_not_confirmed',
                         material_view_hint=dict(color_id=2, source_sequence=seq, pixel=[678, 660]))
        else:
            state['materials'][0].update(center_x=648, center_y=359)
        return state

    t.snapshot = partial_until_move
    arm, obs, _ = t.align_color()
    assert arm['reach_cdeg'] < -7000 and obs['in_grasp_window']
    assert t.first_usable == first_usable
    assert len([c for c in plant.calls if c[0] == 'align']) == 1
    assert not any(c[0] == 'pick' for c in plant.calls)


def test_micro_only_far_target_is_bounded_even_if_start_is_outside_corridor():
    p = Plant()
    for z in (0, 2):
        arm = dict(p.arm,z_cdeg=z)
        pose,_,coarse = tray_correction(arm,np.array([283,24]),230,p.mapping['matrix'],
                                        TrialTuning(),32,allow_coarse=False)
        assert not coarse and 0 < pose[1] <= 241 and -1115 <= pose[2] <= 0
