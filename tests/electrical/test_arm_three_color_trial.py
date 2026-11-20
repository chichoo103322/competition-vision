import copy
import pytest

from tools.arm_three_color_trial import (
    ArmTrial, ThreeColorTrial, TrialTuning, ObservationUnavailable, correction_target, parse_order)


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
    assert len(stamps)==3 and len({f['sequence'] for f in obs['source_frames']})==3
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
