import copy
import math
from types import SimpleNamespace
import pytest
from pi_vision.arm_alignment import ArmCalibration,MechanismPose
from tools.arm_alignment_work import ArmAlignmentWork,validate_config


def calibration():
    # Artificial test calibration; never installed on hardware.
    return dict(measured=True,calibration_id='TEST_ONLY',plane_id='test_top',
        camera_source='/dev/v4l/by-id/test-camera',hsv_profile_sha256='test-hsv',
        origin_radius_mm=200,motor_signs=dict(z=1,yaw=1,reach=1),resolution=[1280,720],
        reference_pixel=[640,360],correction_radial_tangent_mm_per_pixel=[[.2,0],[0,.2]],
        measured_pose_ranges=dict(z=[-80,80],yaw=[-30,30],reach=[-100,200]),
        motor_limits_deg=dict(z=[-720,720],yaw=[-90,90],reach=[-720,720]))


def configuration():
    return dict(enabled=True,mode='alignment_only',firmware_and_clearances_verified=True,
        vision_state_url='http://127.0.0.1:8090/state',station_timeout_s=60,motion_timeout_s=10,
        observation_timeout_s=1,settle_s=.1,tolerance_px=3,confirm_frames=3,confirm_span_s=.2,
        max_frame_gap_s=.7,max_step_mm=10,max_yaw_step_deg=2,max_total_mm=60,max_adjustments=8,
        calibrations=dict(top=calibration()),stations={str(i):dict(layer=0,plane_id='test_top',
            calibration_id='top',target=dict(kind='material',color_id=1)) for i in range(102,108)})


def test_user_transmission_factors_and_inverse():
    c=ArmCalibration(calibration())
    pose=MechanismPose(20,90,50)
    data=calibration()
    data['measured_pose_ranges']['yaw']=[-90,90]
    data['motor_limits_deg']['yaw']=[-270,270]
    c=ArmCalibration(data)
    motor=c.motor_pose(pose)
    assert motor.z_deg==180 and motor.yaw_deg==270
    assert motor.reach_deg==pytest.approx(159.1511936)
    back=c.mechanism_pose(dict(z_deg=motor.z_deg,yaw_deg=motor.yaw_deg,reach_deg=motor.reach_deg))
    assert back.z_mm==20 and back.reach_mm==pytest.approx(50) and back.yaw_deg==90


def test_polar_correction_depends_on_current_extension_and_preserves_height():
    c=ArmCalibration(calibration())
    near,d=c.correct(MechanismPose(5,0,0),(0,20),10,10)
    far,_=c.correct(MechanismPose(5,0,100),(0,20),10,10)
    assert near.yaw_deg>far.yaw_deg>0
    assert near.z_mm==far.z_mm==5 and d==4
    assert near.reach_mm==pytest.approx(math.hypot(200,4)-200)
    limited,_=c.correct(MechanismPose(0,0,0),(0,1000),10,1)
    assert abs(limited.yaw_deg)<=1.00001


@pytest.mark.parametrize('problem',['unmeasured','sign','matrix','range'])
def test_calibration_and_actual_limits_enforced(problem):
    data=calibration()
    if problem=='unmeasured': data['measured']=False
    if problem=='sign': data['motor_signs']['z']=0
    if problem=='matrix': data['correction_radial_tangent_mm_per_pixel']=[[1,1],[1,1]]
    if problem=='range':
        with pytest.raises(ValueError): ArmCalibration(data).motor_pose(MechanismPose(0,31,0))
        return
    with pytest.raises(ValueError): ArmCalibration(data)


class Plant:
    def __init__(self):
        self.now=100.
        self.status=dict(route_id=7,point_index=2,work_id=102,state=3,phase=4)
        self.arm=dict(route_id=7,point_index=2,work_id=102,state_name='WAIT_VISION',fault=0,
            origins_valid=True,command_id=0,aligned_command_id=0,tick=10000,
            z_deg=0,yaw_deg=0,reach_deg=0)
        self.sequence=0
        self.actions=[]
        self.error=20
        self.problem=None
    def arm_status(self):
        self.arm['tick']+=200
        return dict(self.arm)
    def arm_align(self,*args):
        *context,cmd,z,yaw,reach=args
        self.actions.append(('align',args))
        self.arm.update(command_id=cmd,aligned_command_id=cmd,state_name='ALIGNED',
                        z_deg=z,yaw_deg=yaw,reach_deg=reach)
        self.error=0
    def arm_finish(self,*args):
        self.actions.append(('finish',args))
        self.arm.update(command_id=args[-1],state_name='READY_LEAVE')
    def snapshot(self):
        self.sequence+=1
        acquired=self.now-.01
        state=dict(camera_source='/dev/v4l/by-id/test-camera',session_id='session1',
            capture=dict(valid=True,acquired_monotonic_s=acquired),
            detection=dict(materials=True,rings=True),
            hsv_profile_sha256='test-hsv',coordinate_frame='camera_image',coordinate_units='pixels',
            preview=dict(source_width=1280,source_height=720),material_backend='HSVMaterialDetector',
            material_selection='nearest_window_center',material_target_window=[480,240,320,240],
            material_observation=dict(valid=True,status='ok',acquired_monotonic_s=acquired,
                                      source_sequence=self.sequence),
            materials=[dict(center_x=640+self.error,center_y=360,color_id=1,
                            point_kind='top_face_ellipse_center')])
        if self.problem=='old': state['material_observation']['acquired_monotonic_s']=self.now-3
        if self.problem=='duplicate': state['material_observation']['source_sequence']=1
        if self.problem=='prestop': state['material_observation']['acquired_monotonic_s']=99
        if self.problem=='wrong_color': state['materials'][0]['color_id']=2
        if self.problem=='side': state['materials'][0]['point_kind']='contour_area_centroid'
        if self.problem=='lost': state['materials']=[]
        if self.problem=='peripheral': state['material_selection']='all_candidates'
        if self.problem=='camera': state['camera_source']='/dev/v4l/by-id/qr-camera'
        if self.problem=='restart': state['session_id']='session'+str(self.sequence)
        return state
    def advance(self): self.now+=.21


def run_plant(plant,config=None):
    work=ArmAlignmentWork(plant,config or configuration(),plant.snapshot,clock=lambda:plant.now)
    work.start(plant.status)
    for i in range(400):
        result=work.poll(plant.status)
        if result is not None: return work,result
        plant.advance()
    raise AssertionError('never returned')


def test_full_visual_align_reobserve_then_park_without_pick_or_fake_gripper():
    plant=Plant()
    work,result=run_plant(plant)
    assert [a for a,_ in plant.actions]==['align','finish']
    assert result['visual_alignment_confirmed'] and result['physical_grasp_completed'] is False
    assert result['physical_place_completed'] is False
    assert work.phase=='done' and len(work.results)==1


@pytest.mark.parametrize('problem',['old','duplicate','wrong_color','side','lost','peripheral','camera','restart'])
def test_bad_observation_never_releases_station(problem):
    plant=Plant();plant.problem=problem
    with pytest.raises((RuntimeError,TimeoutError)):
        run_plant(plant)
    assert not any(a=='finish' for a,_ in plant.actions)


def test_already_centered_still_requires_align_ack_and_new_observations():
    plant=Plant();plant.error=0
    _,result=run_plant(plant)
    assert [a for a,_ in plant.actions]==['align','finish']
    assert result['adjustments']==0


def test_unsupported_stack_and_group4_staging_refused():
    data=configuration();data['stations']['107']['layer']=1
    with pytest.raises(ValueError,match='stacked'): validate_config(data)
    data=configuration();data['stations']['107']['target']=dict(kind='ring',task_group='group4',task_index=0)
    with pytest.raises(ValueError,match='same-color'): validate_config(data)


def test_not_origin_ready_preflight_never_zeros_or_moves():
    plant=Plant();plant.arm['origins_valid']=False
    work=ArmAlignmentWork(plant,configuration(),plant.snapshot,clock=lambda:plant.now)
    with pytest.raises(RuntimeError,match='never auto-zero'): work.preflight()
    assert not plant.actions


def test_existing_ring_pipeline_is_shared_without_changing_default_detector():
    from unittest.mock import Mock
    from pi_vision.pipeline import configure_ring_pipeline
    from pi_vision.position_digits import NumberedRingDetector
    pipeline=SimpleNamespace(ring_detector=Mock())
    original=pipeline.ring_detector
    configure_ring_pipeline(pipeline)
    assert pipeline.ring_detector is original
    configure_ring_pipeline(pipeline,'morphology',[0,0,640,480],0)
    assert isinstance(pipeline.ring_detector,NumberedRingDetector)
    with pytest.raises(ValueError): configure_ring_pipeline(pipeline,'morphology')


def test_ring_target_uses_position_identity_not_placeholder_scoring_ring():
    plant=Plant()
    data=configuration()
    data['stations']['102']['target']=dict(kind='ring',position_id=2)
    def snapshot():
        state=plant.snapshot()
        state.update(position_identity_enabled=True,ring_observation=state['material_observation'],
            rings=[dict(ring_no=1,position_id=2,position_status='candidate',center_x=640,center_y=360)])
        return state
    work=ArmAlignmentWork(plant,data,snapshot,clock=lambda:plant.now)
    work.start(plant.status)
    for _ in range(30):
        result=work.poll(plant.status)
        if result is not None: break
        plant.advance()
    assert result['target']['position_id']==2
    assert result['physical_place_completed'] is False


def test_recent_but_before_stop_frame_cannot_trigger_alignment():
    plant=Plant();plant.error=0
    snapshots=[]
    def snapshot():
        state=plant.snapshot()
        state['material_observation']['acquired_monotonic_s']=100.05
        snapshots.append(state)
        return state
    work=ArmAlignmentWork(plant,configuration(),snapshot,clock=lambda:plant.now)
    work.start(plant.status)
    work.poll(plant.status)  # WAIT_VISION -> floor=100.1
    plant.advance()
    work.poll(plant.status)  # Frame is recent but acquired before stop+settle.
    assert snapshots and not plant.actions and not work.stable


@pytest.mark.parametrize('problem',['capture','camera','detector'])
def test_source_preflight_fails_before_route_start(problem):
    plant=Plant();plant.arm['state_name']='READY'
    def snapshot():
        state=plant.snapshot()
        if problem=='capture': state['capture']['valid']=False
        if problem=='camera': state['camera_source']='other-camera'
        if problem=='detector': state['detection']['materials']=False
        return state
    work=ArmAlignmentWork(plant,configuration(),snapshot,clock=lambda:plant.now)
    with pytest.raises(RuntimeError): work.preflight()
    assert not plant.actions


@pytest.mark.parametrize('state', ['DEBUG_MOVING', 'DEBUG_HOLD'])
def test_debug_operation_interrupts_station_without_replacement_motion(state):
    plant = Plant()
    work = ArmAlignmentWork(plant, configuration(), plant.snapshot, clock=lambda: plant.now)
    work.start(plant.status)
    plant.arm['state_name'] = state
    with pytest.raises(RuntimeError, match='debug'):
        work.poll(plant.status)
    assert plant.actions == []


class RawSearchPlant(Plant):
    def snapshot(self):
        state = super().snapshot()
        state.update(material_selection='unique_color_in_search_window', material_search_color=1,
                     material_target_window=[0, 0, 1280, 720])
        return state


def test_raw_station_can_start_with_red_top_face_outside_old_center_window():
    plant = RawSearchPlant()
    plant.error = 300
    _, result = run_plant(plant)
    assert result['visual_alignment_confirmed']
    assert [a for a, _ in plant.actions] == ['align', 'finish']


def test_missing_material_uses_measured_search_pose_then_reobserves():
    plant = RawSearchPlant()
    original = plant.snapshot

    def snapshot():
        state = original()
        if not plant.actions:
            state['materials'] = []
        return state

    config = configuration()
    config['stations']['102'].update(search_wait_s=.1,
        search_poses=[dict(z_mm=0, yaw_deg=1, reach_mm=-1)])
    work = ArmAlignmentWork(plant, config, snapshot, clock=lambda: plant.now)
    work.start(plant.status)
    for _ in range(50):
        result = work.poll(plant.status)
        if result is not None:
            break
        plant.advance()
    else:
        raise AssertionError('search did not finish')
    assert result['visual_alignment_confirmed']
    assert work.search_index == 1
    assert plant.actions[0][0] == 'align'
    assert plant.actions[0][1][-3:] == pytest.approx((0, 3, -360/113.1))
    assert plant.actions[-1][0] == 'finish'


def test_raw_color_search_is_rejected_at_processing_station():
    plant = RawSearchPlant()
    plant.status['work_id'] = plant.arm['work_id'] = 103
    with pytest.raises(RuntimeError, match='station'):
        run_plant(plant)


def test_search_pose_cannot_lower_camera_or_exceed_step_limit():
    plant = RawSearchPlant()
    plant.problem = 'lost'
    config = configuration()
    config['stations']['102'].update(search_wait_s=.1,
        search_poses=[dict(z_mm=-1, yaw_deg=0, reach_mm=0)])
    with pytest.raises(RuntimeError, match='fixed-height'):
        run_plant(plant, config)
    assert plant.actions == []


@pytest.mark.parametrize('problem', ['lost', 'old'])
def test_search_exhaustion_and_stale_frame_never_park_or_release(problem):
    plant = RawSearchPlant()
    plant.problem = problem
    config = configuration()
    config['stations']['102'].update(search_wait_s=.1,
        search_poses=[dict(z_mm=0, yaw_deg=1, reach_mm=-1)])
    with pytest.raises(RuntimeError):
        run_plant(plant, config)
    assert not any(action == 'finish' for action, _ in plant.actions)
    assert len(plant.actions) == (1 if problem == 'lost' else 0)
