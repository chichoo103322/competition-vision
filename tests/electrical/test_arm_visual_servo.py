import copy
import math

import numpy as np
import pytest

from pi_vision.arm_visual_servo import AdaptivePixelMap, local_motion_mm, observation_dynamics


MATRIX = [[0,-.22],[.22,0]]


def observed(pixel, stamp, **changes):
    return dict(pixel=list(pixel),pixel_time_s=stamp,track_id=3,
                stationary_for_mapping=True,velocity_reliable=False,**changes)


def test_encoder_displacement_rotates_into_previous_arm_frame():
    a=dict(yaw_cdeg=3000,reach_cdeg=-3000)
    b=dict(yaw_cdeg=3240,reach_cdeg=-3000)
    d=local_motion_mm(a,b,230)
    radius=230-3000*113.1/36000
    assert d==pytest.approx([radius*(math.cos(math.radians(.8))-1),
                            radius*math.sin(math.radians(.8))])


def test_short_stable_encoder_samples_improve_a_biased_mapping_without_mutating_calibration():
    source=copy.deepcopy(MATRIX)
    servo=AdaptivePixelMap(source)
    before=dict(yaw_cdeg=0,reach_cdeg=0)
    after=dict(yaw_cdeg=0,reach_cdeg=-955)
    delta=local_motion_mm(before,after,230)
    true_j=servo.jacobian.copy();true_j[:,0]*=1.25
    p=np.array([680.,390.])
    q=p+true_j@delta
    old_error=np.linalg.norm(servo.jacobian@delta-true_j@delta)
    update=servo.update(observed(p,100),observed(q,101.2),before,after,230,
                        dict(elapsed_s=.7,full_avoidance=False))
    assert update['updated']
    assert np.linalg.norm(servo.jacobian@delta-true_j@delta)<old_error
    assert source==MATRIX
    assert np.linalg.cond(servo.correction_matrix)<20


@pytest.mark.parametrize('problem',['moving','identity','slow','clearance','outlier','gap'])
def test_turntable_motion_identity_changes_and_slow_avoidance_are_never_learned(problem):
    servo=AdaptivePixelMap(MATRIX)
    saved=servo.correction_matrix.copy()
    a=dict(yaw_cdeg=0,reach_cdeg=0);b=dict(yaw_cdeg=0,reach_cdeg=-955)
    first,last=observed([680,390],100),observed([680,376],101.2)
    motion=dict(elapsed_s=.7,full_avoidance=False)
    if problem=='moving':last['stationary_for_mapping']=False
    if problem=='identity':last['track_id']=99
    if problem=='slow':motion['elapsed_s']=8
    if problem=='clearance':motion['full_avoidance']=True
    if problem=='outlier':last['pixel']=[710,410]
    if problem=='gap':last['pixel_time_s']=105
    result=servo.update(first,last,a,b,230,motion)
    assert not result['updated']
    assert servo.correction_matrix==pytest.approx(saved)


def test_recovery_shrinks_steps_then_restores_them_after_real_progress():
    servo=AdaptivePixelMap(MATRIX)
    for _ in range(20):servo.recover()
    assert servo.step_gain==.4
    for _ in range(20):servo.progress()
    assert servo.step_gain==1


def test_short_term_target_prediction_is_bounded_and_never_uses_stale_positions():
    servo=AdaptivePixelMap(MATRIX)
    obs=observed([680,390],100)
    obs.update(velocity_reliable=True,velocity_px_s=[100,50])
    assert np.linalg.norm(servo.predicted_pixel(obs,100.1)-obs['pixel'])<=4.000001
    assert servo.predicted_pixel(obs,102)==pytest.approx(obs['pixel'])
    obs['velocity_reliable']=False
    assert servo.predicted_pixel(obs,100.1)==pytest.approx(obs['pixel'])


def test_velocity_requires_consistent_motion_across_distinct_acquisitions():
    frames=[dict(acquired=100+i*.2) for i in range(3)]
    moving=observation_dynamics([[640,360],[641,360],[642,360]],frames)
    assert moving['velocity_reliable']
    assert moving['velocity_px_s']==pytest.approx([5,0])
    assert not moving['stationary_for_mapping']
    jump=observation_dynamics([[640,360],[640,360],[643,360]],frames)
    assert not jump['velocity_reliable'] and not jump['stationary_for_mapping']
    still=observation_dynamics([[640,360]]*3,frames)
    assert still['stationary_for_mapping'] and not still['velocity_reliable']


@pytest.mark.parametrize('matrix',[[[0,0],[0,0]],[[float('nan'),0],[0,1]]])
def test_invalid_calibration_cannot_enable_adaptation(matrix):
    with pytest.raises(ValueError):AdaptivePixelMap(matrix)
