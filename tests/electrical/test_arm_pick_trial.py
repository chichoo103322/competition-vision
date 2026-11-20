import pytest
from tools.arm_pick_trial import corrected_target


def test_approach_is_limited_and_moves_in_measured_directions():
    arm=dict(yaw_deg=0,reach_deg=0)
    matrix=[[0,-.22],[.22,0]]
    yaw,reach,distance=corrected_target(arm,[679,377],[648,359],matrix,230)
    assert 0<yaw<=1.00001
    assert -8<=reach<0 and distance<=4.00001


def test_unreachable_forward_target_is_not_silently_clipped():
    with pytest.raises(RuntimeError,match='trial'):
        corrected_target(dict(yaw_deg=0,reach_deg=0),[648,330],[648,359],[[0,-.22],[.22,0]],230)
