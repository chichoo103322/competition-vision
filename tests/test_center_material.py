from pi_vision.material_region import center_material
from pi_vision.types import Material
from pi_vision.stream import FrameStreamer
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np


def m(color, x, y):
    return Material(1, color, 0, x, y, 0., 90)


def test_only_middle_color_even_with_larger_high_confidence_side_objects():
    target = m(6, 640, 360)
    assert center_material([m(1, 900, 360), m(5, 200, 360), target], (480, 240, 320, 240), (1280, 720)) == [target]


def test_empty_window_never_falls_back_to_side_colors():
    assert center_material([m(1, 900, 360)], (480, 240, 320, 240), (1280, 720)) == []


def test_two_similarly_centered_colors_are_ambiguous():
    assert center_material([m(1, 635, 360), m(2, 647, 360)], (480, 240, 320, 240), (1280, 720)) == []


def test_stream_clears_middle_target_after_removal_despite_side_color():
    cam=Mock(); pipeline=SimpleNamespace(qr_camera=cam, material_detector=Mock())
    with patch('pi_vision.stream.time.monotonic', return_value=10.):
        stream=FrameStreamer(cam,pipeline,enable_qr=False,enable_rings=False,
                             material_target_window=(480,240,320,240))
        try:
            stream._latest_raw=np.zeros((720,1280,3),np.uint8)
            stream._raw_sequence=1; stream._raw_acquired_at=10.
            stream._finish_detection('materials',[m(6,640,360),m(1,900,360)],1,10.,100.,0,0)
            assert [x['color_id'] for x in stream.get_state()['materials']] == [6]
            stream._finish_detection('materials',[m(1,900,360)],2,10.,100.,0,0)
            state=stream.get_state()
            assert state['materials']==[]
            assert state['material_observation']['status']=='no_target'
        finally:
            stream._executor.shutdown(wait=False)
