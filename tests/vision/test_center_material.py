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


def test_raw_color_search_finds_offcenter_red_despite_centered_other_color():
    from pi_vision.material_region import search_material
    red = m(1, 1100, 600)
    assert search_material([m(2, 640, 360), red], (0, 0, 1280, 720), (1280, 720), 1) == [red]
    assert search_material([red], (480, 240, 320, 240), (1280, 720), 1) == []


def test_raw_color_search_does_not_replace_ambiguous_or_failed_top_face():
    from dataclasses import replace
    from pi_vision.material_region import search_material
    bad = replace(m(1, 1000, 600), localization_valid=False)
    other = m(1, 640, 360)
    assert search_material([bad], (0, 0, 1280, 720), (1280, 720), 1) == [bad]
    assert search_material([bad, other], (0, 0, 1280, 720), (1280, 720), 1) == []


def test_stream_raw_search_selects_requested_color_and_clears_after_removal():
    cam = Mock()
    pipeline = SimpleNamespace(qr_camera=cam, material_detector=Mock())
    with patch('pi_vision.stream.time.monotonic', return_value=10.):
        stream = FrameStreamer(cam, pipeline, enable_qr=False, enable_rings=False,
                               material_target_window=(0, 0, 1280, 720), material_search_color=1)
        try:
            stream._latest_raw = np.zeros((720, 1280, 3), np.uint8)
            stream._raw_sequence = 1
            stream._raw_acquired_at = 10.
            stream._finish_detection('materials', [m(1, 1100, 600), m(2, 640, 360)], 1, 10., 100., 0, 0)
            state = stream.get_state()
            assert state['material_selection'] == 'unique_color_in_search_window'
            assert state['material_search_color'] == 1
            assert state['materials'][0]['center_x'] == 1100
            stream._finish_detection('materials', [m(2, 640, 360)], 2, 10., 100., 0, 0)
            assert stream.get_state()['materials'] == []
        finally:
            stream._executor.shutdown(wait=False)


def test_switching_turntable_color_invalidates_inflight_and_previous_target():
    cam = Mock()
    pipeline = SimpleNamespace(qr_camera=cam, material_detector=Mock())
    with patch('pi_vision.stream.time.monotonic', return_value=10.):
        stream = FrameStreamer(cam, pipeline, enable_qr=False, enable_rings=False,
            material_target_window=(480, 240, 320, 240), material_search_color=1)
        try:
            stream._latest_raw = np.zeros((720, 1280, 3), np.uint8)
            stream._raw_sequence, stream._raw_acquired_at = 1, 10.
            stream._finish_detection('materials', [m(1, 640, 360)], 1, 10., 100., 0, 0)
            assert stream.get_state()['materials']
            stream.set_material_search_color(2)
            assert stream.get_state()['materials'] == []
            assert stream.get_state()['material_search_color'] == 2
            assert not stream._finish_detection('materials', [m(1, 640, 360)], 2, 10., 100., 0, 0)
            assert stream.get_state()['materials'] == []
        finally:
            stream._executor.shutdown(wait=False)
