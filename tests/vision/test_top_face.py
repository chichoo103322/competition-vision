from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock, patch
import cv2
import numpy as np
import pytest
from pi_vision.detectors.hsv_material_detector import HSVMaterialDetector
from pi_vision.material_region import MaterialRegionDetector, center_material
from pi_vision.stream import FrameStreamer
from pi_vision.types import Material


def green(value):
    return tuple(map(int,cv2.cvtColor(np.uint8([[[65,180,value]]]),cv2.COLOR_HSV2BGR)[0,0]))


def scene(body=True):
    image=np.full((300,400,3),220,np.uint8)
    if body:
        cv2.fillPoly(image,[np.array([(115,95),(185,95),(200,170),(100,170)])],green(120))
    cv2.ellipse(image,(150,95),(35,32),0,0,360,green(230),-1)
    return image


def test_top_center_is_not_side_biased_color_centroid():
    image=scene()
    old=HSVMaterialDetector(center_method='centroid').detect(image)[0]
    top=HSVMaterialDetector(center_method='top_face').detect(image)[0]
    assert old.center_y>115
    assert top.localization_valid
    assert abs(top.center_x-150)<=1 and abs(top.center_y-95)<=1
    assert top.point_kind=='top_face_ellipse_center'
    assert top.selection_center_y>115
    assert top.top_face_residual<.035


def test_overhead_round_face_and_roi_return_full_image_coordinates():
    image=scene(body=False)
    full=np.full((450,600,3),220,np.uint8);full[60:360,80:480]=image
    local=HSVMaterialDetector(center_method='top_face').detect(image)[0]
    top=MaterialRegionDetector(HSVMaterialDetector(center_method='top_face'),(80,60,400,300)).detect(full)[0]
    assert top.localization_valid
    assert abs(top.center_x-230)<=1 and abs(top.center_y-155)<=1
    assert top.selection_center_x==local.selection_center_x+80
    assert top.selection_center_y==local.selection_center_y+60


@pytest.mark.parametrize('kind',['rectangle','clipped','tiny_glare'])
def test_no_plausible_top_never_becomes_valid_centroid(kind):
    image=np.full((300,400,3),220,np.uint8)
    if kind=='clipped':cv2.circle(image,(5,100),55,green(230),-1)
    else:
        cv2.rectangle(image,(100,80),(210,180),green(120),-1)
        if kind=='tiny_glare':cv2.circle(image,(150,120),8,green(240),-1)
    found=HSVMaterialDetector(center_method='top_face').detect(image)
    assert found
    assert all(not m.localization_valid for m in found)


def test_stream_invalid_middle_top_cannot_be_replaced_by_side_target():
    camera=Mock();pipeline=SimpleNamespace(qr_camera=camera,material_detector=Mock())
    middle=Material(1,4,0,300,200,0.,90,point_kind='top_face_ellipse_center',
                    localization_valid=False,localization_status='top_face_not_confirmed',
                    selection_center_x=300.,selection_center_y=200.)
    side=replace(middle,color_id=1,center_x=360,localization_valid=True,selection_center_x=360.)
    with patch('pi_vision.stream.time.monotonic',return_value=10.):
        source=FrameStreamer(camera,pipeline,enable_qr=False,enable_rings=False,
                             material_target_window=(200,100,200,200))
        try:
            source._latest_raw=np.zeros((400,600,3),np.uint8)
            source._raw_sequence=1;source._raw_acquired_at=10.
            source._finish_detection('materials',[middle,side],1,10.,100.,0,0)
            state=source.get_state()
            assert state['materials']==[]
            assert state['material_observation']['status']=='no_target'
            assert state['material_localization_status']=='top_face_not_confirmed'
        finally:source._executor.shutdown(wait=False)


def test_round_silhouette_with_visible_dark_side_prefers_inner_top_rim():
    image=np.full((300,400,3),220,np.uint8)
    cv2.ellipse(image,(150,125),(48,47),0,0,360,green(110),-1)
    cv2.circle(image,(155,114),37,green(230),-1)
    candidates=HSVMaterialDetector(center_method='top_face').detect(image)
    assert len(candidates)==1 and candidates[0].localization_valid
    assert abs(candidates[0].center_x-155)<=2
    assert abs(candidates[0].center_y-114)<=2


def test_similar_brightness_yellow_end_is_distinct_from_cylinder_side():
    def yellow(sat, value):
        return tuple(map(int, cv2.cvtColor(np.uint8([[[27, sat, value]]]), cv2.COLOR_HSV2BGR)[0,0]))
    image = np.full((360,640,3), 185, np.uint8)
    # Shape, position and saturation contrast from the yellow disc-edge preview.
    side = yellow(160, 190)
    cv2.line(image, (376,212), (414,249), side, 108)
    cv2.circle(image, (414,249), 54, yellow(212,188), -1)
    obj, = HSVMaterialDetector(center_method='top_face').detect(image)
    assert obj.localization_valid
    assert abs(obj.center_x - 414) <= 2 and abs(obj.center_y - 249) <= 2
    assert obj.center_x - obj.selection_center_x > 10


@pytest.mark.parametrize('rotate', [0, 1, 2, 3])
def test_clipped_side_with_entire_top_visible(rotate):
    # Crop only the bottom of the wall; rotate to exercise all four borders.
    im = np.ascontiguousarray(np.rot90(scene()[:146], rotate))
    obj, = HSVMaterialDetector(center_method='top_face').detect(im)
    expected = [(150,95), (95,249), (249,50), (50,150)][rotate]
    assert obj.localization_valid and np.linalg.norm(np.array([obj.center_x,obj.center_y])-expected) <= 2
    clipped = np.ascontiguousarray(np.rot90(scene()[85:146], rotate))
    assert not any(m.localization_valid for m in HSVMaterialDetector(center_method='top_face').detect(clipped))


def test_failure_image_cache_matches_detection_source_and_expires_old_keys():
    cam = Mock()
    pipe = SimpleNamespace(qr_camera=cam, material_detector=Mock())
    with patch('pi_vision.stream.time.monotonic', return_value=10.):
        s = FrameStreamer(cam, pipe, enable_qr=False, enable_rings=False,
                          material_target_window=(0,0,30,30), material_search_color=2)
        try:
            for seq in range(1, 5):
                frame = np.full((30,30,3), seq, np.uint8)
                s._latest_raw, s._raw_sequence, s._raw_acquired_at = frame, seq, 10.
                assert s._finish_detection('materials', [], seq, 10., 100., 0, 0, source_frame=frame)
            assert s.material_sample(f'{s.session_id}-0-0-1') is None
            assert np.all(s.material_sample(f'{s.session_id}-0-0-2') == 2)
            assert s.material_sample(f'{s.session_id}-1-0-2') is None
        finally:
            s._executor.shutdown(wait=False)


def test_unique_unlocalized_body_hint_is_separate_from_grasp_coordinates():
    cam = Mock()
    pipe = SimpleNamespace(qr_camera=cam, material_detector=Mock())
    m = Material(1,2,0,200,200,0.,90,localization_valid=False,
                 localization_status='clipped_region_no_complete_rim',selection_center_x=201.,selection_center_y=199.)
    with patch('pi_vision.stream.time.monotonic',return_value=10.):
        s=FrameStreamer(cam,pipe,enable_qr=False,enable_rings=False,
                        material_target_window=(0,0,400,300),material_search_color=2)
        try:
            s._latest_raw=np.zeros((300,400,3),np.uint8);s._raw_acquired_at=10.;s._raw_sequence=1
            s._finish_detection('materials',[m],1,10.,100.,0,0)
            state=s.get_state()
            assert state['materials']==[] and state['material_view_hint']==dict(
                color_id=2,source_sequence=1,pixel=[201.,199.])
            s._finish_detection('materials',[m,replace(m,index=2)],2,10.,100.,0,0)
            assert s.get_state()['material_view_hint'] is None
        finally:
            s._executor.shutdown(wait=False)


@pytest.mark.parametrize('show_rim', [False, True])
def test_elongated_yellow_region_requires_internal_rim_for_axial_fit(show_rim):
    from pi_vision.top_face import axial_rim
    image = np.full((360,640,3), 170, np.uint8)
    side = (45,151,168)
    cv2.line(image,(370,215),(428,216),side,104)
    if show_rim:
        cv2.circle(image,(428,216),52,(32,155,170),-1)
    mask = cv2.inRange(image,(0,115,130),(65,180,190))
    contour = max(cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0],key=cv2.contourArea)
    face = axial_rim(image, contour)
    if show_rim:
        assert face and abs(face.center_x-428)<4 and abs(face.center_y-216)<4
    else:
        assert face is None
