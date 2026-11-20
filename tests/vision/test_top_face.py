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
