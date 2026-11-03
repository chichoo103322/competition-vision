"""Render top-face geometry from a saved raw frame, never from a live preview pair."""
import argparse
import json
from dataclasses import asdict
from pathlib import Path
import cv2
from pi_vision.detectors.hsv_material_detector import HSVMaterialDetector
from pi_vision.material_region import MaterialRegionDetector, center_material


def review(image, roi=None, window=None, profile=None):
    results={}
    for method in ('centroid','top_face'):
        detector=HSVMaterialDetector(profile=profile,center_method=method)
        if roi is not None:detector=MaterialRegionDetector(detector,roi)
        found=detector.detect(image)
        if window is not None:found=center_material(found,window,(image.shape[1],image.shape[0]))
        results[method]=[asdict(m) for m in found]
    view=image.copy()
    cv2.rectangle(view,(8,8),(min(image.shape[1]-8,1100),105),(30,30,30),-1)
    cv2.putText(view,'TOP FACE ONLY | raw image pixels | not mechanical calibration',(18,38),
                cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1,cv2.LINE_AA)
    for item in results['top_face']:
        if not item['localization_valid']:
            cv2.putText(view,'No confirmed top face',(18,75),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,160,255),1)
            continue
        cx,cy=item['center_x'],item['center_y']
        cv2.ellipse(view,((float(cx),float(cy)),tuple(item['top_face_axes']),float(item['top_face_angle_deg'])),(255,220,0),2)
        cv2.drawMarker(view,(cx,cy),(0,0,255),cv2.MARKER_CROSS,24,2)
        cv2.putText(view,f'ID {item["color_id"]} | top center ({cx},{cy}) px',(18,75),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),1,cv2.LINE_AA)
    return results,view


def main():
    parser=argparse.ArgumentParser(description='同一原图顶面/整体质心比较，不打开相机')
    parser.add_argument('image');parser.add_argument('--metadata',help='可选同名诊断JSON，恢复ROI、窗口和profile')
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    image=cv2.imread(args.image)
    if image is None:parser.error('image unreadable')
    data=json.loads(Path(args.metadata).read_text()) if args.metadata else {}
    result,view=review(image,data.get('roi'),data.get('material_target_window'),data.get('profile'))
    result.update(source_sequence=data.get('source_sequence'),frame_size=[image.shape[1],image.shape[0]],
                  coordinate_frame='camera_image',units='pixels',metric_coordinates_valid=False)
    if data.get('material_center_method')=='top_face':
        fields=('color_id','center_x','center_y','point_kind','top_face_axes','top_face_angle_deg')
        def geometry(items):return [{k:m.get(k) for k in fields} for m in items]
        result['matches_recorded']=json.loads(json.dumps(geometry([m for m in result['top_face'] if m['localization_valid']])))==geometry(data['materials'])
    out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    cv2.imwrite(str(out/'top_face.png'),view)
    (out/'report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
