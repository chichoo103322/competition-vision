"""Compare optional reference-video methods on saved images; no camera/serial access."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import cv2
from pi_vision.detectors.hsv_material_detector import HSVMaterialDetector
from pi_vision.detectors.morphology_ring_detector import MorphologyRingDetector, RingRegionDetector
from pi_vision.detectors.ring_detector import RingDetector
from pi_vision.hsv_profile import load_profile
from pi_vision.material_region import MaterialRegionDetector
from pi_vision.position_digits import NumberedRingDetector


def compare(image, *, profile=None, material_roi=None, ring_roi=None, orientation=0, output=None):
    report = {'evidence': 'offline_saved_image_comparison', 'units': 'pixels',
              'limitations': 'No ground truth accuracy or calibrated grasp/placement claim',
              'material_roi': material_roi, 'ring_roi': ring_roi,
              'materials': {}, 'rings': {}}
    canvas = image.copy()
    for method, color in [('box', (0, 200, 0)), ('centroid', (0, 0, 255))]:
        detector = HSVMaterialDetector(profile=profile, center_method=method)
        masks = {}
        if material_roi is None:
            found = detector.detect(image, diagnostics=masks)
        else:
            x, y, w, h = material_roi
            found = MaterialRegionDetector(detector, material_roi).detect(image)
            detector.detect(image[y:y+h, x:x+w], diagnostics=masks)
        report['materials'][method] = [asdict(m) for m in found]
        for m in found:
            cv2.drawMarker(canvas, (m.center_x, m.center_y), color, cv2.MARKER_CROSS, 12, 1)
        if output and method == 'box':
            for color_id, mask in masks.items():
                cv2.imwrite(str(output/f'hsv_{color_id}.png'), mask)
    for name, detector in [('hough', RingDetector()), ('morphology', MorphologyRingDetector())]:
        if ring_roi is not None:
            if name == 'morphology' and output:
                x, y, w, h = ring_roi
                if min(x, y) < 0 or min(w, h) <= 0 or x+w > image.shape[1] or y+h > image.shape[0]:
                    raise ValueError('ring ROI exceeds image')
                stages = {}
                detector.detect(image[y:y+h, x:x+w], diagnostics=stages)
                for stage, view in stages.items():
                    cv2.imwrite(str(output/f'ring_{stage}.png'), view)
            detector = RingRegionDetector(detector, ring_roi)
        elif name == 'morphology':
            continue  # Require explicit local region for experimental morphology.
        found = NumberedRingDetector(detector, orientation).detect(image)
        report['rings'][name] = [asdict(r) for r in found]
        for r in found:
            cv2.circle(canvas, (r.center_x, r.center_y), round(r.radius), (255, 80, 0), 1)
            cv2.putText(canvas, f'{name}:P{r.position_id}', (r.center_x+5, r.center_y-8),
                        cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 80, 0), 1)
    if output:
        cv2.imwrite(str(output/'comparison.png'), canvas)
        (output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description='保存图像离线比较：框中心/质心、霍夫/形态学圆环')
    parser.add_argument('image')
    parser.add_argument('--output', required=True)
    parser.add_argument('--hsv-profile')
    parser.add_argument('--material-roi', nargs=4, type=int)
    parser.add_argument('--ring-roi', nargs=4, type=int)
    parser.add_argument('--orientation', type=int, choices=(0, 90, 180, 270), default=0)
    args = parser.parse_args()
    image = cv2.imread(args.image)
    if image is None:
        parser.error('image is unreadable')
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    report = compare(image, profile=load_profile(args.hsv_profile) if args.hsv_profile else None,
                     material_roi=args.material_roi, ring_roi=args.ring_roi,
                     orientation=args.orientation, output=output)
    print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    main()
