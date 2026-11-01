"""Replay full-resolution evidence; never opens a camera or serial device."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path

import cv2

from pi_vision.detectors.hsv_material_detector import HSVMaterialDetector
from pi_vision.hsv_profile import load_profile, profile_hash, validate_profile
from pi_vision.material_region import MaterialRegionDetector, ordered_materials


def replay(directory, candidate=None):
    report = {'samples': [], 'limitations': '历史原图回放；颜色正确性和抓取精度仍需人工及实机确认'}
    for path in sorted(Path(directory).glob('sample_*.json')):
        data = json.loads(path.read_text(encoding='utf-8'))
        if data.get('backend') != 'HSVMaterialDetector' or not data.get('valid'):
            raise ValueError(f'{path.name}: invalid observation or unsupported backend')
        baseline = validate_profile(data['profile'])
        if profile_hash(baseline) != data['profile_sha256']:
            raise ValueError(f'{path.name}: profile hash mismatch')
        image_name = data['image']
        if Path(image_name).name != image_name:
            raise ValueError('image must be in evidence directory')
        image = cv2.imread(str(path.parent / image_name))
        if image is None or [image.shape[1], image.shape[0]] != data['frame_size']:
            raise ValueError(f'{path.name}: missing or invalid full-resolution image')
        kinds = {m.get('point_kind', 'contour_bounding_box_center') for m in data['materials']}
        if not kinds <= {'contour_bounding_box_center', 'contour_area_centroid'} or len(kinds) > 1:
            raise ValueError('unsupported or mixed coordinate point definitions')
        method = 'centroid' if kinds == {'contour_area_centroid'} else 'box'
        def detect(profile):
            detector = HSVMaterialDetector(profile=profile, center_method=method)
            if data['roi'] is not None:
                detector = MaterialRegionDetector(detector, data['roi'])
            return [asdict(m) for m in ordered_materials(detector.detect(image))]
        original = detect(baseline)
        # Tracker IDs are live session metadata, not reproducible detector results.
        def geometry(items):
            return [{key: item.get(key, 'contour_bounding_box_center' if key == 'point_kind' else None)
                     for key in ('index', 'color_id', 'shape_id', 'center_x', 'center_y',
                                 'theta_deg', 'confidence', 'point_kind')} for item in items]
        result = detect(candidate) if candidate is not None else original
        report['samples'].append({
            'source_sequence': data['source_sequence'], 'image': image_name,
            'baseline_matches_recorded': geometry(original) == geometry(data['materials']),
            'profile_sha256': profile_hash(candidate if candidate is not None else baseline),
            'materials': result,
        })
    if not report['samples']:
        raise ValueError('No diagnostic samples found')
    report['baseline_exact_matches'] = sum(s['baseline_matches_recorded'] for s in report['samples'])
    return report


def main():
    parser = argparse.ArgumentParser(description='原图离线回放；可比较候选 HSV 参数，不占用相机')
    parser.add_argument('directory')
    parser.add_argument('--profile', help='候选 YAML 参数，省略时回放样本所记参数')
    parser.add_argument('--output', help='可选 JSON 报告文件')
    args = parser.parse_args()
    report = replay(args.directory, load_profile(args.profile) if args.profile else None)
    if args.output:
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['baseline_exact_matches'] == len(report['samples']) else 1


if __name__ == '__main__':
    raise SystemExit(main())
