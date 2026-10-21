"""On-Pi capture for diagnosing missed/misclassified materials. No serial writes."""

import argparse
from dataclasses import replace
from datetime import datetime
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from pi_vision.camera import Camera
from pi_vision.config import load_config
from pi_vision.detectors.yolo_material_detector import YoloMaterialDetector


def inspect(detector, crop):
    tensor, scale, px, py = detector._preprocess(crop)
    output = detector._session.run(None, {detector._input_name: tensor})[0]
    pred = np.squeeze(output, axis=0)
    if pred.ndim != 2 or pred.shape[0] < 5:
        raise ValueError(f'不支持的模型输出形状: {output.shape}')
    scores = pred[4:]
    # 保留原始各类别分数，包括argmax未选中的类别；诊断阈值不用于生产。
    candidates = detector._postprocess(output, scale, px, py)
    candidates = sorted(candidates, key=lambda row: -row[5])[:30]
    return output, {
        'class_max_scores': [round(float(v), 5) for v in scores.max(axis=1)],
        'candidates_at_diagnostic_floor': [
            {'box_roi_xyxy': [round(float(v), 1) for v in row[:4]],
             'class_index': int(row[4]), 'score': round(float(row[5]), 5)}
            for row in candidates],
    }


def main():
    parser = argparse.ArgumentParser(description='原图和模型诊断；请先停止原视觉程序释放相机')
    parser.add_argument('--config', default='config/vision_config.yaml')
    parser.add_argument('--camera', default='0')
    parser.add_argument('--roi', nargs=4, type=int, default=[480, 60, 520, 450])
    parser.add_argument('--output', default='output/material_diagnosis')
    parser.add_argument('--samples', type=int, default=8)
    parser.add_argument('--jpeg', action='store_true', help='保存质量95的JPEG以减少传输量；默认无损PNG')
    args = parser.parse_args()
    if not 1 <= args.samples <= 100:
        parser.error('--samples 须为1至100')
    config = load_config(args.config)
    folder = Path(args.output) / datetime.now().strftime('%Y%m%d_%H%M%S')
    folder.mkdir(parents=True, exist_ok=False)
    detector = YoloMaterialDetector(config.detection.yolo_model_path, confidence=.03,
                                    iou_threshold=.45)
    detector.load()
    if detector._session is None:
        raise RuntimeError('模型未加载，停止诊断')
    metadata = detector._session.get_modelmeta().custom_metadata_map
    report = {'model_path': detector.model_path,
              'model_sha256': hashlib.sha256(Path(detector.model_path).read_bytes()).hexdigest(),
              'model_metadata': metadata, 'input_shape': detector._session.get_inputs()[0].shape,
              'production_threshold': config.detection.yolo_confidence,
              'diagnostic_floor': .03, 'roi': args.roi, 'undistort': False,
              'image_format': 'jpeg_quality_95' if args.jpeg else 'lossless_png', 'samples': []}
    print('模型类别元数据:', metadata.get('names', '模型未提供names，需要核对训练data.yaml'), flush=True)
    camera = Camera(replace(config.camera, source=args.camera, undistort=False))
    try:
        camera.open()
        for i in range(10 + args.samples):
            ok, frame, timestamp = camera.read()
            if not ok or frame is None:
                raise RuntimeError('相机读取失败')
            if i < 10:
                continue
            x, y, w, h = args.roi
            if x < 0 or y < 0 or w <= 0 or h <= 0 or x+w > frame.shape[1] or y+h > frame.shape[0]:
                raise ValueError('ROI超出画面')
            crop = frame[y:y+h, x:x+w]
            raw, observation = inspect(detector, crop)
            number = i-10
            # 原始画幅，不加标签；JPEG模式明确记录压缩格式。
            suffix = 'jpg' if args.jpeg else 'png'
            options = [cv2.IMWRITE_JPEG_QUALITY, 95] if args.jpeg else []
            if not cv2.imwrite(str(folder/f'frame_{number:02d}.{suffix}'), frame, options):
                raise RuntimeError('原图保存失败')
            np.savez_compressed(folder/f'output_{number:02d}.npz', output=raw)
            observation.update(sample=number, camera_timestamp=timestamp,
                               frame_size=[frame.shape[1], frame.shape[0]])
            report['samples'].append(observation)
            print(f'样本{number+1}/{args.samples}，各类别最高分:', observation['class_max_scores'], flush=True)
    finally:
        camera.release()
        (folder/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(f'诊断完成，文件保存在：{folder.resolve()}。请重新启动原预览命令。', flush=True)


if __name__ == '__main__':
    main()
