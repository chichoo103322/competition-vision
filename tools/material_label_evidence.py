"""Export small, annotated label samples; never change source data or open camera."""
import argparse
import hashlib
import json
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED

import cv2

NAMES = ['red', 'yellow', 'blue', 'green', 'black', 'light_blue', 'base']
SUFFIXES = {'.jpg', '.jpeg', '.png', '.bmp', '.webp', '.tif', '.tiff'}


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser(description='导出标签图片证据，不修改原数据')
    parser.add_argument('--root', type=Path, default=Path.cwd())
    args = parser.parse_args()
    root = args.root.resolve()
    if not (root / 'datasets').is_dir():
        parser.error('请在 competition_vision 工程目录运行')
    records, buckets, errors = [], defaultdict(list), []
    print('正在核对图片与标签；相机服务可以保持运行。', flush=True)
    for folder in sorted((root / 'datasets').rglob('labels')):
        if not folder.is_dir():
            continue
        images = folder.parent / 'images'
        image_map = defaultdict(list)
        for path in images.rglob('*'):
            if path.is_file() and path.suffix.lower() in SUFFIXES:
                image_map[str(path.relative_to(images).with_suffix(''))].append(path)
        for label in sorted(folder.rglob('*.txt')):
            text = label.read_text(errors='replace')
            key = str(label.relative_to(folder).with_suffix(''))
            paths = image_map.get(key, [])
            if len(paths) != 1:
                errors.append(f'{label.relative_to(root)}: 图片匹配数量{len(paths)}')
                continue
            path = paths[0]
            try:
                rows = [[float(v) for v in row.split()] for row in text.splitlines() if row.strip()]
                classes = sorted(set(int(row[0]) for row in rows))
                record = {'image': str(path.relative_to(root)), 'label': str(label.relative_to(root)),
                          'label_text': text, 'image_sha256': digest(path), 'classes': classes}
                index = len(records)
                records.append(record)
                for cls in classes:
                    buckets[(str(folder.relative_to(root)), cls)].append(index)
            except (ValueError, OSError, IndexError) as exc:
                errors.append(f'{label.relative_to(root)}: {exc}')
    # Two deterministically spaced samples per directory/class, not independent validation.
    selected = set()
    for indexes in buckets.values():
        selected.add(indexes[0])
        selected.add(indexes[-1])
    output = root / 'output' / 'training_audit'
    output.mkdir(parents=True, exist_ok=True)
    destination = output / ('label_evidence_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.zip')
    with ZipFile(destination, 'w', ZIP_DEFLATED) as archive:
        for index in sorted(selected):
            record = records[index]
            frame = cv2.imread(str(root / record['image']))
            if frame is None:
                errors.append(record['image'] + ': 图片读取失败')
                continue
            height, width = frame.shape[:2]
            scale = min(1., 640 / max(height, width))
            frame = cv2.resize(frame, (round(width * scale), round(height * scale)), interpolation=cv2.INTER_AREA)
            h, w = frame.shape[:2]
            for line in record['label_text'].splitlines():
                if not line.strip():
                    continue
                row = [float(v) for v in line.split()]
                cls = int(row[0])
                if len(row) == 5:
                    _, x, y, bw, bh = row
                    x1, y1 = round((x-bw/2)*w), round((y-bh/2)*h)
                    x2, y2 = round((x+bw/2)*w), round((y+bh/2)*h)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 2)
                    name = NAMES[cls] if 0 <= cls < len(NAMES) else 'unknown'
                    label = f'{cls}: {name}'
                    cv2.putText(frame, label, (max(0, x1), max(18, y1)), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 0), 3)
                    cv2.putText(frame, label, (max(0, x1), max(18, y1)), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1)
            success, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if success:
                record['preview'] = f'previews/{index:04d}.jpg'
                archive.writestr(record['preview'], encoded.tobytes())
        archive.writestr('evidence.json', json.dumps({'records': records, 'errors': errors,
            'display_names_from_current_data_yaml': NAMES,
            'limitations': '仅抽样查看标注；预览缩放压缩；哈希只识别完全相同文件，不能排除相邻帧泄漏。'}, ensure_ascii=False, indent=2))
    print(f'已收集{len(records)}组标签记录、{sum("preview" in r for r in records)}张预览，未完成项{len(errors)}。')
    print('检查包:', destination)
    print('原始图片、标签、模型均未修改。')


if __name__ == '__main__':
    main()
