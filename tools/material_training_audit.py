"""Read training evidence without opening camera, serial, or loading PT checkpoints."""

import argparse
import csv
import hashlib
import io
import json
import math
from collections import Counter
from datetime import datetime
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile


IMAGE_SUFFIXES = {'.jpg', '.jpeg', '.png', '.bmp', '.webp', '.tif', '.tiff'}


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def inspect_labels(folder):
    images = folder.parent / 'images'
    image_keys = Counter(str(p.relative_to(images).with_suffix(''))
                         for p in images.rglob('*')
                         if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES)
    label_keys = set()
    objects, image_counts = Counter(), Counter()
    invalid, examples, empty = 0, [], 0
    for path in sorted(folder.rglob('*.txt')):
        key = str(path.relative_to(folder).with_suffix(''))
        label_keys.add(key)
        text = path.read_text(errors='replace').strip()
        empty += not bool(text)
        seen = set()
        for number, line in enumerate(text.splitlines(), 1):
            try:
                values = [float(v) for v in line.split()]
                if not all(math.isfinite(v) for v in values):
                    raise ValueError('nonfinite')
                if len(values) != 5 and not (len(values) >= 7 and len(values) % 2 == 1):
                    raise ValueError('不是框或多边形格式')
                cls = int(values[0])
                if values[0] != cls or cls < 0 or not all(0 <= v <= 1 for v in values[1:]):
                    raise ValueError('类别编号或归一化坐标非法')
                if len(values) == 5 and (values[3] <= 0 or values[4] <= 0):
                    raise ValueError('框宽高为零')
                objects[cls] += 1
                seen.add(cls)
            except ValueError as exc:
                invalid += 1
                if len(examples) < 10:
                    examples.append(f'{path.name}:{number}: {exc}')
        image_counts.update(seen)
    return {
        'image_files': sum(image_keys.values()), 'label_files': len(label_keys),
        'empty_label_files': empty,
        'objects_by_class_index': dict(sorted(objects.items())),
        'label_files_by_class_index': dict(sorted(image_counts.items())),
        'images_without_label': len(set(image_keys) - label_keys),
        'labels_without_image': len(label_keys - set(image_keys)),
        'duplicate_image_stems': sum(n > 1 for n in image_keys.values()),
        'invalid_rows': invalid, 'invalid_examples': examples,
    }


def main():
    parser = argparse.ArgumentParser(description='只读训练审计；不打开相机或串口')
    parser.add_argument('--root', type=Path, default=Path.cwd())
    args = parser.parse_args()
    root = args.root.resolve()
    if not (root / 'pi_vision').is_dir():
        parser.error('请在 competition_vision 工程目录运行')
    output = root / 'output' / 'training_audit'
    output.mkdir(parents=True, exist_ok=True)
    bundle = output / ('audit_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.zip')
    report = {'root': str(root), 'datasets': {}, 'runs': {}, 'models': {}, 'errors': [],
              'limitations': '标签计数不验证颜色标注是否正确；未检查图像内容或训练验证泄漏；'
                              '未加载PT权重；SHA仅用于识别文件，不能证明导出对应关系。'}
    print('正在检查训练记录、标签和模型文件；相机服务可以保持运行。', flush=True)
    with ZipFile(bundle, 'w', ZIP_DEFLATED) as archive:
        configs = set((root / 'datasets').rglob('*.yaml'))
        configs.update((root / 'config').glob('vision_config.yaml'))
        for path in sorted(configs):
            archive.write(path, str(path.relative_to(root)))
        for folder in sorted((root / 'datasets').rglob('labels')):
            if not folder.is_dir():
                continue
            name = str(folder.relative_to(root))
            try:
                report['datasets'][name] = inspect_labels(folder)
            except OSError as exc:
                report['errors'].append(f'{name}: {exc}')
        for path in sorted((root / 'runs').rglob('args.yaml')):
            name = str(path.parent.relative_to(root))
            archive.write(path, str(path.relative_to(root)))
            record = {}
            result = path.parent / 'results.csv'
            if result.is_file():
                archive.write(result, str(result.relative_to(root)))
                rows = list(csv.DictReader(io.StringIO(result.read_text(errors='replace'))))
                record['epochs_recorded'] = len(rows)
                record['last_row'] = {k.strip(): v.strip() for k, v in rows[-1].items()
                                      if k is not None and v is not None} if rows else {}
            checkpoint = path.parent / 'weights' / 'best.pt'
            if checkpoint.is_file():
                record['best_pt_bytes'] = checkpoint.stat().st_size
                record['best_pt_sha256'] = sha256(checkpoint)
            report['runs'][name] = record
        try:
            import onnx
        except ImportError:
            onnx = None
        for path in sorted((root / 'models').glob('*')):
            if not path.is_file():
                continue
            record = {'bytes': path.stat().st_size, 'sha256': sha256(path)}
            if path.suffix == '.onnx':
                if onnx is None:
                    record['metadata_status'] = '未安装onnx，跳过；不会自动安装依赖'
                else:
                    try:
                        model = onnx.load(str(path), load_external_data=False)
                        record['metadata'] = {p.key: p.value for p in model.metadata_props}
                        del model
                    except Exception as exc:
                        record['metadata_error'] = str(exc)
            report['models'][path.name] = record
        archive.writestr('audit.json', json.dumps(report, ensure_ascii=False, indent=2))
    for name, stats in report['datasets'].items():
        print(f"{name}: 图片{stats['image_files']}，标签{stats['label_files']}，"
              f"各类实例{stats['objects_by_class_index']}，非法行{stats['invalid_rows']}")
    print(f"训练记录{len(report['runs'])}份，模型文件{len(report['models'])}份。")
    for error in report['errors']:
        print('检查未完成项:', error)
    print('报告包已保存:', bundle)
    print('未修改训练数据、模型或配置；未打开相机或串口。')


if __name__ == '__main__':
    main()
