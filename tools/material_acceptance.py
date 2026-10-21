"""Read-only static material sampling. Uses the running server; opens no camera."""

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time
import urllib.request


COLORS = {1: '红', 2: '黄', 3: '蓝', 4: '绿', 5: '黑', 6: '浅蓝'}


def summarize(records, expected, max_age_ms=1000):
    seen = set()
    fresh = []
    counts = Counter()
    last_sequence = None
    for record in records:
        if 'error' in record:
            counts['request_errors'] += 1
            continue
        state = record['state']
        obs = state.get('material_observation', {})
        sequence, age = obs.get('sequence'), obs.get('age_ms')
        capture_age = state.get('capture', {}).get('age_ms')
        if not isinstance(sequence, int) or sequence <= 0:
            counts['no_observation'] += 1
            continue
        if last_sequence is not None and sequence < last_sequence:
            counts['sequence_regressions'] += 1
            continue
        last_sequence = sequence
        if sequence in seen:
            counts['duplicate_polls'] += 1
            continue
        seen.add(sequence)
        ages = (age, capture_age)
        if any(not isinstance(a, (int, float)) or not math.isfinite(a)
               or a < 0 or a + record['request_ms'] > max_age_ms for a in ages):
            counts['stale_or_unknown_age'] += 1
            continue
        if not state.get('detection', {}).get('materials', False):
            counts['detection_disabled'] += 1
            continue
        fresh.append(state)
    result = {'polls': len(records), **dict(counts), 'fresh_unique_observations': len(fresh),
              'max_age_including_request_ms': max_age_ms, 'colors': {}}
    for color in expected:
        points = []
        missing = ambiguous = 0
        for state in fresh:
            matches = [m for m in state.get('materials', []) if m.get('color_id') == color]
            if not matches:
                missing += 1
            elif len(matches) != 1:
                ambiguous += 1
            else:
                points.append(matches[0])
        item = {'name': COLORS[color], 'single_detection': len(points),
                'missing': missing, 'multiple_detections': ambiguous,
                'single_detection_rate': round(len(points)/len(fresh), 4) if fresh else None}
        if points:
            for axis in ('center_x', 'center_y'):
                values = [p[axis] for p in points]
                item[axis + '_range_px'] = [min(values), max(values)]
                item[axis + '_span_px'] = max(values) - min(values)
        result['colors'][str(color)] = item
    result['unexpected_color_observations'] = sum(
        any(m.get('color_id') not in expected for m in s.get('materials', [])) for s in fresh)
    result['limitations'] = '静止场景抽样，不是全部推理帧；不能验证抓取精度或正确率，需人工确认颜色和目标对应。'
    return result


def main():
    parser = argparse.ArgumentParser(description='物料静止验收：读取现有网页状态，不占用相机或串口')
    parser.add_argument('--url', default='http://127.0.0.1:8090')
    parser.add_argument('--colors', type=int, nargs='+', required=True, choices=COLORS)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--output', default='output/material_diagnostics/acceptance.json')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds 须为正数')
    records = []
    deadline = time.monotonic() + args.seconds
    consecutive_errors = 0
    print('保持相机、物料不动；正在读取识别状态…', flush=True)
    while time.monotonic() < deadline:
        start = time.monotonic()
        try:
            with urllib.request.urlopen(args.url.rstrip('/') + '/state', timeout=min(2, max(.1, deadline-start))) as response:
                state = json.load(response)
            if not isinstance(state, dict):
                raise ValueError('状态接口未返回对象')
            records.append({'request_ms': round((time.monotonic()-start)*1000, 1), 'state': state})
            consecutive_errors = 0
        except Exception as error:
            records.append({'error': str(error)})
            consecutive_errors += 1
            if consecutive_errors >= 3:
                print('连续3次请求失败，提前停止；请检查识别服务与网络。', flush=True)
                break
        remaining = deadline-time.monotonic()
        if remaining > 0:
            time.sleep(min(.2, remaining))
    report = summarize(records, set(args.colors))
    path = Path(args.output).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({'expected_colors': sorted(set(args.colors)),
                                'summary': report, 'records': records}, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f'原始记录已保存：{path.resolve()}')
    return 0 if report['fresh_unique_observations'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
