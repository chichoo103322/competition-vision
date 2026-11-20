"""Read-only HTTP view of the existing UART owner's JSONL; never opens serial."""
import argparse
import gzip
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import struct
import threading
import time
from urllib.parse import parse_qs, urlsplit

from pi_vision import electrical_protocol as p
from pi_vision import arm_protocol as arm

NAMES = {p.UPLOAD: '上传路线', p.UPLOAD_FOR_AREA: '按键绑定上传路线',
         p.START: '开始运行', p.STOP: '停止', p.WORK_DONE: '作业完成放行',
         p.STATUS: '查询状态', p.AREA_ACK: '确认启停区', p.START_AREA: '实体按键选区',
         p.ACK: '命令回复', p.EVENT: '控制器事件', p.VISION: '视觉数据'}
NAMES.update({arm.ARM_COMMAND: '云台指令', arm.ARM_STATUS: '查询云台', arm.ARM_EVENT: '云台反馈'})
PHASES = {4: '等待作业', 5: '连续路径行驶', 6: '工作区原地转向'}


def interpret(kind, body):
    name = NAMES.get(kind, f'未知消息 0x{kind:02X}')
    details = {}
    routine = kind == p.STATUS
    if kind == arm.ARM_EVENT:
        details = arm.arm_status_payload(body)
        name = '云台 · '+details['state_name']
    elif kind in (p.ACK, p.EVENT):
        details = p.status_payload(body)
        request = details['request']
        code = details['code']
        if kind == p.ACK:
            result = p.RESULTS[code] if code < len(p.RESULTS) else str(code)
            name = f"{NAMES.get(request, hex(request))}回复 · {result}"
            routine = request == p.STATUS and code == 0
        else:
            name = '事件 · ' + p.EVENTS.get(code, str(code))
            details['event_name'] = p.EVENTS.get(code, str(code))
        details['phase_name'] = PHASES.get(details['phase'], str(details['phase']))
    elif kind == p.START_AREA:
        details = p.start_area_payload(body)
        name = f"KEY{details['key']} · 选择启停区{details['area']}"
    elif kind == p.AREA_ACK:
        request_id, area = struct.unpack('<IB3x', body)
        details = dict(request_id=request_id, area=area)
    elif kind in (p.UPLOAD, p.UPLOAD_FOR_AREA):
        if kind == p.UPLOAD_FOR_AREA:
            details['request_id'], = struct.unpack_from('<I', body)
            body = body[4:]
        route_id, count, reserved, x, y, yaw = struct.unpack_from('<IHHiii', body)
        if len(body) != 20 + count * 20:
            raise ValueError('路线长度与点数不符')
        points = []
        for i in range(count):
            px, py, pyaw, move, turn, work, flags = struct.unpack_from('<iiiHHHH', body, 20+i*20)
            points.append(dict(point_index=i, x_mm=px, y_mm=py, yaw_deg=pyaw/100,
                               move_rpm=move, turn_rpm=turn, work_id=work, flags=flags))
        details.update(route_id=route_id, point_count=count,
                       initial_pose=dict(x_mm=x, y_mm=y, yaw_deg=yaw/100), points=points)
        name += f' · {count}点'
    elif kind in (p.START, p.STOP):
        if len(body) == 4:
            details['route_id'], = struct.unpack('<I', body)
    elif kind == p.WORK_DONE:
        route, index, success, reserved = struct.unpack('<IHBB', body)
        details = dict(route_id=route, point_index=index, success=success, reserved=reserved)
    return name, details, routine


class LogSource:
    def __init__(self, root):
        self.root = Path(root)
        self.stream = None
        self.identity = None
        self.pending = b''
        self.decoders = {d: p.Decoder(clock=lambda: 0) for d in ('TX', 'RX_RAW')}
        self.entries = deque(maxlen=600)
        self.important = deque(maxlen=300)
        self.serial = 0
        self.last_tx = self.last_rx = self.last_status_at = None
        self.controller = None
        self.last_key = None
        self.warning = None

    def append(self, item):
        self.serial += 1
        entry = dict(item, id=self.serial)
        self.entries.append(entry)
        if not item.get('routine') and not item.get('raw'):
            self.important.append(entry)

    def consume(self, item):
        event = item.get('event', '')
        at = item.get('at')
        if not isinstance(at, (float, int)):
            return
        if event not in self.decoders:
            self.append(dict(at=at, direction='SYS', name=event,
                             details=item, routine=False))
            return
        raw = bytes.fromhex(item.get('hex', ''))
        if event == 'TX':
            self.last_tx = at
        else:
            self.last_rx = at
        packets = self.decoders[event].feed(raw)
        self.append(dict(at=at, direction='TX' if event == 'TX' else 'RX',
                         name='原始数据块', raw=True, routine=True,
                         hex=raw.hex(' '), details={'bytes': len(raw)}))
        for kind, seq, body in packets:
            try:
                name, details, routine = interpret(kind, body)
                error = None
            except (ValueError, struct.error) as exc:
                name, details, routine, error = '报文内容解析失败', {}, False, str(exc)
            if event == 'RX_RAW' and not error:
                if kind in (p.ACK, p.EVENT):
                    self.controller = details
                    self.last_status_at = at
                elif kind == p.START_AREA:
                    self.last_key = dict(details, at=at)
            self.append(dict(at=at, direction='TX' if event == 'TX' else 'RX',
                             name=name, kind=kind, sequence=seq, details=details,
                             routine=routine, error=error, hex=p.frame(kind, seq, body).hex(' ')))

    def update(self):
        path = self.root / 'wire.jsonl'
        try:
            stat = path.stat()
            identity = (stat.st_dev, stat.st_ino)
            if self.stream is None or identity != self.identity or stat.st_size < self.stream.tell():
                if self.stream:
                    # Drain the old inode before following a rotated log.
                    self.read_chunk()
                    self.stream.close()
                self.stream = path.open('rb')
                self.identity = identity
                self.pending = b''
                self.decoders = {d: p.Decoder(clock=lambda: 0) for d in ('TX', 'RX_RAW')}
                if stat.st_size > 1_000_000:
                    self.stream.seek(-1_000_000, 2)
                    self.stream.readline()
            self.read_chunk()
        except FileNotFoundError:
            self.warning = '尚未找到通信日志；等待后台服务生成。'
        except OSError as exc:
            self.warning = f'读取日志失败：{exc}'

    def read_chunk(self):
        data = self.pending + self.stream.read(1_000_000)
        lines = data.split(b'\n')
        self.pending = lines.pop()
        self.warning = None
        for line in lines:
            try:
                self.consume(json.loads(line))
            except (ValueError, TypeError, AttributeError):
                self.warning = '部分日志行损坏，已跳过；原文件未修改。'

    def snapshot(self):
        self.update()
        def read_json(name):
            try:
                return json.loads((self.root/name).read_text())
            except (OSError, ValueError):
                return None
        now = time.time()
        age = None if self.last_status_at is None else max(0, now-self.last_status_at)
        entries = {e['id']: e for e in (*self.important, *self.entries)}
        return dict(now=now, entries=sorted(entries.values(), key=lambda e: e['id']), warning=self.warning,
                    last_tx=self.last_tx, last_rx=self.last_rx, last_status_at=self.last_status_at,
                    status_age_s=age, controller_recent=age is not None and age < 5,
                    controller=self.controller, last_key=self.last_key,
                    service=read_json('status.json'), result=read_json('result.json'))


def create_server(sources, port=8767, historical=False):
    lock = threading.Lock()
    html = Path(__file__).with_suffix('.html').read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_GET(self):
            url = urlsplit(self.path)
            source = sources.get(parse_qs(url.query).get('source', [next(iter(sources))])[0])
            if not source:
                self.send_error(400, 'Unknown source')
                return
            if url.path == '/':
                data, content_type = html, 'text/html; charset=utf-8'
            elif url.path == '/api/state':
                with lock:
                    state = source.snapshot()
                state['sources'] = list(sources)
                state['historical'] = historical
                data, content_type = json.dumps(state, ensure_ascii=False).encode(), 'application/json'
            elif url.path == '/favicon.ico':
                self.send_response(204)
                self.end_headers()
                return
            elif url.path == '/download':
                # Fixed filename only; no paths from the request are opened.
                try:
                    stream = (source.root/'wire.jsonl').open('rb')
                except OSError:
                    self.send_error(404, 'Log not available')
                    return
                with stream:
                    import os
                    remaining = os.fstat(stream.fileno()).st_size
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/x-ndjson')
                    self.send_header('Content-Disposition', 'attachment; filename="wire.jsonl"')
                    self.send_header('Content-Length', str(remaining))
                    self.end_headers()
                    while remaining:
                        chunk = stream.read(min(65536, remaining))
                        if not chunk:
                            break
                        self.wfile.write(chunk)
                        remaining -= len(chunk)
                return
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Cache-Control', 'no-store')
            if url.path == '/api/state' and 'gzip' in self.headers.get('Accept-Encoding', ''):
                data = gzip.compress(data, compresslevel=1)
                self.send_header('Content-Encoding', 'gzip')
                self.send_header('Vary', 'Accept-Encoding')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

    return ThreadingHTTPServer(('127.0.0.1', port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--test-root', type=Path)
    parser.add_argument('--competition-root', type=Path)
    parser.add_argument('--historical', action='store_true', help='label local archived-log preview')
    parser.add_argument('--port', type=int, default=8767)
    args = parser.parse_args()
    def root(override, mode):
        if override is not None:
            return override
        try:
            config = json.loads(Path(f'config/{mode}_button.local.json').read_text())
            return Path(config['output_root'])
        except (OSError, ValueError, KeyError):
            return Path(f'output/{mode}_button')
    server = create_server({'test': LogSource(root(args.test_root, 'chassis')),
                            'competition': LogSource(root(args.competition_root, 'competition'))},
                           args.port, args.historical)
    print(f'通信监看：http://127.0.0.1:{server.server_port}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
