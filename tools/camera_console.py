"""Read-only camera console. Reuses material HTTP frames and QR owner's preview files."""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import time
import threading
import subprocess
from urllib.parse import parse_qs, urlsplit
from urllib.request import urlopen
from tools.qr_station_work import QRStationWork


class QRPreview:
    def __init__(self, root, config):
        self.root, self.config = root, config
        self.lock = threading.Lock()
        self.worker, self.last_view = None, 0.
        self.stop = threading.Event()
        threading.Thread(target=self._watch, daemon=True).start()

    def _release(self):
        if self.worker is not None:
            self.worker.close()
            self.worker = None

    def release(self):
        with self.lock:
            self._release()

    def view(self):
        with self.lock:
            self.last_view = time.monotonic()
            status = json.loads((self.root/'status.json').read_text())
            if status['state'] not in ('READY', 'WAIT_CONTROLLER', 'WAIT_DEVICES'):
                return  # Route worker owns prewarm/scan. View its published frames.
            if self.worker is not None:
                return
            if not self.config:
                raise ValueError('QR camera is not configured')
            device = self.config['camera']
            holders = subprocess.run(['fuser', device], capture_output=True, timeout=2)
            if holders.returncode != 1:
                return
            config = dict(self.config, preview_dir=str(self.root/'qr_camera'))
            config.pop('preview_control_url', None)
            self.worker = QRStationWork(config)
            try:
                self.worker.prewarm()
            except BaseException:
                self._release()
                raise

    def _watch(self):
        while not self.stop.wait(1):
            with self.lock:
                if self.worker is not None and time.monotonic()-self.last_view > 8:
                    self._release()

    def close(self):
        self.stop.set()
        self.release()


def material_snapshot():
    with urlopen('http://127.0.0.1:8090/state', timeout=2) as response:
        return json.load(response)


def qr_snapshot(root, now=None):
    now = time.time() if now is None else now
    try:
        state = json.loads((root/'qr_camera/state.json').read_text())
        age = now-state['acquired_unix_s']
        valid = state.get('active') is True and 0 <= age < 1
        return dict(valid=valid, age_ms=round(age*1000, 1), sequence=state['sequence'])
    except (OSError, ValueError, KeyError, TypeError):
        return dict(valid=False, age_ms=None, sequence=0)


def task_snapshot(root):
    try:
        state = json.loads((root/'material_work.json').read_text())
        if state['phase'] not in ('done', 'stopped') and not 0 <= time.time()-state['updated_unix_s'] < 3:
            return None
        return state
    except (OSError, ValueError, KeyError, TypeError):
        return None


def create_server(root, port=8091, reference_path=None, qr_config=None):
    preview = QRPreview(root, qr_config)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_POST(self):
            if self.headers.get('Sec-Fetch-Site') == 'cross-site':
                self.send_error(403)
                return
            path = urlsplit(self.path).path
            try:
                if path == '/qr/preview':
                    preview.view()
                elif path == '/qr/release':
                    preview.release()
                else:
                    self.send_error(404)
                    return
            except (OSError, ValueError, RuntimeError, subprocess.SubprocessError):
                self.send_error(503, 'QR preview unavailable')
                return
            self.send_response(200)
            self.send_header('Content-Length', '0')
            self.end_headers()

        def do_GET(self):
            url = urlsplit(self.path)
            camera = parse_qs(url.query).get('camera', ['material'])[0]
            if camera not in ('material', 'qr'):
                self.send_error(400)
                return
            try:
                if url.path == '/api/state':
                    if camera == 'material':
                        vision = material_snapshot()
                        state = dict(valid=vision.get('capture', {}).get('valid') is True,
                                     vision=vision, task=task_snapshot(root))
                        if reference_path is not None and reference_path.exists():
                            reference = json.loads(reference_path.read_text())
                            size = [vision['preview']['source_width'], vision['preview']['source_height']]
                            if (reference.get('camera_source') == vision.get('camera_source')
                                    and reference.get('resolution') == size
                                    and reference.get('hsv_profile_sha256') == vision.get('hsv_profile_sha256')):
                                state['reference_pixel'] = reference.get('reference_pixel')
                    else:
                        state = qr_snapshot(root)
                    state['camera'] = camera
                    data, kind = json.dumps(state, ensure_ascii=False).encode(), 'application/json'
                elif url.path == '/frame.jpg':
                    if camera == 'qr':
                        if not qr_snapshot(root)['valid']:
                            self.send_error(503, 'QR camera is closed or no fresh frame')
                            return
                        data = (root/'qr_camera/frame.jpg').read_bytes()
                    else:
                        with urlopen('http://127.0.0.1:8090/frame.jpg', timeout=2) as response:
                            data = response.read()
                    kind = 'image/jpeg'
                else:
                    self.send_error(404)
                    return
            except (OSError, ValueError, KeyError):
                self.send_error(503, 'Camera source unavailable')
                return
            self.send_response(200)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.qr_preview = preview
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path('config/chassis_button.local.json'))
    parser.add_argument('--port', type=int, default=8091)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    server = create_server(Path(config['output_root']), args.port,
                           args.config.resolve().parent/'arm_reference.local.json', config.get('qr_station'))
    try:
        server.serve_forever()
    finally:
        server.qr_preview.close()
        server.server_close()


if __name__ == '__main__':
    main()
