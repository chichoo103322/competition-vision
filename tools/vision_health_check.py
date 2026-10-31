"""Software fault injection through the real HTTP handler. Opens no physical devices."""

import argparse
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import urllib.error
import urllib.request

import numpy as np

from pi_vision import stream
from pi_vision.types import Material


class SyntheticCamera:
    def __init__(self):
        self.fail = False
        self.pause = False
        self.target = True
        self.readers = self.max_readers = self.opens = 0

    def open(self):
        self.opens += 1
        if self.fail:
            raise RuntimeError('injected_camera_open_failure')

    def release(self):
        pass

    def read(self):
        self.readers += 1
        self.max_readers = max(self.max_readers, self.readers)
        try:
            while self.pause:
                time.sleep(.01)
            time.sleep(.02)
            if self.fail:
                return False, None, 0.
            image = np.full((120, 160, 3), 180, np.uint8)
            image[0, 0, 0] = int(self.target)
            return True, image, time.time()
        finally:
            self.readers -= 1


class SyntheticDetector:
    def __init__(self):
        self.mode = 'ok'
        self.entered = threading.Event()
        self.release = threading.Event()

    def detect(self, image):
        mode = self.mode
        if mode == 'error':
            raise RuntimeError('injected_detector_error')
        if mode == 'slow':
            self.entered.set()
            self.release.wait(5)
        return [Material(1, 1, 0, 80, 60, 0., 90)] if image[0, 0, 0] else []


def run_check():
    camera, detector = SyntheticCamera(), SyntheticDetector()
    pipeline = SimpleNamespace(qr_camera=camera, material_detector=detector)
    runner = stream.FrameStreamer(camera, pipeline, enable_qr=False, enable_rings=False,
                                  capture_timeout=.2, result_timeout=.2, detect_interval=.05,
                                  preview_fps=10, draw_overlay=False)
    stream._streamer = runner
    server = stream.ThreadingHTTPServer(('127.0.0.1', 0), stream.StreamHandler)
    address = f'http://127.0.0.1:{server.server_address[1]}'
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    checks = []

    def state():
        with urllib.request.urlopen(address+'/state', timeout=1) as response:
            return json.load(response)

    def wait_for(label, predicate):
        deadline = time.monotonic()+4
        while time.monotonic() < deadline:
            current = state()
            if predicate(current):
                checks.append({'check': label, 'passed': True,
                               'capture_status': current['capture']['status'],
                               'material_status': current['material_observation']['status'],
                               'source_sequence': current['material_observation']['source_sequence']})
                return current
            time.sleep(.02)
        raise AssertionError(f'{label}: {current}')

    def healthy(s):
        return s['capture']['valid'] and s['material_observation']['valid'] and bool(s['materials'])

    runner.start()
    server_thread.start()
    try:
        wait_for('fresh_target', healthy)
        camera.target = False
        wait_for('fresh_no_target', lambda s: s['material_observation']['status']=='no_target' and not s['materials'])
        camera.target = True
        wait_for('target_returns', healthy)
        detector.mode = 'error'
        wait_for('detector_error_not_no_target', lambda s: s['material_observation']['status']=='error' and not s['materials'])
        detector.mode = 'ok'
        wait_for('detector_recovers', healthy)
        detector.mode = 'slow'
        if not detector.entered.wait(2):
            raise AssertionError('slow detector not entered')
        wait_for('processing_timeout', lambda s: s['material_observation']['status']=='timeout' and not s['materials'])
        detector.mode = 'ok'
        detector.release.set()
        wait_for('late_result_discarded_then_new_frame_recovers', healthy)
        camera.pause = True
        wait_for('capture_stall_expires_coordinates', lambda s: s['capture']['status']=='stale' and not s['materials'])
        try:
            urllib.request.urlopen(address+'/frame.jpg', timeout=1)
            raise AssertionError('stale image returned HTTP 200')
        except urllib.error.HTTPError as error:
            if error.code != 503:
                raise
        checks.append({'check': 'stale_image_http_503', 'passed': True})
        camera.pause = False
        wait_for('capture_stall_recovers', healthy)
        camera.fail = True
        wait_for('camera_read_error', lambda s: s['capture']['status']=='error' and not s['materials'])
        wait_for('same_owner_reopen_attempt', lambda s: s['capture']['reopen_attempts']>0)
        camera.fail = False
        wait_for('camera_read_error_recovers', healthy)
        if camera.max_readers != 1:
            raise AssertionError('camera had multiple readers')
        checks.append({'check': 'single_capture_owner', 'passed': True})
        return {'kind': 'software_fault_injection_not_physical_camera_test', 'passed': True,
                'checks': checks, 'max_simultaneous_readers': camera.max_readers}
    finally:
        camera.pause = camera.fail = False
        detector.release.set()
        server.shutdown()
        server.server_close()
        runner.stop()


def main():
    parser = argparse.ArgumentParser(description='软件故障注入，不打开真实相机或串口')
    parser.add_argument('--output')
    args = parser.parse_args()
    report = run_check()
    if args.output:
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
