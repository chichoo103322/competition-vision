"""Local browser QA with synthetic camera frames; no camera or serial access."""

from http.server import ThreadingHTTPServer
import threading
import time

import cv2
import numpy as np

from pi_vision import stream


class SyntheticPreview:
    capture_timeout = 1.
    preview_timeout = 3.
    def __init__(self):
        self.started = time.monotonic()

    def get_preview_frame(self):
        now = time.monotonic()
        sequence = int((now - self.started) * 5)
        image = np.full((720, 1280, 3), (55, 45, 25), np.uint8)
        cv2.rectangle(image, ((sequence * 17) % 550, 130),
                      ((sequence * 17) % 550 + 70, 200), (50, 200, 80), -1)
        cv2.putText(image, f'Browser test frame {sequence}', (30, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.circle(image,(180,190),16,(0,220,220),-1)
        cv2.circle(image,(340,200),16,(220,0,0),-1)
        ok, jpeg = cv2.imencode('.jpg', cv2.resize(image,(640,360)))
        assert ok
        return jpeg.tobytes(), sequence, now

    def get_frame_width(self):
        return 1280

    def get_frame_height(self):
        return 720

    def get_state(self):
        return {'task': None, 'capture': {'status': 'ok', 'valid': True},
                'qr_observation': {'valid': False}, 'ring_observation': {'valid': False},
                'material_observation': {'valid': True, 'status': 'ok', 'age_ms': 0, 'timeout_ms': 1000},
                'qr': None, 'materials': [
                    {'color_id': 2, 'color_name': '黄', 'center_x': 180, 'center_y': 190, 'confidence': 38},
                    {'color_id': 3, 'color_name': '蓝', 'center_x': 340, 'center_y': 200, 'confidence': 36}],
                'material_roi': [100, 80, 400, 220], 'rings': [], 'rotation': '已关闭',
                'detection': {'qr': False, 'materials': True, 'rings': False}}


if __name__ == '__main__':
    stream._streamer = SyntheticPreview()
    server = ThreadingHTTPServer(('127.0.0.1', 8091), stream.StreamHandler)
    print('Synthetic preview browser QA: http://127.0.0.1:8091/', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
