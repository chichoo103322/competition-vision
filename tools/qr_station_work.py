"""本轮按键后预热单个相机，到站后只确认新帧；扫码结束即关闭。"""
from dataclasses import replace
import math
import multiprocessing
import json
import logging
from pathlib import Path
import time

from pi_vision.task_confirmation import TaskCodeConfirmation, parse_competition_task


def publish_preview(directory, frame=None, sequence=0):
    """Publish the existing capture owner's frame; never opens a camera."""
    if directory is None:
        return
    try:
        root = Path(directory)
        root.mkdir(parents=True, exist_ok=True)
        metadata = dict(active=frame is not None, sequence=sequence, acquired_unix_s=time.time())
        if frame is not None:
            import cv2
            width = min(640, frame.shape[1])
            image = cv2.resize(frame, (width, round(frame.shape[0]*width/frame.shape[1])))
            ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 65])
            if not ok:
                return
            temp = root/'frame.tmp'
            temp.write_bytes(jpeg.tobytes())
            temp.replace(root/'frame.jpg')
        temp = root/'state.tmp'
        temp.write_text(json.dumps(metadata))
        temp.replace(root/'state.json')
        if frame is None:
            (root/'frame.jpg').unlink(missing_ok=True)
    except OSError as exc:
        logging.getLogger(__name__).warning('QR preview unavailable: %s', exc)


class FreshQRSession:
    """边界前的帧和到站时正在读取的帧不得参与确认。"""
    def __init__(self, started_at, timeout):
        self.started_at = started_at
        self.deadline = started_at + timeout
        self.confirmation = TaskCodeConfirmation(3, .2, 1.0)
        self.discard = 4  # 保守清掉到站边界附近的驱动队列，不改变确认阈值。
        self.last_sequence = 0
        self.first_read_started_at = None

    def accepts(self, sequence, read_started_at, acquired_at):
        if acquired_at >= self.deadline:
            raise TimeoutError('QR scan timeout; no confirmed task')
        if sequence <= self.last_sequence:
            return False
        self.last_sequence = sequence
        if self.discard:
            self.discard -= 1
            return False
        return read_started_at > self.started_at

    def observe(self, code, read_started_at, acquired_at, completed_at):
        if completed_at >= self.deadline:
            raise TimeoutError('QR decode exceeded deadline; no confirmed task')
        if self.first_read_started_at is None:
            self.first_read_started_at = read_started_at
        locked = self.confirmation.update(code, acquired_at)
        if locked is None:
            return None
        return dict(type='task_code', status='ok', confirmed=True, raw=str(locked),
                    matching_frames=self.confirmation.count,
                    elapsed_s=round(completed_at - self.started_at, 3),
                    first_read_started_at=self.first_read_started_at,
                    acquired_at=acquired_at, confirmed_at=completed_at)


def capture_worker(config, connection, heartbeat):
    """仅此子进程持有Camera，待机持续读帧但不提前解码或保留任务码。"""
    from pi_vision.camera import Camera
    from pi_vision.config import load_config
    from pi_vision.detectors.qr_detector import QRDetector
    from pi_vision.qr_check import inspect_frame
    camera = Camera(replace(load_config(config['vision_config']).camera,
                            source=config['camera']))
    result = None
    try:
        detector = QRDetector()
        camera.open()
        sequence = 0
        session = None
        last_preview = 0.
        while True:
            # Parent death closes the pipe; never keep an orphan camera owner.
            if connection.poll():
                command = connection.recv()
                if command['action'] == 'close':
                    break
                if command['action'] != 'scan' or session is not None:
                    raise RuntimeError('invalid or repeated QR scan command')
                session = FreshQRSession(command['started_at'], config['timeout_s'])
            read_started_at = time.monotonic()
            ok, frame, _ = camera.read()
            acquired_at = time.monotonic()
            if not ok or frame is None or frame.size == 0:
                raise RuntimeError('QR camera read failed')
            heartbeat.value = acquired_at
            sequence += 1
            if acquired_at-last_preview >= .2:
                publish_preview(config.get('preview_dir'), frame, sequence)
                last_preview = acquired_at
            if session is None or not session.accepts(sequence, read_started_at, acquired_at):
                continue
            code, _ = inspect_frame(detector, frame)
            result = session.observe(code, read_started_at, acquired_at, time.monotonic())
            if result is not None:
                result['frame_sequence'] = sequence
                break
    except (Exception, KeyboardInterrupt) as exc:
        result = dict(type='task_code', status='error', confirmed=False,
                      message=f'{type(exc).__name__}: {exc}')
    finally:
        camera.release()
        publish_preview(config.get('preview_dir'))
        heartbeat.value = 0
        # A successful result is visible only after releasing the camera.
        try:
            if result is not None:
                connection.send(result)
        except (OSError, EOFError):
            pass
        connection.close()


class QRStationWork:
    def __init__(self, config, *, clock=time.monotonic):
        self.camera = config['camera']
        if not isinstance(self.camera, str) or not self.camera.startswith('/dev/v4l/by-id/'):
            raise ValueError('qr_station.camera must be an explicit stable /dev/v4l/by-id/ path')
        self.vision_config = str(config.get('vision_config', 'config/vision_config.yaml'))
        self.preview_dir = config.get('preview_dir')
        self.preview_control_url = config.get('preview_control_url')
        if self.preview_control_url not in (None, 'http://127.0.0.1:8091/qr/release'):
            raise ValueError('QR preview handoff must use the local camera console')
        self.timeout = config.get('timeout_s', 15)
        if type(self.timeout) not in (int, float) or not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('qr_station.timeout_s must be positive and finite')
        self.clock = clock
        self.process = self.connection = self.heartbeat = None
        self.started_at = None
        self.result = None
        self.closed = False

    def prewarm(self):
        if self.process is not None or self.closed:
            raise RuntimeError('QR camera session already used')
        if self.preview_control_url:
            from urllib.request import Request, urlopen
            request = Request(self.preview_control_url, data=b'{}', headers={'Content-Type': 'application/json'})
            with urlopen(request, timeout=4):
                pass  # Viewer owner has released the camera before route capture starts.
        context = multiprocessing.get_context('spawn')
        self.connection, child = context.Pipe()
        self.heartbeat = context.Value('d', 0)
        config = dict(camera=self.camera, vision_config=self.vision_config, timeout_s=self.timeout,
                      preview_dir=self.preview_dir)
        self.process = context.Process(target=capture_worker,
            args=(config, child, self.heartbeat), daemon=True)
        try:
            self.process.start()
        except BaseException:
            self.process = None
            self.close()
            raise
        finally:
            child.close()

    def ready(self):
        if self.process is None or self.closed or not self.process.is_alive():
            return False
        last = self.heartbeat.value
        return last > 0 and 0 <= self.clock() - last < 1

    def start(self):
        if self.started_at is not None:
            raise RuntimeError('QR scan already started for this route')
        if not self.ready():
            raise RuntimeError('QR camera not prewarmed or frames stale')
        self.started_at = self.clock()
        self.connection.send(dict(action='scan', started_at=self.started_at))

    def poll(self):
        if self.result is not None:
            return self.result
        if self.started_at is None or self.closed:
            raise RuntimeError('QR scan not started')
        if self.clock() >= self.started_at + self.timeout:
            raise TimeoutError('QR camera worker deadline exceeded; no confirmed task')
        if not self.connection.poll():
            if not self.process.is_alive():
                raise RuntimeError('QR camera worker exited without confirmation')
            return None
        result = self.connection.recv()
        if (result.get('type') != 'task_code' or result.get('status') != 'ok' or
                result.get('confirmed') is not True):
            raise RuntimeError(f'QR scan failed: {result}')
        raw = str(parse_competition_task(result['raw']))
        if (result.get('matching_frames', 0) < 3 or
                result['first_read_started_at'] <= self.started_at or
                not self.started_at < result['acquired_at'] <= result['confirmed_at'] < self.started_at + self.timeout):
            raise RuntimeError('QR result failed frame freshness checks')
        self.result = dict(result, raw=raw, camera=self.camera)
        self.close()
        return self.result

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.process is not None:
            if self.process.is_alive():
                try:
                    self.connection.send(dict(action='close'))
                except (OSError, EOFError):
                    pass
                self.process.join(timeout=.25)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=.5)
            if self.process.is_alive():
                self.process.kill()
                self.process.join(timeout=.5)
        if self.connection is not None:
            self.connection.close()
        publish_preview(self.preview_dir)
