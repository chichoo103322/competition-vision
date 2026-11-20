"""Fault, clock, generation and stale-data regression tests; no physical devices."""

import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from pi_vision.config import default_config
from pi_vision.observation import Observation
from pi_vision.protocol import SCAN_MODE_MATERIAL, SCAN_MODE_QR
from pi_vision.stream import FrameStreamer, SerialController
from pi_vision.task_confirmation import parse_competition_task
from pi_vision.types import Material, Ring
from pi_vision import stream as stream_module


MATERIAL = Material(1, 1, 0, 20, 30, 0., 80)
POINTS = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], np.float32)
TASK = parse_competition_task('156+123+516+231')


class ObservationTests(unittest.TestCase):
    def test_empty_success_error_and_stale_are_different(self):
        obs = Observation()
        capture = {'valid': True, 'status': 'ok'}
        self.assertEqual(obs.state(10, True, capture, 1, False)['status'], 'waiting')
        obs.finish(1, 10, 1000, 10.1)
        self.assertEqual(obs.state(10.2, True, capture, 1, False)['status'], 'no_target')
        self.assertTrue(obs.state(10.2, True, capture, 1, False)['valid'])
        self.assertFalse(obs.state(11, True, capture, 1, True)['valid'])
        self.assertFalse(obs.state(9, True, capture, 1, True)['valid'])
        obs.finish(2, 11, 1001, 11.1, 'ValueError: failure')
        self.assertEqual(obs.state(11.2, True, capture, 1, False)['status'], 'error')
        obs.finish(3, 12, 1002, 12.1)
        state = obs.state(12.2, True, capture, 1, True)
        self.assertTrue(state['valid'])
        self.assertEqual(state['errors'], 1)

    def test_running_work_expires_without_waiting_for_a_native_detector(self):
        obs = Observation(busy_since=10)
        self.assertEqual(obs.state(11, True, {'valid': True, 'status': 'ok'}, 1, True)['status'], 'timeout')


class StreamValidityTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.clock = patch('pi_vision.stream.time.monotonic', side_effect=lambda: self.now)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.camera = Mock()
        self.detector = Mock()
        self.pipeline = SimpleNamespace(qr_camera=self.camera, material_detector=self.detector,
                                        ring_detector=Mock(), qr_detector=Mock())
        self.stream = FrameStreamer(self.camera, self.pipeline, enable_qr=False, enable_rings=False)
        self.addCleanup(self.stream._executor.shutdown, wait=False, cancel_futures=True)
        self.capture()

    def capture(self):
        self.stream._raw_sequence += 1
        self.stream._raw_acquired_at = self.now
        self.stream._raw_wall_time = 1000 + self.now
        self.stream._latest_raw = np.zeros((40, 60, 3), np.uint8)
        self.stream._capture_error = None

    def publish(self, result=None, error=None):
        return self.stream._finish_detection('materials', [MATERIAL] if result is None else result,
            self.stream._raw_sequence, self.stream._raw_acquired_at,
            self.stream._raw_wall_time, self.stream._qr_generation, self.stream._capture_epoch, error)

    def test_timestamps_frame_id_and_empty_success(self):
        self.publish()
        state = self.stream.get_state()
        self.assertEqual(state['capture']['acquired_unix_s'], 1010.)
        self.assertEqual(state['material_observation']['source_sequence'], 1)
        self.assertEqual(state['material_observation']['age_ms'], 0)
        self.assertEqual(state['coordinate_units'], 'pixels')
        self.assertEqual(len(self.stream.get_materials()), 1)
        first_id = state['materials'][0]['track_id']
        self.assertEqual(state['materials'][0]['tracking_status'], 'new')
        self.now += .1
        self.capture()
        self.publish([Material(1, 1, 0, 24, 32, 0., 80)])
        state = self.stream.get_state()
        self.assertEqual(state['materials'][0]['track_id'], first_id)
        self.assertEqual(state['materials'][0]['tracking_status'], 'updated')
        self.assertEqual((state['materials'][0]['center_x'], state['materials'][0]['center_y']), (24, 32))
        self.now += .1
        self.capture()
        self.publish([])
        state = self.stream.get_state()
        self.assertEqual(state['materials'], [])
        self.assertEqual(state['material_observation']['status'], 'no_target')
        self.assertTrue(state['material_observation']['valid'])
        self.assertFalse(state['material_observation']['has_target'])

    def test_result_expires_while_capture_continues(self):
        self.publish()
        self.now = 11.
        self.capture()
        state = self.stream.get_state()
        self.assertTrue(state['capture']['valid'])
        self.assertEqual(state['materials'], [])
        self.assertEqual(state['material_observation']['status'], 'stale')
        self.assertEqual(self.stream.get_materials(), [])

    def test_blocked_first_read_also_has_a_deadline(self):
        self.stream._raw_sequence = 0
        self.assertEqual(self.stream.get_state()['capture']['status'], 'waiting')
        self.now = 11.1
        self.assertEqual(self.stream.get_state()['capture']['status'], 'stale')

    def test_malformed_qr_points_never_lock_a_task(self):
        self.stream.set_scan_detection(SCAN_MODE_QR)
        for index in range(3):
            self.now = 10.1+index*.11
            self.capture()
            self.stream._accept_qr_observation((TASK, [[float('nan'), 0]]*4), index+2,
                                               self.now, self.stream._qr_generation)
        self.assertIsNone(self.stream.get_qr())
        self.assertEqual(self.stream.get_state()['qr_observation']['status'], 'error')

    def test_failed_capture_invalidates_inflight_work_and_jpeg(self):
        self.publish()
        epoch = self.stream._capture_epoch
        self.stream._latest_jpeg = b'old'
        self.stream._jpeg_sequence = 1
        self.stream._jpeg_acquired_at = self.now
        self.stream._capture_failed('disconnected')
        self.assertFalse(self.stream.get_state()['capture']['valid'])
        self.assertEqual(self.stream.get_materials(), [])
        self.assertIsNone(self.stream.get_preview_frame()[0])
        self.now += .1
        self.capture()
        self.assertFalse(self.stream.get_state()['material_observation']['valid'])
        self.assertFalse(self.stream._finish_detection('materials', [MATERIAL], 1, 10, 1010, 0, epoch))
        self.assertEqual(self.stream.get_materials(), [])
        self.publish()
        self.assertEqual(self.stream.get_state()['material_observation']['status'], 'ok')

    def test_utc_clock_jump_does_not_extend_freshness(self):
        self.publish()
        with patch('pi_vision.stream.time.time', return_value=-10000):
            self.now = 11.1
            self.assertFalse(self.stream.get_state()['capture']['valid'])

    def test_mode_change_discards_old_generation(self):
        generation = self.stream._qr_generation
        self.stream.set_scan_detection(SCAN_MODE_MATERIAL)
        self.assertFalse(self.stream._finish_detection('materials', [MATERIAL], 1, 10, 1010,
                                                      generation, self.stream._capture_epoch))
        self.assertEqual(self.stream.get_materials(), [])

    def test_same_source_is_processed_once(self):
        self.detector.detect.return_value = [MATERIAL]
        def sleep(_):
            if self.detector.detect.call_count:
                self.stream._running = False
        self.stream._running = True
        with patch('pi_vision.stream.time.sleep', side_effect=sleep):
            self.stream._detect_loop()
        self.assertEqual(self.detector.detect.call_count, 1)
        self.assertEqual(self.stream.get_state()['material_observation']['sequence'], 1)

    def test_detector_error_does_not_hide_independent_ring_success(self):
        self.stream.enable_rings = True
        def detect(_):
            self.stream._running = False
            raise ValueError('broken material detector')
        self.detector.detect.side_effect = detect
        self.pipeline.ring_detector.detect.return_value = [Ring(1, 20, 30, 80)]
        self.stream._running = True
        with patch('pi_vision.stream.time.sleep'):
            self.stream._detect_loop()
        state = self.stream.get_state()
        self.assertEqual(state['material_observation']['status'], 'error')
        self.assertFalse(state['material_observation']['valid'])
        self.assertEqual(state['ring_observation']['status'], 'ok')
        self.assertEqual(len(self.stream.get_rings()), 1)

    def test_late_success_is_timeout_not_new_coordinates(self):
        def detect(_):
            self.now = 11.5
            self.capture()
            self.stream._running = False
            return [MATERIAL]
        self.detector.detect.side_effect = detect
        self.stream._running = True
        with patch('pi_vision.stream.time.sleep'):
            self.stream._detect_loop()
        self.assertEqual(self.stream.get_state()['material_observation']['status'], 'timeout')
        self.assertEqual(self.stream.get_materials(), [])

    def test_qr_confirmation_deadline_and_latched_task_have_separate_lifetimes(self):
        self.stream.set_scan_detection(SCAN_MODE_QR)
        for index in range(3):
            self.now = 10.1 + index*.11
            self.capture()
            self.stream._accept_qr_observation((TASK, POINTS), index+2, self.now, self.stream._qr_generation, 1000+self.now)
        self.assertIsNotNone(self.stream.get_qr())
        self.now = 12
        state = self.stream.get_state()
        self.assertIsNone(state['qr'])
        self.assertIsNone(self.stream.get_qr()[1])
        self.assertEqual(state['task']['text'], str(TASK))
        self.assertTrue(state['task']['latched'])
        self.stream.set_scan_detection(SCAN_MODE_QR)
        self.now = 22.1
        self.capture()
        self.stream._accept_qr_observation((TASK, POINTS), 20, self.now, self.stream._qr_generation)
        self.assertEqual(self.stream.get_state()['qr_session']['status'], 'timeout')
        self.assertIsNone(self.stream.get_qr())

    def test_read_failure_recovers_in_same_capture_owner(self):
        calls = [0]
        def read():
            calls[0] += 1
            self.now += .1
            if calls[0] <= 5:
                return False, None, 0
            self.stream._running = False
            return True, np.zeros((40, 60, 3), np.uint8), 1000+self.now
        self.camera.read.side_effect = read
        self.stream._running = True
        with patch('pi_vision.stream.time.sleep'):
            self.stream._capture_loop()
        self.camera.open.assert_called_once()
        self.assertTrue(self.stream.get_state()['capture']['valid'])
        self.assertEqual(self.stream.get_state()['capture']['errors'], 5)
        self.assertIsNone(self.stream.get_state()['capture']['error'])

    def test_preview_encoding_failure_does_not_become_camera_failure(self):
        def read():
            self.stream._running = False
            return True, np.zeros((40, 60, 3), np.uint8), 1010.
        self.camera.read.side_effect = read
        self.stream._running = True
        with patch('pi_vision.stream.cv2.imencode', return_value=(False, None)), patch('pi_vision.stream.time.sleep'):
            self.stream._capture_loop()
        state = self.stream.get_state()
        self.assertTrue(state['capture']['valid'])
        self.assertFalse(state['preview']['valid'])
        self.assertEqual(state['preview']['error'], 'preview_encode_failed')

    def test_serial_never_reuses_held_coordinate_after_failure_or_loss(self):
        controller = SerialController('unused', 115200, self.stream, default_config())
        controller._link = Mock()
        controller._target_color = 1
        self.publish()
        controller._report_material_once()
        controller._link.send_found.assert_called_once()
        self.now += .1
        self.capture()
        self.publish([])
        controller._report_material_once()
        self.stream._capture_failed('disconnected')
        controller._report_material_once()
        self.assertEqual(controller._link.send_not_found.call_count, 2)
        self.assertEqual(controller._link.send_found.call_count, 1)

    def test_invalid_timeouts_fail_before_opening_any_device(self):
        for value in (0, -1, float('inf'), float('nan'), True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                FrameStreamer(self.camera, self.pipeline, capture_timeout=value)
        self.camera.open.assert_not_called()

    def test_stop_invalidates_latched_task(self):
        self.stream._latest_qr = (TASK, POINTS)
        self.stream.stop()
        self.assertIsNone(self.stream.get_qr())
        self.assertIsNone(self.stream.get_state()['task'])
        self.assertFalse(self.stream.get_state()['capture']['valid'])

    def test_partial_startup_releases_resources_and_does_not_open_camera_if_port_busy(self):
        for bind_fails in (True, False):
            with self.subTest(bind_fails=bind_fails):
                fake, server = Mock(), Mock()
                fake.start.side_effect = RuntimeError('injected startup failure')
                with patch('sys.argv', ['stream', '--material-backend', 'hsv']), \
                        patch.object(stream_module, 'load_config', return_value=default_config()), \
                        patch.object(stream_module, 'VisionPipeline'), \
                        patch.object(stream_module, 'Camera'), \
                        patch.object(stream_module, 'FrameStreamer', return_value=fake), \
                        patch.object(stream_module, 'ThreadingHTTPServer', side_effect=OSError('port busy') if bind_fails else None, return_value=server), \
                        self.assertRaises((OSError, RuntimeError)):
                    stream_module.main()
                fake.stop.assert_called_once()
                if bind_fails:
                    fake.start.assert_not_called()
                else:
                    server.server_close.assert_called_once()


class NativeTimeoutTests(unittest.TestCase):
    def test_blocked_worker_does_not_queue_unbounded_jobs_or_publish_late_result(self):
        release, entered = threading.Event(), threading.Event()
        camera, detector = Mock(), Mock()
        def detect(_):
            entered.set()
            release.wait(2)
            return [MATERIAL]
        detector.detect.side_effect = detect
        stream = FrameStreamer(camera, SimpleNamespace(qr_camera=camera, material_detector=detector),
                               enable_qr=False, enable_rings=False, result_timeout=.03, capture_timeout=5)
        stream._latest_raw = np.zeros((40, 60, 3), np.uint8)
        stream._raw_sequence = 1
        stream._raw_acquired_at = time.monotonic()
        stream._raw_wall_time = time.time()
        stream._running = True
        worker = threading.Thread(target=stream._detect_loop)
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            time.sleep(.12)
            state = stream.get_state()
            self.assertEqual(state['material_observation']['status'], 'timeout')
            self.assertEqual(state['materials'], [])
            self.assertEqual(detector.detect.call_count, 1)
        finally:
            stream._running = False
            release.set()
            worker.join(1)
            stream._executor.shutdown(wait=True, cancel_futures=True)
