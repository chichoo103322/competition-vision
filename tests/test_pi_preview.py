"""Regression checks for duplicate MJPEG traffic and shared-lock blocking."""

import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from pi_vision import stream


class PreviewTests(unittest.TestCase):
    def setUp(self):
        self.camera = Mock()
        self.streamer = stream.FrameStreamer(
            self.camera, SimpleNamespace(qr_camera=self.camera),
            enable_qr=False, enable_materials=False, enable_rings=False,
            draw_overlay=False, preview_fps=10)
        self.addCleanup(self.streamer._executor.shutdown, wait=False)

    def handler(self):
        handler = stream.StreamHandler.__new__(stream.StreamHandler)
        handler.send_response = Mock()
        handler.send_header = Mock()
        handler.end_headers = Mock()
        handler.wfile = io.BytesIO()
        return handler

    def test_http_does_not_resend_one_jpeg_as_multiple_frames(self):
        clock = [1.0]
        def sleep(seconds):
            clock[0] += max(seconds, 0.001)
        source = Mock()
        source.get_preview_frame.side_effect = [
            (b'first-jpeg', 1, 0.9), (b'first-jpeg', 1, 0.9),
            (b'latest-jpeg', 5, 1.0), BrokenPipeError(),
        ]
        handler = self.handler()
        with patch.object(stream, '_streamer', source), \
                patch.object(stream.time, 'monotonic', side_effect=lambda: clock[0]), \
                patch.object(stream.time, 'sleep', side_effect=sleep):
            handler._send_stream()
        output = handler.wfile.getvalue()
        self.assertEqual(output.count(b'--frame\r\n'), 2)
        self.assertEqual(output.count(b'first-jpeg'), 1)
        self.assertIn(b'X-Frame-Id: 5\r\n', output)
        self.assertIn(b'Content-Length: 11\r\n\r\nlatest-jpeg', output)

    def test_capture_publishes_raw_frames_faster_than_preview_encoding(self):
        clock = [0.0]
        image = np.zeros((20, 30, 3), np.uint8)
        count = [0]
        def read():
            count[0] += 1
            clock[0] += 0.03
            if count[0] == 6:
                self.streamer._running = False
            return True, image, 9000 + clock[0]
        self.camera.read.side_effect = read
        self.streamer._running = True
        with patch.object(stream.time, 'monotonic', side_effect=lambda: clock[0]), \
                patch.object(stream.time, 'sleep'), \
                patch.object(stream.cv2, 'imencode', return_value=(True, np.array([1, 2], np.uint8))) as encode:
            self.streamer._capture_loop()
        self.assertEqual(self.streamer._raw_sequence, 6)
        self.assertEqual(encode.call_count, 2)
        with patch.object(stream.time, 'monotonic', return_value=clock[0]):
            self.assertEqual(self.streamer.get_preview_frame()[1], 2)
        self.assertEqual(self.streamer._latest_raw.shape, image.shape)

    def test_jpeg_encoding_never_holds_shared_state_lock(self):
        image = np.zeros((20, 30, 3), np.uint8)
        def read():
            self.streamer._running = False
            return True, image, 1000
        self.camera.read.side_effect = read
        def encode(*_):
            available = self.streamer._lock.acquire(blocking=False)
            self.assertTrue(available, 'encoding held the shared camera/state lock')
            if available:
                self.streamer._lock.release()
            return True, np.array([1, 2, 3], np.uint8)
        self.streamer._running = True
        with patch.object(stream.cv2, 'imencode', side_effect=encode), \
                patch.object(stream.time, 'sleep'):
            self.streamer._capture_loop()
        self.assertEqual(self.streamer.get_preview_frame()[0], b'\x01\x02\x03')

    def test_state_reports_capture_and_preview_freshness_independently(self):
        self.streamer._raw_sequence = 10
        self.streamer._raw_acquired_at = 5.9
        self.streamer._jpeg_sequence = 3
        self.streamer._jpeg_acquired_at = 5.5
        self.streamer._latest_jpeg = b'abc'
        with patch.object(stream.time, 'monotonic', return_value=6):
            state = self.streamer.get_state()
        self.assertEqual({k:state['capture'][k] for k in ('sequence', 'age_ms')}, {'sequence': 10, 'age_ms': 100.0})
        self.assertTrue(state['capture']['valid'])
        self.assertEqual({key:state['preview'][key] for key in ('sequence','age_ms','jpeg_bytes')},
                         {'sequence': 3, 'age_ms': 500.0, 'jpeg_bytes': 3})

    def test_no_frame_yet_has_no_misleading_zero_age(self):
        state = self.streamer.get_state()
        self.assertIsNone(state['capture']['age_ms'])
        self.assertIsNone(state['preview']['age_ms'])

    def test_snapshot_returns_one_complete_latest_jpeg_and_frame_headers(self):
        source = Mock()
        source.get_preview_frame.return_value = (b'latest-jpeg', 12, 5.5)
        source.get_frame_width.return_value = 1280
        source.get_frame_height.return_value = 720
        source.capture_timeout = 1.
        source.preview_timeout = 3.
        handler = self.handler()
        with patch.object(stream, '_streamer', source), \
                patch.object(stream.time, 'monotonic', return_value=6):
            handler._send_frame()
        handler.send_response.assert_called_once_with(200)
        self.assertEqual(handler.wfile.getvalue(), b'latest-jpeg')
        self.assertIn(('X-Frame-Id', '12'), [call.args for call in handler.send_header.call_args_list])
        self.assertIn(('X-Capture-Age-Ms', '500'), [call.args for call in handler.send_header.call_args_list])
        self.assertIn(('X-Frame-Max-Age-Ms', '1000.0'), [call.args for call in handler.send_header.call_args_list])
        self.assertIn(('X-Preview-Max-Age-Ms', '3000.0'), [call.args for call in handler.send_header.call_args_list])
        self.assertIn(('X-Source-Width', '1280'), [call.args for call in handler.send_header.call_args_list])
        self.assertIn(('X-Source-Height', '720'), [call.args for call in handler.send_header.call_args_list])

    def test_snapshot_without_frames_reports_retryable_unavailable(self):
        source = Mock()
        source.get_preview_frame.return_value = (None, 0, 0)
        handler = self.handler()
        with patch.object(stream, '_streamer', source):
            handler._send_frame()
        handler.send_response.assert_called_once_with(503)
        self.assertEqual(handler.wfile.getvalue(), b'')

    def test_snapshot_cache_busting_query_routes_to_snapshot(self):
        handler = self.handler()
        handler.path = '/frame.jpg?t=123'
        handler._send_frame = Mock()
        handler.do_GET()
        handler._send_frame.assert_called_once()

    def test_only_preview_is_resized_while_detection_and_coordinate_size_remain_full(self):
        image = np.zeros((720, 1280, 3), np.uint8)
        def read():
            self.streamer._running = False
            return True, image, 1000
        self.camera.read.side_effect = read
        self.streamer._running = True
        with patch.object(stream.cv2, 'imencode', return_value=(True, np.array([1, 2], np.uint8))) as encode, \
                patch.object(stream.time, 'sleep'):
            self.streamer._capture_loop()
        self.assertEqual(encode.call_args.args[1].shape, (360, 640, 3))
        self.assertEqual(self.streamer._latest_raw.shape, (720, 1280, 3))
        self.assertEqual((self.streamer.get_frame_width(), self.streamer.get_frame_height()), (1280, 720))

    def test_invalid_preview_rate_or_width_is_rejected(self):
        for options in ({'preview_fps':0}, {'preview_fps':float('nan')}, {'preview_width':0}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                stream.FrameStreamer(self.camera, SimpleNamespace(qr_camera=self.camera), **options)

    def test_display_budget_does_not_extend_capture_validity(self):
        self.streamer._raw_sequence = self.streamer._jpeg_sequence = 1
        self.streamer._raw_acquired_at = self.streamer._jpeg_acquired_at = 5.
        self.streamer._latest_jpeg = b'jpeg'
        self.assertEqual(self.streamer.preview_timeout, 3.)
        with patch.object(stream.time, 'monotonic', return_value=6.):
            self.assertFalse(self.streamer.get_state()['capture']['valid'])
            self.assertIsNone(self.streamer.get_preview_frame()[0])

    def test_invalid_display_budget_is_rejected_before_camera_open(self):
        for value in (0, -1, float('nan'), float('inf'), True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                stream.FrameStreamer(self.camera, SimpleNamespace(qr_camera=self.camera), preview_timeout=value)
        self.camera.open.assert_not_called()


if __name__ == '__main__':
    unittest.main()
