"""Exercise production QR paths without cameras, model inference or serial I/O."""

import contextlib
import io
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from pi_vision.app import VisionApp
from pi_vision.config import default_config
from pi_vision.pipeline import VisionPipeline
from pi_vision.protocol import CMD_RESCAN, CMD_SCAN, CMD_STOP_SCAN, SCAN_MODE_QR
from pi_vision.stream import FrameStreamer, SerialController
from pi_vision.task_confirmation import parse_competition_task
from pi_vision.types import TaskCode


A = parse_competition_task("156+123+516+231")
B = parse_competition_task("452+321+254+312")
POINTS = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], dtype=np.float32)


class PipelineConfirmationTests(unittest.TestCase):
    def run_sequence(self, codes, **options):
        pipeline = VisionPipeline.__new__(VisionPipeline)
        pipeline.qr_detector = Mock()
        pipeline.qr_detector.detect.side_effect = codes
        now = [0.0]
        def read():
            now[0] += 0.11
            return np.zeros((20, 20, 3), np.uint8), 9000 + now[0]
        pipeline.read_qr_frame = Mock(side_effect=read)
        with patch("pi_vision.pipeline.time.monotonic", side_effect=lambda: now[0]):
            result = pipeline.detect_task_code_stable(max_frames=len(codes), **options)
        return result, pipeline

    def test_does_not_return_highest_vote_when_below_threshold(self):
        for codes in ([A], [A, A], [None, A, None], [A, B, A]):
            with self.subTest(codes=codes):
                self.assertIsNone(self.run_sequence(codes)[0])

    def test_new_frames_after_a_miss_must_confirm_again(self):
        result, pipeline = self.run_sequence([A, None, A, A, A])
        self.assertEqual(result, A)
        self.assertEqual(pipeline.read_qr_frame.call_count, 5)

    def test_invalid_task_is_never_returned(self):
        invalid = TaskCode.parse("445+123+545+321")
        self.assertIsNone(self.run_sequence([invalid] * 4)[0])

    def test_timeout_and_cancel_return_no_task(self):
        self.assertIsNone(self.run_sequence([A] * 3, timeout=0.25)[0])
        stop = threading.Event()
        stop.set()
        result, pipeline = self.run_sequence([A] * 3, stop_event=stop)
        self.assertIsNone(result)
        pipeline.read_qr_frame.assert_not_called()


class StreamConfirmationTests(unittest.TestCase):
    def setUp(self):
        self.camera = Mock()
        self.pipeline = SimpleNamespace(qr_camera=self.camera, qr_detector=Mock())
        self.streamer = FrameStreamer(self.camera, self.pipeline,
                                      enable_materials=False, enable_rings=False)
        self.streamer._qr_session_started_at = 0
        self.addCleanup(self.streamer._executor.shutdown, wait=False)
        self.addCleanup(patch.stopall)
        patch("builtins.print").start()

    def offer(self, code, sequence, timestamp, generation=None):
        result = None if code is None else (code, POINTS)
        generation = self.streamer.get_qr_generation() if generation is None else generation
        self.streamer._accept_qr_observation(result, sequence, timestamp, generation)

    def lock(self, code=A, start=1):
        for index in range(3):
            self.offer(code, index + start, time.monotonic() + index * 0.11)

    def test_candidate_not_visible_as_confirmed_state_or_serial_result(self):
        self.offer(A, 1, 0.1)
        self.assertIsNone(self.streamer.get_qr())
        state = self.streamer.get_state()
        self.assertIsNone(state["qr"])
        self.assertEqual(state["qr_candidate"], str(A))
        self.assertFalse(state["qr_confirmed"])

    def test_reprocessing_one_cached_frame_cannot_create_three_votes(self):
        for stamp in (0.1, 0.3, 0.5):
            self.offer(A, 1, stamp)
        self.assertIsNone(self.streamer.get_qr())
        self.assertEqual(self.streamer.get_state()["qr_matching_frames"], 1)

    def test_lock_survives_miss_and_other_code_until_a_new_scan(self):
        self.lock()
        self.offer(None, 4, time.monotonic() + 0.4)
        self.offer(B, 5, time.monotonic() + 0.5)
        self.assertEqual(self.streamer.get_qr()[0], A)
        self.streamer.set_scan_detection(SCAN_MODE_QR)
        self.assertIsNone(self.streamer.get_qr())
        self.lock(B, start=6)
        self.assertEqual(self.streamer.get_qr()[0], B)

    def test_late_result_from_before_rescan_cannot_contaminate_new_session(self):
        generation = self.streamer.get_qr_generation()
        self.streamer.set_scan_detection(SCAN_MODE_QR)
        self.offer(A, 1, time.monotonic(), generation)
        self.assertEqual(self.streamer.get_state()["qr_matching_frames"], 0)

    def test_new_session_rejects_frames_captured_before_command(self):
        old_acquired_at = time.monotonic()
        self.streamer.set_scan_detection(SCAN_MODE_QR)
        self.offer(A, 1, old_acquired_at)
        self.assertIsNone(self.streamer.get_qr())
        self.assertEqual(self.streamer.get_state()["qr_matching_frames"], 0)

    def test_stop_clears_result_and_disables_acceptance(self):
        self.lock()
        self.streamer.set_scan_detection(None)
        self.lock(B, start=6)
        self.assertIsNone(self.streamer.get_qr())

    def test_invalid_tasks_reset_votes(self):
        self.offer(A, 1, 0.1)
        self.offer(TaskCode.parse("445+123+545+321"), 2, 0.2)
        self.assertEqual(self.streamer.get_state()["qr_matching_frames"], 0)

    def test_detection_loop_discards_result_if_rescan_arrives_during_decode(self):
        self.streamer._latest_raw = np.zeros((20, 20, 3), np.uint8)
        self.streamer._raw_sequence = 1
        self.streamer._raw_acquired_at = time.monotonic()
        self.streamer._running = True
        def decode(_):
            self.streamer.set_scan_detection(SCAN_MODE_QR)
            self.streamer._running = False
            return A, POINTS
        self.pipeline.qr_detector.detect_with_points.side_effect = decode
        with patch("pi_vision.stream.time.sleep"):
            self.streamer._detect_loop()
        self.assertEqual(self.streamer.get_state()["qr_matching_frames"], 0)


class SerialReportingTests(unittest.TestCase):
    def setUp(self):
        self.streamer = Mock()
        self.streamer.get_qr_generation.return_value = 7
        self.controller = SerialController("unused", 115200, self.streamer, default_config())
        self.controller._link = Mock()
        self.controller._scanning = True
        self.controller._scan_mode = SCAN_MODE_QR
        self.event = Mock()
        self.event.is_set.return_value = False

    def run_loop(self, clock=None):
        with contextlib.redirect_stdout(io.StringIO()):
            if clock is None:
                self.controller._qr_loop(self.event, 7)
            else:
                with patch("pi_vision.stream.time.monotonic", side_effect=clock):
                    self.controller._qr_loop(self.event, 7)

    def test_waits_for_confirmed_result_then_reports_once(self):
        self.streamer.get_qr.side_effect = [None, None, (A, POINTS)]
        self.run_loop()
        self.controller._link.send_taskcode.assert_called_once_with(str(A))
        self.controller._link.send_not_found.assert_not_called()
        self.assertEqual(self.event.wait.call_count, 2)

    def test_only_timeout_reports_not_found(self):
        self.streamer.get_qr.return_value = None
        self.run_loop(clock=[0, 0.1, 10.1])
        self.controller._link.send_not_found.assert_called_once()
        self.controller._link.send_taskcode.assert_not_called()

    def test_stopped_worker_cannot_send_or_be_revived(self):
        self.event.is_set.return_value = True
        self.streamer.get_qr.return_value = (A, POINTS)
        self.run_loop()
        self.controller._link.send_taskcode.assert_not_called()
        self.controller._link.send_not_found.assert_not_called()

    def test_changed_generation_cannot_publish_old_task(self):
        self.streamer.get_qr_generation.return_value = 8
        self.streamer.get_qr.return_value = (A, POINTS)
        self.run_loop()
        self.controller._link.send_taskcode.assert_not_called()

    def test_stop_arriving_after_cache_read_prevents_publish(self):
        def get_result():
            self.event.is_set.return_value = True
            return A, POINTS
        self.streamer.get_qr.side_effect = get_result
        self.run_loop()
        self.controller._link.send_taskcode.assert_not_called()

    def test_scan_and_rescan_reset_detection_and_use_async_wait(self):
        with patch.object(self.controller, "_start_qr_loop") as start, \
                patch.object(self.controller, "_stop_qr_thread"), \
                patch.object(self.controller, "_stop_ring_thread"), \
                contextlib.redirect_stdout(io.StringIO()):
            self.controller._on_frame(CMD_SCAN, bytes([SCAN_MODE_QR]))
            self.controller._on_frame(CMD_RESCAN, b"")
        self.assertEqual(start.call_count, 2)
        self.assertEqual(self.streamer.set_scan_detection.call_count, 2)
        self.controller._link.send_taskcode.assert_not_called()

    def test_previous_ring_worker_cannot_keep_reporting_in_qr_mode(self):
        self.controller._ring_loop(threading.Event())
        self.streamer.get_rings.assert_not_called()
        self.controller._link.send_not_found.assert_not_called()


class AppConfirmationTests(unittest.TestCase):
    def setUp(self):
        self.link = Mock()
        self.pipeline = Mock()
        self.app = VisionApp(default_config(), self.link, self.pipeline)

    def test_legacy_continuous_entry_uses_stable_pipeline(self):
        self.pipeline.detect_task_code_stable.return_value = A
        with contextlib.redirect_stdout(io.StringIO()):
            self.app._qr_loop(threading.Event())
        self.pipeline.qr_detector.detect.assert_not_called()
        self.link.send_taskcode.assert_called_once_with(str(A))
        self.assertEqual(self.pipeline.detect_task_code_stable.call_args.kwargs["min_confirmed"], 3)

    def test_stop_during_pipeline_read_prevents_any_result_report(self):
        event = threading.Event()
        def decode(**_):
            event.set()
            return A
        self.pipeline.detect_task_code_stable.side_effect = decode
        self.app._run_qr_once(event)
        self.link.send_taskcode.assert_not_called()
        self.link.send_not_found.assert_not_called()

    def test_no_confirmation_reports_not_found(self):
        self.pipeline.detect_task_code_stable.return_value = None
        with contextlib.redirect_stdout(io.StringIO()):
            self.app._run_qr_once(threading.Event())
        self.link.send_not_found.assert_called_once()
        self.link.send_taskcode.assert_not_called()


if __name__ == "__main__":
    unittest.main()
