"""树莓派现有 Camera/QRDetector 接口的扫码验收测试。"""

import contextlib
import io
import json
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

from pi_vision import qr_check
from pi_vision.config import default_config
from pi_vision.task_confirmation import TaskCodeConfirmation, parse_competition_task
from pi_vision.types import TaskCode


class CompetitionTaskTests(unittest.TestCase):
    def test_valid_task_keeps_existing_protocol_type(self):
        code = parse_competition_task(" 156+123+516+231\n")
        self.assertIsInstance(code, TaskCode)
        self.assertEqual(str(code), "156+123+516+231")

    def test_rejects_semantically_invalid_payloads(self):
        for raw in ("445+123+545+321", "156+123+512+231", "156+113+516+231",
                    "156+123+516+456", "176+123+716+231", "156123516231",
                    "１５６+１２３+５１６+２３１"):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                parse_competition_task(raw)

    def test_real_pixel_decoding_and_strict_validation(self):
        root = Path(__file__).resolve().parents[1]
        for name, expected in (("task_156.png", "candidate"),
                               ("invalid_task.png", "invalid_task")):
            with self.subTest(name=name):
                frame = cv2.imread(str(root / "examples" / "qr" / name))
                code, status = qr_check.inspect_frame(qr_check.QRDetector(), frame)
                self.assertEqual(status, expected)
                self.assertEqual(code is not None, expected == "candidate")


class ConfirmationTests(unittest.TestCase):
    def setUp(self):
        self.a = parse_competition_task("156+123+516+231")
        self.b = parse_competition_task("452+321+254+312")
        self.confirm = TaskCodeConfirmation()

    def test_one_observation_never_becomes_success_when_no_more_votes_arrive(self):
        self.assertIsNone(self.confirm.update(self.a, 0))
        for timestamp in (0.11, 0.22, 0.33):
            self.assertIsNone(self.confirm.update(None, timestamp))
        self.assertIsNone(self.confirm.locked_code)

    def test_both_count_and_time_are_required(self):
        for timestamp in (0, 0.01, 0.02):
            self.assertIsNone(self.confirm.update(self.a, timestamp))
        self.assertEqual(self.confirm.update(self.a, 0.21), self.a)

    def test_missing_invalid_conflicting_and_late_frames_restart_votes(self):
        scenarios = (
            (self.a, None, self.a, self.a, self.a),
            (self.a, TaskCode.parse("445+123+545+321"), self.a, self.a, self.a),
            (self.a, self.a, self.b, self.b, self.b),
        )
        for codes in scenarios:
            with self.subTest(codes=codes):
                confirm = TaskCodeConfirmation()
                for index, code in enumerate(codes):
                    result = confirm.update(code, index * 0.11)
                    self.assertEqual(result, code if index == 4 else None)
        self.confirm.update(self.a, 0)
        self.confirm.update(self.a, 0.11)
        self.assertIsNone(self.confirm.update(self.a, 2))
        self.assertEqual(self.confirm.count, 1)

    def test_lock_requires_explicit_reset_before_accepting_another_task(self):
        for timestamp in (0, 0.11, 0.22):
            self.confirm.update(self.a, timestamp)
        self.assertEqual(self.confirm.update(self.b, 0.33), self.a)
        self.confirm.reset()
        self.assertIsNone(self.confirm.update(self.b, 0.44))

    def test_invalid_configuration_and_timestamps_are_rejected(self):
        for kwargs in ({"frames": 0}, {"frames": True}, {"min_seconds": -1},
                       {"min_seconds": float("nan")}, {"max_gap": 0}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                TaskCodeConfirmation(**kwargs)
        self.confirm.update(self.a, 1)
        for timestamp in (1, 0.9, float("nan")):
            with self.subTest(timestamp=timestamp), self.assertRaises(ValueError):
                self.confirm.update(self.a, timestamp)


class QRCheckCommandTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.output = io.StringIO()
        self.frame = np.full((200, 200, 3), 255, np.uint8)
        self.camera = Mock()
        self.camera.read.side_effect = self.read_frame
        self.decoder = Mock()
        self.decoder.detect.return_value = parse_competition_task("156+123+516+231")

    def read_frame(self):
        self.now += 0.11
        return True, self.frame, 1000 + self.now

    def run_command(self, *options):
        with patch.object(qr_check, "Camera", return_value=self.camera), \
                patch.object(qr_check, "load_config", return_value=default_config()), \
                patch.object(qr_check, "QRDetector", return_value=self.decoder), \
                patch.object(qr_check.time, "monotonic", side_effect=lambda: self.now), \
                contextlib.redirect_stdout(self.output), contextlib.redirect_stderr(io.StringIO()):
            return qr_check.main(list(options))

    def result(self):
        return json.loads(self.output.getvalue())

    def test_camera_scan_reuses_existing_interface_and_releases_on_success(self):
        self.assertEqual(self.run_command("--camera", "0"), 0)
        self.assertTrue(self.result()["confirmed"])
        self.assertEqual(self.camera.read.call_count, 3)
        self.camera.release.assert_called_once()

    def test_timeout_does_not_return_unconfirmed_candidate(self):
        self.assertEqual(self.run_command("--timeout", "0.25"), 1)
        self.assertEqual(self.result()["status"], "timeout")
        self.assertIsNone(self.result()["raw"])
        self.camera.release.assert_called_once()

    def test_decode_finishing_after_deadline_is_rejected(self):
        def decode(_):
            self.now += 2
            return parse_competition_task("156+123+516+231")
        self.decoder.detect.side_effect = decode
        self.assertEqual(self.run_command("--timeout", "1", "--confirm-frames", "1",
                                          "--min-confirm-seconds", "0"), 1)
        self.assertFalse(self.result()["confirmed"])

    def test_camera_read_failure_is_an_unconfirmed_error(self):
        self.camera.read.side_effect = lambda: (False, None, 0)
        self.assertEqual(self.run_command(), 2)
        self.assertEqual(self.result()["status"], "error")
        self.assertFalse(self.result()["confirmed"])
        self.camera.release.assert_called_once()

    def test_camera_open_failure_still_releases_device(self):
        self.camera.open.side_effect = RuntimeError("camera busy")
        self.assertEqual(self.run_command(), 2)
        self.assertEqual(self.result()["status"], "error")
        self.camera.release.assert_called_once()

    def test_single_image_can_never_claim_realtime_confirmation(self):
        with patch.object(qr_check.cv2, "imread", return_value=self.frame):
            self.assertEqual(self.run_command("--image", "sample.png"), 0)
        self.assertEqual(self.result()["status"], "image_ok")
        self.assertFalse(self.result()["confirmed"])
        self.camera.open.assert_not_called()

    def test_invalid_roi_is_an_unconfirmed_error(self):
        self.assertEqual(self.run_command("--roi", "0", "0", "999", "999"), 2)
        self.assertFalse(self.result()["confirmed"])
        self.camera.release.assert_called_once()


if __name__ == "__main__":
    unittest.main()
