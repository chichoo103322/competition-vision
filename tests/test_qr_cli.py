"""Camera-loop contract tests: no hardware and no wall-clock sleeps."""

import contextlib
import io
import json
import unittest
from unittest.mock import Mock, patch

import numpy as np

from robot_perception import cli
from robot_perception.task_code import TaskObservation, parse_task_code


class QRCommandTests(unittest.TestCase):
    def setUp(self):
        self.clock = 0.0
        self.camera = Mock()
        self.frame = np.full((100, 100, 3), 255, np.uint8)
        self.camera.read.side_effect = self.read
        self.plan = parse_task_code("156+123+516+231")
        self.decoder = Mock()
        self.decoder.decode.return_value = TaskObservation("candidate", self.plan)
        self.stdout = io.StringIO()
        self.stderr = io.StringIO()

    def read(self):
        self.clock += 0.11
        return True, self.frame

    def run_command(self, *options):
        args = cli.build_parser().parse_args(["qr", *options])
        with patch.object(cli, "open_camera", return_value=self.camera), \
                patch.object(cli, "TaskCodeDecoder", return_value=self.decoder), \
                patch.object(cli.time, "monotonic", side_effect=lambda: self.clock), \
                contextlib.redirect_stdout(self.stdout), contextlib.redirect_stderr(self.stderr):
            return cli.qr(args)

    def result(self):
        lines = self.stdout.getvalue().splitlines()
        self.assertEqual(len(lines), 1)
        return json.loads(lines[0])

    def test_live_scan_confirms_then_emits_one_plan_and_releases_camera(self):
        self.assertEqual(self.run_command("--camera", "0", "--diagnostics"), 0)
        result = self.result()
        self.assertTrue(result["confirmed"])
        self.assertEqual(result["raw"], self.plan.raw)
        self.assertEqual(self.camera.read.call_count, 3)
        self.assertEqual(len(self.stderr.getvalue().splitlines()), 3)
        self.camera.release.assert_called_once()

    def test_missing_observation_resets_live_confirmation(self):
        self.decoder.decode.side_effect = [
            TaskObservation("candidate", self.plan), TaskObservation("not_found"),
            *[TaskObservation("candidate", self.plan)] * 3,
        ]
        self.assertEqual(self.run_command("--camera", "0"), 0)
        self.assertEqual(self.camera.read.call_count, 5)
        self.assertTrue(self.result()["confirmed"])

    def test_timeout_never_emits_a_candidate_as_confirmed(self):
        self.assertEqual(self.run_command("--camera", "0", "--timeout", "0.25"), 1)
        result = self.result()
        self.assertEqual(result["status"], "timeout")
        self.assertFalse(result["confirmed"])
        self.assertNotIn("raw", result)
        self.camera.release.assert_called_once()

    def test_slow_decode_past_deadline_cannot_succeed(self):
        def decode(*_):
            self.clock += 1
            return TaskObservation("candidate", self.plan)
        self.decoder.decode.side_effect = decode
        self.assertEqual(self.run_command("--camera", "0", "--timeout", "0.5",
                                          "--confirm-frames", "1",
                                          "--min-confirm-seconds", "0"), 1)
        self.assertFalse(self.result()["confirmed"])

    def test_camera_failure_releases_device_and_produces_no_plan(self):
        self.camera.read.side_effect = lambda: (False, None)
        with self.assertRaisesRegex(RuntimeError, "camera did not return"):
            self.run_command("--camera", "0")
        self.camera.release.assert_called_once()
        self.assertEqual(self.stdout.getvalue(), "")

    def test_image_is_only_an_unconfirmed_inspection(self):
        with patch.object(cli, "read_image", return_value=self.frame):
            self.assertEqual(self.run_command("--image", "sample.png"), 0)
        self.assertFalse(self.result()["confirmed"])
        self.camera.read.assert_not_called()

    def test_bad_configuration_is_rejected_before_camera_use(self):
        for options in (("--timeout", "nan"), ("--timeout", "-1"),
                        ("--confirm-frames", "0"), ("--max-frame-gap", "0")):
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.run_command("--camera", "0", *options)
        self.camera.read.assert_not_called()


if __name__ == "__main__":
    unittest.main()
