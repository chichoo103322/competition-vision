import unittest

import cv2
import numpy as np
from unittest.mock import Mock

from robot_perception.task_code import (
    InvalidTaskCode, TaskCodeDecoder, TaskConfirmation, decode_task_frame, parse_task_code,
)


class TaskCodeTests(unittest.TestCase):
    def test_second_batch_storage_follows_color(self):
        plan = parse_task_code("452+321+254+312")
        self.assertEqual([move.color for move in plan.first_batch],
                         ["green", "black", "yellow"])
        self.assertEqual([move.rough_position for move in plan.second_batch], [3, 1, 2])
        self.assertEqual([move.storage_position for move in plan.second_batch], [1, 2, 3])

    def test_rejects_malformed_or_mismatched_tasks(self):
        for payload in ("452+321+254", "452+321+256+312", "452+331+254+312",
                        "445+321+254+312", "452+321+254+112"):
            with self.subTest(payload=payload), self.assertRaises(InvalidTaskCode):
                parse_task_code(payload)

    def test_reads_real_qr_pixels(self):
        encoder = cv2.QRCodeEncoder_create()
        qr = encoder.encode("452+321+254+312")
        large = cv2.resize(qr, (400, 400), interpolation=cv2.INTER_NEAREST)
        frame = cv2.cvtColor(large, cv2.COLOR_GRAY2BGR)
        plan = decode_task_frame(frame)
        self.assertIsNotNone(plan)
        self.assertEqual(plan.raw, "452+321+254+312")

    def test_rulebook_example_stacks_second_batch_by_color(self):
        plan = parse_task_code("156+123+516+231")
        self.assertEqual([move.rough_position for move in plan.second_batch], [2, 3, 1])
        self.assertEqual([move.storage_position for move in plan.second_batch], [2, 1, 3])


class TaskDecoderTests(unittest.TestCase):
    def setUp(self):
        self.decoder = TaskCodeDecoder()
        self.frame = np.full((500, 1000, 3), 255, np.uint8)

    def stamp(self, payload, x):
        qr = cv2.QRCodeEncoder_create().encode(payload)
        qr = cv2.resize(qr, (350, 350), interpolation=cv2.INTER_NEAREST)
        self.frame[75:425, x:x + 350] = cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)

    def test_empty_and_blank_frames(self):
        for frame in (None, np.array([], dtype=np.uint8)):
            self.assertEqual(self.decoder.decode(frame).status, "empty_frame")
        self.assertEqual(self.decoder.decode(self.frame).status, "not_found")

    def test_real_invalid_qr_is_rejected(self):
        self.stamp("445+123+545+321", 75)
        result = self.decoder.decode(self.frame)
        self.assertEqual(result.status, "invalid_payload")
        self.assertIsNone(result.plan)

    def test_two_distinct_real_qrs_are_ambiguous(self):
        self.stamp("156+123+516+231", 75)
        self.stamp("452+321+254+312", 575)
        result = self.decoder.decode(self.frame)
        self.assertEqual(result.status, "ambiguous")
        self.assertIsNone(result.plan)

    def test_roi_excludes_other_qr(self):
        self.stamp("156+123+516+231", 75)
        self.stamp("452+321+254+312", 575)
        result = self.decoder.decode(self.frame, (0, 0, 500, 500))
        self.assertEqual(result.plan.raw, "156+123+516+231")

    def test_real_qr_under_controlled_image_transformations(self):
        self.stamp("156+123+516+231", 75)
        source = np.float32([[0, 0], [999, 0], [999, 499], [0, 499]])
        destination = np.float32([[45, 25], [950, 0], [999, 470], [0, 499]])
        transform = cv2.getPerspectiveTransform(source, destination)
        samples = {
            "rotation": cv2.rotate(self.frame, cv2.ROTATE_90_CLOCKWISE),
            "dim": (self.frame.astype(np.float32) * 0.45 + 15).astype(np.uint8),
            "mild_blur": cv2.GaussianBlur(self.frame, (3, 3), 0),
            "perspective": cv2.warpPerspective(self.frame, transform, (1000, 500),
                                                borderValue=(255, 255, 255)),
        }
        for name, frame in samples.items():
            with self.subTest(name=name):
                result = self.decoder.decode(frame)
                self.assertIsNotNone(result.plan)
                self.assertEqual(result.plan.raw, "156+123+516+231")

    def test_rejects_invalid_roi(self):
        for roi in ((-1, 0, 10, 10), (0, 0, 0, 10), (0, 0, 1001, 500)):
            with self.subTest(roi=roi), self.assertRaises(ValueError):
                self.decoder.decode(self.frame, roi)

    def test_fallback_when_multi_decoder_misses_single_code(self):
        self.decoder.detector = Mock()
        self.decoder.detector.detectAndDecodeMulti.return_value = (False, (), None, ())
        self.decoder.detector.detectAndDecode.return_value = ("156+123+516+231", None, None)
        self.assertEqual(self.decoder.decode(self.frame).plan.raw, "156+123+516+231")

    def test_non_utf8_qr_does_not_crash_reader(self):
        self.decoder.detector = Mock()
        self.decoder.detector.detectAndDecodeMulti.side_effect = UnicodeDecodeError(
            "utf8", b"\xff", 0, 1, "invalid byte")
        self.assertEqual(self.decoder.decode(self.frame).status, "invalid_payload")


class TaskConfirmationTests(unittest.TestCase):
    def setUp(self):
        self.a = parse_task_code("156+123+516+231")
        self.b = parse_task_code("452+321+254+312")
        self.confirm = TaskConfirmation()

    def test_needs_both_frame_count_and_elapsed_time(self):
        for now in (0, 0.01, 0.02):
            self.assertIsNone(self.confirm.update(self.a, now))
        self.assertEqual(self.confirm.update(self.a, 0.21), self.a)

    def test_long_interval_does_not_replace_required_frame_count(self):
        self.assertIsNone(self.confirm.update(self.a, 0))
        self.assertIsNone(self.confirm.update(self.a, 0.3))
        self.assertEqual(self.confirm.update(self.a, 0.6), self.a)

    def test_missing_or_invalid_frame_restarts_confirmation(self):
        for now in (0, 0.11):
            self.assertIsNone(self.confirm.update(self.a, now))
        self.assertIsNone(self.confirm.update(None, 0.22))
        for now in (0.33, 0.44):
            self.assertIsNone(self.confirm.update(self.a, now))
        self.assertEqual(self.confirm.update(self.a, 0.55), self.a)

    def test_switching_codes_cannot_share_votes(self):
        self.confirm.update(self.a, 0)
        self.confirm.update(self.a, 0.11)
        self.assertIsNone(self.confirm.update(self.b, 0.22))
        self.assertIsNone(self.confirm.update(self.b, 0.33))
        self.assertEqual(self.confirm.update(self.b, 0.44), self.b)

    def test_camera_gap_restarts_confirmation(self):
        self.confirm.update(self.a, 0)
        self.confirm.update(self.a, 0.11)
        self.assertIsNone(self.confirm.update(self.a, 2))
        self.assertIsNone(self.confirm.update(self.a, 2.11))
        self.assertEqual(self.confirm.update(self.a, 2.22), self.a)

    def test_lock_survives_missing_and_conflicting_codes_until_reset(self):
        for now in (0, 0.11, 0.22):
            self.confirm.update(self.a, now)
        self.assertEqual(self.confirm.update(None, 0.33), self.a)
        self.assertEqual(self.confirm.update(self.b, 0.44), self.a)
        self.confirm.reset()
        self.assertIsNone(self.confirm.update(self.b, 1))
        self.assertIsNone(self.confirm.update(self.b, 1.11))
        self.assertEqual(self.confirm.update(self.b, 1.22), self.b)

    def test_rejects_invalid_settings_and_clock(self):
        for kwargs in ({"frames": 0}, {"frames": 2.5}, {"frames": True},
                       {"min_seconds": -1}, {"min_seconds": float("nan")},
                       {"max_gap": 0}, {"max_gap": float("inf")}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                TaskConfirmation(**kwargs)
        for now in (float("nan"), float("inf")):
            with self.subTest(now=now), self.assertRaises(ValueError):
                self.confirm.update(self.a, now)
        self.confirm.update(self.a, 1)
        for now in (1, 0.9):
            with self.subTest(now=now), self.assertRaises(ValueError):
                self.confirm.update(self.a, now)


if __name__ == "__main__":
    unittest.main()
