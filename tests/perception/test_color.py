import unittest

import cv2
import numpy as np

from robot_perception.color import ColorConfirmation, ColorDetector


class ColorTests(unittest.TestCase):
    def test_detects_six_material_colors_in_region(self):
        samples = {
            "red": (0, 0, 220),
            "yellow": (0, 220, 220),
            "blue": (220, 80, 0),
            "green": (0, 180, 0),
            "black": (25, 25, 25),
            "light_blue": (225, 190, 120),
        }
        detector = ColorDetector(min_area_px=100)
        for color, bgr in samples.items():
            with self.subTest(color=color):
                image = np.full((160, 160, 3), 245, np.uint8)
                cv2.circle(image, (80, 80), 30, bgr, thickness=-1)
                results = detector.detect(image, (35, 35, 90, 90))
                self.assertTrue(any(result.color == color for result in results))

    def test_confirmation_requires_consecutive_frames(self):
        image = np.full((160, 160, 3), 245, np.uint8)
        cv2.circle(image, (80, 80), 30, (0, 0, 220), thickness=-1)
        detections = ColorDetector(min_area_px=100).detect(image)
        confirm = ColorConfirmation(3)
        self.assertIsNone(confirm.update(detections, "red"))
        self.assertIsNone(confirm.update([], "red"))
        self.assertIsNone(confirm.update(detections, "red"))
        self.assertIsNone(confirm.update(detections, "red"))
        self.assertEqual(confirm.update(detections, "red").color, "red")


if __name__ == "__main__":
    unittest.main()
