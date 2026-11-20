"""Experimental color backend: identity, false positives and pipeline selection."""

import unittest
from unittest.mock import patch

import cv2
import numpy as np

from pi_vision.config import default_config
from pi_vision.detectors.hsv_material_detector import HSVMaterialDetector
from pi_vision.material_region import MaterialRegionDetector
from pi_vision.pipeline import VisionPipeline


def background():
    hsv = np.full((400, 600, 3), (15, 100, 140), np.uint8)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def paint(image, center, hsv, radius=38):
    bgr = cv2.cvtColor(np.uint8([[hsv]]), cv2.COLOR_HSV2BGR)[0, 0]
    cv2.circle(image, center, radius, tuple(map(int, bgr)), -1)


class HSVMaterialTests(unittest.TestCase):
    def test_six_distinct_colors_including_black_and_lightblue(self):
        image = background()
        colors = [(0, 240, 220), (27, 220, 220), (115, 240, 220),
                  (65, 180, 220), (0, 0, 45), (98, 130, 240)]
        for i, color in enumerate(colors):
            paint(image, (100+200*(i % 3), 100+200*(i//3)), color)
        found = HSVMaterialDetector().detect(image)
        self.assertEqual([m.color_id for m in found], [1, 2, 3, 4, 5, 6])

    def test_empty_scene_and_small_dark_speck_are_not_materials(self):
        image = background()
        paint(image, (100, 100), (0, 0, 30), radius=8)
        self.assertEqual(HSVMaterialDetector().detect(image), [])

    def test_same_color_objects_are_not_silently_collapsed(self):
        image = background()
        paint(image, (100, 100), (65, 180, 220))
        paint(image, (400, 100), (65, 180, 220))
        self.assertEqual(len(HSVMaterialDetector().detect(image)), 2)

    def test_removed_object_does_not_leave_cached_coordinates(self):
        detector = HSVMaterialDetector()
        image = background()
        paint(image, (100, 100), (0, 0, 40))
        self.assertEqual(len(detector.detect(image)), 1)
        self.assertEqual(detector.detect(background()), [])
        self.assertEqual(detector.detect(None), [])

    def test_region_wrapper_preserves_full_camera_coordinates(self):
        image = np.full((600, 900, 3), 255, np.uint8)
        image[100:500, 200:800] = background()
        paint(image, (400, 300), (98, 130, 240))
        found = MaterialRegionDetector(HSVMaterialDetector(), (200, 100, 600, 400)).detect(image)
        self.assertEqual(len(found), 1)
        self.assertLessEqual(abs(found[0].center_x-400), 1)
        self.assertLessEqual(abs(found[0].center_y-300), 1)

    def test_hsv_selection_does_not_load_yolo(self):
        with patch('pi_vision.pipeline.YoloMaterialDetector.load') as load:
            config = default_config()
            pipeline = VisionPipeline(config, material_backend='hsv')
        load.assert_not_called()
        self.assertIsInstance(pipeline.material_detector, HSVMaterialDetector)

    def test_saturated_blue_stays_blue_when_hue_moves_into_cyan_band(self):
        for hue in (98, 102, 105, 108, 115):
            with self.subTest(hue=hue):
                image = background()
                paint(image, (200, 200), (hue, 220, 220), radius=60)
                self.assertEqual([m.color_id for m in HSVMaterialDetector().detect(image)], [3])

    def test_local_cyan_highlight_does_not_split_one_blue_object(self):
        image = background()
        paint(image, (200, 200), (110, 220, 210), radius=65)
        paint(image, (215, 200), (100, 130, 240), radius=23)
        found = HSVMaterialDetector().detect(image)
        self.assertEqual([m.color_id for m in found], [3])
        self.assertLessEqual(abs(found[0].center_x-200), 1)

    def test_real_blue_and_lightblue_with_small_gap_remain_two_targets(self):
        image = background()
        paint(image, (180, 200), (110, 220, 210), radius=50)
        paint(image, (286, 200), (98, 130, 240), radius=50)
        self.assertEqual([m.color_id for m in HSVMaterialDetector().detect(image)], [3, 6])

    def test_two_touching_color_cores_remain_two_targets(self):
        image = background()
        paint(image, (180, 200), (110, 220, 210), radius=50)
        paint(image, (270, 200), (98, 130, 240), radius=50)
        self.assertEqual([m.color_id for m in HSVMaterialDetector().detect(image)], [3, 6])

    def test_large_enclosed_highlight_belongs_to_one_blue_object(self):
        image = background()
        paint(image, (200, 200), (110, 220, 210), radius=65)
        paint(image, (200, 200), (98, 130, 240), radius=38)
        self.assertEqual([m.color_id for m in HSVMaterialDetector().detect(image)], [3])

    def test_enclosed_dark_blue_patch_belongs_to_one_lightblue_object(self):
        image = background()
        paint(image, (200, 200), (98, 130, 240), radius=65)
        paint(image, (200, 200), (110, 220, 210), radius=38)
        self.assertEqual([m.color_id for m in HSVMaterialDetector().detect(image)], [6])


if __name__ == '__main__':
    unittest.main()
