"""物料区域检测的坐标一致性、输出顺序和观测时间测试。"""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from pi_vision.material_region import MaterialRegionDetector, ordered_materials
from pi_vision.pipeline import VisionPipeline
from pi_vision.stream import FrameStreamer
from pi_vision.types import Material


def material(color=2, x=10, y=20, score=40, index=1):
    return Material(index, color, 0, x, y, 0.0, score)


class MaterialRegionTests(unittest.TestCase):
    def test_roi_uses_crop_but_returns_full_frame_coordinates(self):
        source = Mock()
        source.detect.return_value = [material()]
        detector = MaterialRegionDetector(source, (100, 80, 200, 150))
        frame = np.zeros((400, 600, 3), np.uint8)
        frame[80:230, 100:300] = 99
        result = detector.detect(frame)
        crop = source.detect.call_args.args[0]
        self.assertEqual(crop.shape, (150, 200, 3))
        self.assertTrue(np.all(crop == 99))
        self.assertEqual((result[0].center_x, result[0].center_y), (110, 100))
        self.assertEqual(source.detect.return_value[0].center_x, 10)

    def test_confidence_changes_cannot_swap_color_rows(self):
        a = ordered_materials([material(3, score=39), material(2, score=32)])
        b = ordered_materials([material(2, score=39), material(3, score=32)])
        self.assertEqual([item.color_id for item in a], [2, 3])
        self.assertEqual([item.color_id for item in b], [2, 3])
        self.assertEqual([item.index for item in b], [1, 2])

    def test_ordering_does_not_freeze_or_invent_positions(self):
        self.assertEqual(ordered_materials([material(x=123)])[0].center_x, 123)
        self.assertEqual(ordered_materials([material(x=456)])[0].center_x, 456)
        self.assertEqual(ordered_materials([]), [])

    def test_region_outside_actual_image_is_rejected(self):
        detector = MaterialRegionDetector(Mock(), (100, 80, 200, 150))
        with self.assertRaises(ValueError):
            detector.detect(np.zeros((100, 100, 3), np.uint8))
        detector.detector.detect.assert_not_called()

    def test_invalid_regions_are_rejected(self):
        for roi in ((0, 0, 0, 100), (-1, 0, 10, 10), (0, 0, True, 10),
                    (0, 0, 3.5, 10), (0, 0, 10)):
            with self.subTest(roi=roi), self.assertRaises(ValueError):
                MaterialRegionDetector(Mock(), roi)

    def test_centers_outside_crop_are_not_reported(self):
        source = Mock()
        source.detect.return_value = [material(x=-1), material(x=200), material(x=50)]
        result = MaterialRegionDetector(source, (100, 80, 200, 150)).detect(
            np.zeros((400, 600, 3), np.uint8))
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].center_x, 150)

    def test_roi_can_be_changed_or_disabled_without_double_offsets(self):
        pipeline = VisionPipeline.__new__(VisionPipeline)
        source = Mock()
        source.detect.return_value = [material(x=10, y=10)]
        pipeline.material_detector = source
        pipeline.set_material_roi((100, 80, 100, 100))
        pipeline.set_material_roi((200, 180, 100, 100))
        result = pipeline.detect_materials(np.zeros((400, 600, 3), np.uint8))
        self.assertEqual(result[0].center_x, 210)
        pipeline.set_material_roi(None)
        self.assertIs(pipeline.material_detector, source)

    def test_empty_frame_does_not_reuse_old_detection(self):
        detector = MaterialRegionDetector(Mock(), (0, 0, 100, 100))
        self.assertEqual(detector.detect(None), [])
        self.assertEqual(detector.detect(np.array([], np.uint8)), [])

    def test_production_detection_loop_publishes_fixed_order_and_observation_id(self):
        camera, detector = Mock(), Mock()
        pipeline = SimpleNamespace(qr_camera=camera, material_detector=detector)
        streamer = FrameStreamer(camera, pipeline, enable_qr=False,
                                 enable_materials=True, enable_rings=False)
        self.addCleanup(streamer._executor.shutdown, wait=False)
        streamer._latest_raw = np.zeros((20, 30, 3), np.uint8)
        streamer._raw_sequence = 10
        streamer._raw_acquired_at = 1.0
        streamer._running = True
        def detect(_):
            streamer._running = False
            return [material(3, score=39), material(2, score=32)]
        detector.detect.side_effect = detect
        with patch('pi_vision.stream.time.sleep'):
            streamer._detect_loop()
        state = streamer.get_state()
        self.assertEqual([item['color_id'] for item in state['materials']], [2, 3])
        self.assertEqual(state['material_observation']['sequence'], 1)
        self.assertEqual(state['material_observation']['source_sequence'], 10)


if __name__ == '__main__':
    unittest.main()
