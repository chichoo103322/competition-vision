"""Field profiles must be opt-in, validated and reproducible from raw evidence."""

from dataclasses import asdict
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch
from unittest.mock import Mock

import cv2
import numpy as np

from pi_vision.diagnostics import DiagnosticRecorder
from pi_vision.hsv_profile import load_profile, validate_profile, profile_hash
from pi_vision.detectors.hsv_material_detector import HSVMaterialDetector
from pi_vision.material_region import MaterialRegionDetector
from tools.hsv_replay import replay
from pi_vision.camera import Camera
from pi_vision.config import CameraConfig, load_config
from pi_vision.stream import FrameStreamer


class ProfileTests(unittest.TestCase):
    def test_stream_records_the_detection_source_not_the_newest_capture(self):
        for fails in (False, True):
            with self.subTest(fails=fails):
                image = np.zeros((40, 60, 3), np.uint8)
                recorder, camera, detector = Mock(), Mock(), Mock()
                detector.profile = validate_profile()
                streamer = FrameStreamer(camera, SimpleNamespace(qr_camera=camera, material_detector=detector),
                                         enable_qr=False, enable_rings=False, diagnostic_recorder=recorder)
                streamer._latest_raw = image
                streamer._raw_sequence = 7
                streamer._raw_acquired_at = 1.
                streamer._raw_wall_time = 12345.
                streamer._running = True
                def detect(frame):
                    streamer._latest_raw = np.ones((40, 60, 3), np.uint8)
                    streamer._raw_sequence = 8
                    streamer._running = False
                    if fails:
                        raise ValueError('detection failed')
                    return []
                detector.detect.side_effect = detect
                with patch('pi_vision.stream.time.sleep'), patch('pi_vision.stream.time.monotonic', return_value=1.1):
                    streamer._detect_loop()
                streamer._executor.shutdown()
                saved_frame, meta = recorder.offer.call_args.args
                self.assertIs(saved_frame, image)
                self.assertEqual(meta['source_sequence'], 7)
                self.assertEqual(meta['acquired_unix_s'], 12345.)
                self.assertEqual(meta['valid'], not fails)

    def test_device_controls_are_opt_in_and_failure_releases_camera(self):
        capture = Mock()
        with patch('pi_vision.camera.cv2.VideoCapture', return_value=capture), patch('pi_vision.camera.subprocess.run') as run:
            camera = Camera(CameraConfig(undistort=False))
            camera.open()
            run.assert_not_called()
            camera.release()
            run.side_effect = OSError('unsupported control')
            camera = Camera(CameraConfig(undistort=False, v4l2_controls={'gain': 64}))
            with self.assertRaises(OSError):
                camera.open()
            self.assertFalse(camera.is_open)
            self.assertEqual(run.call_args.args[0], ['v4l2-ctl', '--device', '/dev/video0', '--set-ctrl', 'gain=64'])

    def test_camera_yaml_preserves_controls_and_rejects_unknown_names(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'camera.yaml'
            path.write_text('camera:\n  v4l2_controls:\n    white_balance_automatic: 0\n    white_balance_temperature: 4600\n')
            config = load_config(path)
            self.assertEqual(config.camera.v4l2_controls['white_balance_automatic'], 0)
            path.write_text('camera:\n  v4l2_controls:\n    typo: 1\n')
            with self.assertRaises(ValueError):
                load_config(path)

    def test_example_preserves_baseline(self):
        self.assertEqual(load_profile('config/hsv_profile.example.yaml'), validate_profile())

    def test_invalid_and_misspelled_profiles_are_rejected(self):
        invalid = [[], {'version': True}, {'version': 2}, {'colour': {}},
                   {'shape': {'min_fill': float('nan')}}, {'shape': {'max_aspect': 0}},
                   {'bands': {6: [[[0, 0, 0], [1, 2, 3]]]}},
                   {'bands': {1: [[[170, 0, 0], [9, 255, 255]]]}},
                   {'blue': {'cyan_h_max': 180}}, {'shape': {'min_area_ratio': .9}}]
        for raw in invalid:
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                validate_profile(raw)

    def test_candidate_changes_only_explicit_threshold_and_keeps_business_ids(self):
        image = np.full((200, 200, 3), 255, np.uint8)
        red = tuple(map(int, cv2.cvtColor(np.uint8([[[0, 110, 220]]]), cv2.COLOR_HSV2BGR)[0, 0]))
        cv2.circle(image, (100, 100), 40, red, -1)
        self.assertEqual(HSVMaterialDetector().detect(image), [])
        profile = validate_profile({'bands': {1: [[[0, 100, 120], [9, 255, 255]]]}})
        self.assertEqual([m.color_id for m in HSVMaterialDetector(profile=profile).detect(image)], [1])
        self.assertEqual(HSVMaterialDetector().detect(image), [])

    def test_bounded_recorder_and_exact_full_frame_replay(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp) / 'evidence'
            image = np.full((300, 400, 3), 255, np.uint8)
            cv2.circle(image, (200, 150), 40, (0, 0, 220), -1)
            profile = validate_profile()
            detector = MaterialRegionDetector(HSVMaterialDetector(profile=profile), (100, 50, 200, 200))
            materials = [asdict(m) for m in detector.detect(image)]
            meta = {'source_sequence': 10, 'valid': True, 'backend': 'HSVMaterialDetector',
                    'profile': profile, 'profile_sha256': profile_hash(profile),
                    'frame_size': [400, 300], 'roi': [100, 50, 200, 200], 'materials': materials}
            recorder = DiagnosticRecorder(folder, samples=1, interval=.2)
            recorder.offer(image, meta)
            recorder.offer(image, dict(meta, source_sequence=11))
            recorder.close()
            self.assertEqual(recorder.state()['saved'], 1)
            self.assertEqual(replay(folder)['baseline_exact_matches'], 1)
            np.testing.assert_array_equal(cv2.imread(str(folder / 'sample_0001.png')), image)
            self.assertGreater(materials[0]['center_x'], 100)
            stored = folder / 'sample_0001.json'
            data = json.loads(stored.read_text())
            data['profile_sha256'] = 'changed'
            stored.write_text(json.dumps(data))
            with self.assertRaises(ValueError):
                replay(folder)

    def test_full_disk_queue_drops_without_blocking_detector(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(DiagnosticRecorder, '_write_loop'):
            recorder = DiagnosticRecorder(Path(tmp)/'evidence', samples=30, interval=.2)
            with patch('pi_vision.diagnostics.time.monotonic', side_effect=[1, 2, 3, 4]):
                recorder.offer(None, {'source_sequence': 1})
                recorder.offer(None, {'source_sequence': 1})
                recorder.offer(None, {'source_sequence': 2})
                recorder.offer(None, {'source_sequence': 3})
            self.assertEqual(recorder.queued, 2)
            self.assertEqual(recorder.dropped, 1)
            recorder.close()
