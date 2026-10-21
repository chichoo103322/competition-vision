import unittest

from robot_perception.lidar import SafetyMonitor, Scan


def make_scan(front: float = 1.0, side: float = 1.0) -> Scan:
    ranges = [1.0] * 360
    ranges[180] = front
    ranges[270] = side
    return Scan(tuple(ranges), -180, 1)


class LidarTests(unittest.TestCase):
    def test_stop_then_blocked_then_clear(self):
        monitor = SafetyMonitor()
        self.assertEqual(monitor.tick(0).state, "STOP")
        self.assertEqual(monitor.ingest(make_scan(0.2), 0.0, "A-B").state, "STOP")
        self.assertEqual(monitor.ingest(make_scan(0.2), 0.1, "A-B").state, "STOP")
        blocked = monitor.ingest(make_scan(0.2), 0.2, "A-B")
        self.assertEqual(blocked.state, "BLOCKED")
        self.assertEqual(blocked.segment_id, "A-B")
        self.assertTrue(blocked.stop_requested)
        self.assertEqual(monitor.ingest(make_scan(), 0.3).state, "CLEAR")

    def test_side_object_does_not_stop_forward_motion(self):
        monitor = SafetyMonitor()
        self.assertEqual(monitor.ingest(make_scan(side=0.1), 0).state, "CLEAR")

    def test_stale_or_invalid_scan_stops(self):
        monitor = SafetyMonitor()
        monitor.ingest(make_scan(), 1)
        self.assertEqual(monitor.tick(1.31).reason, "stale_scan")
        invalid = Scan(tuple([float("nan")] * 360), -180, 1)
        self.assertEqual(monitor.ingest(invalid, 2).reason, "invalid_scan")

    def test_scan_input_validation(self):
        with self.assertRaises(ValueError):
            Scan.from_dict({"ranges_m": [1] * 360, "angle_min_deg": 0,
                            "angle_increment_deg": 0})


if __name__ == "__main__":
    unittest.main()
