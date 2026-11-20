import unittest

from tools.material_acceptance import summarize


def record(sequence, materials=None, age=20, request_ms=10):
    return {'request_ms': request_ms, 'state': {
        'material_observation': {'sequence': sequence, 'age_ms': age},
        'capture': {'age_ms': 20}, 'detection': {'materials': True},
        'materials': materials or []}}


class AcceptanceTests(unittest.TestCase):
    def test_duplicates_and_stale_data_cannot_inflate_detection_rate(self):
        detected = [{'color_id': 4, 'center_x': 100, 'center_y': 80}]
        result = summarize([record(1, detected), record(1, detected),
                            record(2), record(3, detected, age=2000)], {4})
        self.assertEqual(result['fresh_unique_observations'], 2)
        self.assertEqual(result['colors']['4']['single_detection_rate'], .5)
        self.assertEqual(result['duplicate_polls'], 1)
        self.assertEqual(result['stale_or_unknown_age'], 1)

    def test_ambiguous_color_is_not_arbitrarily_selected(self):
        a = {'color_id': 5, 'center_x': 10, 'center_y': 20}
        result = summarize([record(1, [a, a]), record(2, [a]),
                            record(3, [{**a, 'center_x': 13}])], {5})
        self.assertEqual(result['colors']['5']['multiple_detections'], 1)
        self.assertEqual(result['colors']['5']['center_x_span_px'], 3)

    def test_network_failure_does_not_become_an_object_miss(self):
        result = summarize([{'error': 'timeout'}], {6})
        self.assertIsNone(result['colors']['6']['single_detection_rate'])
        self.assertEqual(result['request_errors'], 1)

    def test_transfer_delay_and_restart_are_not_accepted_as_fresh(self):
        result = summarize([record(10, age=10, request_ms=1100), record(2)], {1})
        self.assertEqual(result['fresh_unique_observations'], 0)
        self.assertEqual(result['sequence_regressions'], 1)

    def test_fault_and_reprocessed_source_cannot_count_as_fresh_detection(self):
        a, b, c = record(1), record(2), record(3)
        a['state']['material_observation'].update(valid=False, status='error')
        b['state']['material_observation'].update(source_sequence=10, valid=True)
        c['state']['material_observation'].update(source_sequence=10, valid=True)
        result = summarize([a, b, c], {1})
        self.assertEqual(result['fresh_unique_observations'], 1)
        self.assertEqual(result['invalid_observations'], 1)
        self.assertEqual(result['duplicate_polls'], 1)

    def test_a_new_process_session_is_not_mixed_with_the_original_baseline(self):
        a, b = record(1), record(2)
        a['state']['session_id'] = 'old'
        b['state']['session_id'] = 'new'
        result = summarize([a, b], {1})
        self.assertEqual(result['fresh_unique_observations'], 1)
        self.assertEqual(result['session_changes'], 1)


if __name__ == '__main__':
    unittest.main()
