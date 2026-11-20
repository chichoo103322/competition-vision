"""Task planning/feedback simulation only; never opens a camera or motor port."""

from copy import deepcopy
import itertools
import unittest

from pi_vision.mission import Phase, TaskCoordinator, build_plan
from pi_vision.mission_demo import MockBackend, simulate


TASK = '156+123+516+231'


class PlanTests(unittest.TestCase):
    def test_second_batch_stacks_by_first_batch_color_not_fourth_group(self):
        for capacity in (1, 2, 3):
            plan = build_plan(TASK, capacity, 'start_B')
            stacked = [s.params for s in plan if s.operation == 'place_from_tray'
                       and s.params['zone'] == 'staging' and s.params['batch'] == 2]
            self.assertEqual([(p['color_id'], p['position_id'], p['layer']) for p in stacked],
                             [(5, 2, 1), (1, 1, 1), (6, 3, 1)])
            self.assertEqual(plan[-2].params['home_id'], 'start_B')
            for batch in (1, 2):
                rough_places = [i for i, s in enumerate(plan) if s.operation == 'place_from_tray'
                                and s.params['batch'] == batch and s.params['zone'] == 'rough']
                rough_picks = [i for i, s in enumerate(plan) if s.operation == 'pick_to_tray'
                               and s.params['batch'] == batch and s.params['zone'] == 'rough']
                self.assertLess(max(rough_places), min(rough_picks))

    def test_complete_flow_for_all_colors_and_capacities(self):
        for colors in itertools.combinations('123456', 3):
            for capacity in (1, 2, 3):
                first = ''.join(colors)
                second = first[::-1]
                with self.subTest(colors=colors, capacity=capacity):
                    engine, trace = simulate(first+'+312+'+second+'+231', capacity)
                    self.assertEqual(engine.phase, Phase.COMPLETED)
                    self.assertEqual(engine.counts, {'picked': 12, 'placed': 12, 'stacked': 3})
                    self.assertEqual(engine.tray, {})
                    self.assertTrue(all(c['simulation'] for c in trace))
                    self.assertEqual(len({c['command_id'] for c in trace}), len(trace))

    def test_rejects_invalid_code_capacity_and_home(self):
        for code, capacity, home in [('156+123+512+231', 1, 'A'), (TASK, True, 'A'),
                                     (TASK, 0, 'A'), (TASK, 4, 'A'), (TASK, 1, '')]:
            with self.subTest(code=code, capacity=capacity), self.assertRaises(ValueError):
                build_plan(code, capacity, home)


class CoordinatorTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.engine = TaskCoordinator(simulation=True, clock=lambda: self.now,
                                      wall_clock=lambda: self.now+1700000000)
        self.mock = MockBackend(TASK)
        self.engine.start('start_A')

    def reply(self):
        self.now += .15
        return self.mock.reply(self.engine.pending.to_dict(), self.now+1700000000)

    def reach(self, operation):
        for _ in range(200):
            if self.engine.pending.operation == operation:
                return
            self.engine.drain_commands()
            self.engine.feedback(self.reply())
        self.fail('could not reach '+operation)

    def test_read_task_requires_fresh_continuous_confirmations(self):
        first = self.reply()
        self.engine.feedback(first)
        for _ in range(5):
            self.now += .01
            self.assertFalse(self.engine.feedback(first))
        self.assertEqual(self.engine.phase, Phase.READING_TASK)
        self.engine.feedback(self.reply())
        invalid = self.reply()
        invalid['data']['task_code'] = '156+123+512+231'
        self.engine.feedback(invalid)
        self.assertIsNone(self.engine.task)
        for _ in range(3):
            self.engine.feedback(self.reply())
        self.assertEqual(str(self.engine.task), TASK)

    def test_invalid_observation_breaks_qr_confirmation(self):
        for _ in range(2):
            self.engine.feedback(self.reply())
        invalid = self.reply()
        invalid['data']['valid'] = False
        self.engine.feedback(invalid)
        self.engine.feedback(self.reply())
        self.assertIsNone(self.engine.task)

    def test_camera_restart_cannot_complete_previous_qr_votes(self):
        for _ in range(2):
            self.engine.feedback(self.reply())
        message = self.reply()
        message['data']['source_session_id'] = 'new-camera-session'
        self.engine.feedback(message)
        self.assertIsNone(self.engine.task)

    def test_timeouts_are_configurable_but_not_nonfinite(self):
        for options in ({'observation_timeout': 0}, {'timeouts': {'navigate': float('nan')}},
                        {'timeouts': {'not_an_action': 1}}, {'capacity': False}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                TaskCoordinator(**options)

    def test_ack_is_not_action_completion_or_count(self):
        self.reach('pick_to_tray')
        message = self.reply()
        message['status'] = 'accepted'
        pending = self.engine.pending
        self.assertFalse(self.engine.feedback(message))
        self.assertEqual(self.engine.pending, pending)
        self.assertEqual(self.engine.counts['picked'], 0)
        message['status'] = 'done'
        self.assertTrue(self.engine.feedback(message))
        self.assertEqual(self.engine.counts['picked'], 1)
        self.assertFalse(self.engine.feedback(message))
        self.assertEqual(self.engine.counts['picked'], 1)

    def test_wrong_mode_run_or_command_cannot_advance(self):
        for key, value in [('run_id', 'old-run'), ('command_id', 'old-command'),
                           ('simulation', False), ('schema_version', True)]:
            message = self.reply()
            message[key] = value
            self.assertFalse(self.engine.feedback(message))
        self.assertIsNone(self.engine.task)

    def test_real_coordinator_rejects_simulated_feedback(self):
        engine = TaskCoordinator(clock=lambda: self.now, wall_clock=lambda: self.now+1700000000)
        engine.start()
        fake = self.reply()
        fake.update(run_id=engine.run_id, command_id=engine.pending.command_id)
        self.assertFalse(engine.feedback(fake))
        self.assertIsNone(engine.task)

    def test_material_rejects_wrong_stale_ambiguous_or_nonpixel_targets(self):
        self.reach('scan_material')
        mutations = [lambda d: d.update(acquired_unix_s=d['acquired_unix_s']-2),
                     lambda d: d.update(acquired_unix_s=d['acquired_unix_s']+2),
                     lambda d: d.update(units='mm'),
                     lambda d: d.update(zone='staging'),
                     lambda d: d.update(camera_role='qr'),
                     lambda d: d.update(targets=d['targets']*2),
                     lambda d: d['targets'][0].update(u=float('nan')),
                     lambda d: d['targets'][0].update(v=720),
                     lambda d: d['targets'][0].update(color_id=2),
                     lambda d: d.update(status='no_target')]
        for mutate in mutations:
            message = self.reply()
            mutate(message['data'])
            self.assertFalse(self.engine.feedback(message))
            self.assertEqual(self.engine.pending.operation, 'scan_material')
        self.assertTrue(self.engine.feedback(self.reply()))
        target = self.engine.pending.params['target']
        self.assertEqual(target['units'], 'pixels')
        self.assertEqual(target['width'], 1280)
        self.assertTrue(self.engine.pending.params['requires_calibrated_backend'])

    def test_camera_failure_issues_stop_and_never_advances(self):
        self.reach('scan_material')
        message = self.reply()
        message['data'].update(valid=False, status='camera_error')
        self.engine.feedback(message)
        self.assertEqual(self.engine.phase, Phase.STOPPING)
        self.assertEqual(self.engine.pending.operation, 'stop')
        self.assertEqual(self.engine.counts['picked'], 0)

    def test_timeout_rejects_late_success_and_does_not_replay_motion(self):
        self.reach('pick_to_tray')
        late = self.reply()
        self.now += 15
        self.assertFalse(self.engine.feedback(late))
        self.assertEqual(self.engine.pending.operation, 'stop')
        self.assertEqual(self.engine.counts['picked'], 0)
        commands = self.engine.drain_commands()
        self.assertEqual([c['operation'] for c in commands], ['stop'])
        self.engine.feedback(self.reply())
        self.assertEqual(self.engine.phase, Phase.FAULT)

    def test_stop_timeout_blocks_restart_until_stop_confirmed(self):
        self.engine.cancel()
        self.now += 3
        self.engine.tick()
        self.assertEqual(self.engine.phase, Phase.FAULT)
        with self.assertRaises(ValueError):
            self.engine.start()
        self.engine.feedback(self.reply())  # A late STOP ack is safe to accept.
        self.assertTrue(self.engine.stop_confirmed)
        old_run = self.engine.run_id
        self.engine.start()
        self.assertNotEqual(self.engine.run_id, old_run)

    def test_unverified_pick_or_placement_is_not_success(self):
        for operation, flag in [('pick_to_tray', 'loaded'), ('place_from_tray', 'placement_verified')]:
            self.setUp()
            self.reach(operation)
            message = self.reply()
            message['data'][flag] = False
            before = dict(self.engine.counts)
            self.engine.feedback(message)
            self.assertEqual(self.engine.phase, Phase.STOPPING)
            self.assertEqual(self.engine.counts, before)

    def test_second_batch_stack_requires_correct_base_color(self):
        for _ in range(300):
            current = self.engine.pending
            if current.operation == 'locate_position' and current.params.get('layer') == 1:
                break
            self.engine.feedback(self.reply())
        else:
            self.fail('no stack location')
        message = self.reply()
        message['data']['targets'][0]['base_color_id'] = 2
        self.assertFalse(self.engine.feedback(message))
        self.assertEqual(self.engine.pending.operation, 'locate_position')
        self.assertTrue(self.engine.feedback(self.reply()))
        self.assertEqual(self.engine.pending.operation, 'place_from_tray')

    def test_cancel_and_new_run_reject_old_feedback(self):
        old = self.reply()
        self.engine.cancel()
        self.assertFalse(self.engine.feedback(old))
        self.engine.feedback(self.reply())
        self.assertEqual(self.engine.phase, Phase.CANCELLED)
        self.engine.start()
        self.assertFalse(self.engine.feedback(old))

    def test_monotonic_deadline_does_not_follow_wall_clock_jump(self):
        self.engine.wall_clock = lambda: -999999
        self.engine.tick()
        self.assertEqual(self.engine.phase, Phase.READING_TASK)
        self.now += 10
        self.engine.tick()
        self.assertEqual(self.engine.phase, Phase.STOPPING)

    def test_mock_failure_is_explicit_not_completed(self):
        engine, trace = simulate(TASK, 3, 'pick_to_tray')
        self.assertEqual(engine.phase, Phase.FAULT)
        self.assertTrue(engine.stop_confirmed)
        self.assertEqual(engine.counts, {'picked': 0, 'placed': 0, 'stacked': 0})
        self.assertEqual(trace[-1]['operation'], 'stop')


if __name__ == '__main__':
    unittest.main()
