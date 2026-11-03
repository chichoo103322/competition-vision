import json

import pytest

from tools.chassis_button_service import (atomic_json, new_tick, wait_fresh_key,
                                         lidar_fresh, wait_ready_key)


class Client:
    def __init__(self, keys, ticks=None, state=0):
        self.keys = iter(keys)
        self.ticks = iter(ticks or [110, 120, 130])
        self.state = state

    def status(self):
        return dict(tick=next(self.ticks), state=self.state, fault=0, chassis_fault=0)

    def wait_start_area(self, timeout, cancelled):
        key = next(self.keys)
        if key is None:
            raise TimeoutError()
        return dict(request_id=1, tick=key, area=1, key=1)


def test_key_before_ready_is_not_replayed_but_new_press_is_accepted():
    log = []
    client = Client([90, None, 125])
    selected = wait_fresh_key(client, 100, lambda: True,
                              lambda **e: log.append(e), sleep=lambda _: None)
    assert selected['tick'] == 125
    assert log[0]['event'] == 'IGNORED_OLD_KEY'


def test_no_key_never_starts_and_cancel_exits():
    polls = []
    with pytest.raises(InterruptedError):
        wait_fresh_key(Client([None]), 100, lambda: True, lambda **_: None,
                       cancelled=lambda: bool(polls), sleep=lambda _: polls.append(1))


def test_stale_lidar_at_key_does_not_start():
    with pytest.raises(RuntimeError, match='lidar not ready'):
        wait_fresh_key(Client([115]), 100, lambda: False, lambda **_: None)


@pytest.mark.parametrize('state', [2, 3, 6])
def test_no_arming_from_active_or_faulted_controller(state):
    with pytest.raises(RuntimeError, match='active/faulted'):
        wait_fresh_key(Client([115], state=state), 100, lambda: True, lambda **_: None)


def test_reboot_requires_rearm():
    with pytest.raises(RuntimeError, match='rebooted'):
        wait_fresh_key(Client([5], ticks=[10]), 100, lambda: True, lambda **_: None)
    assert new_tick(5, 0xfffffff0)  # legitimate uint32 wrap


def test_standby_waits_for_idle_then_uses_only_a_fresh_key():
    class BusyThenIdle(Client):
        def __init__(self):
            super().__init__([None, 125], ticks=[100, 101, 102, 103, 104], state=0)
            self.statuses = iter([
                dict(tick=100, state=2, fault=0, chassis_fault=0),
                dict(tick=101, state=6, fault=1, chassis_fault=0),
                dict(tick=102, state=5, fault=0, chassis_fault=0),
                dict(tick=103, state=0, fault=0, chassis_fault=0),
                dict(tick=104, state=0, fault=0, chassis_fault=0),
            ])
        def status(self):
            return next(self.statuses)
    states = []
    client, selected = BusyThenIdle(), None
    _, selected = wait_ready_key(client, lambda: True,
        lambda name, **_: states.append(name), lambda **_: None, sleep=lambda _: None)
    assert selected['tick'] == 125
    assert states.count('WAIT_CONTROLLER') == 1
    assert states.count('READY') == 1


def test_atomic_state_replaces_existing_file(tmp_path):
    path = tmp_path/'status.json'
    atomic_json(path, dict(state='READY'))
    atomic_json(path, dict(state='DONE'))
    assert json.loads(path.read_text())['state'] == 'DONE'
    assert not path.with_name('status.json.tmp').exists()


@pytest.mark.parametrize('value', [False, None, -1, 1000, float('nan')])
def test_lidar_must_be_fresh_before_key(monkeypatch, value):
    monkeypatch.setattr('tools.chassis_button_service.http_json',
        lambda _: dict(valid=True, age_ms=value, frame=dict(seq=10)))
    # False is bool, not an age value, and should be refused as malformed data.
    assert not lidar_fresh('http://127.0.0.1:8766/state')


def test_competition_entry_has_no_implicit_test_fallback(tmp_path):
    from tools.competition_button_service import main
    path = tmp_path/'config.json'
    path.write_text('{}')
    with pytest.raises(SystemExit) as error:
        main(['--config', str(path)])
    assert error.value.code == 2


@pytest.mark.parametrize('simulation', [True, None])
def test_competition_refuses_mock_or_unspecified_backend(monkeypatch, simulation):
    from types import SimpleNamespace
    from tools.chassis_button_service import load_competition_backend
    backend = SimpleNamespace(simulation=simulation, ready=lambda: True,
                              plan=lambda **_: {}, on_work=lambda *_: True)
    monkeypatch.setattr('tools.chassis_button_service.importlib.import_module',
                        lambda _: SimpleNamespace(create=lambda config: backend))
    with pytest.raises(ValueError, match='simulation=False'):
        load_competition_backend('backend:create', {})


def test_standby_timeout_recovers_and_discards_keys_pressed_while_offline(monkeypatch):
    from tools.chassis_button_service import wait_ready_key
    class RecoveringClient(Client):
        transport = object()
        clock = staticmethod(lambda: 0)
        def status(self):
            tick = next(self.ticks)
            if tick is None:
                raise TimeoutError('STM32 off')
            return dict(tick=tick, state=0, fault=0, chassis_fault=0)
    first = RecoveringClient([], ticks=[100, None])
    restored = RecoveringClient([150, 225], ticks=[200, 210, 230])
    monkeypatch.setattr('tools.chassis_button_service.ElectricalClient', lambda *a, **k: restored)
    events, states = [], []
    client, selected = wait_ready_key(first, lambda: True,
        lambda state, **kw: states.append(state), lambda **e: events.append(e), sleep=lambda _: None)
    assert client is restored and selected['tick'] == 225
    assert states == ['WAIT_CONTROLLER', 'READY', 'WAIT_CONTROLLER', 'READY']
    assert [e['event'] for e in events] == ['STANDBY_RECONNECT', 'IGNORED_OLD_KEY']


def test_controller_off_at_start_keeps_waiting_and_can_be_cancelled():
    from tools.chassis_button_service import wait_ready_key
    class Offline:
        def status(self):
            raise TimeoutError('off')
    cancel = []
    with pytest.raises(InterruptedError):
        wait_ready_key(Offline(), lambda: True, lambda *a, **k: None, lambda **k: None,
                       cancelled=lambda: bool(cancel), sleep=lambda _: cancel.append(True))
