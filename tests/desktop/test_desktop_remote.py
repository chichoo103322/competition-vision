from pathlib import Path
from unittest.mock import Mock

from tools import desktop_remote as remote


def test_existing_backends_are_reused_without_process_commands(monkeypatch):
    run = Mock(side_effect=AssertionError('existing service must be reused'))
    monkeypatch.setattr(remote.subprocess, 'run', run)
    remote.ensure_monitor(Path('/unused'), {})
    assert remote.ensure_radar(Path('/unused'), {}) is None
    run.assert_not_called()


def test_missing_radar_does_not_start_any_process(tmp_path, monkeypatch):
    import json
    config = tmp_path/'config'
    config.mkdir()
    (config/'chassis_button.local.json').write_text(json.dumps({'lidar_device': str(tmp_path/'absent')}))
    spawn = Mock()
    monkeypatch.setattr(remote, 'spawn', spawn)
    assert 'USB' in remote.ensure_radar(tmp_path, None)
    spawn.assert_not_called()


def test_busy_uart_does_not_get_a_second_collector(tmp_path, monkeypatch):
    import json
    config = tmp_path/'config'
    config.mkdir()
    device = tmp_path/'device'
    device.touch()
    (config/'chassis_button.local.json').write_text(json.dumps({'lidar_device': str(device)}))
    monkeypatch.setattr(remote.subprocess, 'run', Mock(return_value=Mock(returncode=0)))
    spawn = Mock()
    monkeypatch.setattr(remote, 'spawn', spawn)
    assert '正在使用' in remote.ensure_radar(tmp_path, None)
    spawn.assert_not_called()


def test_backend_status_separates_http_from_real_scan_and_mcu_data():
    state = remote.summarize({'valid': False}, {'controller_recent': False})
    assert state['radar']['http_ready'] is True
    assert state['radar']['valid'] is False
    assert state['communication']['http_ready'] is True
    assert state['communication']['valid'] is False
    state = remote.summarize({'valid': True, 'frame': {'frequency_hz': 6, 'points': [[0, 1, 3]]}},
                             {'controller_recent': True})
    assert state['radar']['valid'] is True
    assert '1 点' in state['radar']['message']
    assert state['communication']['valid'] is True
    state = remote.summarize(None, {'controller_recent': False}, monitor_error='按键待机服务启动失败')
    assert state['communication']['message'] == '按键待机服务启动失败'


def test_formal_uart_owner_is_reused_without_starting_test_service(monkeypatch):
    run = Mock(return_value=Mock(returncode=0, stdout=b'activating\n'))
    monkeypatch.setattr(remote.subprocess, 'run', run)
    assert remote.ensure_controller() is False
    assert run.call_count == 1
    assert run.call_args.args[0] == [
        'systemctl', '--user', 'show', '-p', 'ActiveState', '--value', 'competition-button.service']


def test_inactive_services_start_standby_once_without_restart(monkeypatch):
    run = Mock(side_effect=[Mock(returncode=0, stdout=b'inactive\n'),
                           Mock(returncode=0, stdout=b'inactive\n'), Mock(returncode=0)])
    monkeypatch.setattr(remote.subprocess, 'run', run)
    assert remote.ensure_controller() is True
    assert run.call_args.args[0] == ['systemctl', '--user', 'start', 'chassis-button.service']
