"""Executed on the Pi over SSH. Starts existing services and preview backends."""
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request


def emit(event, **fields):
    print(json.dumps(dict(event=event, **fields), ensure_ascii=False), flush=True)


def probe(url):
    try:
        with urllib.request.urlopen(url, timeout=2) as response:
            data = json.load(response)
            return data if isinstance(data, dict) else None
    except (OSError, ValueError, urllib.error.URLError):
        return None


def spawn(command, project, log_path):
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open('ab') as log:
        subprocess.Popen(command, cwd=project, stdin=subprocess.DEVNULL,
                         stdout=log, stderr=subprocess.STDOUT, start_new_session=True)


def ensure_monitor(project, state):
    if state is not None:
        return
    script = project/'tools/communication_monitor.py'
    if not script.exists():
        raise RuntimeError('树莓派缺少通讯监看程序，请先部署 tools/communication_monitor.py')
    result = subprocess.run(
        ['systemctl', '--user', 'start', 'communication-monitor.service'],
        capture_output=True, timeout=10)
    if result.returncode:
        spawn([sys.executable, '-u', '-m', 'tools.communication_monitor'], project,
              project/'output/desktop/communication.log')


def ensure_controller():
    """Reuse the active formal/test UART owner, otherwise start test standby."""
    for name in ('competition-button.service', 'chassis-button.service'):
        result = subprocess.run(['systemctl', '--user', 'show', '-p', 'ActiveState', '--value', name],
                                capture_output=True, timeout=5)
        state = result.stdout.decode(errors='replace').strip()
        if state in ('active', 'activating', 'reloading', 'deactivating'):
            return False
    result = subprocess.run(['systemctl', '--user', 'start', 'chassis-button.service'],
                            capture_output=True, timeout=10)
    if result.returncode:
        raise RuntimeError('按键待机服务启动失败，请查看树莓派 chassis-button.service 日志')
    return True


def ensure_radar(project, state):
    if state is not None:
        return None
    config_path = project/'config/chassis_button.local.json'
    if not config_path.exists():
        return '缺少树莓派本机雷达启动配置'
    config = json.loads(config_path.read_text())
    device = Path(config['lidar_device'])
    if not device.exists():
        return '等待雷达USB接入树莓派'
    holders = subprocess.run(['fuser', str(device)], capture_output=True, timeout=3)
    if holders.returncode != 1:
        return '雷达串口正在使用，等待现有后端就绪'
    command = config['lidar_command']
    if not isinstance(command, list) or '--seconds' not in command:
        return '雷达启动配置不完整'
    if command[command.index('--seconds')+1] != '0':
        return '雷达启动配置需要 --seconds 0'
    spawn(command, project, Path(config['output_root'])/'lidar_service.log')
    return '正在启动雷达扫描'


def ensure_cameras(state):
    if state is not None:
        return
    for name in ('material-vision.service', 'camera-console.service'):
        result = subprocess.run(['systemctl', '--user', 'start', name], capture_output=True, timeout=10)
        if result.returncode:
            raise RuntimeError('相机服务启动失败 '+name)


def summarize(radar, monitor, radar_message=None, monitor_error=None, camera=None, camera_error=None):
    radar_info = dict(http_ready=radar is not None, valid=False, message=radar_message or '等待雷达后端')
    if radar is not None:
        radar_info['valid'] = radar.get('valid') is True
        frame = radar.get('frame') or {}
        if radar_info['valid']:
            radar_info['message'] = f"扫描中 · {frame.get('frequency_hz', '—')} Hz · {len(frame.get('points', []))} 点"
        else:
            radar_info['message'] = radar.get('error') or '雷达后端已启动，等待有效扫描'
    comm_info = dict(http_ready=monitor is not None, valid=False,
                     message=monitor_error or '正在启动通讯监看')
    if monitor is not None:
        comm_info['valid'] = monitor.get('controller_recent') is True and not monitor_error
        comm_info['message'] = monitor_error or (
            'STM32持续回复' if comm_info['valid'] else '监看已启动，等待STM32有效回复')
    camera_info = dict(http_ready=camera is not None, valid=bool(camera and camera.get('valid')),
                       message=camera_error or ('物料相机在线' if camera and camera.get('valid') else '等待相机服务'))
    return dict(radar=radar_info, communication=comm_info, camera=camera_info)


def main():
    project = Path.home()/'competition_vision'
    if not project.is_dir():
        emit('error', message=f'树莓派项目目录不存在 {project}')
        return 1
    emit('connected', message='SSH已连接，检查后端服务')
    lock_path = Path(tempfile.gettempdir())/f'robot-preview-start-{os.getuid()}.lock'
    with lock_path.open('a') as lock:
        while True:
            radar = probe('http://127.0.0.1:8766/state')
            monitor = probe('http://127.0.0.1:8767/api/state')
            camera = probe('http://127.0.0.1:8091/api/state?camera=material')
            camera_error = None
            radar_message = monitor_error = None
            locked = False
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                locked = True
                controller_started = False
                try:
                    controller_started = ensure_controller()
                except (OSError, subprocess.SubprocessError, RuntimeError) as exc:
                    monitor_error = str(exc)
                # Recheck after taking the cross-client startup lock.
                if monitor is None:
                    try:
                        ensure_monitor(project, probe('http://127.0.0.1:8767/api/state'))
                    except (OSError, ValueError, subprocess.SubprocessError, RuntimeError) as exc:
                        monitor_error = str(exc)
                try:
                    ensure_cameras(camera)
                except (OSError, subprocess.SubprocessError, RuntimeError) as exc:
                    camera_error = str(exc)
                if radar is None and controller_started:
                    radar_message = '按键待机服务已启动，正在预热雷达'
                elif radar is None:
                    try:
                        radar_message = ensure_radar(project, probe('http://127.0.0.1:8766/state'))
                    except (OSError, ValueError, KeyError, IndexError, subprocess.SubprocessError) as exc:
                        radar_message = f'雷达启动失败 {exc}'
            except BlockingIOError:
                radar_message = '其他连接正在启动后端，稍候自动检查'
            finally:
                if locked:
                    fcntl.flock(lock, fcntl.LOCK_UN)
            emit('services', **summarize(radar, monitor, radar_message, monitor_error, camera, camera_error))
            time.sleep(3)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (BrokenPipeError, KeyboardInterrupt):
        pass
