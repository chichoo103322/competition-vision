"""PyQt desktop entry point for the Pi's radar and communication previews."""
import argparse
import json
from pathlib import Path
import re
import socket
import sys
import time

from PyQt6.QtCore import QProcess, QSettings, QTimer, QUrl, Qt
from PyQt6.QtNetwork import QLocalServer, QLocalSocket
from PyQt6.QtWidgets import (
    QApplication, QCheckBox, QFileDialog, QHBoxLayout, QLabel, QLineEdit,
    QMainWindow, QPlainTextEdit, QPushButton, QStackedWidget, QTabWidget,
    QVBoxLayout, QWidget,
)
from PyQt6.QtWebEngineCore import QWebEngineProfile
from PyQt6.QtWebEngineWidgets import QWebEngineView


def reserve_ports():
    """Choose separate ports so existing terminal tunnels can keep running."""
    sockets = [socket.socket() for _ in range(3)]
    try:
        for item in sockets:
            item.bind(('127.0.0.1', 0))
        return [item.getsockname()[1] for item in sockets]
    finally:
        for item in sockets:
            item.close()


def remote_source():
    root = Path(getattr(sys, '_MEIPASS', Path(__file__).parent))
    return (root/'desktop_remote.py').read_bytes()


def ssh_arguments(ports, address=''):
    arguments = [
        '-T', '-S', 'none', '-o', 'BatchMode=yes',
        '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=15',
        '-o', 'ExitOnForwardFailure=yes', '-o', 'ServerAliveInterval=10',
        '-o', 'ServerAliveCountMax=3', '-o', 'IPQoS=none',
    ]
    if address:
        if not re.fullmatch(r'[A-Za-z0-9_.:%-]+', address) or address.startswith('-'):
            raise ValueError('请填写IP或主机名，例如 cuiguo.local，不包含用户名和空格')
        arguments += ['-o', 'HostName='+address]
    for local, remote in zip(ports, (8766, 8767, 8091)):
        arguments += ['-L', f'127.0.0.1:{local}:127.0.0.1:{remote}']
    return arguments + ['pi-vision', 'python3 -u -']


class PreviewTab(QWidget):
    def __init__(self, title, parent=None, html_file=None):
        super().__init__(parent)
        self.title = title
        self.html_file = html_file
        self.loaded = False
        self.url = QUrl()
        self.stack = QStackedWidget()
        self.placeholder = QLabel('连接树莓派后显示'+title+'界面')
        self.placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.placeholder.setWordWrap(True)
        self.web = QWebEngineView()
        self.web.loadFinished.connect(self.loaded_page)
        self.web.page().renderProcessTerminated.connect(self.renderer_failed)
        self.stack.addWidget(self.placeholder)
        self.stack.addWidget(self.web)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.stack)

    def show_message(self, message):
        self.placeholder.setText(message)
        self.stack.setCurrentWidget(self.placeholder)

    def reset(self, message):
        self.loaded = False
        self.url = QUrl()
        self.web.stop()
        self.web.setUrl(QUrl('about:blank'))
        self.show_message(message)

    def open_backend(self, port):
        self.url = QUrl(f'http://127.0.0.1:{port}/')
        if not self.loaded:
            self.loaded = True
            self.stack.setCurrentWidget(self.web)
            self.load_page()

    def load_page(self):
        if self.html_file:
            root = Path(getattr(sys, '_MEIPASS', Path(__file__).parent))
            self.web.setHtml((root/self.html_file).read_text(), self.url)
        else:
            self.web.setUrl(self.url)

    def reload(self):
        if self.url.isValid() and self.url.scheme() == 'http':
            self.loaded = True
            self.stack.setCurrentWidget(self.web)
            self.load_page()

    def loaded_page(self, success):
        if not success and self.loaded:
            self.show_message('页面加载超时。连接恢复后点击“刷新当前界面”。')

    def renderer_failed(self, *_):
        self.loaded = False
        self.show_message('界面进程已退出，正在重新加载。')
        QTimer.singleShot(1500, self.reload)


class RobotConsole(QMainWindow):
    def __init__(self, auto_connect=True):
        super().__init__()
        self.setWindowTitle('小车调试台 · 相机、雷达与通讯')
        self.resize(1280, 900)
        self.setMinimumSize(840, 620)
        self.settings = QSettings('CompetitionRobot', 'RobotConsole')
        self.process = None
        self.wanted = False
        self.connected = False
        self.stdout_buffer = bytearray()
        self.stderr_tail = ''
        self.last_heartbeat = 0.0
        self.retry_delay = 5
        self.ports = []
        self.last_services = None
        self.connection_address = ''
        self.retry_timer = QTimer(self)
        self.retry_timer.setSingleShot(True)
        self.retry_timer.timeout.connect(self.start_connection)
        self.watchdog = QTimer(self)
        self.watchdog.setInterval(2000)
        self.watchdog.timeout.connect(self.check_connection)
        self.build_ui()
        self.watchdog.start()
        if auto_connect and self.auto_connect.isChecked():
            QTimer.singleShot(300, self.connect_requested)

    def build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(18, 16, 18, 14)
        title = QLabel('小车调试台')
        title.setObjectName('title')
        self.connection_label = QLabel('尚未连接树莓派')
        self.connection_label.setWordWrap(True)
        layout.addWidget(title)
        layout.addWidget(self.connection_label)

        controls = QHBoxLayout()
        self.address = QLineEdit(self.settings.value('address', ''))
        self.address.setPlaceholderText('使用已配置的 pi-vision，可选填树莓派新IP或 cuiguo.local')
        self.address.setAccessibleName('树莓派连接地址')
        self.address.returnPressed.connect(self.connect_requested)
        self.connect_button = QPushButton('连接树莓派')
        self.connect_button.clicked.connect(self.connect_requested)
        self.disconnect_button = QPushButton('断开连接')
        self.disconnect_button.clicked.connect(self.disconnect_requested)
        self.disconnect_button.setEnabled(False)
        self.refresh_button = QPushButton('刷新当前界面')
        self.refresh_button.clicked.connect(lambda: self.tabs.currentWidget().reload())
        controls.addWidget(self.address, 1)
        for button in (self.connect_button, self.disconnect_button, self.refresh_button):
            controls.addWidget(button)
        layout.addLayout(controls)

        status = QHBoxLayout()
        self.radar_status = QLabel('雷达 · 等待连接')
        self.comm_status = QLabel('通讯 · 等待连接')
        self.camera_status = QLabel('相机 · 等待连接')
        status.addWidget(self.radar_status, 1)
        status.addWidget(self.comm_status, 1)
        status.addWidget(self.camera_status, 1)
        self.auto_connect = QCheckBox('打开软件自动连接')
        self.auto_connect.setChecked(self.settings.value('auto_connect', True, type=bool))
        self.auto_connect.toggled.connect(lambda checked: self.settings.setValue('auto_connect', checked))
        status.addWidget(self.auto_connect)
        layout.addLayout(status)

        self.tabs = QTabWidget()
        self.radar_tab = PreviewTab('雷达')
        self.comm_tab = PreviewTab('通讯')
        self.camera_tab = PreviewTab('相机', html_file='camera_console.html')
        self.tabs.addTab(self.camera_tab, '相机与物料对齐')
        self.tabs.addTab(self.radar_tab, '雷达与地图')
        self.tabs.addTab(self.comm_tab, 'STM32通讯')
        layout.addWidget(self.tabs, 1)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.document().setMaximumBlockCount(250)
        self.log.setMaximumHeight(135)
        self.log.hide()
        layout.addWidget(self.log)
        footer = QHBoxLayout()
        show_log = QCheckBox('连接日志')
        show_log.toggled.connect(self.log.setVisible)
        save_log = QPushButton('保存连接日志')
        save_log.clicked.connect(self.save_log)
        footer.addWidget(show_log)
        footer.addWidget(save_log)
        footer.addStretch()
        hint = QLabel('连接后启用实体按键待机。关闭软件后，树莓派服务继续运行。')
        hint.setObjectName('hint')
        footer.addWidget(hint)
        layout.addLayout(footer)
        self.setStyleSheet('''
            QMainWindow, QWidget { background: #111619; color: #edf2f4; }
            QWidget { font-size: 14px; }
            QLabel#title { font-size: 23px; font-weight: 600; }
            QLabel#hint { color: #b7c4cc; font-size: 12px; }
            QLineEdit, QPlainTextEdit { background: #1a2227; border: 1px solid #61737e;
                border-radius: 5px; padding: 8px; selection-background-color: #436c60; }
            QPushButton { background: #25353e; border: 1px solid #71858f;
                border-radius: 5px; padding: 8px 12px; }
            QPushButton:hover { background: #3c515c; }
            QPushButton:disabled { color: #7c8c96; border-color: #47565e; }
            QLineEdit:focus, QPushButton:focus { border-color: #60e8bc; }
            QTabWidget::pane { border: 1px solid #47565e; }
            QTabBar::tab { padding: 11px 22px; background: #1a2227; color: #b7c4cc; }
            QTabBar::tab:selected { background: #25353e; color: #60e8bc; }
            QCheckBox { spacing: 6px; }
        ''')

    def record(self, message):
        self.log.appendPlainText(time.strftime('%H:%M:%S')+'  '+message)

    def connect_requested(self):
        address = self.address.text().strip()
        try:
            ssh_arguments([1, 2], address)
        except ValueError as exc:
            self.connection_label.setText(str(exc))
            return
        self.settings.setValue('address', address)
        self.connection_address = address
        self.wanted = True
        self.retry_delay = 5
        self.stop_process()
        self.start_connection()

    def start_connection(self):
        if not self.wanted:
            return
        self.retry_timer.stop()
        self.ports = reserve_ports()
        self.stdout_buffer.clear()
        self.stderr_tail = ''
        self.last_services = None
        self.connected = False
        self.last_heartbeat = time.monotonic()
        self.radar_tab.reset('正在连接树莓派，随后自动启动雷达后端。')
        self.comm_tab.reset('正在连接树莓派，随后自动启动通讯监看。')
        self.camera_tab.reset('正在连接树莓派相机服务。')
        self.radar_status.setText('雷达 · 等待连接')
        self.comm_status.setText('通讯 · 等待连接')
        self.camera_status.setText('相机 · 等待连接')
        self.connection_label.setText('正在建立SSH连接…')
        self.connect_button.setText('重新连接')
        self.disconnect_button.setEnabled(True)
        process = QProcess(self)
        self.process = process
        process.started.connect(lambda: self.send_bootstrap(process))
        process.readyReadStandardOutput.connect(lambda: self.read_output(process))
        process.readyReadStandardError.connect(lambda: self.read_errors(process))
        process.finished.connect(lambda *_: self.finished(process))
        process.errorOccurred.connect(lambda error: self.process_error(process, error))
        process.setProgram('/usr/bin/ssh')
        process.setArguments(ssh_arguments(self.ports, self.connection_address))
        self.record('连接树莓派 '+(self.connection_address or 'pi-vision'))
        process.start()

    def send_bootstrap(self, process):
        if process is self.process:
            try:
                process.write(remote_source())
                process.closeWriteChannel()
            except OSError as exc:
                self.stderr_tail = '启动文件读取失败 '+str(exc)
                process.kill()

    def read_output(self, process):
        if process is not self.process:
            return
        self.stdout_buffer.extend(bytes(process.readAllStandardOutput()))
        while b'\n' in self.stdout_buffer:
            line, _, rest = self.stdout_buffer.partition(b'\n')
            self.stdout_buffer = bytearray(rest)
            try:
                item = json.loads(line)
                self.handle_message(item)
            except (ValueError, TypeError, KeyError):
                self.record(line.decode('utf-8', errors='replace'))

    def handle_message(self, item):
        self.last_heartbeat = time.monotonic()
        if item['event'] == 'connected':
            self.connected = True
            self.retry_delay = 5
            self.connection_label.setText('已连接树莓派，正在检查后端…')
            self.record(item['message'])
        elif item['event'] == 'error':
            self.stderr_tail = item['message']
            self.connection_label.setText(item['message'])
            self.record(item['message'])
        elif item['event'] == 'services':
            self.connection_label.setText('已连接树莓派 · 后端自动检查中')
            for key, tab, label, port, title in (
                ('radar', self.radar_tab, self.radar_status, self.ports[0], '雷达'),
                ('communication', self.comm_tab, self.comm_status, self.ports[1], '通讯'),
                ('camera', self.camera_tab, self.camera_status, self.ports[2], '相机'),
            ):
                info = item.get(key, dict(http_ready=False, valid=False, message='等待后端'))
                label.setText(title+' · '+info['message'])
                label.setStyleSheet('color: '+('#60e8bc' if info['valid'] else '#ffbe70'))
                if info['http_ready']:
                    tab.open_backend(port)
                elif tab.loaded:
                    tab.reset(info['message'])
                else:
                    tab.show_message(info['message'])
            if item != self.last_services:
                self.record(item['radar']['message']+' / '+item['communication']['message'])
                self.last_services = item

    def read_errors(self, process):
        if process is self.process:
            text = bytes(process.readAllStandardError()).decode('utf-8', errors='replace').strip()
            if text:
                self.stderr_tail = (self.stderr_tail+'\n'+text)[-2000:]
                self.record(text)

    def process_error(self, process, error):
        if process is self.process and error == QProcess.ProcessError.FailedToStart:
            self.stderr_tail = '无法启动系统SSH程序'
            self.finished(process)

    def finished(self, process):
        if process is not self.process:
            return
        self.read_errors(process)
        self.process = None
        process.deleteLater()
        self.connected = False
        reason = self.stderr_tail.strip() or '连接中断或树莓派无响应'
        self.radar_tab.reset(reason)
        self.comm_tab.reset(reason)
        self.camera_tab.reset(reason)
        self.radar_status.setText('雷达 · 连接已中断')
        self.comm_status.setText('通讯 · 连接已中断')
        self.camera_status.setText('相机 · 连接已中断')
        if self.wanted:
            delay = self.retry_delay
            self.connection_label.setText(f'{reason}\n{delay}秒后自动重新连接，可填写新IP并点“重新连接”。')
            self.record(f'{delay}秒后自动重连')
            self.retry_timer.start(delay*1000)
            self.retry_delay = min(delay*2, 30)

    def check_connection(self):
        if self.process is None:
            return
        age = time.monotonic()-self.last_heartbeat
        timeout = 30 if self.connected else 60
        if age > timeout:
            self.stderr_tail = '树莓派连接超时，请检查两台设备的网络和树莓派供电'
            self.process.kill()
        elif self.connected and age > 12:
            self.connection_label.setText(f'网络响应延迟，最近一次树莓派状态在{int(age)}秒前')

    def stop_process(self):
        self.retry_timer.stop()
        process = self.process
        self.process = None
        if process is not None:
            process.terminate()
            if not process.waitForFinished(500):
                process.kill()
                process.waitForFinished(500)
            process.deleteLater()

    def disconnect_requested(self):
        self.wanted = False
        self.stop_process()
        self.connected = False
        self.connection_label.setText('已断开桌面连接')
        self.radar_status.setText('雷达 · 已断开')
        self.comm_status.setText('通讯 · 已断开')
        self.camera_status.setText('相机 · 已断开')
        self.radar_tab.reset('点击“连接树莓派”打开雷达界面')
        self.comm_tab.reset('点击“连接树莓派”打开通讯界面')
        self.camera_tab.reset('点击“连接树莓派”打开相机界面')
        self.disconnect_button.setEnabled(False)
        self.connect_button.setText('连接树莓派')

    def save_log(self):
        filename, _ = QFileDialog.getSaveFileName(self, '保存连接日志', '小车连接日志.txt', '文本文件 (*.txt)')
        if filename:
            try:
                Path(filename).write_text(self.log.toPlainText(), encoding='utf-8')
            except OSError as exc:
                self.connection_label.setText('日志保存失败 '+str(exc))

    def closeEvent(self, event):
        self.disconnect_requested()
        event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-auto-connect', action='store_true')
    args = parser.parse_args()
    app = QApplication(sys.argv[:1])
    app.setApplicationName('小车调试台')
    app.setOrganizationName('CompetitionRobot')
    # A second double-click raises the existing window and shares its tunnels.
    name = 'competition-robot-console'
    socket_client = QLocalSocket()
    socket_client.connectToServer(name)
    if socket_client.waitForConnected(200):
        socket_client.write(b'raise')
        socket_client.waitForBytesWritten(200)
        return 0
    server = QLocalServer()
    if not server.listen(name):
        # A simultaneous launch may have acquired the name since the first probe.
        socket_client.connectToServer(name)
        if socket_client.waitForConnected(300):
            socket_client.write(b'raise')
            socket_client.waitForBytesWritten(200)
            return 0
        QLocalServer.removeServer(name)
        if not server.listen(name):
            raise RuntimeError('无法创建桌面应用连接，请关闭其他小车调试台后重试')
    window = RobotConsole(auto_connect=not args.no_auto_connect)
    app.aboutToQuit.connect(window.disconnect_requested)

    def raise_window():
        connection = server.nextPendingConnection()
        if connection:
            connection.disconnectFromServer()
            connection.deleteLater()
        window.showNormal()
        window.raise_()
        window.activateWindow()

    server.newConnection.connect(raise_window)
    # Native save dialog keeps the existing web UI's log/image downloads usable.
    def download(item):
        filename, _ = QFileDialog.getSaveFileName(window, '保存文件', item.downloadFileName())
        if filename:
            path = Path(filename)
            item.setDownloadDirectory(str(path.parent))
            item.setDownloadFileName(path.name)
            item.accept()
        else:
            item.cancel()

    QWebEngineProfile.defaultProfile().downloadRequested.connect(download)
    window.show()
    return app.exec()


if __name__ == '__main__':
    raise SystemExit(main())
