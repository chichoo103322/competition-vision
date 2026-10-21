"""基于电控 v3.1 协议的串口链路。"""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable, Optional

from .protocol import (
    FrameSplitter,
    cmd_ack,
    cmd_found,
    cmd_heartbeat,
    cmd_not_found,
    cmd_obstacle,
    cmd_place_check,
    cmd_ring_center,
    cmd_taskcode,
)

LOGGER = logging.getLogger(__name__)


class SerialLink:
    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 0.05,
        on_frame: Optional[Callable[[int, bytes], None]] = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.on_frame = on_frame

        self._serial = None
        self._thread = None
        self._running = False
        self._write_lock = threading.Lock()
        self._splitter = FrameSplitter()

    @property
    def is_open(self) -> bool:
        return self._serial is not None and bool(self._serial.is_open)

    def open(self) -> None:
        if self.is_open:
            raise RuntimeError("串口已经打开")
        import serial

        self._serial = serial.Serial(
            self.port,
            self.baudrate,
            timeout=self.timeout,
        )
        self._running = True
        self._thread = threading.Thread(
            target=self._reader_loop,
            name="pi-vision-serial-reader",
            daemon=True,
        )
        self._thread.start()
        LOGGER.info("串口已打开: %s @ %d", self.port, self.baudrate)

    def close(self) -> None:
        self._running = False
        thread = self._thread
        self._thread = None
        if thread is not None:
            thread.join(timeout=0.5)
        serial_port = self._serial
        self._serial = None
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception as exc:  # 尽力关闭
                LOGGER.debug("关闭串口失败: %s", exc)

    def __enter__(self) -> "SerialLink":
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    def send_frame(self, frame: bytes) -> bool:
        if not self.is_open:
            raise RuntimeError("串口未打开")
        with self._write_lock:
            written = self._serial.write(frame)
            self._serial.flush()
        return written == len(frame)

    def send_taskcode(self, task) -> bool:
        return self.send_frame(cmd_taskcode(task))

    def send_found(
        self,
        color: int,
        center_x: int,
        center_y: int,
        offset_deg: float,
        dist_cm: int,
    ) -> bool:
        return self.send_frame(
            cmd_found(color, center_x, center_y, offset_deg, dist_cm)
        )

    def send_ring_center(self, ring_no: int, center_x: int, center_y: int) -> bool:
        return self.send_frame(cmd_ring_center(ring_no, center_x, center_y))

    def send_not_found(self) -> bool:
        return self.send_frame(cmd_not_found())

    def send_place_check(self, zone: int, pos: int, ok: int) -> bool:
        return self.send_frame(cmd_place_check(zone, pos, ok))

    def send_obstacle(self, offset_deg: float, dist_cm: int) -> bool:
        return self.send_frame(cmd_obstacle(offset_deg, dist_cm))

    def send_heartbeat(self) -> bool:
        return self.send_frame(cmd_heartbeat())

    def send_ack(self, orig_cmd: int, status: int = 0) -> bool:
        return self.send_frame(cmd_ack(orig_cmd, status))

    def _reader_loop(self) -> None:
        while self._running:
            try:
                chunk = self._serial.read(64)
                if chunk:
                    for cmd, params in self._splitter.feed(chunk):
                        if self.on_frame is not None:
                            self.on_frame(cmd, params)
            except OSError as exc:
                # 串口读取错误（如 UART 瞬断/设备抖动）不应杀死读线程：
                # 记录后重连重试，串口恢复后自动继续收发，避免主控收不到上报。
                LOGGER.error("串口读取失败，1s 后重连: %s", exc)
                time.sleep(1.0)
                try:
                    if self._serial is not None:
                        self._serial.close()
                except Exception:
                    pass
                try:
                    import serial as _serial_mod
                    self._serial = _serial_mod.Serial(
                        self.port, self.baudrate, timeout=self.timeout
                    )
                except Exception as exc2:
                    LOGGER.error("串口重连失败: %s", exc2)
                    time.sleep(1.0)
                continue
            except Exception as exc:
                LOGGER.exception("串口读取异常: %s", exc)
                time.sleep(0.01)
            time.sleep(0.001)
