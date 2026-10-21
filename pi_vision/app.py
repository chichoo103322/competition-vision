"""应用层：按电控 v3.1 协议处理扫描控制并上报视觉结果。"""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable, Optional

from .config import VisionConfig
from .pipeline import VisionPipeline
from .rotation_gate import RotationGate
from .material_tracking import MaterialStability
from .protocol import (
    CMD_RESCAN,
    CMD_SCAN,
    CMD_SET_COLOR,
    CMD_STOP_SCAN,
    SCAN_MODE_MATERIAL,
    SCAN_MODE_QR,
    SCAN_MODE_VERIFY,
    ST_ERR,
    ST_OK,
)
from .serial_link import SerialLink

LOGGER = logging.getLogger(__name__)


class VisionApp:
    def __init__(
        self,
        config: VisionConfig,
        serial_link: Optional[SerialLink] = None,
        pipeline: Optional[VisionPipeline] = None,
        on_frame: Optional[Callable[[int, bytes], None]] = None,
    ) -> None:
        self.config = config
        self.serial_link = serial_link
        self.pipeline = pipeline
        self._user_on_frame = on_frame

        self._scan_mode: Optional[int] = None
        self._target_color: Optional[int] = None
        self._scan_thread = None
        self._scan_stop = threading.Event()
        self._qr_stop = threading.Event()
        self._qr_report_lock = threading.Lock()
        self._heartbeat_thread = None
        self._running = False
        self._rotation_gate = None
        if config.detection.rotation_gate:
            self._rotation_gate = RotationGate(
                move_px=config.detection.rotation_move_px,
                settle_frames=config.detection.rotation_settle_frames,
                window_frames=config.detection.rotation_window_frames,
                stable_px=config.detection.rotation_stable_px,
            )
        # 物料遮挡丢失容忍：抗机械臂半闭合等遮挡导致的短暂漏检（中心几乎不抖动，不做平滑）
        self._mat_stab = MaterialStability(
            lost_tol_s=config.detection.material_lost_tol
        )

    def start(self) -> None:
        self._running = True
        if self.serial_link is not None:
            self.serial_link.on_frame = self._on_frame
            self.serial_link.open()
        if self.pipeline is not None and self.pipeline.camera is not None:
            self.pipeline.camera.open()
        if (
            self.pipeline is not None
            and self.pipeline.qr_camera is not None
            and self.pipeline.qr_camera is not self.pipeline.camera
        ):
            try:
                self.pipeline.qr_camera.open()
            except Exception as exc:
                LOGGER.warning("扫码相机打开失败，二维码功能不可用: %s", exc)
                self.pipeline.qr_camera = None
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True
        )
        self._heartbeat_thread.start()

    def stop(self) -> None:
        self._running = False
        self._stop_scan_thread()
        if self.serial_link is not None:
            self.serial_link.close()
        if self.pipeline is not None and self.pipeline.camera is not None:
            self.pipeline.camera.release()
        if (
            self.pipeline is not None
            and self.pipeline.qr_camera is not None
            and self.pipeline.qr_camera is not self.pipeline.camera
        ):
            self.pipeline.qr_camera.release()

    def run_forever(self) -> None:
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass

    def _on_frame(self, cmd: int, params: bytes) -> None:
        if self._user_on_frame is not None:
            self._user_on_frame(cmd, params)

        if cmd == CMD_SCAN:
            self._reply_ack(cmd)
            self._handle_scan(params)
        elif cmd == CMD_STOP_SCAN:
            self._reply_ack(cmd)
            self._handle_stop_scan()
        elif cmd == CMD_SET_COLOR:
            self._reply_ack(cmd)
            self._handle_set_color(params)
        elif cmd == CMD_RESCAN:
            self._reply_ack(cmd)
            self._handle_rescan()

    def _reply_ack(self, orig_cmd: int, status: int = ST_OK) -> None:
        if self.serial_link is not None:
            self.serial_link.send_ack(orig_cmd, status)

    def _handle_scan(self, params: bytes) -> None:
        mode = params[0] if params else SCAN_MODE_QR
        self._stop_scan_thread()
        self._scan_mode = mode
        print(f"[扫描开始] 模式={mode} ({self._mode_name(mode)})")

        if mode == SCAN_MODE_QR:
            self._qr_stop = threading.Event()
            self._scan_thread = threading.Thread(
                target=self._qr_loop, args=(self._qr_stop,), daemon=True)
            self._scan_thread.start()
        elif mode == SCAN_MODE_MATERIAL:
            if self._rotation_gate is not None:
                self._rotation_gate.reset()
            self._mat_stab.reset()
            self._scan_stop.clear()
            self._scan_thread = threading.Thread(
                target=self._material_loop, daemon=True
            )
            self._scan_thread.start()
        elif mode == SCAN_MODE_VERIFY:
            self._scan_stop.clear()
            self._scan_thread = threading.Thread(
                target=self._ring_loop, daemon=True
            )
            self._scan_thread.start()

    def _handle_stop_scan(self) -> None:
        print(f"[扫描停止] 模式={self._scan_mode} ({self._mode_name(self._scan_mode)})")
        self._stop_scan_thread()

    def _handle_set_color(self, params: bytes) -> None:
        if params:
            self._target_color = params[0]
            LOGGER.info("目标颜色设置为 %d", self._target_color)

    def _handle_rescan(self) -> None:
        if self._scan_mode is None:
            self._handle_scan(bytes([SCAN_MODE_QR]))
            return
        self._handle_scan(bytes([self._scan_mode]))

    def _stop_scan_thread(self) -> None:
        with self._qr_report_lock:
            self._qr_stop.set()
        self._scan_stop.set()
        if self._scan_thread is not None:
            self._scan_thread.join(timeout=1.0)
            self._scan_thread = None

    def _run_qr_once(self, stop_event=None) -> None:
        if self.pipeline is None or self.serial_link is None:
            return
        stop_event = self._qr_stop if stop_event is None else stop_event
        code = None
        try:
            code = self.pipeline.detect_task_code_stable(
                max_frames=60, min_confirmed=3, timeout=10.0, stop_event=stop_event)
        except Exception as exc:
            LOGGER.exception("读码失败: %s", exc)
        with self._qr_report_lock:
            if stop_event.is_set():
                return
            if code is None:
                self.serial_link.send_not_found()
                print("[二维码] 未确认任务码", flush=True)
            else:
                self.serial_link.send_taskcode(str(code))
                print(f"[二维码] 上报已确认任务: {code}", flush=True)

    def _qr_loop(self, stop_event=None) -> None:
        """扫码会话锁定后仅上报一次，需新的扫码/重扫命令开始下一轮。"""
        self._run_qr_once(stop_event)

    def _mode_name(self, mode) -> str:
        names = {
            SCAN_MODE_QR: "读二维码",
            SCAN_MODE_MATERIAL: "找物料",
            SCAN_MODE_VERIFY: "放置验证",
        }
        return names.get(mode, "未知")

    def _run_verify_once(self) -> None:
        if self.serial_link is None:
            return
        # TODO: 放置验证的视觉判定，先按正确上报占位
        self.serial_link.send_place_check(zone=1, pos=1, ok=1)

    def _ring_loop(self) -> None:
        if self.pipeline is None or self.serial_link is None:
            return
        while not self._scan_stop.is_set():
            try:
                frame, _ = self.pipeline.read_frame()
                rings = self.pipeline.ring_detector.detect(frame)
                if rings:
                    target = min(
                        rings,
                        key=lambda item: (
                            item.center_x - frame.shape[1] / 2.0
                        ) ** 2
                        + (item.center_y - frame.shape[0] / 2.0) ** 2,
                    )
                    self.serial_link.send_ring_center(
                        target.ring_no, target.center_x, target.center_y
                    )
                else:
                    self.serial_link.send_not_found()
            except Exception as exc:
                LOGGER.exception("圆环上报失败: %s", exc)
                time.sleep(0.05)
            time.sleep(0.1)

    def _material_loop(self) -> None:
        if self.pipeline is None or self.serial_link is None:
            return
        while not self._scan_stop.is_set():
            try:
                frame, _ = self.pipeline.read_frame()
                materials = self.pipeline.material_detector.detect(frame)
                target = None
                if self._target_color is not None:
                    for item in materials:
                        if item.color_id == self._target_color:
                            target = item
                            break
                now = time.time()
                if target is None:
                    # 物料被遮挡短暂漏检：在丢失容忍期内沿用最近稳定中心继续上报，
                    # 避免遮挡瞬间 send_not_found 中断导致主控误判/延迟。
                    report = self._mat_stab.report_center(now)
                    if report is None:
                        self.serial_link.send_not_found()
                        continue
                    cx, cy = report
                    angle = self._offset_angle(
                        cx,
                        frame.shape[1],
                        self.config.camera.fov_h_deg,
                    )
                    self.serial_link.send_found(
                        self._target_color,
                        int(round(cx)),
                        int(round(cy)),
                        angle,
                        self.config.fixed_distance_cm,
                    )
                    # 不 mark_stable：丢失容忍不续期，超过 lost_tol 后 report_center 返 None
                    continue
                else:
                    cx, cy = self._mat_stab.observe(
                        target.center_x, target.center_y, now
                    )
                    allow = True
                    if self._rotation_gate is not None:
                        allow = self._rotation_gate.update(cx, cy)
                if allow:
                    angle = self._offset_angle(
                        cx,
                        frame.shape[1],
                        self.config.camera.fov_h_deg,
                    )
                    self.serial_link.send_found(
                        target.color_id,
                        int(round(cx)),
                        int(round(cy)),
                        angle,
                        self.config.fixed_distance_cm,
                    )
                    self._mat_stab.mark_stable(now)
                else:
                    self.serial_link.send_not_found()
            except Exception as exc:
                LOGGER.exception("物料上报失败: %s", exc)
                time.sleep(0.05)
            time.sleep(0.05)

    def _offset_angle(self, center_x: int, width: int, fov_h_deg: float) -> float:
        if width <= 0:
            return 0.0
        normalized = (center_x - width / 2.0) / (width / 2.0)
        return normalized * (fov_h_deg / 2.0)

    def _heartbeat_loop(self) -> None:
        while self._running:
            if self.serial_link is not None:
                try:
                    self.serial_link.send_heartbeat()
                except Exception as exc:
                    LOGGER.warning("心跳发送失败: %s", exc)
            time.sleep(1.0)
