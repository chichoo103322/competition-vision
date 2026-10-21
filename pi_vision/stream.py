"""树莓派摄像头 MJPEG 推流服务，可在电脑浏览器实时查看。"""

from __future__ import annotations

import argparse
import json
import logging
import threading
import time
from urllib.parse import urlsplit
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

from .camera import Camera
from .config import CameraConfig, VisionConfig, load_config
from .pipeline import VisionPipeline
from .preview import draw_materials, draw_qr, draw_rings
from .rotation_gate import RotationGate
from .material_tracking import MaterialStability
from .task_confirmation import TaskCodeConfirmation, parse_competition_task
from .preview_page import FRAME_CLIENT_JS
from .material_region import MaterialRegionDetector, ordered_materials
from .protocol import (
    COLOR,
    CMD_RESCAN,
    CMD_SCAN,
    CMD_SET_COLOR,
    CMD_STOP_SCAN,
    SCAN_MODE_MATERIAL,
    SCAN_MODE_QR,
    SCAN_MODE_VERIFY,
    ST_OK,
)
from .serial_link import SerialLink

LOGGER = logging.getLogger(__name__)

_streamer = None


def draw_qr_status(canvas, qr_result) -> None:
    """在画面左上角常驻显示二维码识别状态。"""
    if qr_result is None:
        text = "二维码: 未识别"
        color = (0, 0, 255)
    else:
        text = f"二维码: {qr_result[0]}"
        color = (0, 255, 0)
    font = cv2.FONT_HERSHEY_SIMPLEX
    (text_w, text_h), baseline = cv2.getTextSize(text, font, 0.7, 2)
    cv2.rectangle(
        canvas,
        (8, 8),
        (8 + text_w + 12, 8 + text_h + baseline + 8),
        (0, 0, 0),
        -1,
    )
    cv2.putText(canvas, text, (14, 8 + text_h + 4), font, 0.7, color, 2)


class FrameStreamer:
    """后台抓帧、画检测结果，并生成 JPEG 帧供 HTTP 客户端读取。"""

    def __init__(
        self,
        camera: Camera,
        pipeline: VisionPipeline,
        qr_camera: Optional[Camera] = None,
        rotation_gate: Optional[RotationGate] = None,
        enable_qr: bool = True,
        enable_materials: bool = True,
        enable_rings: bool = True,
        quality: int = 80,
        detect_interval: float = 0.3,
        draw_overlay: bool = True,
        preview_width: int = 640,
        preview_fps: float = 5.0,
    ) -> None:
        self._camera = camera
        self._qr_camera = qr_camera if qr_camera is not None else (pipeline.qr_camera or camera)
        self._pipeline = pipeline
        self.enable_qr = enable_qr
        self.enable_materials = enable_materials
        self.enable_rings = enable_rings
        self.quality = quality
        self.detect_interval = max(0.05, detect_interval)
        self.draw_overlay = draw_overlay
        self._latest = None
        self._latest_jpeg = None
        self._jpeg_sequence = 0
        self._jpeg_acquired_at = 0.0
        self._next_preview_at = 0.0
        if preview_width < 1 or not 0 < preview_fps <= 30:
            raise ValueError("预览宽度须为正数，帧率须在 0—30 之间")
        self.preview_width = preview_width
        self.preview_interval = 1.0 / preview_fps
        self._latest_raw = None
        self._latest_qr = None
        self._qr_confirmation = TaskCodeConfirmation()
        self._qr_candidate = None
        self._qr_generation = 0
        self._qr_session_started_at = time.monotonic()
        self._last_qr_sequence = -1
        self._raw_sequence = 0
        self._raw_acquired_at = 0.0
        self._independent_qr_sequence = 0
        self._latest_materials = []
        self._material_sequence = 0
        self._material_source_sequence = 0
        self._material_acquired_at = 0.0
        self._material_completed_at = 0.0
        self._latest_rings = []
        self._last_printed_qr = None
        self._rotation_gate = rotation_gate
        self._target_color = None
        self._rotation_allowed = True
        self._rotation_state = (
            "已关闭" if rotation_gate is None else "等待首次换位"
        )
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._detect_thread = None
        # 物料 + 圆环并行检测，避免串行叠加拖慢节拍
        self._executor = ThreadPoolExecutor(max_workers=2)

    def start(self) -> None:
        self._camera.open()
        if self._qr_camera is not self._camera:
            try:
                self._qr_camera.open()
            except Exception as exc:
                LOGGER.warning("扫码相机打开失败，二维码将使用主相机画面: %s", exc)
                self._qr_camera = self._camera
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._detect_thread = threading.Thread(target=self._detect_loop, daemon=True)
        self._thread.start()
        self._detect_thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._detect_thread is not None:
            self._detect_thread.join(timeout=1.0)
        self._executor.shutdown(wait=False)
        self._camera.release()
        if self._qr_camera is not self._camera:
            self._qr_camera.release()

    def get_jpeg(self):
        """返回最新一帧的 JPEG 字节，没有帧时返回 None。"""
        with self._lock:
            return self._latest_jpeg

    def get_preview_frame(self):
        """原子返回 JPEG 与采集信息，HTTP 客户端用序号跳过重复帧。"""
        with self._lock:
            return self._latest_jpeg, self._jpeg_sequence, self._jpeg_acquired_at

    def get_qr(self):
        """仅返回本轮已锁定结果；候选不会交给串口。"""
        with self._lock:
            return self._latest_qr

    def get_qr_generation(self):
        with self._lock:
            return self._qr_generation

    def _accept_qr_observation(self, result, sequence, acquired_at, generation):
        """丢弃重扫前的在途解码和重复缓存帧，锁定后保留本轮任务。"""
        with self._lock:
            if (generation != self._qr_generation or not self.enable_qr
                    or acquired_at < self._qr_session_started_at
                    or sequence <= self._last_qr_sequence):
                return
            self._last_qr_sequence = sequence
            code = result[0] if result is not None else None
            if code is not None:
                try:
                    code = parse_competition_task(str(code))
                except ValueError:
                    code = None
            self._qr_candidate = code
            locked = self._qr_confirmation.update(code, acquired_at)
            if locked is not None and self._latest_qr is None:
                self._latest_qr = (locked, np.asarray(result[1]).copy())
                print(f"[二维码] 已确认: {locked}", flush=True)

    def get_materials(self):
        with self._lock:
            return list(self._latest_materials)

    def get_rings(self):
        with self._lock:
            return list(self._latest_rings)

    def set_target_color(self, color) -> None:
        with self._lock:
            self._target_color = color

    def set_scan_detection(self, mode: Optional[int]) -> None:
        """按电控扫描模式控制检测算法启停（算法开关）。

        电控到相应环节才发 0x21 开启对应算法，0x22 / 未收到指令则全关：
        - None/未知：全部算法不启动（只推流画面）
        - SCAN_MODE_QR：只跑二维码
        - SCAN_MODE_MATERIAL：只跑物料+旋转门控（关闭圆环，提速）
        - SCAN_MODE_VERIFY：只跑圆环
        每个夹取间隙电控发 0x22 即可全关，下一次 0x21 再开启。
        """
        with self._lock:
            self._qr_generation += 1
            self._qr_session_started_at = time.monotonic()
            self._qr_confirmation.reset()
            self._last_qr_sequence = -1
            self._qr_candidate = None
            self._latest_qr = None
            if mode == SCAN_MODE_QR:
                self.enable_qr, self.enable_materials, self.enable_rings = True, False, False
            elif mode == SCAN_MODE_MATERIAL:
                self.enable_qr, self.enable_materials, self.enable_rings = False, True, False
            elif mode == SCAN_MODE_VERIFY:
                self.enable_qr, self.enable_materials, self.enable_rings = False, False, True
            else:
                self.enable_qr, self.enable_materials, self.enable_rings = False, False, False

    def reset_rotation_gate(self) -> None:
        with self._lock:
            if self._rotation_gate is not None:
                self._rotation_gate.reset()
                self._rotation_state = "等待首次换位"

    def get_rotation_state(self) -> str:
        with self._lock:
            return self._rotation_state

    def get_rotation_allowed(self) -> bool:
        with self._lock:
            if self._rotation_gate is None:
                return True
            return self._rotation_allowed

    def get_frame_width(self) -> int:
        with self._lock:
            return 0 if self._latest is None else self._latest.shape[1]

    def get_frame_height(self) -> int:
        with self._lock:
            return 0 if self._latest is None else self._latest.shape[0]

    def get_state(self) -> dict:
        with self._lock:
            now = time.monotonic()
            qr = None
            if self._latest_qr is not None:
                code, points = self._latest_qr
                qr = {
                    "text": str(code),
                    "points": [
                        [float(x), float(y)]
                        for x, y in np.asarray(points, dtype=np.float32).reshape(-1, 2)
                    ],
                }
            materials = [
                {
                    "color_id": item.color_id,
                    "color_name": COLOR.get(item.color_id, str(item.color_id)),
                    "center_x": item.center_x,
                    "center_y": item.center_y,
                    "confidence": item.confidence,
                }
                for item in self._latest_materials
            ]
            rings = [
                {
                    "ring_no": item.ring_no,
                    "center_x": item.center_x,
                    "center_y": item.center_y,
                    "radius": item.radius,
                }
                for item in self._latest_rings
            ]
            return {
                "qr": qr,
                "qr_candidate": str(self._qr_candidate) if self._qr_candidate else None,
                "qr_matching_frames": self._qr_confirmation.count,
                "qr_confirmed": self._latest_qr is not None,
                "capture": {
                    "sequence": self._raw_sequence,
                    "age_ms": round((now - self._raw_acquired_at) * 1000, 1)
                    if self._raw_sequence else None,
                },
                "preview": {
                    "sequence": self._jpeg_sequence,
                    "age_ms": round((now - self._jpeg_acquired_at) * 1000, 1)
                    if self._jpeg_sequence else None,
                    "jpeg_bytes": len(self._latest_jpeg) if self._latest_jpeg else 0,
                    "source_width": self._latest_raw.shape[1] if self._latest_raw is not None else 0,
                    "source_height": self._latest_raw.shape[0] if self._latest_raw is not None else 0,
                },
                "materials": materials,
                "material_observation": {
                    "sequence": self._material_sequence,
                    "source_sequence": self._material_source_sequence,
                    "age_ms": round((now - self._material_acquired_at) * 1000, 1)
                    if self._material_sequence else None,
                    "processed_ms": round((self._material_completed_at - self._material_acquired_at) * 1000, 1)
                    if self._material_sequence else None,
                },
                "material_roi": list(self._pipeline.material_detector.roi)
                if isinstance(getattr(self._pipeline, "material_detector", None), MaterialRegionDetector)
                else None,
                "material_backend": type(
                    self._pipeline.material_detector.detector
                    if isinstance(getattr(self._pipeline, "material_detector", None), MaterialRegionDetector)
                    else getattr(self._pipeline, "material_detector", None)
                ).__name__,
                "rings": rings,
                "rotation": self._rotation_state,
                # 识别开关状态（算法开关：电控 0x21/0x22 按环节开启/关闭三种识别）
                "detection": {
                    "qr": bool(self.enable_qr),
                    "materials": bool(self.enable_materials),
                    "rings": bool(self.enable_rings),
                },
            }

    def _capture_loop(self) -> None:
        while self._running:
            try:
                ok, frame, _ = self._camera.read()
                acquired_at = time.monotonic()
                if not ok:
                    time.sleep(0.05)
                    continue
                with self._lock:
                    self._latest_raw = frame.copy()
                    self._raw_sequence += 1
                    self._raw_acquired_at = acquired_at
                    qr_result = self._latest_qr
                    materials = self._latest_materials
                    rings = self._latest_rings

                # 采集/识别持续更新全分辨率原图；独立限制预览编码频率。
                if acquired_at < self._next_preview_at:
                    continue
                self._next_preview_at = acquired_at + self.preview_interval
                canvas = frame.copy()
                if self.draw_overlay:
                    if self.enable_qr:
                        draw_qr(canvas, qr_result)
                        draw_qr_status(canvas, qr_result)
                    if self.enable_materials:
                        draw_materials(canvas, materials)
                    if self.enable_rings:
                        draw_rings(canvas, rings)

                # 编码耗时不占用共享锁，避免 /state、识别和取帧互相等待。
                preview = canvas
                if canvas.shape[1] > self.preview_width:
                    height = max(1, round(canvas.shape[0] * self.preview_width / canvas.shape[1]))
                    preview = cv2.resize(canvas, (self.preview_width, height), interpolation=cv2.INTER_AREA)
                ok, buf = cv2.imencode(
                    ".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, self.quality]
                )
                if ok:
                    jpeg = buf.tobytes()
                    with self._lock:
                        self._latest = canvas
                        self._latest_jpeg = jpeg
                        self._jpeg_sequence += 1
                        self._jpeg_acquired_at = acquired_at
            except Exception as exc:
                LOGGER.exception("抓帧失败: %s", exc)
                time.sleep(0.05)
            time.sleep(0.005)

    def _detect_loop(self) -> None:
        next_detect = time.time()
        while self._running:
            with self._lock:
                frame = self._latest_raw
                sequence = self._raw_sequence
                acquired_at = self._raw_acquired_at
                generation = self._qr_generation
                enable_qr = self.enable_qr
                material_source_sequence = sequence
                material_acquired_at = acquired_at
            if frame is None:
                time.sleep(0.01)
                continue
            qr_result = None
            materials = []
            rings = []
            mat_f = ring_f = None
            try:
                if enable_qr:
                    qr_frame = frame
                    if self._qr_camera is not self._camera:
                        ok, qr_frame, _ = self._qr_camera.read()
                        acquired_at = time.monotonic()
                        self._independent_qr_sequence += 1
                        sequence = self._independent_qr_sequence
                        if not ok:
                            qr_frame = None
                    if qr_frame is not None:
                        qr_result = self._pipeline.qr_detector.detect_with_points(qr_frame)
                # 物料 + 圆环并行提交，避免串行叠加拖慢节拍
                if self.enable_materials:
                    mat_f = self._executor.submit(
                        self._pipeline.material_detector.detect, frame
                    )
                if self.enable_rings:
                    ring_f = self._executor.submit(
                        self._pipeline.ring_detector.detect, frame
                    )
                if mat_f is not None:
                    materials = mat_f.result()
                if ring_f is not None:
                    rings = ring_f.result()
            except Exception as exc:
                LOGGER.exception("检测失败: %s", exc)

            if enable_qr:
                self._accept_qr_observation(qr_result, sequence, acquired_at, generation)

            with self._lock:
                self._latest_materials = ordered_materials(materials)
                if mat_f is not None:
                    self._material_sequence += 1
                    self._material_source_sequence = material_source_sequence
                    self._material_acquired_at = material_acquired_at
                    self._material_completed_at = time.monotonic()
                self._latest_rings = rings
                if self._rotation_gate is not None:
                    target = None
                    if self._target_color is not None:
                        for item in materials:
                            if item.color_id == self._target_color:
                                target = item
                                break
                    if target is None and materials:
                        target = materials[0]
                    if target is not None:
                        self._rotation_allowed = self._rotation_gate.update(
                            target.center_x, target.center_y
                        )
                        self._rotation_state = self._rotation_gate.status
                    else:
                        self._rotation_state = "未检测到物料"
            next_detect += self.detect_interval
            wait = next_detect - time.time()
            if wait > 0:
                time.sleep(wait)
            else:
                next_detect = time.time()


class SerialController:
    """推流与串口一体模式：同一进程内处理主控扫描指令。"""

    def __init__(
        self,
        serial_port: str,
        baudrate: int,
        streamer: FrameStreamer,
        config: VisionConfig,
    ) -> None:
        self._streamer = streamer
        self._config = config
        self._link = SerialLink(
            serial_port, baudrate=baudrate, on_frame=self._on_frame
        )
        self._scan_mode = None
        self._target_color = None
        self._scanning = False
        self._stop = threading.Event()
        self._threads = []
        self._qr_thread = None
        self._qr_stop = threading.Event()
        self._qr_report_lock = threading.Lock()
        # 启动默认算法开关由 main() 统一算好传入 FrameStreamer（预览和一体模式一致）
        self._ring_thread = None
        self._ring_stop = threading.Event()
        # 物料遮挡丢失容忍：抗机械臂半闭合等遮挡导致的短暂漏检
        self._mat_stab = MaterialStability(
            lost_tol_s=config.detection.material_lost_tol
        )

    def start(self) -> None:
        self._link.open()
        heartbeat = threading.Thread(target=self._heartbeat_loop, daemon=True)
        report = threading.Thread(target=self._material_loop, daemon=True)
        heartbeat.start()
        report.start()
        self._threads = [heartbeat, report]

    def stop(self) -> None:
        self._stop.set()
        self._stop_qr_thread()
        self._stop_ring_thread()
        self._link.close()

    def _on_frame(self, cmd: int, params: bytes) -> None:
        if cmd in (CMD_SCAN, CMD_STOP_SCAN, CMD_SET_COLOR, CMD_RESCAN):
            self._link.send_ack(cmd, ST_OK)

        if cmd == CMD_SCAN:
            self._stop_qr_thread()
            self._stop_ring_thread()
            self._scan_mode = params[0] if params else SCAN_MODE_QR
            self._scanning = True
            print(f"[扫描开始] 模式={self._scan_mode} ({self._mode_name(self._scan_mode)})")
            # 算法开关：只启动本环节所需算法（物料模式关圆环提速；停止时全关）
            self._streamer.set_scan_detection(self._scan_mode)
            if self._scan_mode == SCAN_MODE_QR:
                self._start_qr_loop()
            elif self._scan_mode == SCAN_MODE_MATERIAL:
                self._streamer.reset_rotation_gate()
                self._mat_stab.reset()
                pass
            elif self._scan_mode == SCAN_MODE_VERIFY:
                self._stop_ring_thread()
                self._start_ring_loop()
        elif cmd == CMD_STOP_SCAN:
            self._stop_qr_thread()
            self._scanning = False
            print(f"[扫描停止] 模式={self._scan_mode} ({self._mode_name(self._scan_mode)})")
            self._streamer.set_scan_detection(None)
            self._stop_ring_thread()
        elif cmd == CMD_SET_COLOR:
            if params:
                self._target_color = params[0]
                self._streamer.set_target_color(self._target_color)
        elif cmd == CMD_RESCAN:
            self._stop_qr_thread()
            self._stop_ring_thread()
            if self._scan_mode == SCAN_MODE_QR:
                self._scanning = True
                self._streamer.set_scan_detection(SCAN_MODE_QR)
                self._start_qr_loop()
            elif self._scan_mode == SCAN_MODE_MATERIAL:
                self._scanning = True
            elif self._scan_mode == SCAN_MODE_VERIFY:
                self._scanning = True
                self._start_ring_loop()

    def _send_qr_result(self) -> None:
        """兼容旧调用入口；异步等待本轮确认，不使用 0.2 秒旧缓存。"""
        self._stop_qr_thread()
        self._start_qr_loop()

    def _start_qr_loop(self) -> None:
        self._qr_stop = threading.Event()
        self._qr_thread = threading.Thread(
            target=self._qr_loop,
            args=(self._qr_stop, self._streamer.get_qr_generation()), daemon=True)
        self._qr_thread.start()

    def _stop_qr_thread(self) -> None:
        with self._qr_report_lock:
            self._qr_stop.set()
        if self._qr_thread is not None:
            self._qr_thread.join(timeout=1.0)
            self._qr_thread = None

    def _qr_loop(self, stop_event, generation) -> None:
        """每条扫码/重扫命令最多上报一次；10 秒无确认才报告未找到。"""
        deadline = time.monotonic() + 10.0
        while not stop_event.is_set() and not self._stop.is_set():
            qr_result = self._streamer.get_qr()
            with self._qr_report_lock:
                if (stop_event.is_set() or self._stop.is_set() or not self._scanning
                        or self._scan_mode != SCAN_MODE_QR
                        or generation != self._streamer.get_qr_generation()):
                    return
                if time.monotonic() >= deadline:
                    self._link.send_not_found()
                    print("[二维码] 确认超时", flush=True)
                    return
                if qr_result is not None:
                    code = parse_competition_task(str(qr_result[0]))
                    self._link.send_taskcode(str(code))
                    print(f"[二维码] 上报已确认任务: {code}", flush=True)
                    return
            stop_event.wait(0.05)

    def _start_ring_loop(self) -> None:
        self._ring_stop = threading.Event()
        self._ring_thread = threading.Thread(
            target=self._ring_loop, args=(self._ring_stop,), daemon=True)
        self._ring_thread.start()

    def _stop_ring_thread(self) -> None:
        self._ring_stop.set()
        if self._ring_thread is not None:
            self._ring_thread.join(timeout=1.0)
            self._ring_thread = None

    def _ring_loop(self, stop_event) -> None:
        while (self._scanning and self._scan_mode == SCAN_MODE_VERIFY
               and not self._stop.is_set() and not stop_event.is_set()):
            rings = self._streamer.get_rings()
            if rings:
                target = min(
                    rings,
                    key=lambda item: (
                        item.center_x - self._streamer.get_frame_width() / 2.0
                    ) ** 2
                    + (item.center_y - self._streamer.get_frame_height() / 2.0) ** 2,
                )
                self._link.send_ring_center(
                    target.ring_no, target.center_x, target.center_y
                )
            else:
                self._link.send_not_found()
            stop_event.wait(0.1)

    def _mode_name(self, mode) -> str:
        names = {
            SCAN_MODE_QR: "读二维码",
            SCAN_MODE_MATERIAL: "找物料",
            SCAN_MODE_VERIFY: "放置验证",
        }
        return names.get(mode, "未知")

    def _material_loop(self) -> None:
        while not self._stop.is_set():
            if self._scanning and self._scan_mode == SCAN_MODE_MATERIAL:
                materials = self._streamer.get_materials()
                target = None
                if self._target_color is not None:
                    for item in materials:
                        if item.color_id == self._target_color:
                            target = item
                            break
                now = time.time()
                if target is None:
                    # 遮挡漏检：容忍期内沿用最近稳定中心，避免上报中断
                    report = self._mat_stab.report_center(now)
                    if report is None:
                        self._link.send_not_found()
                    else:
                        cx, cy = report
                        width = self._streamer.get_frame_width()
                        angle = self._offset_angle(
                            cx, width, self._config.camera.fov_h_deg
                        )
                        self._link.send_found(
                            self._target_color,
                            int(round(cx)),
                            int(round(cy)),
                            angle,
                            self._config.fixed_distance_cm,
                        )
                    continue
                cx, cy = self._mat_stab.observe(
                    target.center_x, target.center_y, now
                )
                allow = self._streamer.get_rotation_allowed()
                if allow:
                    width = self._streamer.get_frame_width()
                    angle = self._offset_angle(
                        cx, width, self._config.camera.fov_h_deg
                    )
                    self._link.send_found(
                        target.color_id,
                        int(round(cx)),
                        int(round(cy)),
                        angle,
                        self._config.fixed_distance_cm,
                    )
                    self._mat_stab.mark_stable(now)
                else:
                    self._link.send_not_found()
            time.sleep(0.1)

    def _offset_angle(self, center_x: int, width: int, fov_h_deg: float) -> float:
        if width <= 0:
            return 0.0
        normalized = (center_x - width / 2.0) / (width / 2.0)
        return normalized * (fov_h_deg / 2.0)

    def _heartbeat_loop(self) -> None:
        while not self._stop.is_set():
            try:
                self._link.send_heartbeat()
            except Exception as exc:
                LOGGER.warning("心跳发送失败: %s", exc)
            time.sleep(1.0)


class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        path = urlsplit(self.path).path
        if path == "/":
            self._send_index()
        elif path == "/stream":
            self._send_stream()
        elif path == "/frame.jpg":
            self._send_frame()
        elif path == "/state":
            self._send_state()
        else:
            self.send_error(404)

    def _send_index(self) -> None:
        html = (
            "<!DOCTYPE html><html><head><meta charset='utf-8'>"
            "<title>树莓派视觉预览</title></head>"
            "<style>html,body{margin:0;height:100%;background:#111;overflow:hidden}"
            "#video{position:absolute;width:100vw;height:100vh;object-fit:contain;background:#111}"
            "#overlay{position:absolute;inset:0;width:100%;height:100%;pointer-events:none}"
            "</style></head>"
            "<body><img id='video' alt='等待相机画面'><svg id='overlay' xmlns='http://www.w3.org/2000/svg'></svg>"
            "<div style='position:fixed;left:10px;bottom:10px;z-index:10;background:#111;color:#fff;padding:8px;font:16px sans-serif'>"
            "<span id='video-status' role='status' aria-live='polite'>正在连接相机画面…</span> "
            "<button id='reconnect-video' type='button' style='margin-left:8px;padding:5px 10px;cursor:pointer'>重新连接</button></div>"
            "<div id='status' style='position:fixed;top:10px;right:10px;color:#fff;background:rgba(0,0,0,0.65);padding:6px 10px;font:14px sans-serif;z-index:9'>加载中...</div>"
            "<div id='rot' style='position:fixed;top:44px;right:10px;color:#fff;background:#555;padding:4px 10px;font:13px sans-serif;z-index:9'>转盘:未扫描</div>"
            "<div id='coords' style='position:fixed;bottom:10px;right:10px;color:#fff;background:rgba(0,0,0,0.65);padding:8px 10px;font:14px sans-serif;z-index:9;line-height:1.6'>物料:<br>圆环:</div>"
            "<script>"
            "const img=document.getElementById('video'),svg=document.getElementById('overlay');"
            "let sourceWidth=0,sourceHeight=0;"
            "function applyViewBox(){svg.setAttribute('viewBox','0 0 '+(sourceWidth||img.naturalWidth||640)+' '+(sourceHeight||img.naturalHeight||480));}"
            + FRAME_CLIENT_JS +
            "function esc(s){return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');}"
            "let qrCache=null,matCache=[],ringCache=[],qrT=0,matT=0,ringT=0,materialRoi=null;"
            "function render(){let h='';"
            "if(materialRoi){const [x,y,w,hgt]=materialRoi;h+='<rect x=\"'+x+'\" y=\"'+y+'\" width=\"'+w+'\" height=\"'+hgt+'\" fill=\"none\" stroke=\"#ffffff\" stroke-width=\"2\" stroke-dasharray=\"8 6\"/>';"
            "h+='<text x=\"'+(x+8)+'\" y=\"'+(y+24)+'\" fill=\"#ffffff\" font-size=\"18\">物料识别区域</text>';}"
            "if(qrCache){h+='<polygon points=\"'+qrCache.points.map(p=>p[0]+','+p[1]).join(' ')+'\" fill=\"none\" stroke=\"#00ff66\" stroke-width=\"2\"/>';"
            "const p=qrCache.points[0];h+='<text x=\"'+(p[0]+10)+'\" y=\"'+(p[1]-8)+'\" fill=\"#00ff66\" font-size=\"18\">'+esc(qrCache.text)+'</text>';}"
            "matCache.forEach(m=>{h+='<circle cx=\"'+m.center_x+'\" cy=\"'+m.center_y+'\" r=\"2.25\" fill=\"#00ff66\"/>';"
            "h+='<text x=\"'+(m.center_x+8)+'\" y=\"'+(m.center_y-6)+'\" fill=\"#00ff66\" font-size=\"16\">'+(m.color_name||('C'+m.color_id))+'</text>';});"
            "ringCache.forEach(r=>{h+='<circle cx=\"'+r.center_x+'\" cy=\"'+r.center_y+'\" r=\"'+r.radius+'\" fill=\"none\" stroke=\"#ff3366\" stroke-width=\"0.6\"/>';"
            "h+='<circle cx=\"'+r.center_x+'\" cy=\"'+r.center_y+'\" r=\"1\" fill=\"#ff3366\"/>';"
            "h+='<text x=\"'+(r.center_x+8)+'\" y=\"'+(r.center_y-6)+'\" fill=\"#ff3366\" font-size=\"16\">R'+r.ring_no+'</text>';});"
            "svg.innerHTML=h;}"
            "async function loop(){const controller=new AbortController();const deadline=setTimeout(()=>controller.abort(),2500);try{"
            "const response=await fetch('/state',{cache:'no-store',signal:controller.signal});if(!response.ok)throw new Error('HTTP '+response.status);"
            "const s=await response.json();const now=Date.now();let changed=false;"
            "materialRoi=s.material_roi||null;changed=true;"
            "if(s.qr){qrCache=s.qr;qrT=now;changed=true;}else if(now-qrT>1000){qrCache=null;changed=true;}"
            "if(s.materials.length){matCache=s.materials;matT=now;changed=true;}else if(now-matT>1000){matCache=[];changed=true;}"
            "if(s.rings.length){ringCache=s.rings;ringT=now;changed=true;}else if(now-ringT>1000){ringCache=[];changed=true;}"
            "if(changed)render();"
            "let coords='物料:<br>';matCache.forEach(m=>{coords+=(m.color_name||('C'+m.color_id))+' ('+m.center_x+','+m.center_y+') 分数 '+m.confidence+'<br>';});"
            "coords+='圆环:<br>';ringCache.forEach(r=>{coords+='R'+r.ring_no+' ('+r.center_x+','+r.center_y+')<br>';});"
            "document.getElementById('coords').innerHTML=coords;"
            "const d=s.detection||{qr:false,materials:false,rings:false};"
            "document.getElementById('status').textContent='QR:'+(s.qr?s.qr.text:'无')+' 物料:'+s.materials.length+' 圆环:'+s.rings.length+' 开关:二维码'+(d.qr?'开':'关')+' 物料'+(d.materials?'开':'关')+' 圆环'+(d.rings?'开':'关')+' 坐标画幅:'+sourceWidth+'x'+sourceHeight;"
            "const rs=s.rotation||'未扫描';const rot=document.getElementById('rot');"
            "rot.textContent='转盘:'+rs;"
            "rot.style.background=rs.indexOf('稳定')>=0?'#1b8f3c':(rs.indexOf('换位')>=0?'#c0392b':(rs.indexOf('等待')>=0?'#d68910':'#555'));"
            "}catch(e){document.getElementById('status').textContent='识别状态暂时断开，正在重试…';}finally{clearTimeout(deadline);}}"
            "let stateTimer=null,stateStopped=false;async function pollState(){await loop();if(!stateStopped)stateTimer=setTimeout(pollState,200);}pollState();"
            "window.addEventListener('pagehide',()=>{stateStopped=true;clearTimeout(stateTimer);});"
            "window.addEventListener('pageshow',()=>{if(stateStopped){stateStopped=false;pollState();}});"
            "window.addEventListener('resize',applyViewBox);img.addEventListener('load',applyViewBox);applyViewBox();"
            "</script></body></html>"
        )
        body = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_state(self) -> None:
        body = json.dumps(_streamer.get_state()).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store, no-cache")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_frame(self) -> None:
        jpeg, sequence, acquired_at = _streamer.get_preview_frame()
        self.send_response(200 if jpeg else 503)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Content-Length", str(len(jpeg) if jpeg else 0))
        if jpeg:
            self.send_header("X-Frame-Id", str(sequence))
            self.send_header("X-Source-Width", str(_streamer.get_frame_width()))
            self.send_header("X-Source-Height", str(_streamer.get_frame_height()))
            self.send_header("X-Capture-Age-Ms", str(max(0, round(
                (time.monotonic() - acquired_at) * 1000))))
        else:
            self.send_header("Retry-After", "1")
        self.end_headers()
        if jpeg:
            self.wfile.write(jpeg)

    def _send_stream(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.end_headers()
        last_sequence = -1
        next_send_at = time.monotonic()
        try:
            while True:
                now = time.monotonic()
                if now < next_send_at:
                    time.sleep(min(0.05, next_send_at - now))
                    continue
                jpeg, sequence, acquired_at = _streamer.get_preview_frame()
                if jpeg is None or sequence == last_sequence:
                    time.sleep(0.02)
                    continue
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"X-Frame-Id: {sequence}\r\n".encode())
                age_ms = max(0, round((time.monotonic() - acquired_at) * 1000))
                self.wfile.write(f"X-Capture-Age-Ms: {age_ms}\r\n".encode())
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
                last_sequence = sequence
                # 每次发送都重新取最新帧，慢客户端不会在应用层排队补发。
                next_send_at = time.monotonic() + 0.1
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, fmt, *args) -> None:
        pass


def resolve_startup_detection(args: argparse.Namespace, config: VisionConfig):
    """计算启动时三种识别算法的初始开关状态。

    优先级（高 → 低）：
    1. 单项命令行参数：--no-qr / --materials / --no-materials / --rings / --no-rings，
       只决定对应算法（可与 --algorithms-on 组合，如全开但关圆环）；
    2. 总开关参数：--no-algorithms-on（全关）> --algorithms-on（全开）；
    3. config 的 detection.algorithms_default_on。
    """
    if args.no_algorithms_on:
        default_on = False
    elif args.algorithms_on:
        default_on = True
    else:
        default_on = config.detection.algorithms_default_on
    enable_qr = False if args.no_qr else default_on
    enable_materials = (
        False if args.no_materials else (True if args.materials else default_on)
    )
    enable_rings = False if args.no_rings else (True if args.rings else default_on)
    return enable_qr, enable_materials, enable_rings


def primary_lan_ip() -> str:
    """取本机局域网源 IP，用于启动时打印正确的预览地址。

    Debian 系 hostname 会解析到 127.0.1.1，不能直接用 gethostbyname；
    UDP connect 不会真的发包，仅让内核按路由选出源地址。
    """
    import socket

    for peer in ("192.168.137.1", "8.8.8.8"):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect((peer, 80))
                ip = s.getsockname()[0]
                if not ip.startswith("127."):
                    return ip
        except Exception:
            continue
    try:
        ip = socket.gethostbyname(socket.gethostname())
        if not ip.startswith("127."):
            return ip
    except Exception:
        pass
    return "127.0.0.1"


def main() -> None:
    parser = argparse.ArgumentParser(description="树莓派摄像头 MJPEG 推流服务")
    parser.add_argument("--config", default="config/vision_config.yaml", help="配置文件路径")
    parser.add_argument("--port", type=int, default=8090, help="HTTP 端口")
    parser.add_argument("--quality", type=int, default=55, help="仅预览 JPEG 质量 1-100，默认 55")
    parser.add_argument("--preview-width", type=int, default=640, help="仅预览宽度，默认 640；不改变识别原图")
    parser.add_argument("--preview-fps", type=float, default=5.0, help="仅预览最大帧率，默认 5")
    parser.add_argument("--no-qr", action="store_true", help="关闭二维码检测")
    parser.add_argument("--materials", action="store_true", default=None, help="开启物料检测")
    parser.add_argument("--rings", action="store_true", default=None, help="开启圆环检测")
    parser.add_argument("--no-materials", action="store_true", help="强制关闭物料检测")
    parser.add_argument("--no-rings", action="store_true", help="强制关闭圆环检测")
    parser.add_argument("--rotation-gate", action="store_true", default=None, help="强制开启转盘旋转门控")
    parser.add_argument("--no-rotation-gate", action="store_true", help="强制关闭转盘旋转门控")
    parser.add_argument("--undistort", action="store_true", default=None, help="强制开启畸变矫正")
    parser.add_argument("--no-undistort", action="store_true", help="强制关闭畸变矫正")
    parser.add_argument("--width", type=int, default=0, help="覆盖相机宽度")
    parser.add_argument("--height", type=int, default=0, help="覆盖相机高度")
    parser.add_argument("--camera", default="", help="覆盖主相机编号/路径，如 4 或 /dev/video4")
    parser.add_argument("--material-roi", type=int, nargs=4, metavar=("X", "Y", "W", "H"),
                        help="限定物料识别区域；输出坐标仍相对于完整画面")
    parser.add_argument("--material-backend", choices=['auto', 'hsv'], default='auto',
                        help="物料后端：auto沿用YOLO；hsv为实验性六色轮廓检测")
    parser.add_argument("--fast", action="store_true", help="仅用 zxing 快速识别，关闭 OpenCV 兜底")
    parser.add_argument("--serial-port", default="", help="一体模式下打开串口，例如 /dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=115200, help="串口波特率")
    parser.add_argument("--qr-camera", default="", help="扫码相机编号/路径，默认用主相机")
    parser.add_argument("--detect-interval", type=float, default=0.2, help="检测间隔秒数，默认 0.2")
    parser.add_argument("--detect-every", type=int, default=None, help="兼容旧参数：N/10 秒检测一次")
    parser.add_argument("--overlay", action="store_true", help="使用 OpenCV 直接绘制覆盖层（默认前端矢量渲染）")
    parser.add_argument("--algorithms-on", action="store_true", default=None, help="启动时识别算法默认全开（覆盖 config algorithms_default_on；--no-qr/--no-materials/--no-rings 仍可单独关）")
    parser.add_argument("--no-algorithms-on", action="store_true", default=None, help="启动时识别算法默认全关（覆盖 config algorithms_default_on；--materials/--rings 仍可单独开）")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)
    config = load_config(args.config)
    if args.camera:
        config.camera.source = args.camera
    if args.width:
        config.camera.width = args.width
    if args.height:
        config.camera.height = args.height
    if args.qr_camera:
        config.camera.qr_source = args.qr_camera
    if args.no_rotation_gate:
        config.detection.rotation_gate = False
    elif args.rotation_gate:
        config.detection.rotation_gate = True
    if args.no_undistort:
        config.camera.undistort = False
    elif args.undistort:
        config.camera.undistort = True
    detect_interval = args.detect_interval
    if args.detect_every is not None:
        detect_interval = args.detect_every / 10.0
    camera = Camera(config.camera)
    qr_camera = None
    if config.camera.qr_source:
        qr_config = CameraConfig(
            source=config.camera.qr_source,
            backend=config.camera.backend,
            width=config.camera.width,
            height=config.camera.height,
            fps=config.camera.fps,
            flip=config.camera.flip,
            fov_h_deg=config.camera.fov_h_deg,
            undistort=config.camera.undistort,
        )
        qr_camera = Camera(qr_config)
    pipeline = VisionPipeline(config, camera=camera, qr_camera=qr_camera,
                              material_backend=args.material_backend)
    if args.material_roi:
        pipeline.set_material_roi(args.material_roi)
    if args.fast:
        pipeline.qr_detector.use_opencv_fallback = False
    rotation_gate = None
    if config.detection.rotation_gate:
        rotation_gate = RotationGate(
            move_px=config.detection.rotation_move_px,
            settle_frames=config.detection.rotation_settle_frames,
            window_frames=config.detection.rotation_window_frames,
            stable_px=config.detection.rotation_stable_px,
        )

    # 启动级算法开关：--algorithms-on/--no-algorithms-on 覆盖 config 的
    # algorithms_default_on，得到三种识别的默认全开/全关；单项参数
    # --no-qr/--materials/--no-materials/--rings/--no-rings 优先级更高，
    # 只覆盖对应算法（如 --algorithms-on --no-rings = 开二维码+物料、关圆环提速）。
    # 必须在构造前一次性算好传入，禁止构造后再整体重设——那会覆盖掉单项参数。
    enable_qr, enable_materials, enable_rings = resolve_startup_detection(
        args, config
    )

    global _streamer
    _streamer = FrameStreamer(
        camera,
        pipeline,
        rotation_gate=rotation_gate,
        enable_qr=enable_qr,
        enable_materials=enable_materials,
        enable_rings=enable_rings,
        quality=args.quality,
        detect_interval=detect_interval,
        draw_overlay=args.overlay,
        preview_width=args.preview_width,
        preview_fps=args.preview_fps,
    )
    _streamer.start()

    serial_controller = None
    if args.serial_port:
        serial_controller = SerialController(
            args.serial_port, args.baud, _streamer, config
        )
        serial_controller.start()
        print(f"一体模式已启用，串口: {args.serial_port} @ {args.baud}", flush=True)

    httpd = ThreadingHTTPServer(("0.0.0.0", args.port), StreamHandler)
    print(f"预览地址: http://{primary_lan_ip()}:{args.port}/ （本机访问也可用 http://127.0.0.1:{args.port}/）", flush=True)
    print("Ctrl+C 停止", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
        if serial_controller is not None:
            serial_controller.stop()
        _streamer.stop()


if __name__ == "__main__":
    main()
