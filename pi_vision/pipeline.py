"""视觉流水线：把相机和多个检测器组合成统一接口。"""

from __future__ import annotations

import logging
import math
import time
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np

from .camera import Camera
from .config import VisionConfig
from .detectors import (
    ColorClassifier,
    MaterialDetector,
    QRDetector,
    RingDetector,
    ShapeClassifier,
    YoloMaterialDetector,
)
from .types import Material, Ring, TaskCode
from .task_confirmation import TaskCodeConfirmation, parse_competition_task
from .material_region import MaterialRegionDetector

LOGGER = logging.getLogger(__name__)


class VisionPipeline:
    def __init__(
        self,
        config: VisionConfig,
        camera: Optional[Camera] = None,
        qr_camera: Optional[Camera] = None,
        material_backend: str = 'auto',
        hsv_profile=None,
    ):
        self.config = config
        self.camera = camera
        self.qr_camera = qr_camera if qr_camera is not None else camera
        self.qr_detector = QRDetector()
        self.color_classifier = ColorClassifier(config.colors)
        self.yolo_detector = YoloMaterialDetector(
            config.detection.yolo_model_path,
            confidence=config.detection.yolo_confidence,
        )
        if material_backend not in ('auto', 'hsv'):
            raise ValueError('material_backend must be auto or hsv')
        if material_backend == 'hsv':
            from .detectors.hsv_material_detector import HSVMaterialDetector
            self.material_detector = HSVMaterialDetector(profile=hsv_profile)
            LOGGER.info('物料后端：实验性HSV颜色轮廓；坐标为轮廓框中心，尚需实机验收')
        elif Path(config.detection.yolo_model_path).exists():
            try:
                self.yolo_detector.load()
                if self.yolo_detector._session is not None:
                    self.material_detector = self.yolo_detector
                else:
                    self.material_detector = MaterialDetector(self.color_classifier)
            except Exception as exc:
                LOGGER.warning("YOLO 模型加载失败，使用颜色分割: %s", exc)
                self.material_detector = MaterialDetector(self.color_classifier)
        else:
            self.material_detector = MaterialDetector(self.color_classifier)
        self.ring_detector = RingDetector(
            min_rings_per_cluster=config.detection.min_rings_per_cluster,
            hough_enabled=config.detection.ring_hough,
            hough_param2=config.detection.ring_hough_param2,
            min_radius=config.detection.ring_min_radius,
            hollow_check=config.detection.ring_hollow,
            hollow_threshold=config.detection.ring_hollow_threshold,
            smooth_alpha=config.detection.ring_smooth_alpha,
            refine_center=config.detection.ring_refine_center,
            refine_max_offset_ratio=config.detection.ring_refine_max_offset_ratio,
        )
        self.shape_classifier = ShapeClassifier()

    def read_frame(self):
        """从相机读取一帧，未配置相机时抛出异常。"""
        if self.camera is None:
            raise RuntimeError("未配置相机")
        ok, frame, timestamp = self.camera.read()
        if not ok:
            raise RuntimeError("读取相机帧失败")
        return frame, timestamp

    def read_qr_frame(self):
        """从扫码相机读取一帧；未单独配置时退回主相机。"""
        if self.qr_camera is None:
            return self.read_frame()
        ok, frame, timestamp = self.qr_camera.read()
        if not ok:
            raise RuntimeError("读取扫码相机帧失败")
        return frame, timestamp

    def detect_task_code(
        self, frame: Optional[np.ndarray] = None, max_frames: int = 5
    ) -> Optional[TaskCode]:
        """读取二维码任务码，返回首次成功解出的任务码。"""
        for _ in range(max_frames):
            current = frame
            if current is None:
                current, _ = self.read_qr_frame()
            code = self.qr_detector.detect(current)
            if code is not None:
                try:
                    return parse_competition_task(str(code))
                except ValueError:
                    pass
            if frame is not None:
                break
        return None

    def detect_task_code_stable(
        self, max_frames: int = 30, min_confirmed: int = 3,
        timeout: float = 10.0, stop_event=None,
    ) -> Optional[TaskCode]:
        """仅返回连续确认的合法任务；预算耗尽时不返回最高票候选。"""
        if max_frames < 1 or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("扫码帧数和超时时间必须为正数")
        confirmation = TaskCodeConfirmation(frames=min_confirmed)
        deadline = time.monotonic() + timeout
        for _ in range(max_frames):
            if time.monotonic() >= deadline or (stop_event and stop_event.is_set()):
                return None
            frame, _ = self.read_qr_frame()
            acquired_at = time.monotonic()
            try:
                code = self.qr_detector.detect(frame)
            except UnicodeError:
                code = None
            if time.monotonic() >= deadline or (stop_event and stop_event.is_set()):
                return None
            locked = confirmation.update(code, acquired_at)
            if locked is not None:
                return locked
        return None

    def detect_materials(
        self, frame: Optional[np.ndarray] = None
    ) -> List[Material]:
        """识别物料颜色与位置。"""
        current = frame
        if current is None:
            current, _ = self.read_frame()
        return self.material_detector.detect(current)

    def set_material_roi(self, roi) -> None:
        """统一包装当前检测后端，保留输出的全图坐标。"""
        detector = self.material_detector
        if isinstance(detector, MaterialRegionDetector):
            detector = detector.detector
        self.material_detector = detector if roi is None else MaterialRegionDetector(detector, roi)

    def detect_rings(self, frame: Optional[np.ndarray] = None) -> List[Ring]:
        """识别放置圆环的位置。"""
        current = frame
        if current is None:
            current, _ = self.read_frame()
        return self.ring_detector.detect(current)

    def draw_debug(
        self,
        frame: np.ndarray,
        materials: Optional[List[Material]] = None,
        rings: Optional[List[Ring]] = None,
    ) -> np.ndarray:
        """在调试图上画出检测结果。"""
        canvas = frame.copy()
        for material in materials or []:
            cv2.circle(canvas, (material.center_x, material.center_y), 8, (0, 255, 0), -1)
            cv2.putText(
                canvas,
                f"C{material.color_id}",
                (material.center_x + 10, material.center_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 0),
                2,
            )
        for ring in rings or []:
            cv2.circle(canvas, (ring.center_x, ring.center_y), 12, (255, 0, 0), 2)
            cv2.putText(
                canvas,
                f"R{ring.ring_no}",
                (ring.center_x + 14, ring.center_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 0),
                2,
            )
        return canvas
