"""二维码任务码识别。"""

from __future__ import annotations

from typing import Optional

import cv2
import numpy as np

from ..types import TaskCode

try:
    import zxingcpp

    ZXING_AVAILABLE = True
except ImportError:
    ZXING_AVAILABLE = False


class QRDetector:
    def __init__(self, use_opencv_fallback: bool = True) -> None:
        self._detector = cv2.QRCodeDetector()
        self.use_opencv_fallback = use_opencv_fallback

    def detect(self, frame: np.ndarray) -> Optional[TaskCode]:
        """从单帧图像中读取任务码。"""
        result = self.detect_with_points(frame)
        if result is None:
            return None
        return result[0]

    def detect_with_points(self, frame: np.ndarray):
        """读取任务码，同时返回二维码四个角点。

        返回 (TaskCode, points)，points 为 (4, 2) 的角点数组。
        """
        zxing_result = self._try_zxing(frame)
        if zxing_result is not None:
            return zxing_result
        if not self.use_opencv_fallback:
            return None

        candidates = [(frame, 1.0)]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        candidates.append((gray, 1.0))

        upscaled = cv2.resize(
            gray, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC
        )
        candidates.append((upscaled, 2.0))

        blur = cv2.GaussianBlur(gray, (0, 0), 1.0)
        sharp = cv2.addWeighted(gray, 1.5, blur, -0.5, 0)
        candidates.append((sharp, 1.0))

        for image, scale in candidates:
            result = self._try_decode(image, scale)
            if result is not None:
                return result
        return None

    def _try_zxing(self, frame: np.ndarray):
        """优先使用 zxing-cpp 解码，识别率比 OpenCV 更高。"""
        if not ZXING_AVAILABLE:
            return None
        try:
            result = zxingcpp.read_barcode(frame)
        except Exception:
            return None
        if result is None or not result.valid or not result.text:
            return None
        try:
            code = TaskCode.parse(result.text)
        except ValueError:
            return None
        pos = result.position
        pts = np.array(
            [
                [pos.top_left.x, pos.top_left.y],
                [pos.top_right.x, pos.top_right.y],
                [pos.bottom_right.x, pos.bottom_right.y],
                [pos.bottom_left.x, pos.bottom_left.y],
            ],
            dtype=np.float32,
        )
        return code, pts

    def _try_decode(self, image: np.ndarray, scale: float):
        """尝试解码一张候选图，并把角点坐标换算回原图。"""
        try:
            text, points, _ = self._detector.detectAndDecode(image)
        except cv2.error:
            return None
        if not text or points is None:
            return None
        try:
            pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
            if len(pts) < 3 or cv2.contourArea(pts) <= 0:
                return None
            code = TaskCode.parse(text)
        except (ValueError, cv2.error):
            return None
        return code, pts / scale
