"""相机抽象，统一 USB 相机与 CSI 相机接入。"""

from __future__ import annotations

import time

import cv2
import numpy as np

from .config import CameraConfig


class Camera:
    def __init__(self, config: CameraConfig):
        self.config = config
        self._capture = None
        self._map1 = None
        self._map2 = None

    @property
    def is_open(self) -> bool:
        return self._capture is not None and bool(self._capture.isOpened())

    def open(self) -> None:
        """打开相机并应用分辨率、帧率等参数。"""
        if self.is_open:
            raise RuntimeError("相机已经打开")
        source = self.config.source
        backend = self._resolve_backend()
        video_source = int(source) if source.isdigit() else source
        self._capture = cv2.VideoCapture(video_source, backend)
        if not self._capture.isOpened():
            self._capture = None
            raise RuntimeError(f"无法打开相机: {source}")
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        self._capture.set(cv2.CAP_PROP_FPS, self.config.fps)
        self._prepare_undistort()

    def read(self):
        """读取一帧，返回 (是否成功, BGR 图像, 时间戳)。"""
        if not self.is_open:
            raise RuntimeError("相机未打开")
        ok, frame = self._capture.read()
        if not ok:
            return False, None, 0.0
        if self.config.flip:
            frame = cv2.flip(frame, self.config.flip)
        if self._map1 is not None and self._map2 is not None:
            frame = cv2.remap(frame, self._map1, self._map2, cv2.INTER_LINEAR)
        return True, frame, time.time()

    def release(self) -> None:
        """释放相机资源。"""
        if self._capture is not None:
            self._capture.release()
        self._capture = None

    def _resolve_backend(self) -> int:
        """把配置里的后端名称解析为 OpenCV 后端常量。"""
        backend_name = (self.config.backend or "v4l2").lower()
        if backend_name == "v4l2" and hasattr(cv2, "CAP_V4L2"):
            return cv2.CAP_V4L2
        if backend_name == "gstreamer" and hasattr(cv2, "CAP_GSTREAMER"):
            return cv2.CAP_GSTREAMER
        if backend_name == "ffmpeg" and hasattr(cv2, "CAP_FFMPEG"):
            return cv2.CAP_FFMPEG
        if backend_name == "any":
            return cv2.CAP_ANY
        raise RuntimeError(f"不支持的相机后端: {self.config.backend}")

    def _prepare_undistort(self) -> None:
        """如果启用了畸变矫正且配置了标定参数，预生成去畸变映射表。"""
        if not self.config.undistort:
            self._map1 = None
            self._map2 = None
            return
        matrix = self.config.calibration_matrix
        dist = self.config.distortion_coeffs
        if len(matrix) != 9 or len(dist) < 4:
            return
        width = int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH) or self.config.width)
        height = int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT) or self.config.height)
        camera_matrix = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
        if len(self.config.calibration_size) >= 2:
            calib_w = self.config.calibration_size[0]
            calib_h = self.config.calibration_size[1]
            if calib_w > 0 and calib_h > 0 and (calib_w, calib_h) != (width, height):
                sx = width / calib_w
                sy = height / calib_h
                camera_matrix = np.array(
                    [
                        [camera_matrix[0, 0] * sx, 0, camera_matrix[0, 2] * sx],
                        [0, camera_matrix[1, 1] * sy, camera_matrix[1, 2] * sy],
                        [0, 0, 1],
                    ],
                    dtype=np.float64,
                )
        dist_coeffs = np.asarray(dist, dtype=np.float64)
        new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            camera_matrix,
            dist_coeffs,
            (width, height),
            alpha=0.5,
            centerPrincipalPoint=True,
        )
        self._map1, self._map2 = cv2.initUndistortRectifyMap(
            camera_matrix,
            dist_coeffs,
            None,
            new_camera_matrix,
            (width, height),
            cv2.CV_32FC1,
        )

    def __enter__(self) -> "Camera":
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.release()
