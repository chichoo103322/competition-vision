"""物料检测：颜色分割 + 轮廓定位。"""

from __future__ import annotations

from typing import List

import cv2
import numpy as np

from ..types import Material
from .color_classifier import ColorClassifier


class MaterialDetector:
    def __init__(
        self,
        color_classifier: ColorClassifier,
        min_area: int = 180,
        max_area: int = 12000,
        morph_kernel: int = 5,
        min_solidity: float = 0.5,
        min_circularity: float = 0.15,
        max_aspect: float = 4.0,
    ) -> None:
        self._classifier = color_classifier
        self.min_area = min_area
        self.max_area = max_area
        self.min_solidity = min_solidity
        self.min_circularity = min_circularity
        self.max_aspect = max_aspect
        self._kernel = np.ones((morph_kernel, morph_kernel), dtype=np.uint8)

    def detect(self, frame: np.ndarray) -> List[Material]:
        """返回图像中的物料列表。"""
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        _, dist, allowed = self._classifier.nearest_color_ids(lab)
        # 逐像素比较最近颜色距离，生成前景掩码
        mask = (dist <= allowed).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        materials = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area or area > self.max_area:
                continue
            hull_area = cv2.contourArea(cv2.convexHull(contour))
            solidity = area / hull_area if hull_area > 0 else 0.0
            perimeter = cv2.arcLength(contour, True)
            circularity = (4 * np.pi * area) / (perimeter ** 2) if perimeter > 0 else 0.0
            rect = cv2.minAreaRect(contour)
            (rect_w, rect_h) = rect[1]
            aspect = max(rect_w, rect_h) / min(rect_w, rect_h) if min(rect_w, rect_h) > 0 else 99.0
            if (
                solidity < self.min_solidity
                or circularity < self.min_circularity
                or aspect > self.max_aspect
            ):
                continue
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue
            center_x = int(moments["m10"] / moments["m00"])
            center_y = int(moments["m01"] / moments["m00"])
            _, _, theta_deg = rect
            patch = frame[
                max(0, center_y - 12) : center_y + 12,
                max(0, center_x - 12) : center_x + 12,
            ]
            lab_patch = cv2.cvtColor(patch, cv2.COLOR_BGR2LAB)
            median_lab = np.median(lab_patch.reshape(-1, 3), axis=0)
            color_id = self._classifier.color_id_for_lab(median_lab)
            if color_id == 0:
                continue
            materials.append(
                Material(
                    index=len(materials) + 1,
                    color_id=color_id,
                    shape_id=0,
                    center_x=center_x,
                    center_y=center_y,
                    theta_deg=theta_deg,
                    confidence=90,
                )
            )
        return materials
