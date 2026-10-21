"""基于 LAB 颜色中心的物料颜色分类器。"""

from __future__ import annotations

from typing import List, Optional, Tuple

import cv2
import numpy as np

from ..config import ColorSpec


class ColorClassifier:
    def __init__(self, colors: List[ColorSpec]) -> None:
        self._colors = list(colors)
        self._centers = np.array([color.lab_center for color in self._colors], dtype=np.float32)
        self._ids = np.array([color.color_id for color in self._colors], dtype=np.int32)
        self._max_dist = np.array(
            [color.max_distance for color in self._colors], dtype=np.float32
        )
        self._distance_scale = 1.0

    def set_distance_scale(self, scale: float) -> None:
        """调整颜色识别的整体灵敏度，越大越容易匹配。"""
        self._distance_scale = float(scale)

    def set_color_center(self, color_id: int, lab_center, max_distance: Optional[float] = None) -> None:
        """用实际采样更新某个颜色的 LAB 中心。"""
        for index, color in enumerate(self._colors):
            if color.color_id == color_id:
                new_center = [float(v) for v in lab_center]
                self._centers[index] = np.asarray(new_center, dtype=np.float32)
                if max_distance is not None:
                    self._max_dist[index] = float(max_distance)
                self._colors[index] = ColorSpec(
                    color_id=color.color_id,
                    name=color.name,
                    lab_center=new_center,
                    max_distance=float(self._max_dist[index]),
                )
                return
        raise ValueError(f"未配置颜色编号: {color_id}")

    @property
    def color_specs(self) -> List[ColorSpec]:
        """返回当前颜色配置，用于保存到 YAML。"""
        return list(self._colors)

    def nearest_color_ids(
        self, lab_image: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """返回最近颜色 id、最小距离和该颜色允许的最大距离。"""
        lab = np.asarray(lab_image, dtype=np.float32)
        flat = lab.reshape(-1, 3)
        diff = flat[:, None, :] - self._centers[None, :, :]
        dist = np.linalg.norm(diff, axis=2)
        nearest = np.argmin(dist, axis=1)
        min_dist = dist[np.arange(len(flat)), nearest]
        return (
            self._ids[nearest].reshape(lab.shape[:2]),
            min_dist.reshape(lab.shape[:2]),
            (self._max_dist[nearest] * self._distance_scale).reshape(lab.shape[:2]),
        )

    def color_id_for_lab(self, lab: np.ndarray) -> int:
        """把一个 LAB 像素值归类为颜色 id，超出阈值返回 0。"""
        lab = np.asarray(lab, dtype=np.float32).reshape(-1, 3)[0]
        diff = lab[None, :] - self._centers
        dist = np.linalg.norm(diff, axis=1)
        index = int(np.argmin(dist))
        if dist[index] <= self._max_dist[index] * self._distance_scale:
            return int(self._ids[index])
        return 0

    @property
    def max_distances(self) -> np.ndarray:
        """各颜色允许的最大 LAB 距离。"""
        return self._max_dist * self._distance_scale

    def color_id_for_patch(self, patch_bgr: np.ndarray) -> int:
        """把一小块 BGR 图像的平均 LAB 归类为颜色 id。"""
        lab = cv2.cvtColor(patch_bgr, cv2.COLOR_BGR2LAB)
        mean = lab.reshape(-1, 3).mean(axis=0)
        return self.color_id_for_lab(mean)
