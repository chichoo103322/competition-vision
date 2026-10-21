"""合成测试场景，用于没有硬件时开发视觉算法。"""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

# BGR 颜色，用于合成图像
COLOR_BGR = {
    1: (0, 0, 230),
    2: (0, 215, 235),
    3: (235, 120, 40),
    4: (60, 185, 70),
    5: (28, 28, 28),
    6: (235, 200, 110),
}


class SyntheticScene:
    def __init__(self, size: Tuple[int, int] = (640, 480), background: int = 255):
        self.width, self.height = size
        self.background = background

    def blank(self) -> np.ndarray:
        """生成空白背景帧。"""
        return np.full((self.height, self.width, 3), self.background, dtype=np.uint8)

    def render_materials(
        self,
        color_ids: Sequence[int],
        positions: Optional[List[Tuple[int, int]]] = None,
    ) -> np.ndarray:
        """在图像中画若干彩色圆形物料。"""
        if positions is None:
            positions = [(220, 240), (320, 240), (420, 240)]
        frame = self.blank()
        for color_id, (x, y) in zip(color_ids, positions):
            cv2.circle(frame, (x, y), 28, COLOR_BGR[int(color_id)], -1)
        return frame

    def render_rings(
        self,
        ring_nos: Optional[Sequence[int]] = None,
        positions: Optional[List[Tuple[int, int]]] = None,
        arc: float = 360.0,
        occlude: bool = False,
        contrast: float = 1.0,
    ) -> np.ndarray:
        """在图像中画一组同心圆环和环号。

        arc: 360 画整圆；< 360 只画部分圆弧（模拟遮挡导致的形态不完整）。
        occlude: 用白色横条盖住圆环中段（模拟强光/遮挡造成的缺口）。
        contrast: 越低圆环越接近背景（模拟强光洗白），1.0 为原样。
        """
        if ring_nos is None:
            ring_nos = [1, 2, 3, 4, 5]
        if positions is None:
            positions = [(320, 240)] * len(ring_nos)
        frame = self.blank()
        for index, (ring_no, (x, y)) in enumerate(zip(ring_nos, positions)):
            radius = 40 + index * 30
            contrast = max(0.0, min(1.0, contrast))
            color = tuple(
                int(255 - (255 - c) * contrast) for c in (100, 100, 100)
            )
            if arc >= 359.0:
                cv2.circle(frame, (x, y), radius, color, 3)
            else:
                cv2.ellipse(frame, (x, y), (radius, radius), 0, 0, arc, color, 3)
            if occlude:
                # 白色横条盖住圆环中段：圆环断成上下两段圆弧
                cv2.rectangle(
                    frame,
                    (x - radius - 10, y - 12),
                    (x + radius + 10, y + 8),
                    (255, 255, 255),
                    -1,
                )
            cv2.putText(
                frame,
                str(ring_no),
                (x - 8, y + 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (60, 60, 60),
                2,
            )
        return frame
