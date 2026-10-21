"""形状识别占位模块，决赛阶段再实现。"""

from __future__ import annotations

import numpy as np


class ShapeClassifier:
    def classify(self, patch_bgr: np.ndarray) -> int:
        """返回形状 id。

        初赛三种物料同形状，当前统一返回 0；
        决赛需要补充圆柱、方形、三角、球、锥及组合体的分类。
        """
        return 0
