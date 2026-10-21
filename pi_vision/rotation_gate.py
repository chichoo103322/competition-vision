"""转盘旋转门控：转盘换位后位置稳定才放行物料坐标上报。"""

from __future__ import annotations

from collections import deque


class RotationGate:
    """根据目标物料中心最近几帧的累计位移判断转盘是否在换位。

    规则：
    - 首次启动不直接放行，必须观察到一次换位后才可能放行；
    - 最近 window_frames 帧累计位移超过 move_px 判为换位中，不放行；
    - 换位结束后连续 settle_frames 帧位移都很小，才放行上报。
    """

    def __init__(
        self,
        move_px: float = 40.0,
        settle_frames: int = 5,
        window_frames: int = 3,
        stable_px: float = 6.0,
    ) -> None:
        self.move_px = max(1.0, float(move_px))
        self.settle_frames = max(1, int(settle_frames))
        self.window_frames = max(2, int(window_frames))
        self.stable_px = max(1.0, float(stable_px))
        self._history = deque(maxlen=self.window_frames)
        self._last = None
        self._settle_count = 0
        self._seen_rotation = False

    def reset(self) -> None:
        """开始新一轮扫描时重置，重新学习第一次换位。"""
        self._history.clear()
        self._last = None
        self._settle_count = 0
        self._seen_rotation = False

    @property
    def status(self) -> str:
        """当前门控状态，用于推流页面显示。"""
        if not self._seen_rotation:
            return "等待首次换位"
        if self._settle_count < self.settle_frames:
            return "换位中"
        return "已稳定可上报"

    def update(self, x: float, y: float) -> bool:
        """喂入当前帧目标中心，返回 True 表示允许上报坐标。"""
        self._history.append((float(x), float(y)))
        step = 0.0
        if self._last is not None:
            step = (
                (float(x) - self._last[0]) ** 2
                + (float(y) - self._last[1]) ** 2
            ) ** 0.5
        self._last = (float(x), float(y))
        if len(self._history) < 2:
            return False
        points = list(self._history)
        moved = 0.0
        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            moved += ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        if step > self.stable_px or moved > self.move_px:
            # 累计位移偏大：判为换位中，持续重置稳定计数
            self._seen_rotation = True
            self._settle_count = 0
            return False
        self._settle_count += 1
        if self._settle_count >= self.settle_frames:
            return self._seen_rotation
        return False
