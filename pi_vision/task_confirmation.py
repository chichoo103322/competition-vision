"""初赛任务码的严格校验与连续确认，供扫码验收及后续主流程接入。"""

from __future__ import annotations

import math
import re

from .types import TaskCode


def parse_competition_task(text: str) -> TaskCode:
    """校验初赛的颜色、顺序和位置约束；不进行任务规划。"""
    raw = text.strip()
    if not re.fullmatch(r"[1-6]{3}\+[1-3]{3}\+[1-6]{3}\+[1-3]{3}", raw):
        raise ValueError("任务码须为 CCC+PPP+CCC+PPP，颜色 1—6，位置 1—3")
    code = TaskCode.parse(raw)
    if len(set(code.group1)) != 3 or set(code.group1) != set(code.group3):
        raise ValueError("两批物料须为相同的三种不同颜色")
    if set(code.group2) != set("123") or set(code.group4) != set("123"):
        raise ValueError("两组位置须分别为 1、2、3 的排列")
    return code


class TaskCodeConfirmation:
    """只统计新采集帧；断码、换码或间隔过长会重新累计。

    时间戳须来自 time.monotonic()。确认结果保持到 reset()；驱动是否
    重复返回缓存帧不在本类可观测范围内。
    """

    def __init__(self, frames: int = 3, min_seconds: float = 0.2,
                 max_gap: float = 1.0):
        if isinstance(frames, bool) or not isinstance(frames, int) or frames < 1:
            raise ValueError("确认帧数须为正整数")
        if not math.isfinite(min_seconds) or min_seconds < 0:
            raise ValueError("最短确认时间须为非负有限数")
        if not math.isfinite(max_gap) or max_gap <= 0:
            raise ValueError("最大帧间隔须为正有限数")
        self.frames = frames
        self.min_seconds = min_seconds
        self.max_gap = max_gap
        self.reset()

    def reset(self):
        self.candidate = None
        self.count = 0
        self.locked_code = None
        self._started_at = None
        self._last_at = None

    def update(self, code: TaskCode | None, now: float) -> TaskCode | None:
        if not math.isfinite(now):
            raise ValueError("采集时间戳须为有限数")
        if self.locked_code is not None:
            return self.locked_code
        if self._last_at is not None and now <= self._last_at:
            raise ValueError("新帧的单调时间戳必须严格递增")
        gap = self._last_at is not None and now - self._last_at > self.max_gap
        self._last_at = now
        if code is not None:
            try:
                code = parse_competition_task(str(code))
            except ValueError:
                code = None
        if code is None:
            self.candidate = None
            self.count = 0
            self._started_at = None
            return None
        if gap or self.candidate != code:
            self.candidate = code
            self.count = 1
            self._started_at = now
        else:
            self.count += 1
        if self.count >= self.frames and now - self._started_at >= self.min_seconds:
            self.locked_code = code
        return self.locked_code
