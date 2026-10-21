"""物料遮挡丢失容忍：机械臂半闭合遮挡转盘送来的物料时，保持上报不中断。

赛场场景：电控在识别/夹取物料时让机械臂半闭合，夹爪会短暂遮挡转盘送来的
物料，导致视觉**漏检**（target None）→ 若直接 send_not_found 会中断上报，
造成主控误判/延迟、无法及时夹取。

（实测物料中心几乎不抖动，因此不做平滑；只需在遮挡漏检时，于容忍期内沿用
最近一次"已稳定"的中心继续上报，避免 not_found 中断。）

本模块只维护"最近稳定中心"并判断是否仍在丢失容忍期内：
- observe(): 检测到物料，记录中心（不做平滑）；
- mark_stable(): 门控判定"已稳定允许上报"的那一帧调用，用于记住稳定时刻；
- report_center(): 漏检时，若在容忍期内且之前已稳定，返回上一个中心，否则 None。
"""

from __future__ import annotations


class MaterialStability:
    """记录最近稳定中心，并在遮挡漏检的容忍期内继续上报该中心。"""

    def __init__(self, lost_tol_s: float = 0.3):
        self.lost_tol_s = float(lost_tol_s)
        self._last_good = None         # (cx, cy, ts) 最近一次观测到的中心
        self._last_stable_ts = None    # 最近一次"已稳定允许上报"的时间

    def reset(self) -> None:
        """开始新一轮扫描时清空状态。"""
        self._last_good = None
        self._last_stable_ts = None

    def observe(self, cx: float, cy: float, ts: float):
        """本帧检测到物料：记录中心，返回该中心（原样，不做平滑）。"""
        self._last_good = (float(cx), float(cy), ts)
        return float(cx), float(cy)

    def mark_stable(self, ts: float) -> None:
        """记录本帧门控判定"已稳定允许上报"。"""
        self._last_stable_ts = ts

    def report_center(self, ts: float):
        """漏检时返回应上报的中心 (cx, cy)。

        - 若从未检测到物料或从未稳定过 → None（应上报 not_found）；
        - 若仍在上次稳定时刻的容忍期内 → 返回最近中心，维持上报；
        - 否则 → None（物料确实丢失）。
        """
        if self._last_good is None or self._last_stable_ts is None:
            return None
        if ts - self._last_stable_ts <= self.lost_tol_s:
            return self._last_good[0], self._last_good[1]
        return None
