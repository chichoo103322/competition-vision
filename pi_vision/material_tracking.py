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

from dataclasses import replace
import math


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


class MaterialTracker:
    """Short-lived image-space association; does not smooth or preserve output points."""

    def __init__(self, max_gap_s=0.75, max_displacement_px=180.):
        if not math.isfinite(max_gap_s) or max_gap_s <= 0 or not math.isfinite(max_displacement_px) or max_displacement_px <= 0:
            raise ValueError('tracker limits must be finite and positive')
        self.max_gap_s = float(max_gap_s)
        self.max_displacement_px = float(max_displacement_px)
        self._next_id = 1
        self._tracks = {}

    def reset(self):
        self._tracks.clear()

    def update(self, materials, acquired_at):
        """Return this frame's detections with service-local IDs and association status."""
        if not math.isfinite(acquired_at):
            raise ValueError('acquisition time must be finite')
        active = {track_id: track for track_id, track in self._tracks.items()
                  if 0 <= acquired_at-track['time'] <= self.max_gap_s}
        edges = []
        for index, item in enumerate(materials):
            candidates = []
            for track_id, track in active.items():
                if track['color_id'] != item.color_id:
                    continue
                distance = math.hypot(item.center_x-track['x'], item.center_y-track['y'])
                allowed = min(self.max_displacement_px,
                              35. + 240. * max(0., acquired_at-track['time']))
                if distance <= allowed:
                    candidates.append((distance, track_id))
            candidates.sort()
            if candidates and (len(candidates) == 1 or
                               candidates[1][0]-candidates[0][0] > max(8., candidates[0][0]*.15)):
                edges.append((candidates[0][0], index, candidates[0][1]))

        assignments = {}
        used_tracks = set()
        for _, index, track_id in sorted(edges):
            if index not in assignments and track_id not in used_tracks:
                assignments[index] = track_id
                used_tracks.add(track_id)

        out = []
        next_tracks = {track_id: track for track_id, track in active.items()
                       if track_id not in used_tracks}
        for index, item in enumerate(materials):
            track_id = assignments.get(index)
            status = 'updated' if track_id is not None else 'new'
            if track_id is None:
                track_id = self._next_id
                self._next_id += 1
            next_tracks[track_id] = dict(color_id=item.color_id, x=float(item.center_x),
                                         y=float(item.center_y), time=float(acquired_at))
            out.append(replace(item, track_id=track_id, tracking_status=status))
        self._tracks = next_tracks
        return out
        return None
