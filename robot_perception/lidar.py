"""Fail-safe, chassis-aware obstacle monitoring for a 2D lidar."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class Scan:
    ranges_m: tuple[float, ...]
    angle_min_deg: float
    angle_increment_deg: float

    @classmethod
    def from_dict(cls, data: dict) -> "Scan":
        ranges = data.get("ranges_m")
        if not isinstance(ranges, list) or len(ranges) < 10:
            raise ValueError("ranges_m must contain at least 10 readings")
        angle_min = float(data["angle_min_deg"])
        increment = float(data["angle_increment_deg"])
        if not math.isfinite(angle_min) or not math.isfinite(increment) or increment <= 0:
            raise ValueError("invalid scan angles")
        if increment * len(ranges) > 720:
            raise ValueError("scan spans more than two revolutions")
        return cls(tuple(float(value) for value in ranges), angle_min, increment)


@dataclass(frozen=True)
class ObstacleStatus:
    state: str
    reason: str
    stop_requested: bool
    segment_id: str | None
    distance_m: float | None

    def to_dict(self) -> dict:
        return {"type": "obstacle", **asdict(self)}


class SafetyMonitor:
    def __init__(
        self,
        robot_width_m: float = 0.24,
        side_margin_m: float = 0.03,
        stop_distance_m: float = 0.35,
        stale_after_s: float = 0.3,
        blocked_scans: int = 3,
        min_valid_points: int = 10,
    ) -> None:
        if robot_width_m <= 0 or side_margin_m < 0 or stop_distance_m <= 0:
            raise ValueError("invalid safety geometry")
        if stale_after_s <= 0 or blocked_scans < 1 or min_valid_points < 1:
            raise ValueError("invalid safety timing")
        self.half_corridor = robot_width_m / 2 + side_margin_m
        self.stop_distance_m = stop_distance_m
        self.stale_after_s = stale_after_s
        self.blocked_scans = blocked_scans
        self.min_valid_points = min_valid_points
        self.last_scan_at: float | None = None
        self.obstructed_scans = 0
        self.status = ObstacleStatus("STOP", "waiting_for_scan", True, None, None)

    def ingest(self, scan: Scan, now: float, segment_id: str | None = None) -> ObstacleStatus:
        valid_points = 0
        closest: float | None = None
        for index, distance in enumerate(scan.ranges_m):
            if not math.isfinite(distance) or not 0.05 <= distance <= 8.0:
                continue
            valid_points += 1
            angle = math.radians(scan.angle_min_deg + index * scan.angle_increment_deg)
            forward = distance * math.cos(angle)
            lateral = distance * math.sin(angle)
            if forward > 0 and abs(lateral) <= self.half_corridor:
                closest = forward if closest is None else min(closest, forward)

        self.last_scan_at = now
        if valid_points < self.min_valid_points:
            self.obstructed_scans = 0
            self.status = ObstacleStatus("STOP", "invalid_scan", True, segment_id, None)
        elif closest is not None and closest <= self.stop_distance_m:
            self.obstructed_scans += 1
            state = "BLOCKED" if self.obstructed_scans >= self.blocked_scans else "STOP"
            reason = "route_blocked" if state == "BLOCKED" else "obstacle_ahead"
            self.status = ObstacleStatus(state, reason, True, segment_id, round(closest, 3))
        else:
            self.obstructed_scans = 0
            self.status = ObstacleStatus("CLEAR", "path_clear", False, segment_id, None)
        return self.status

    def tick(self, now: float) -> ObstacleStatus:
        if self.last_scan_at is None or now - self.last_scan_at > self.stale_after_s:
            self.obstructed_scans = 0
            self.status = ObstacleStatus(
                "STOP", "stale_scan" if self.last_scan_at is not None else "waiting_for_scan",
                True, self.status.segment_id, None,
            )
        return self.status
