"""Decode the preliminary-round QR payload into a transport plan."""

from __future__ import annotations

import re
import math
from dataclasses import asdict, dataclass

import cv2
import numpy as np


COLORS = {
    "1": "red",
    "2": "yellow",
    "3": "blue",
    "4": "green",
    "5": "black",
    "6": "light_blue",
}


class InvalidTaskCode(ValueError):
    """The QR payload is not a valid preliminary-round task."""


@dataclass(frozen=True)
class Move:
    order: int
    color_code: int
    color: str
    rough_position: int
    storage_position: int


@dataclass(frozen=True)
class TaskPlan:
    raw: str
    first_batch: tuple[Move, ...]
    second_batch: tuple[Move, ...]

    def to_dict(self) -> dict:
        return asdict(self)


def parse_task_code(raw: str) -> TaskPlan:
    code = raw.strip()
    if not re.fullmatch(r"[1-6]{3}\+[1-3]{3}\+[1-6]{3}\+[1-3]{3}", code):
        raise InvalidTaskCode("expected CCC+PPP+CCC+PPP")

    first_colors, first_positions, second_colors, second_positions = code.split("+")
    if len(set(first_colors)) != 3 or set(second_colors) != set(first_colors):
        raise InvalidTaskCode("each batch must use the same three distinct colors")
    if set(first_positions) != set("123") or set(second_positions) != set("123"):
        raise InvalidTaskCode("each position group must be a permutation of 1, 2, 3")

    storage_by_color = dict(zip(first_colors, first_positions))
    first = tuple(
        Move(i + 1, int(color), COLORS[color], int(first_positions[i]), int(first_positions[i]))
        for i, color in enumerate(first_colors)
    )
    second = tuple(
        Move(i + 1, int(color), COLORS[color], int(second_positions[i]), int(storage_by_color[color]))
        for i, color in enumerate(second_colors)
    )
    return TaskPlan(code, first, second)


@dataclass(frozen=True)
class TaskObservation:
    status: str
    plan: TaskPlan | None = None


class TaskCodeDecoder:
    """Reuse one OpenCV detector; never choose between conflicting QR payloads."""

    def __init__(self) -> None:
        self.detector = cv2.QRCodeDetector()

    def decode(
        self, frame: np.ndarray, roi: tuple[int, int, int, int] | None = None
    ) -> TaskObservation:
        if frame is None or frame.size == 0:
            return TaskObservation("empty_frame")
        height, width = frame.shape[:2]
        if roi is not None:
            x, y, w, h = roi
            if x < 0 or y < 0 or w <= 0 or h <= 0 or x + w > width or y + h > height:
                raise ValueError("ROI must fit entirely inside the image")
            frame = frame[y:y + h, x:x + w]

        try:
            _, payloads, _, _ = self.detector.detectAndDecodeMulti(frame)
            payloads = set(payloads) - {""}
            if not payloads:
                payload, _, _ = self.detector.detectAndDecode(frame)
                payloads = {payload} - {""}
        except UnicodeError:
            return TaskObservation("invalid_payload")
        if not payloads:
            return TaskObservation("not_found")
        if len(payloads) != 1:
            return TaskObservation("ambiguous")
        try:
            return TaskObservation("candidate", parse_task_code(payloads.pop()))
        except InvalidTaskCode:
            return TaskObservation("invalid_payload")


class TaskConfirmation:
    """Latch a plan after consecutive matching observations over a time interval.

    Each update must describe a newly acquired frame, with a strictly increasing
    monotonic timestamp in seconds. Missing/invalid observations, a different
    code, or an excessive inter-frame gap restart confirmation. Once locked,
    the plan is immutable until the caller explicitly resets for a new round.
    """

    def __init__(
        self, frames: int = 3, min_seconds: float = 0.2, max_gap: float = 1.0
    ) -> None:
        if isinstance(frames, bool) or not isinstance(frames, int) or frames < 1:
            raise ValueError("confirmation frames must be a positive integer")
        if not math.isfinite(min_seconds) or min_seconds < 0:
            raise ValueError("minimum confirmation seconds must be finite and nonnegative")
        if not math.isfinite(max_gap) or max_gap <= 0:
            raise ValueError("maximum frame gap must be finite and positive")
        self.frames = frames
        self.min_seconds = min_seconds
        self.max_gap = max_gap
        self.reset()

    def reset(self) -> None:
        self.locked_plan: TaskPlan | None = None
        self.candidate: TaskPlan | None = None
        self.count = 0
        self._started_at: float | None = None
        self._last_at: float | None = None

    def update(self, plan: TaskPlan | None, now: float) -> TaskPlan | None:
        if not math.isfinite(now):
            raise ValueError("frame timestamp must be finite")
        if self.locked_plan is not None:
            return self.locked_plan
        if self._last_at is not None and now <= self._last_at:
            raise ValueError("frame timestamps must strictly increase")
        gap = self._last_at is not None and now - self._last_at > self.max_gap
        self._last_at = now
        if plan is None:
            self.candidate = None
            self.count = 0
            self._started_at = None
            return None
        if gap or self.candidate is None or self.candidate.raw != plan.raw:
            self.candidate = plan
            self.count = 1
            self._started_at = now
        else:
            self.count += 1
        if self.count >= self.frames and now - self._started_at >= self.min_seconds:
            self.locked_plan = plan
        return self.locked_plan


def decode_task_frame(frame: np.ndarray) -> TaskPlan | None:
    """Single-image inspection only; use TaskConfirmation before executing a task."""
    return TaskCodeDecoder().decode(frame).plan
