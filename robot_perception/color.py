"""Calibratable HSV material detection and multi-frame confirmation."""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
import json
from pathlib import Path

import cv2
import numpy as np


DEFAULT_RANGES: dict[str, list[list[int]]] = {
    "red": [[0, 12, 90, 55, 255, 255], [170, 179, 90, 55, 255, 255]],
    "yellow": [[18, 38, 85, 65, 255, 255]],
    "green": [[40, 85, 65, 45, 255, 255]],
    "blue": [[95, 132, 140, 40, 255, 255]],
    "light_blue": [[85, 112, 45, 75, 139, 255]],
    "black": [[0, 179, 0, 0, 255, 72]],
}


@dataclass(frozen=True)
class Detection:
    color: str
    x: int
    y: int
    confidence: float
    area_px: int

    def to_dict(self) -> dict:
        return asdict(self)


def load_ranges(path: str | Path | None = None) -> dict[str, list[list[int]]]:
    if path is None:
        return DEFAULT_RANGES
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if set(data) != set(DEFAULT_RANGES):
        raise ValueError("color profile must define all six colors")
    for color, bands in data.items():
        if not bands or any(len(band) != 6 for band in bands):
            raise ValueError(f"invalid HSV bands for {color}")
        for h_low, h_high, s_low, v_low, s_high, v_high in bands:
            if not (0 <= h_low <= h_high <= 179 and 0 <= s_low <= s_high <= 255
                    and 0 <= v_low <= v_high <= 255):
                raise ValueError(f"invalid HSV limits for {color}")
    return data


class ColorDetector:
    def __init__(
        self,
        ranges: dict[str, list[list[int]]] | None = None,
        min_area_px: int = 250,
        min_confidence: float = 0.35,
    ) -> None:
        self.ranges = ranges or DEFAULT_RANGES
        self.min_area_px = min_area_px
        self.min_confidence = min_confidence

    def detect(
        self, frame: np.ndarray, roi: tuple[int, int, int, int] | None = None
    ) -> list[Detection]:
        if frame is None or frame.size == 0:
            return []
        height, width = frame.shape[:2]
        x0, y0, roi_width, roi_height = roi or (0, 0, width, height)
        if x0 < 0 or y0 < 0 or roi_width <= 0 or roi_height <= 0 \
                or x0 + roi_width > width or y0 + roi_height > height:
            raise ValueError("ROI must fit entirely inside the image")
        crop = frame[y0:y0 + roi_height, x0:x0 + roi_width]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        kernel = np.ones((3, 3), np.uint8)
        found: list[Detection] = []

        for color, bands in self.ranges.items():
            mask = np.zeros(hsv.shape[:2], np.uint8)
            for h_low, h_high, s_low, v_low, s_high, v_high in bands:
                mask |= cv2.inRange(
                    hsv, (h_low, s_low, v_low), (h_high, s_high, v_high)
                )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = int(cv2.contourArea(contour))
                if area < self.min_area_px:
                    continue
                box_x, box_y, box_w, box_h = cv2.boundingRect(contour)
                fill = area / max(1, box_w * box_h)
                confidence = round(min(1.0, fill), 3)
                if confidence < self.min_confidence:
                    continue
                moments = cv2.moments(contour)
                if moments["m00"] == 0:
                    continue
                found.append(Detection(
                    color,
                    x0 + round(moments["m10"] / moments["m00"]),
                    y0 + round(moments["m01"] / moments["m00"]),
                    confidence,
                    area,
                ))
        return sorted(found, key=lambda detection: detection.area_px, reverse=True)


class ColorConfirmation:
    def __init__(self, frames: int = 3) -> None:
        if frames < 1:
            raise ValueError("frames must be positive")
        self.history: deque[Detection | None] = deque(maxlen=frames)
        self.frames = frames

    def update(self, detections: list[Detection], target: str) -> Detection | None:
        match = next((item for item in detections if item.color == target), None)
        self.history.append(match)
        if len(self.history) == self.frames and all(self.history):
            return match
        return None
