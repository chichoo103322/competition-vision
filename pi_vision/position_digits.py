"""Opt-in ring-center digit candidates, never left-to-right position numbering.

Template similarity is a shape score, not an OCR probability. Actual printed
fonts, perspective and occlusion require field validation before actuation.
"""

from dataclasses import dataclass
import math
import cv2
import numpy as np
from .types import Ring


def normalize(mask):
    ys, xs = np.where(mask > 0)
    if not len(xs):
        return None
    crop = mask[ys.min():ys.max()+1, xs.min():xs.max()+1]
    scale = min(28/crop.shape[1], 36/crop.shape[0])
    resized = cv2.resize(crop, (max(1, round(crop.shape[1]*scale)),
                                max(1, round(crop.shape[0]*scale))), interpolation=cv2.INTER_NEAREST)
    out = np.zeros((44, 36), np.uint8)
    y, x = (44-resized.shape[0])//2, (36-resized.shape[1])//2
    out[y:y+resized.shape[0], x:x+resized.shape[1]] = resized
    return out > 0


@dataclass(frozen=True)
class NumberedRing(Ring):
    position_id: int | None = None
    digit_score: float = 0.
    digit_margin: float = 0.
    position_status: str = 'unreadable'
    point_kind: str = 'ring_candidate_center'
    digit_orientation_deg: int | None = None


class DigitReader:
    def __init__(self, orientation_degrees, *, min_score=.78, min_margin=.08):
        if type(orientation_degrees) is not int or orientation_degrees not in (0, 90, 180, 270):
            raise ValueError('explicit digit correction orientation 0/90/180/270 required')
        if not 0 < min_score <= 1 or not 0 < min_margin < 1:
            raise ValueError('invalid digit similarity limits')
        self.orientation_degrees = orientation_degrees
        self.min_score, self.min_margin = min_score, min_margin
        self.templates = {digit: [] for digit in (1, 2, 3)}
        for digit in self.templates:
            for font in (cv2.FONT_HERSHEY_SIMPLEX, cv2.FONT_HERSHEY_DUPLEX,
                         cv2.FONT_HERSHEY_COMPLEX, cv2.FONT_HERSHEY_TRIPLEX):
                for scale in (1., 1.5, 2., 2.5):
                    for thickness in (2, 3, 4):
                        image = np.zeros((110, 110), np.uint8)
                        cv2.putText(image, str(digit), (12, 85), font, scale, 255, thickness, cv2.LINE_8)
                        self.templates[digit].append(normalize(image))

    @staticmethod
    def _remove_plan_dimension_lines(binary):
        height, width = binary.shape
        horizontal = cv2.getStructuringElement(cv2.MORPH_RECT, (max(9, round(width*.62)), 1))
        vertical = cv2.getStructuringElement(cv2.MORPH_RECT, (1, max(9, round(height*.82))))
        lines = cv2.morphologyEx(binary, cv2.MORPH_OPEN, horizontal)
        lines |= cv2.morphologyEx(binary, cv2.MORPH_OPEN, vertical)
        return cv2.subtract(binary, lines)

    def read(self, frame, ring):
        digit, score, margin, _ = self.read_detail(frame, ring)
        return digit, score, margin

    def read_detail(self, frame, ring):
        cx, cy, radius = ring.center_x, ring.center_y, ring.radius
        if not all(math.isfinite(v) for v in (cx, cy, radius)) or radius < 15:
            return None, 0., 0., None
        half = int(radius*.55)
        x, y = int(round(cx)), int(round(cy))
        h, w = frame.shape[:2]
        if x-half < 0 or y-half < 0 or x+half >= w or y+half >= h:
            return None, 0., 0., None
        crop = frame[y-half:y+half+1, x-half:x+half+1]
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY) if crop.ndim == 3 else crop
        if int(gray.max())-int(gray.min()) < 30:
            return None, 0., 0., None
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        binary = self._remove_plan_dimension_lines(binary)
        n, labels, stats, _ = cv2.connectedComponentsWithStats(binary, 8)
        components = sorted(range(1, n), key=lambda i: stats[i, cv2.CC_STAT_AREA], reverse=True)
        if not components:
            return None, 0., 0., None
        index = components[0]
        bx, by, bw, bh, area = map(int, stats[index])
        if (area < max(8, binary.size*.003) or area > binary.size*.45
                or bh < max(8, radius*.15) or bh > gray.shape[0]*.85
                or bw < 2 or bx <= 1 or by <= 1
                or bx+bw >= gray.shape[1]-1 or by+bh >= gray.shape[0]-1
                or len(components) > 1 and stats[components[1], cv2.CC_STAT_AREA] > area*.2):
            return None, 0., 0., None
        component = np.where(labels == index, 255, 0).astype(np.uint8)
        scores = {digit: (0., None) for digit in self.templates}
        for offset in (0, 90, 180, 270):
            orientation = (self.orientation_degrees+offset) % 360
            glyph = normalize(np.rot90(component, k=-(orientation//90)).copy())
            for digit, templates in self.templates.items():
                score = max(2.*np.count_nonzero(glyph & template)/
                            (np.count_nonzero(glyph)+np.count_nonzero(template))
                            for template in templates)
                if score > scores[digit][0]:
                    scores[digit] = (score, orientation)
        ranked = sorted(scores, key=lambda digit: scores[digit][0], reverse=True)
        score, orientation = scores[ranked[0]]
        margin = score-scores[ranked[1]][0]
        accepted = score >= self.min_score and margin >= self.min_margin
        return ranked[0] if accepted else None, float(score), float(margin), orientation if accepted else None


class NumberedRingDetector:
    position_identity_enabled = True

    def __init__(self, detector, orientation_degrees):
        self.detector = detector
        self.reader = DigitReader(orientation_degrees)

    def detect(self, frame):
        out = []
        for ring in self.detector.detect(frame):
            number, score, margin, orientation = self.reader.read_detail(frame, ring)
            out.append(NumberedRing(ring.ring_no, ring.center_x, ring.center_y,
                ring.confidence, ring.radius, number, score, margin,
                'candidate' if number else 'unreadable',
                'ring_candidate_center', orientation))
        ids = [r.position_id for r in out if r.position_id is not None]
        if len(ids) != len(set(ids)):
            from dataclasses import replace
            out = [replace(r, position_id=None, position_status='duplicate_digit_conflict') for r in out]
        return out
