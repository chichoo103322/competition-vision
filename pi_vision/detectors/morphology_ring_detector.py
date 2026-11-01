"""Opt-in near-planar ring localization: threshold/dilate/blur/erode/centroid.

Parameters are working-image pixels, not values calibrated for a real camera.
Raw concentric edge support rejects disks and marks; no measurement of scoring layers.
"""
from dataclasses import dataclass
import math
import cv2
import numpy as np
from ..types import Ring


@dataclass(frozen=True)
class MorphologyRing(Ring):
    point_kind: str = 'ring_candidate_center'
    center_method: str = 'morphology_external_contour_centroid'
    radial_support_layers: int = 0


class MorphologyRingDetector:
    def __init__(self, *, work_width=640, dilate_iterations=4, erode_iterations=2,
                 min_area_ratio=.002, max_area_ratio=.8):
        if (type(work_width) is not int or work_width < 64 or
                type(dilate_iterations) is not int or not 0 <= dilate_iterations <= 10 or
                type(erode_iterations) is not int or not 0 <= erode_iterations <= 10 or
                not 0 < min_area_ratio < max_area_ratio < 1):
            raise ValueError('invalid morphology ring parameters')
        self.work_width = work_width
        self.dilate_iterations, self.erode_iterations = dilate_iterations, erode_iterations
        self.min_area_ratio, self.max_area_ratio = min_area_ratio, max_area_ratio

    def detect(self, frame, *, diagnostics=None):
        if diagnostics is not None:
            diagnostics.clear()
        if frame is None or frame.size == 0:
            return []
        scale = min(1., self.work_width/frame.shape[1])
        work = cv2.resize(frame, (round(frame.shape[1]*scale), round(frame.shape[0]*scale)))
        gray = cv2.cvtColor(work, cv2.COLOR_BGR2GRAY)
        if int(gray.max())-int(gray.min()) < 30:
            return []
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        dilated = cv2.dilate(binary, kernel, iterations=self.dilate_iterations)
        blurred = cv2.blur(dilated, (3, 3))
        _, smoothed = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)
        eroded = cv2.erode(smoothed, kernel, iterations=self.erode_iterations)
        if diagnostics is not None:
            diagnostics.update(binary=binary, dilated=dilated, blurred=blurred, eroded=eroded)
        edges = cv2.Canny(gray, 50, 150)
        contours, _ = cv2.findContours(eroded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        height, width = gray.shape
        out = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if not self.min_area_ratio*gray.size <= area <= self.max_area_ratio*gray.size:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            perimeter = cv2.arcLength(hull, True)
            if (x <= 1 or y <= 1 or x+w >= width-1 or y+h >= height-1 or
                    max(w, h)/min(w, h) > 1.6 or perimeter <= 0 or
                    4*math.pi*hull_area/perimeter**2 < .8 or
                    area/max(hull_area, 1.) < .8 or area/(w*h) < .6):
                continue
            m = cv2.moments(contour)
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            radius = (w+h)/4
            # Multiple radial peaks must cover several directions in the original edges.
            radii = np.arange(max(5, round(radius*.4)), round(radius)+2)
            angles = np.linspace(0, 2*math.pi, 96, endpoint=False)
            xs = np.rint(cx+radii[:, None]*np.cos(angles)).astype(int)
            ys = np.rint(cy+radii[:, None]*np.sin(angles)).astype(int)
            support = (edges[np.clip(ys, 0, height-1), np.clip(xs, 0, width-1)] > 0).mean(axis=1)
            groups = []
            for r, value in zip(radii, support):
                if value >= .25:
                    if not groups or r-groups[-1][-1] > max(2., radius*.03):
                        groups.append([])
                    groups[-1].append(r)
            if len(groups) < 3:
                continue
            out.append(MorphologyRing(1, round(cx/scale), round(cy/scale),
                                      90, float(np.median([np.mean(g) for g in groups]))/scale,
                                      radial_support_layers=len(groups)))
        return sorted(out, key=lambda r: (r.center_x, r.center_y))


class RingRegionDetector:
    """Crop before localization; always restore original full-frame coordinates."""
    def __init__(self, detector, roi):
        if len(roi) != 4 or any(type(v) is not int for v in roi):
            raise ValueError('ring ROI must be four integers')
        x, y, w, h = roi
        if min(x, y) < 0 or min(w, h) <= 0:
            raise ValueError('invalid ring ROI')
        self.detector, self.roi = detector, tuple(roi)

    def detect(self, frame):
        from dataclasses import replace
        x, y, w, h = self.roi
        if x+w > frame.shape[1] or y+h > frame.shape[0]:
            raise ValueError('ring ROI exceeds image')
        return [replace(r, center_x=r.center_x+x, center_y=r.center_y+y)
                for r in self.detector.detect(frame[y:y+h, x:x+w])]
