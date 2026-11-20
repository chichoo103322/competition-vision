"""Numbered placement rings measured from independent circular edge layers."""
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .detectors.ring_detector import RingDetector
from .position_digits import DigitReader, NumberedRing, normalize


class PlacementRings:
    def __init__(self, resolution):
        self.resolution = tuple(resolution)
        # Hough only proposes a location. A shadow can hide a layer or fail
        # the center brightness check while several visible arcs still fit.
        self.detector = RingDetector(min_rings_per_cluster=1, hollow_check=False,
                                     center_tolerance=22)
        # The production reader retains its original templates and thresholds.
        self.reader = DigitReader(0)
        for directory in (
            Path('/usr/share/fonts/truetype/dejavu'),
            Path('/usr/share/fonts/truetype/liberation'),
            Path('/usr/share/fonts/truetype/liberation2'),
            Path('/System/Library/Fonts/Supplemental'),
        ):
            for name in ('DejaVuSans.ttf', 'LiberationSans-Regular.ttf', 'Arial.ttf'):
                path = directory / name
                if not path.is_file():
                    continue
                font = ImageFont.truetype(str(path), 64)
                for digit in self.reader.templates:
                    for stroke in (0, 1, 2):
                        canvas = Image.new('L', (96, 96))
                        ImageDraw.Draw(canvas).text((10, 4), str(digit), font=font,
                                                     fill=255, stroke_width=stroke)
                        self.reader.templates[digit].append(normalize(cv2.threshold(np.asarray(canvas), 128, 255, cv2.THRESH_BINARY)[1]))

    def detect(self, frame):
        if (frame.shape[1], frame.shape[0]) != self.resolution:
            raise ValueError('相机实际分辨率与放置测试配置不一致')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 45, 130)
        result = []
        for ring in self.detector.detect(frame):
            refined = fit_ring_center(edges, ring)
            if refined is None:
                continue
            x, y, inner_radius = refined
            ring = replace(ring, center_x=x, center_y=y)
            # Crop out the dark inner ring before matching the printed digit.
            digit, score, margin, orientation = read_inner_digit(self.reader, gray, ring, inner_radius)
            result.append(NumberedRing(
                ring.ring_no, x, y, ring.confidence, ring.radius, digit, score, margin,
                'candidate' if digit else 'unreadable', 'ring_circle_center',
                orientation, 'independent_concentric_circle_fit'))
        return result


def read_inner_digit(reader, gray, ring, inner_radius):
    half = int(inner_radius * .89)
    x, y = round(ring.center_x), round(ring.center_y)
    h, w = gray.shape
    if half < 10 or min(x, y, w-1-x, h-1-y) < half:
        return None, 0., 0., None
    crop = gray[y-half:y+half+1, x-half:x+half+1].copy()
    yy, xx = np.ogrid[-half:half+1, -half:half+1]
    background = int(np.percentile(crop, 80))
    crop[xx**2 + yy**2 > (inner_radius*.89)**2] = background
    # Pad so the existing reader's line removal and border tests keep full glyphs.
    crop = cv2.copyMakeBorder(crop, 5, 5, 5, 5, cv2.BORDER_CONSTANT, value=background)
    center = half + 5
    crop_ring = replace(ring, center_x=center, center_y=center, radius=(center+.1)/.55)
    upright = reader.read_detail(crop, crop_ring)
    if upright[0] is not None:
        return upright
    # Digits turn with the camera during arm yaw. The base reader already
    # searches right angles, so only cover the remaining ±45°.
    matches = {}
    for angle in (-45, -30, -15, 15, 30, 45):
        transform = cv2.getRotationMatrix2D((center, center), angle, 1)
        rotated = cv2.warpAffine(crop, transform, crop.shape[::-1], borderValue=background)
        digit, score, margin, orientation = reader.read_detail(rotated, crop_ring)
        if digit is not None and (digit not in matches or score > matches[digit][1]):
            matches[digit] = digit, score, margin, (orientation-angle) % 360
    ranked = sorted(matches.values(), key=lambda result: result[1], reverse=True)
    if len(ranked) == 1 or (ranked and ranked[0][1]-ranked[1][1] >= reader.min_margin):
        return ranked[0]
    return upright


def fit_ring_center(edges, ring):
    """Reject disagreement between rings rather than using a coarse center."""
    x, y, radius = ring.center_x, ring.center_y, ring.radius
    h, w = edges.shape
    if radius < 15 or min(x, y, w-1-x, h-1-y) < radius*1.34:
        return None
    angles = np.arange(180) * (2*np.pi/180)
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    step = .75
    radii = np.arange(radius*.66, radius*1.38, step)
    origin = np.array([x, y], dtype=float)
    fits = []
    for iteration in range(2):
        samples = origin + radii[:, None, None]*directions
        xs, ys = np.rint(samples).astype(np.int32).transpose(2, 0, 1)
        support = edges[np.clip(ys, 0, h-1), np.clip(xs, 0, w-1)] != 0
        strength = support.mean(axis=1)
        candidates = np.argsort(-strength)
        fits = []
        for k in candidates:
            if strength[k] < .12:
                break
            if any(abs(radii[k]-candidate) < max(2.5, radius*.035)
                   for candidate, *_ in fits):
                continue
            band = abs(radii-radii[k]) <= max(2.3, radius*(.10 if iteration == 0 else .04))
            nearest = np.argmin(np.where(support & band[:, None],
                                         abs(radii-radii[k])[:, None], np.inf), axis=0)
            angle = np.arange(len(angles))
            valid = support[nearest, angle] & band[nearest]
            if min(v.sum() for v in np.array_split(valid, 4)) < 7:
                continue
            points = np.column_stack((xs[nearest, angle], ys[nearest, angle]))[valid] - origin
            design = np.column_stack((2*points, np.ones(len(points))))
            dx, dy, c = np.linalg.lstsq(design, (points**2).sum(axis=1), rcond=None)[0]
            fit_radius = np.sqrt(max(0., c+dx*dx+dy*dy))
            residual = np.abs(np.linalg.norm(points-[dx, dy], axis=1)-fit_radius)
            if (fit_radius > 0 and np.hypot(dx, dy) <= radius*.18
                    and np.median(residual) <= max(1.45, radius*.023)):
                fits.append((radii[k], origin+[dx, dy], fit_radius))
        if len(fits) < 2:
            return None
        origin = np.median([f[1] for f in fits], axis=0)
    agreed = [(point, r) for _, point, r in fits
              if np.linalg.norm(point-origin) <= max(2.2, radius*.035)]
    if len(agreed) < 2 or np.ptp([r for _, r in agreed]) < radius*.09:
        return None
    center = np.median([point for point, _ in agreed], axis=0)
    return float(center[0]), float(center[1]), float(min(r for _, r in agreed))
