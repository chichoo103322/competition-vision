"""Experimental six-color segmentation for the preliminary-round materials.

Thresholds are based on one camera/lighting setup. This is an explicitly selected
alternative to YOLO, not a general object recognizer or a calibrated grasp point.
"""

import math

import cv2
import numpy as np

from ..types import Material


class HSVMaterialDetector:
    def __init__(self, min_area_ratio=.012, max_area_ratio=.4):
        if not 0 < min_area_ratio < max_area_ratio < 1:
            raise ValueError('invalid contour area ratios')
        self.min_area_ratio = min_area_ratio
        self.max_area_ratio = max_area_ratio

    def detect(self, frame):
        if frame is None or frame.size == 0:
            return []
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
        ranges = {
            1: [((0, 130, 120), (9, 255, 255)), ((170, 130, 120), (179, 255, 255))],
            2: [((20, 70, 100), (34, 255, 255))],
            # Extract a whole blue-family region before assigning its color.
            # Local hue changes across one object must not create two identities.
            3: [((86, 60, 60), (135, 255, 255))],
            4: [((35, 65, 65), (85, 255, 255))],
            5: [((0, 0, 0), (179, 255, 80))],
        }
        height, width = frame.shape[:2]
        area = width * height
        # Scale morphology with image size, preserving shape at different resolutions.
        kernel_size = max(3, int(round(min(width, height)/100)) | 1)
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        close_kernel = np.ones((kernel_size*2-1, kernel_size*2-1), np.uint8)
        found = []
        for color_id, bands in ranges.items():
            mask = np.zeros((height, width), np.uint8)
            for low, high in bands:
                mask |= cv2.inRange(hsv, np.array(low), np.array(high))
            # Closing can bridge a narrow gap between a real blue and cyan pair.
            if color_id != 3:
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            regions = (self._blue_regions(frame, hsv, mask, contours) if color_id == 3
                       else [(color_id, contour) for contour in contours])
            for output_color, contour in regions:
                contour_area = cv2.contourArea(contour)
                if not self.min_area_ratio*area <= contour_area <= self.max_area_ratio*area:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                perimeter = cv2.arcLength(contour, True)
                if perimeter <= 0 or min(w, h) <= 0:
                    continue
                circularity = 4*math.pi*contour_area/(perimeter*perimeter)
                fill = contour_area/(w*h)
                aspect = max(w, h)/min(w, h)
                hull_area = cv2.contourArea(cv2.convexHull(contour))
                solidity = contour_area/hull_area if hull_area else 0
                if circularity < .3 or solidity < .65 or fill < .4 or aspect > 3:
                    continue
                # Purely a contour shape score, never a detection probability.
                score = int(round(min(1., circularity)*100))
                found.append(Material(0, output_color, 0, int(round(x+w/2)),
                                      int(round(y+h/2)), 0., score))
        found.sort(key=lambda item: (item.color_id, item.center_x, item.center_y))
        from dataclasses import replace
        return [replace(item, index=i+1) for i, item in enumerate(found)]

    @staticmethod
    def _blue_regions(frame, hsv, mask, contours):
        """Classify whole regions; split distinct touching blue/cyan cores.

        Same experimental color calibration as the prior component classifier.
        Watershed resolves mixed regions instead of dropping both materials.
        """
        regions = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            component = np.zeros((h, w), np.uint8)
            cv2.drawContours(component, [contour], -1, 255, -1, offset=(-x, -y))
            component &= mask[y:y+h, x:x+w]
            inside = component != 0
            values = hsv[y:y+h, x:x+w]
            samples = values[inside]
            if not len(samples):
                continue
            cyan = inside & (values[:, :, 0] <= 101) & (values[:, :, 1] <= 160) & (values[:, :, 2] >= 170)
            blue = inside & ((values[:, :, 0] >= 106) | (values[:, :, 1] >= 190))
            count = np.count_nonzero(inside)
            if np.count_nonzero(cyan)/count >= .25 and np.count_nonzero(blue)/count >= .25:
                # An enclosed highlight is part of the surrounding object.
                cyan_contours, _ = cv2.findContours(cyan.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                blue_contours, _ = cv2.findContours(blue.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                blue_envelope = np.zeros_like(component)
                cv2.drawContours(blue_envelope, blue_contours, -1, 255, -1)
                if np.all(blue_envelope[cyan] != 0):
                    regions.append((3, contour))
                    continue
                cyan_envelope = np.zeros_like(component)
                cv2.drawContours(cyan_envelope, cyan_contours, -1, 255, -1)
                if np.all(cyan_envelope[blue] != 0):
                    regions.append((6, contour))
                    continue
                markers = np.zeros((h+2, w+2), np.int32)
                markers[:] = 1
                markers[1:-1, 1:-1][inside] = 0
                markers[1:-1, 1:-1][blue] = 2
                markers[1:-1, 1:-1][cyan] = 3
                image = cv2.copyMakeBorder(frame[y:y+h, x:x+w], 1, 1, 1, 1, cv2.BORDER_REPLICATE)
                cv2.watershed(image, markers)
                for marker, color in ((2, 3), (3, 6)):
                    part = ((markers[1:-1, 1:-1] == marker) & inside).astype(np.uint8)*255
                    parts, _ = cv2.findContours(part, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    for part_contour in parts:
                        regions.append((color, part_contour + np.array([[[x, y]]], np.int32)))
                continue
            hue, saturation, value = np.median(samples, axis=0)
            if hue >= 104 or saturation >= 170:
                regions.append((3, contour))
            elif value >= 140:
                regions.append((6, contour))
        return regions
