"""在限定区域内检测物料，输出仍使用完整相机画面的像素坐标。"""

from dataclasses import replace


def ordered_materials(materials):
    """颜色优先的确定顺序；不把模型分数排名当成物料身份。"""
    ordered = sorted(materials, key=lambda item: (
        item.color_id, item.center_x, item.center_y))
    return [replace(item, index=index + 1) for index, item in enumerate(ordered)]


class MaterialRegionDetector:
    def __init__(self, detector, roi):
        if len(roi) != 4 or any(isinstance(v, bool) or not isinstance(v, int) for v in roi):
            raise ValueError("物料识别区域须为四个整数：X Y 宽 高")
        x, y, width, height = roi
        if x < 0 or y < 0 or width <= 0 or height <= 0:
            raise ValueError("物料识别区域位置须非负，宽高须为正数")
        self.detector = detector
        self.roi = tuple(roi)

    def detect(self, frame):
        if frame is None or frame.size == 0:
            return []
        x, y, width, height = self.roi
        image_height, image_width = frame.shape[:2]
        if x + width > image_width or y + height > image_height:
            raise ValueError(f"物料识别区域 {self.roi} 超出实际画面 {image_width}×{image_height}")
        crop = frame[y:y + height, x:x + width]
        found = self.detector.detect(crop)
        # 边界外中心不能成为可执行定位结果。
        return ordered_materials([
            replace(item, center_x=item.center_x + x, center_y=item.center_y + y,
                    selection_center_x=item.selection_center_x+x if item.selection_center_x is not None else None,
                    selection_center_y=item.selection_center_y+y if item.selection_center_y is not None else None)
            for item in found
            if 0 <= item.center_x < width and 0 <= item.center_y < height
        ])


def center_material(materials, window, frame_size, ambiguity_px=12.):
    """Choose by distance within a fixed observation window; never use outside targets."""
    import math
    if len(window) != 4 or any(type(v) is not int for v in window):
        raise ValueError('target window must be four integers')
    x, y, w, h = window
    width, height = frame_size
    if min(x, y) < 0 or min(w, h) <= 0 or x+w > width or y+h > height:
        raise ValueError('target window exceeds frame')
    if not math.isfinite(ambiguity_px) or ambiguity_px < 0:
        raise ValueError('invalid ambiguity margin')
    candidates = []
    for m in materials:
        # Select the object before considering top-face success. Never switch to a side object.
        mx = m.selection_center_x if m.selection_center_x is not None else m.center_x
        my = m.selection_center_y if m.selection_center_y is not None else m.center_y
        if x <= mx < x+w and y <= my < y+h:
            candidates.append((math.hypot(mx-(x+w/2), my-(y+h/2)), m))
    candidates.sort(key=lambda pair: pair[0])
    if not candidates or len(candidates) > 1 and candidates[1][0]-candidates[0][0] <= ambiguity_px:
        return []
    return ordered_materials([candidates[0][1]])
