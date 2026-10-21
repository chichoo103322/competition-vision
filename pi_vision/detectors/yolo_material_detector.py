"""基于 ONNX 的 YOLOv8 物料检测器。"""

from __future__ import annotations

import logging
from typing import List, Optional, Tuple

import cv2
import numpy as np

from ..types import Material

LOGGER = logging.getLogger(__name__)

# 底座：内部区分类别（识别为"底座"而非物料），不作为物料返回/上报/显示。
# 与协议 COLOR[7]='底座'、训练 data.yaml names 里 base 对应。
BASE_COLOR_ID = 7


class YoloMaterialDetector:
    """用 YOLOv8 ONNX 模型检测物料位置和颜色。

    类别顺序固定为：red yellow blue green black light_blue，
    对应颜色编号 1-6。
    """

    def __init__(
        self,
        model_path: str = "",
        confidence: float = 0.3,
        iou_threshold: float = 0.45,
        input_size: int = 320,
    ) -> None:
        self.model_path = model_path
        self.confidence = confidence
        self.iou_threshold = iou_threshold
        self.input_size = input_size
        self._session = None
        self._input_name = ""

    def load(self) -> None:
        """加载 ONNX 模型。"""
        if not self.model_path:
            raise RuntimeError("未配置 YOLO 模型路径")
        try:
            import onnxruntime as ort
        except ImportError:
            LOGGER.warning("未安装 onnxruntime，无法加载 YOLO 模型")
            return
        self._session = ort.InferenceSession(
            self.model_path, providers=["CPUExecutionProvider"]
        )
        self._input_name = self._session.get_inputs()[0].name
        input_shape = self._session.get_inputs()[0].shape
        if len(input_shape) >= 4 and isinstance(input_shape[2], int) and isinstance(input_shape[3], int):
            self.input_size = input_shape[2]
        LOGGER.info("YOLO 模型已加载: %s", self.model_path)

    def detect(self, frame: np.ndarray) -> List[Material]:
        """对一帧图像执行推理，返回物料列表。"""
        if self._session is None:
            return []
        input_tensor, scale, pad_x, pad_y = self._preprocess(frame)
        outputs = self._session.run(None, {self._input_name: input_tensor})
        detections = self._postprocess(outputs[0], scale, pad_x, pad_y)
        # 跨类别重叠去重：同一目标双报（如阴影下蓝+浅蓝）只保留最高置信度颜色
        detections = self._dedupe_overlap(detections)

        materials = []
        for index, (x1, y1, x2, y2, cls_id, conf) in enumerate(detections):
            color_id = int(cls_id) + 1
            # 底座（color_id=7）是内部区分类别：不当作物料返回，
            # 从而不进物料列表/不上报/不显示，保证不误识别成物料。
            if color_id == BASE_COLOR_ID:
                continue
            materials.append(
                Material(
                    index=index + 1,
                    color_id=color_id,
                    shape_id=0,
                    center_x=int(round((x1 + x2) / 2)),
                    center_y=int(round((y1 + y2) / 2)),
                    theta_deg=0.0,
                    confidence=int(round(conf * 100)),
                )
            )
        return materials

    def _preprocess(self, frame: np.ndarray):
        """letterbox 缩放到模型输入尺寸，返回 (tensor, scale, pad_x, pad_y)。"""
        height, width = frame.shape[:2]
        scale = min(self.input_size / height, self.input_size / width)
        new_w = max(1, int(round(width * scale)))
        new_h = max(1, int(round(height * scale)))
        resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        canvas = np.full(
            (self.input_size, self.input_size, 3), 114, dtype=np.uint8
        )
        pad_x = (self.input_size - new_w) // 2
        pad_y = (self.input_size - new_h) // 2
        canvas[pad_y : pad_y + new_h, pad_x : pad_x + new_w] = resized
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        tensor = np.transpose(rgb, (2, 0, 1))[None, ...]
        return tensor, scale, pad_x, pad_y

    def _postprocess(
        self, output: np.ndarray, scale: float, pad_x: int, pad_y: int
    ):
        """解析 YOLOv8 ONNX 输出 [1, 4+nc, N]，做置信度过滤和 NMS。"""
        pred = np.squeeze(output, axis=0)  # (4+nc, N)
        if pred.ndim != 2:
            return []
        pred = pred.T  # (N, 4+nc)
        boxes = pred[:, :4]
        class_scores = pred[:, 4:]
        class_ids = np.argmax(class_scores, axis=1)
        scores = class_scores[np.arange(len(class_scores)), class_ids]

        keep = scores >= self.confidence
        boxes = boxes[keep]
        scores = scores[keep]
        class_ids = class_ids[keep]

        if len(boxes) == 0:
            return []

        x_center, y_center, w, h = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        x1 = (x_center - w / 2 - pad_x) / scale
        y1 = (y_center - h / 2 - pad_y) / scale
        x2 = (x_center + w / 2 - pad_x) / scale
        y2 = (y_center + h / 2 - pad_y) / scale
        candidate = np.stack([x1, y1, x2, y2, class_ids, scores], axis=1)

        picked = self._nms(candidate)
        return [tuple(row) for row in picked]

    def _dedupe_overlap(self, detections, iou_threshold: float = 0.45):
        """跨类别重叠去重：同一目标被多个颜色类别同时检出（如阴影下浅蓝被判蓝、
        蓝+浅蓝双报）时，只保留置信度最高的颜色，消除同一位置双报/多报。"""
        if len(detections) <= 1:
            return detections
        dets = sorted(detections, key=lambda d: -d[5])
        kept = []
        for d in dets:
            dup = False
            for k in kept:
                iou = float(self._iou(np.asarray(d[:4]), np.asarray([k[:4]]))[0])
                if iou >= iou_threshold:
                    dup = True
                    break
            if not dup:
                kept.append(d)
        return kept

    def _nms(self, detections: np.ndarray):
        """按类别分组做 NMS，避免不同颜色的近距离物料互相抑制。"""
        picked = []
        for cls_id in np.unique(detections[:, 4]):
            group = detections[detections[:, 4] == cls_id]
            order = np.argsort(-group[:, 5])
            while len(order) > 0:
                idx = order[0]
                picked.append(group[idx])
                rest = order[1:]
                if len(rest) == 0:
                    break
                iou = self._iou(group[idx], group[rest])
                order = rest[iou < self.iou_threshold]
        return picked

    def _iou(self, a, b) -> np.ndarray:
        x1 = np.maximum(a[0], b[:, 0])
        y1 = np.maximum(a[1], b[:, 1])
        x2 = np.minimum(a[2], b[:, 2])
        y2 = np.minimum(a[3], b[:, 3])
        inter = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
        area_a = (a[2] - a[0]) * (a[3] - a[1])
        area_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
        union = area_a + area_b - inter
        return np.where(union > 0, inter / np.maximum(union, 1e-6), 0.0)
