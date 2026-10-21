"""视觉检测器集合。"""

from .color_classifier import ColorClassifier
from .material_detector import MaterialDetector
from .qr_detector import QRDetector
from .ring_detector import RingDetector
from .shape_classifier import ShapeClassifier
from .yolo_material_detector import YoloMaterialDetector

__all__ = [
    "ColorClassifier",
    "MaterialDetector",
    "QRDetector",
    "RingDetector",
    "ShapeClassifier",
    "YoloMaterialDetector",
]
