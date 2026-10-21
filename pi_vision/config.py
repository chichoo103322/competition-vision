"""配置数据结构和 YAML 配置加载。"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import List

import yaml


@dataclass
class SerialConfig:
    """串口参数。"""

    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    timeout: float = 0.05
    write_timeout: float = 0.2
    ack_timeout: float = 0.2
    ack_retries: int = 3


@dataclass
class CameraConfig:
    """相机参数。"""

    source: str = "0"
    qr_source: str = ""
    backend: str = "v4l2"
    width: int = 1280
    height: int = 720
    fps: int = 30
    flip: int = 0
    fov_h_deg: float = 60.0
    undistort: bool = True
    calibration_matrix: List[float] = field(default_factory=list)
    distortion_coeffs: List[float] = field(default_factory=list)
    calibration_size: List[int] = field(default_factory=list)
    world_homography: List[float] = field(default_factory=list)


@dataclass
class ColorSpec:
    """一种参赛物料颜色的参考中心，使用 LAB 空间。"""

    color_id: int
    name: str
    lab_center: List[float]
    max_distance: float = 30.0


@dataclass
class TaskConfig:
    """任务码格式。"""

    group_count: int = 4
    group_length: int = 3


@dataclass
class DebugConfig:
    """调试输出配置。"""

    save_dir: str = "debug_frames"
    show: bool = False


@dataclass
class CollectConfig:
    """网页采集工具配置（collect_web.py）。"""

    upload_url: str = ""
    keep_local: bool = False
    upload_timeout: float = 5.0


@dataclass
class DetectionConfig:
    """检测功能开关。"""

    qr: bool = True
    materials: bool = True
    rings: bool = False
    qr_continuous: bool = False
    yolo_model_path: str = "models/best.onnx"
    yolo_confidence: float = 0.3
    min_rings_per_cluster: int = 2
    ring_hough: bool = True
    ring_hough_param2: float = 0.8
    ring_min_radius: float = 25.0
    ring_hollow: bool = True
    ring_hollow_threshold: float = 15.0
    ring_smooth_alpha: float = 0.25
    ring_refine_center: bool = True
    ring_refine_max_offset_ratio: float = 0.35
    material_lost_tol: float = 0.3
    # 启动时识别算法默认状态：false=全部关闭（等电控 0x21 按环节开启），
    # true=全部开启（电控不发开关也照常识别）。启动可用 --algorithms-on/--no-algorithms-on 覆盖。
    algorithms_default_on: bool = False
    rotation_gate: bool = True
    rotation_move_px: float = 40.0
    rotation_settle_frames: int = 2
    rotation_window_frames: int = 3
    rotation_stable_px: float = 6.0


@dataclass
class VisionConfig:
    """整体配置。"""

    serial: SerialConfig = field(default_factory=SerialConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    colors: List[ColorSpec] = field(default_factory=list)
    task: TaskConfig = field(default_factory=TaskConfig)
    debug: DebugConfig = field(default_factory=DebugConfig)
    detection: DetectionConfig = field(default_factory=DetectionConfig)
    collect: CollectConfig = field(default_factory=CollectConfig)
    fixed_distance_cm: int = 20


DEFAULT_COLORS = [
    ColorSpec(1, "红色", [123.0, 202.0, 190.0]),
    ColorSpec(2, "黄色", [217.0, 118.0, 213.0]),
    ColorSpec(3, "蓝色", [132.0, 146.0, 63.0]),
    ColorSpec(4, "绿色", [170.0, 72.0, 180.0]),
    ColorSpec(5, "黑色", [27.0, 128.0, 128.0]),
    ColorSpec(6, "浅蓝", [195.0, 111.0, 102.0]),
]


def default_config() -> VisionConfig:
    """返回带默认颜色的基础配置。"""
    return VisionConfig(colors=list(DEFAULT_COLORS))


def _update_serial(config: SerialConfig, raw: dict) -> None:
    config.port = raw.get("port", config.port)
    config.baudrate = int(raw.get("baudrate", config.baudrate))
    config.timeout = float(raw.get("timeout", config.timeout))
    config.write_timeout = float(raw.get("write_timeout", config.write_timeout))
    config.ack_timeout = float(raw.get("ack_timeout", config.ack_timeout))
    config.ack_retries = int(raw.get("ack_retries", config.ack_retries))


def _update_camera(config: CameraConfig, raw: dict) -> None:
    config.source = str(raw.get("source", config.source))
    config.qr_source = str(raw.get("qr_source", config.qr_source))
    config.backend = str(raw.get("backend", config.backend))
    config.width = int(raw.get("width", config.width))
    config.height = int(raw.get("height", config.height))
    config.fps = int(raw.get("fps", config.fps))
    config.flip = int(raw.get("flip", config.flip))
    config.fov_h_deg = float(raw.get("fov_h_deg", config.fov_h_deg))
    config.undistort = bool(raw.get("undistort", config.undistort))
    config.calibration_matrix = [
        float(v) for v in raw.get("calibration_matrix", config.calibration_matrix)
    ]
    config.distortion_coeffs = [
        float(v) for v in raw.get("distortion_coeffs", config.distortion_coeffs)
    ]
    config.calibration_size = [
        int(v) for v in raw.get("calibration_size", config.calibration_size)
    ]
    config.world_homography = [
        float(v) for v in raw.get("world_homography", config.world_homography)
    ]


def _parse_colors(raw: list) -> List[ColorSpec]:
    colors = []
    for item in raw:
        colors.append(
            ColorSpec(
                color_id=int(item["color_id"]),
                name=str(item.get("name", "")),
                lab_center=[float(v) for v in item["lab_center"]],
                max_distance=float(item.get("max_distance", 30.0)),
            )
        )
    return colors


def load_config(path) -> VisionConfig:
    """从 YAML 文件加载配置，缺失字段使用默认值。"""
    config = default_config()
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))

    if isinstance(raw.get("serial"), dict):
        _update_serial(config.serial, raw["serial"])
    if isinstance(raw.get("camera"), dict):
        _update_camera(config.camera, raw["camera"])
    if isinstance(raw.get("colors"), list):
        config.colors = _parse_colors(raw["colors"])
    if isinstance(raw.get("task"), dict):
        config.task.group_count = int(raw["task"].get("group_count", config.task.group_count))
        config.task.group_length = int(raw["task"].get("group_length", config.task.group_length))
    if isinstance(raw.get("debug"), dict):
        config.debug.save_dir = str(raw["debug"].get("save_dir", config.debug.save_dir))
        config.debug.show = bool(raw["debug"].get("show", config.debug.show))
    if isinstance(raw.get("detection"), dict):
        detection_raw = raw["detection"]
        config.detection.qr = bool(detection_raw.get("qr", config.detection.qr))
        config.detection.materials = bool(
            detection_raw.get("materials", config.detection.materials)
        )
        config.detection.rings = bool(
            detection_raw.get("rings", config.detection.rings)
        )
        config.detection.qr_continuous = bool(
            detection_raw.get("qr_continuous", config.detection.qr_continuous)
        )
        config.detection.yolo_model_path = str(
            detection_raw.get("yolo_model_path", config.detection.yolo_model_path)
        )
        config.detection.yolo_confidence = float(
            detection_raw.get("yolo_confidence", config.detection.yolo_confidence)
        )
        config.detection.min_rings_per_cluster = int(
            detection_raw.get(
                "min_rings_per_cluster", config.detection.min_rings_per_cluster
            )
        )
        config.detection.ring_hough = bool(
            detection_raw.get("ring_hough", config.detection.ring_hough)
        )
        config.detection.ring_hough_param2 = float(
            detection_raw.get(
                "ring_hough_param2", config.detection.ring_hough_param2
            )
        )
        config.detection.ring_min_radius = float(
            detection_raw.get("ring_min_radius", config.detection.ring_min_radius)
        )
        config.detection.ring_hollow = bool(
            detection_raw.get("ring_hollow", config.detection.ring_hollow)
        )
        config.detection.ring_hollow_threshold = float(
            detection_raw.get(
                "ring_hollow_threshold", config.detection.ring_hollow_threshold
            )
        )
        config.detection.ring_smooth_alpha = float(
            detection_raw.get(
                "ring_smooth_alpha", config.detection.ring_smooth_alpha
            )
        )
        config.detection.rotation_gate = bool(
            detection_raw.get("rotation_gate", config.detection.rotation_gate)
        )
        config.detection.rotation_move_px = float(
            detection_raw.get("rotation_move_px", config.detection.rotation_move_px)
        )
        config.detection.rotation_settle_frames = int(
            detection_raw.get(
                "rotation_settle_frames", config.detection.rotation_settle_frames
            )
        )
        config.detection.rotation_window_frames = int(
            detection_raw.get(
                "rotation_window_frames", config.detection.rotation_window_frames
            )
        )
        config.detection.rotation_stable_px = float(
            detection_raw.get(
                "rotation_stable_px", config.detection.rotation_stable_px
            )
        )
        config.detection.material_lost_tol = float(
            detection_raw.get(
                "material_lost_tol", config.detection.material_lost_tol
            )
        )
        config.detection.algorithms_default_on = bool(
            detection_raw.get(
                "algorithms_default_on", config.detection.algorithms_default_on
            )
        )
    if isinstance(raw.get("collect"), dict):
        collect_raw = raw["collect"]
        config.collect.upload_url = str(
            collect_raw.get("upload_url", config.collect.upload_url)
        )
        config.collect.keep_local = bool(
            collect_raw.get("keep_local", config.collect.keep_local)
        )
        config.collect.upload_timeout = float(
            collect_raw.get("upload_timeout", config.collect.upload_timeout)
        )
    config.fixed_distance_cm = int(raw.get("fixed_distance_cm", config.fixed_distance_cm))
    return config


def save_config(config: VisionConfig, path) -> None:
    """把当前配置写回 YAML 文件。"""
    data = {
        "serial": {
            "port": config.serial.port,
            "baudrate": config.serial.baudrate,
            "timeout": config.serial.timeout,
            "write_timeout": config.serial.write_timeout,
            "ack_timeout": config.serial.ack_timeout,
            "ack_retries": config.serial.ack_retries,
        },
        "camera": {
            "source": config.camera.source,
            "qr_source": config.camera.qr_source,
            "backend": config.camera.backend,
            "width": config.camera.width,
            "height": config.camera.height,
            "fps": config.camera.fps,
            "flip": config.camera.flip,
            "fov_h_deg": float(config.camera.fov_h_deg),
            "undistort": bool(config.camera.undistort),
            "calibration_matrix": [float(v) for v in config.camera.calibration_matrix],
            "distortion_coeffs": [float(v) for v in config.camera.distortion_coeffs],
            "calibration_size": [int(v) for v in config.camera.calibration_size],
            "world_homography": [float(v) for v in config.camera.world_homography],
        },
        "colors": [
            {
                "color_id": color.color_id,
                "name": color.name,
                "lab_center": [float(v) for v in color.lab_center],
                "max_distance": float(color.max_distance),
            }
            for color in config.colors
        ],
        "task": {
            "group_count": config.task.group_count,
            "group_length": config.task.group_length,
        },
        "debug": {
            "save_dir": config.debug.save_dir,
            "show": config.debug.show,
        },
        "detection": {
            "qr": config.detection.qr,
            "materials": config.detection.materials,
            "rings": config.detection.rings,
            "qr_continuous": config.detection.qr_continuous,
            "yolo_model_path": config.detection.yolo_model_path,
            "yolo_confidence": float(config.detection.yolo_confidence),
            "min_rings_per_cluster": int(config.detection.min_rings_per_cluster),
            "ring_hough": bool(config.detection.ring_hough),
            "ring_hough_param2": float(config.detection.ring_hough_param2),
            "ring_min_radius": float(config.detection.ring_min_radius),
            "ring_hollow": bool(config.detection.ring_hollow),
            "ring_hollow_threshold": float(config.detection.ring_hollow_threshold),
            "ring_smooth_alpha": float(config.detection.ring_smooth_alpha),
            "rotation_gate": config.detection.rotation_gate,
            "rotation_move_px": float(config.detection.rotation_move_px),
            "rotation_settle_frames": int(config.detection.rotation_settle_frames),
            "rotation_window_frames": int(config.detection.rotation_window_frames),
            "rotation_stable_px": float(config.detection.rotation_stable_px),
            "material_lost_tol": float(config.detection.material_lost_tol),
            "algorithms_default_on": bool(config.detection.algorithms_default_on),
        },
        "collect": {
            "upload_url": config.collect.upload_url,
            "keep_local": bool(config.collect.keep_local),
            "upload_timeout": float(config.collect.upload_timeout),
        },
        "fixed_distance_cm": config.fixed_distance_cm,
    }
    Path(path).write_text(
        yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
