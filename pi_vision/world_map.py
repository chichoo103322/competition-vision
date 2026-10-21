"""像素坐标到抓取平面世界坐标的转换（平面单应）。

坐标系约定（重要）：
    输入的 (u, v) 必须是「运行时相机管线」输出的帧像素——即 pi_vision.camera.Camera
    读出的帧（已按配置应用分辨率、翻转与去畸变）。单应矩阵 H 由
    tools/calibrate_plane.py 标定，标定工具与主程序使用同一条相机管线，
    因此 H 与运行时像素定义在同一坐标系上。

    调用时不要再传入相机内参/畸变做二次去畸变：整帧已经去畸变过了，
    重复处理会让坐标错位。

    世界坐标单位与标定时输入的方格边长/原点一致（通常为 mm）。
"""

from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np


def _resolve_homography(homography: Sequence[float]) -> np.ndarray:
    """校验并返回 3x3 单应矩阵，非法输入抛 ValueError。"""
    if homography is None or len(homography) != 9:
        raise ValueError("world_homography 需要 9 个元素（3x3 矩阵）")
    h = np.asarray(homography, dtype=np.float64).reshape(3, 3)
    if not np.all(np.isfinite(h)):
        raise ValueError("world_homography 含非有限值")
    if np.linalg.matrix_rank(h) < 3:
        raise ValueError("world_homography 退化（不可逆）")
    return h


def _transform(points: np.ndarray, h: np.ndarray) -> np.ndarray:
    """批量把 (N,2) 像素点映射到世界坐标，返回 (N,2)。"""
    homo = np.hstack([points, np.ones((len(points), 1))])
    mapped = h @ homo.T
    w = mapped[2, :]
    if np.any(np.abs(w) < 1e-12):
        raise ValueError("像素点被映射到无穷远（单应矩阵退化）")
    out = mapped[:2, :] / w
    if not np.all(np.isfinite(out)):
        raise ValueError("世界坐标计算出现非有限值")
    return out.T


def pixel_to_world(
    u: float,
    v: float,
    homography: Sequence[float],
) -> Tuple[float, float]:
    """单个像素点 (u, v) → 世界平面坐标 (X, Y)。"""
    h = _resolve_homography(homography)
    x, y = _transform(np.array([[float(u), float(v)]], dtype=np.float64), h)[0]
    return float(x), float(y)


def pixels_to_world(points, homography: Sequence[float]) -> np.ndarray:
    """批量像素点 (N,2) → 世界坐标 (N,2)，用于一帧内多个目标。"""
    h = _resolve_homography(homography)
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    return _transform(pts, h)


def pixel_to_world_from_config(config, u: float, v: float) -> Tuple[float, float]:
    """从 VisionConfig 读取 camera.world_homography 并转换单个像素点。"""
    return pixel_to_world(u, v, config.camera.world_homography)
