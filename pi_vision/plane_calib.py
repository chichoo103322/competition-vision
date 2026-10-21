"""平面标定的核心算法：棋盘网格编号与像素→世界单应拟合。

供 tools/calibrate_plane.py 使用，独立成模块便于单元测试。

坐标系约定：输入角点是标定画面（与运行时同一条相机管线，即
pi_vision.camera.Camera 输出的帧）上的像素坐标；世界坐标单位由调用方
给定的方格边长与原点决定（通常为 mm）。
"""

from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np


def assign_grid_indices(
    corners: np.ndarray,
    origin_idx: int,
    x_idx: int,
    y_idx: int,
    cols: int,
    rows: int,
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """根据三个锚点确定每个棋盘角点的 (col, row) 网格编号。

    corners 必须是 cv2.findChessboardCorners 的行优先顺序输出
    （长度 cols*rows）。三个锚点由用户点选（点选会吸附到最近角点）：
      origin_idx —— 原点角点（世界系原点）
      x_idx      —— 与原点同行、+X 方向的远端角点
      y_idx      —— 与原点同列、+Y 方向的远端角点

    返回值：两个整数数组 (cols_arr, rows_arr)，第 i 个元素是 corners[i]
    的网格编号，且原点恒为 (0,0)、x 锚点恒为 (cols-1,0)、y 锚点恒为
    (0,rows-1)。锚点不构成合法网格（点重、方向点错、数量不符）时
    返回 None。
    """
    cols = int(cols)
    rows = int(rows)
    if cols < 2 or rows < 2 or len(corners) != cols * rows:
        return None
    origin_idx = int(origin_idx)
    x_idx = int(x_idx)
    y_idx = int(y_idx)
    if not (
        0 <= origin_idx < len(corners)
        and 0 <= x_idx < len(corners)
        and 0 <= y_idx < len(corners)
    ):
        return None

    base = np.arange(len(corners))

    def check(col_arr, row_arr):
        co, ro = int(col_arr[origin_idx]), int(row_arr[origin_idx])
        if co not in (0, cols - 1) or ro not in (0, rows - 1):
            return None
        # +X 锚点：与原点同行、列方向的对侧远端
        if not (
            int(row_arr[x_idx]) == ro
            and int(col_arr[x_idx]) == (0 if co == cols - 1 else cols - 1)
        ):
            return None
        # +Y 锚点：与原点同列、行方向的对侧远端
        if not (
            int(col_arr[y_idx]) == co
            and int(row_arr[y_idx]) == (0 if ro == rows - 1 else rows - 1)
        ):
            return None
        # 归一化：原点移到 (0,0)，X/Y 朝向锚点方向
        if co == cols - 1:
            col_arr = cols - 1 - col_arr
        if ro == rows - 1:
            row_arr = rows - 1 - row_arr
        return col_arr, row_arr

    # findChessboardCorners 行优先：stride-1 的轴可能是棋盘行也可能是
    # 棋盘列，再加上行列方向的反转，共 2x4=8 种网格朝向，逐一验证锚点。
    for col_arr, row_arr in (
        (base % cols, base // cols),
        (base // rows, base % rows),
    ):
        for flip_col in (False, True):
            for flip_row in (False, True):
                c = cols - 1 - col_arr if flip_col else col_arr
                r = rows - 1 - row_arr if flip_row else row_arr
                result = check(c, r)
                if result is not None:
                    return result
    return None


def build_world_points(
    grid_cols: np.ndarray,
    grid_rows: np.ndarray,
    square_mm: float,
    origin_x_mm: float,
    origin_y_mm: float,
) -> np.ndarray:
    """把网格编号转成世界坐标 (N,2)：X 沿列方向、Y 沿行方向，单位 mm。"""
    grid_cols = np.asarray(grid_cols, dtype=np.float64)
    grid_rows = np.asarray(grid_rows, dtype=np.float64)
    world = np.empty((len(grid_cols), 2), dtype=np.float32)
    world[:, 0] = origin_x_mm + grid_cols * square_mm
    world[:, 1] = origin_y_mm + grid_rows * square_mm
    return world


def fit_plane_homography(
    px_points: np.ndarray,
    world_points: np.ndarray,
    ransac_reproj: float = 3.0,
) -> Tuple[Optional[np.ndarray], float, float, str]:
    """用 RANSAC 拟合像素→世界单应，返回 (H, 平均误差, 最大误差, 错误信息)。

    失败时 H 为 None、error 非空；误差单位为世界坐标单位（mm）。
    """
    px = np.asarray(px_points, dtype=np.float32).reshape(-1, 2)
    world = np.asarray(world_points, dtype=np.float32).reshape(-1, 2)
    if len(px) < 4 or len(px) != len(world):
        return None, 0.0, 0.0, "角点数量不足"
    h, _ = cv2.findHomography(px, world, cv2.RANSAC, ransac_reproj)
    if h is None:
        return None, 0.0, 0.0, "单应计算失败"
    proj = cv2.perspectiveTransform(px.reshape(-1, 1, 2), h).reshape(-1, 2)
    err = np.linalg.norm(proj - world, axis=1)
    return h, float(err.mean()), float(err.max()), ""
