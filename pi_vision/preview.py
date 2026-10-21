"""实时预览调试工具：自己控制检测开关、灵敏度和截图保存。"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np

from .camera import Camera
from .config import CameraConfig, VisionConfig, load_config, save_config
from .pipeline import VisionPipeline


def draw_qr(canvas, result) -> None:
    """框出二维码并在旁边显示识别到的任务码。"""
    if result is None:
        return
    code, points = result
    pts = np.asarray(points, dtype=np.int32).reshape(-1, 2)
    cv2.polylines(canvas, [pts], True, (0, 255, 0), 3)

    label = f"任务码: {code}"
    font = cv2.FONT_HERSHEY_SIMPLEX
    (text_w, text_h), baseline = cv2.getTextSize(label, font, 0.7, 2)
    x0 = int(pts[0][0]) + 12
    y0 = max(text_h + 8, int(pts[0][1]) - 12)
    x0 = max(4, min(x0, canvas.shape[1] - text_w - 8))
    cv2.rectangle(
        canvas,
        (x0 - 6, y0 - text_h - 6),
        (x0 + text_w + 6, y0 + baseline + 6),
        (0, 0, 0),
        -1,
    )
    cv2.putText(canvas, label, (x0, y0), font, 0.7, (0, 255, 0), 2)


def draw_materials(canvas, materials) -> None:
    for item in materials:
        cv2.circle(canvas, (item.center_x, item.center_y), 4, (0, 255, 0), -1)
        cv2.putText(
            canvas,
            f"C{item.color_id}",
            (item.center_x + 8, item.center_y - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )


def draw_rings(canvas, rings) -> None:
    for item in rings:
        cv2.circle(canvas, (item.center_x, item.center_y), 4, (255, 0, 0), -1)
        if item.radius > 0:
            cv2.circle(
                canvas,
                (item.center_x, item.center_y),
                int(round(item.radius)),
                (255, 0, 0),
                1,
            )
        cv2.putText(
            canvas,
            f"R{item.ring_no}",
            (item.center_x + 8, item.center_y - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            1,
        )


def draw_status(
    canvas,
    code,
    camera_source,
    show_materials: bool,
    show_rings: bool,
    distance_scale: float,
    calibration_hint: str,
) -> None:
    status = (
        f"任务码: {code or '-'} | 摄像头: {camera_source} | 灵敏度: {distance_scale:.2f} | "
        f"物料: {'开' if show_materials else '关'} | "
        f"圆环: {'开' if show_rings else '关'}"
    )
    if calibration_hint:
        status += f" | {calibration_hint}"
    cv2.putText(
        canvas,
        status,
        (12, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
    )
    cv2.putText(
        canvas,
        "q退出 | c切换摄像头 | 1-6记录颜色 | x保存校准 | m物料 | r圆环 | s截图 | [ ]灵敏度",
        (12, canvas.shape[0] - 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )


def detect_cameras(config: CameraConfig, max_index: int = 5) -> list:
    """自动探测本机可用的摄像头编号。"""
    found = []
    for index in range(max_index + 1):
        probe_config = CameraConfig(
            source=str(index),
            backend=config.backend,
            width=320,
            height=240,
            fps=30,
            flip=0,
        )
        camera = Camera(probe_config)
        try:
            camera.open()
            ok, frame, _ = camera.read()
            if ok and frame is not None:
                found.append(index)
        except Exception:
            pass
        finally:
            camera.release()
    return found


def run_preview(
    config: VisionConfig,
    camera_source: str = "",
    config_path: str = "config/vision_config.yaml",
) -> None:
    detected = detect_cameras(config.camera)
    print(f"检测到摄像头: {detected}")
    available_sources = [str(index) for index in detected]
    if camera_source:
        current_source = camera_source
    elif available_sources:
        current_source = available_sources[0]
    else:
        print("未检测到可用摄像头")
        return
    if current_source not in available_sources:
        available_sources.append(current_source)

    camera = None
    pipeline = VisionPipeline(config, camera=None)

    def open_camera(source: str) -> None:
        nonlocal camera, current_source
        if camera is not None:
            camera.release()
        new_config = config.camera
        new_config.source = str(source)
        camera = Camera(new_config)
        camera.open()
        pipeline.camera = camera
        current_source = str(source)
        print(f"已切换摄像头: {current_source}")

    open_camera(current_source)
    save_dir = Path(config.debug.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    show_materials = config.detection.materials
    show_rings = config.detection.rings
    distance_scale = 1.0
    frame_counter = 0
    calibration_hint = ""
    try:
        while True:
            frame, _ = pipeline.read_frame()
            qr_result = pipeline.qr_detector.detect_with_points(frame)
            code = qr_result[0] if qr_result is not None else None
            materials = pipeline.material_detector.detect(frame) if show_materials else []
            rings = pipeline.ring_detector.detect(frame) if show_rings else []

            canvas = frame.copy()
            draw_qr(canvas, qr_result)
            draw_materials(canvas, materials)
            draw_rings(canvas, rings)
            draw_status(
                canvas,
                code,
                current_source,
                show_materials,
                show_rings,
                distance_scale,
                calibration_hint,
            )
            cv2.imshow("视觉预览", canvas)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("m"):
                show_materials = not show_materials
            elif key == ord("r"):
                show_rings = not show_rings
            elif key == ord("c"):
                if current_source in available_sources:
                    next_index = (available_sources.index(current_source) + 1) % len(
                        available_sources
                    )
                else:
                    next_index = 0
                open_camera(available_sources[next_index])
            elif ord("1") <= key <= ord("6"):
                color_id = key - ord("0")
                height, width = frame.shape[:2]
                patch = frame[
                    max(0, height // 2 - 20) : height // 2 + 20,
                    max(0, width // 2 - 20) : width // 2 + 20,
                ]
                lab_patch = cv2.cvtColor(patch, cv2.COLOR_BGR2LAB)
                mean_lab = np.mean(lab_patch.reshape(-1, 3), axis=0)
                pipeline.color_classifier.set_color_center(color_id, mean_lab)
                calibration_hint = f"颜色{color_id}已记录"
                print(f"颜色 {color_id} 校准: {mean_lab.round(1).tolist()}")
            elif key == ord("x"):
                config.colors = pipeline.color_classifier.color_specs
                config.camera.source = current_source
                save_config(config, config_path)
                calibration_hint = f"已保存到 {config_path}"
                print(f"配置已保存: {config_path}")
            elif key == ord("s"):
                frame_counter += 1
                path = save_dir / f"preview_{frame_counter:04d}.png"
                cv2.imwrite(str(path), canvas)
                print(f"已保存: {path}")
            elif key in (ord("]"), ord("="), ord("+")):
                distance_scale = min(3.0, distance_scale + 0.05)
                pipeline.color_classifier.set_distance_scale(distance_scale)
            elif key in (ord("["), ord("-"), ord("_")):
                distance_scale = max(0.1, distance_scale - 0.05)
                pipeline.color_classifier.set_distance_scale(distance_scale)
    except KeyboardInterrupt:
        pass
    finally:
        if camera is not None:
            camera.release()
        cv2.destroyAllWindows()


def main() -> None:
    if sys.platform.startswith("linux") and not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        print("当前是无图形显示环境（headless），无法打开预览窗口。")
        print("请改用网页推流: python3 -m pi_vision.stream --port 8090，或网页采集: python3 tools/collect_web.py")
        return
    parser = argparse.ArgumentParser(description="视觉预览调试工具")
    parser.add_argument("--config", default="config/vision_config.yaml", help="配置文件路径")
    parser.add_argument("--camera", default="", help="覆盖相机编号或设备路径")
    parser.add_argument("--undistort", action="store_true", default=None, help="强制开启畸变矫正")
    parser.add_argument("--no-undistort", action="store_true", help="强制关闭畸变矫正")
    args = parser.parse_args()

    config = load_config(args.config)
    if args.no_undistort:
        config.camera.undistort = False
    elif args.undistort:
        config.camera.undistort = True
    run_preview(config, args.camera, config_path=args.config)


if __name__ == "__main__":
    main()
