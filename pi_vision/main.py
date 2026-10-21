"""上位机入口：支持合成场景演示与真实串口+相机运行。"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

import cv2
import numpy as np

from .app import VisionApp
from .camera import Camera
from .config import CameraConfig, VisionConfig, load_config
from .pipeline import VisionPipeline
from .serial_link import SerialLink
from .synthetic import SyntheticScene


def make_qr_camera(config: VisionConfig):
    """按配置创建独立的扫码相机，未配置时返回 None。"""
    if not config.camera.qr_source:
        return None
    qr_config = CameraConfig(
        source=config.camera.qr_source,
        backend=config.camera.backend,
        width=config.camera.width,
        height=config.camera.height,
        fps=config.camera.fps,
        flip=config.camera.flip,
        fov_h_deg=config.camera.fov_h_deg,
        undistort=config.camera.undistort,
    )
    return Camera(qr_config)


def run_simulated(config: VisionConfig) -> None:
    """使用合成场景验证检测器链路。"""
    scene = SyntheticScene()
    pipeline = VisionPipeline(config)
    print("=== 合成场景演示 ===")

    material_frame = scene.render_materials([1, 2, 6])
    materials = pipeline.detect_materials(material_frame)
    print("物料识别:")
    for item in materials:
        print(f"  color={item.color_id} center=({item.center_x},{item.center_y})")

    ring_frame = scene.render_rings([1, 2, 3, 4, 5])
    rings = pipeline.detect_rings(ring_frame)
    print("圆环识别:")
    for item in rings:
        print(f"  ring={item.ring_no} center=({item.center_x},{item.center_y})")

    try:
        import qrcode
    except ImportError:
        print("未安装 qrcode，跳过二维码任务码演示")
        return
    qr_image = qrcode.make("156+123+516+231")
    frame = np.array(qr_image.convert("RGB"))[:, :, ::-1]
    frame = np.pad(frame, ((40, 40), (40, 40), (0, 0)), constant_values=255)
    frame = cv2.resize(frame, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
    code = pipeline.detect_task_code(frame=frame)
    print("任务码识别:", code)


def run_camera_test(config: VisionConfig, max_frames: int = 5, show: bool = False) -> None:
    """只打开相机测试视觉链路，不连接串口。"""
    camera = Camera(config.camera)
    qr_camera = make_qr_camera(config)
    pipeline = VisionPipeline(config, camera=camera, qr_camera=qr_camera)
    camera.open()
    if qr_camera is not None and qr_camera is not camera:
        qr_camera.open()
    save_dir = Path(config.debug.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)
    try:
        for index in range(max_frames):
            frame, _ = pipeline.read_frame()
            materials = pipeline.detect_materials(frame)
            rings = pipeline.detect_rings(frame)
            code = pipeline.detect_task_code()
            print(
                f"frame {index + 1}: task={code} "
                f"materials={len(materials)} rings={len(rings)}"
            )
            for item in materials:
                print(
                    f"  物料 color={item.color_id} "
                    f"center=({item.center_x},{item.center_y})"
                )
            for item in rings:
                print(
                    f"  圆环 ring={item.ring_no} "
                    f"center=({item.center_x},{item.center_y})"
                )
            debug_frame = pipeline.draw_debug(frame, materials, rings)
            save_path = save_dir / f"frame_{index + 1:02d}.png"
            cv2.imwrite(str(save_path), debug_frame)
            print(f"  已保存: {save_path}")
            if show:
                cv2.imshow("vision", debug_frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        camera.release()
        if qr_camera is not None and qr_camera is not camera:
            qr_camera.release()
        if show:
            cv2.destroyAllWindows()


def run_qr_test(config: VisionConfig, max_frames: int = 600, show: bool = False) -> None:
    """只测试二维码识别，不连接串口。"""
    camera = Camera(config.camera)
    qr_camera = make_qr_camera(config)
    pipeline = VisionPipeline(config, camera=camera, qr_camera=qr_camera)
    camera.open()
    if qr_camera is not None and qr_camera is not camera:
        qr_camera.open()
    save_dir = Path(config.debug.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)
    try:
        for index in range(max_frames):
            frame, _ = pipeline.read_qr_frame()
            code = pipeline.qr_detector.detect(frame)
            if code is not None:
                print(f"检测到任务码: {code}")
                save_path = save_dir / f"qr_detected_{index + 1:04d}.png"
                cv2.imwrite(str(save_path), frame)
                print(f"已保存: {save_path}")
                return
            if show:
                cv2.imshow("qr-test", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        print("未在指定帧数内检测到二维码")
    finally:
        camera.release()
        if qr_camera is not None and qr_camera is not camera:
            qr_camera.release()
        if show:
            cv2.destroyAllWindows()
        if show:
            cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(description="2027 智能搬运视觉上位机")
    parser.add_argument("--config", default="config/vision_config.yaml", help="配置文件路径")
    parser.add_argument("--port", help="串口设备，例如 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, help="波特率")
    parser.add_argument("--simulate", action="store_true", help="使用合成场景演示")
    parser.add_argument("--camera-test", action="store_true", help="只测试相机视觉链路")
    parser.add_argument("--qr-test", action="store_true", help="只测试二维码识别")
    parser.add_argument("--frames", type=int, default=5, help="相机测试采集帧数")
    parser.add_argument("--preview", action="store_true", help="相机测试时显示预览窗口")
    parser.add_argument("--qr-camera", default="", help="扫码相机编号/路径，默认用主相机")
    parser.add_argument("--undistort", action="store_true", default=None, help="强制开启畸变矫正")
    parser.add_argument("--no-undistort", action="store_true", help="强制关闭畸变矫正")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)
    config = load_config(args.config)
    if args.port:
        config.serial.port = args.port
    if args.baud:
        config.serial.baudrate = args.baud
    if args.qr_camera:
        config.camera.qr_source = args.qr_camera
    if args.no_undistort:
        config.camera.undistort = False
    elif args.undistort:
        config.camera.undistort = True

    if args.simulate:
        run_simulated(config)
        return
    if args.camera_test:
        run_camera_test(config, max_frames=args.frames, show=args.preview)
        return
    if args.qr_test:
        run_qr_test(config, max_frames=max(args.frames, 600), show=args.preview)
        return

    camera = Camera(config.camera)
    qr_camera = make_qr_camera(config)
    pipeline = VisionPipeline(config, camera=camera, qr_camera=qr_camera)
    link = SerialLink(
        config.serial.port,
        baudrate=config.serial.baudrate,
        timeout=config.serial.timeout,
    )
    app = VisionApp(config, serial_link=link, pipeline=pipeline)
    app.start()
    print("视觉上位机已启动，等待主控串口指令；Ctrl+C 停止")

    try:
        app.run_forever()
    except KeyboardInterrupt:
        print("\n正在停止")
    finally:
        app.stop()


if __name__ == "__main__":
    main()
