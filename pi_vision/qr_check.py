"""独立的扫码验收入口：使用已有相机/解码器，不启动串口或 YOLO。"""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
import math
import sys
import time

import cv2

from .camera import Camera
from .config import load_config
from .detectors.qr_detector import QRDetector
from .task_confirmation import TaskCodeConfirmation, parse_competition_task


def emit(data, stream=None):
    print(json.dumps(data, ensure_ascii=False), file=stream or sys.stdout, flush=True)


def inspect_frame(detector, frame, roi=None):
    if frame is None or frame.size == 0:
        return None, "empty_frame"
    if roi is not None:
        x, y, w, h = roi
        height, width = frame.shape[:2]
        if x < 0 or y < 0 or w <= 0 or h <= 0 or x + w > width or y + h > height:
            raise ValueError("扫码区域须完全位于实际画面内，参数为 X Y 宽 高")
        frame = frame[y:y + h, x:x + w]
    try:
        code = detector.detect(frame)
    except UnicodeError:
        return None, "invalid_payload"
    if code is None:
        return None, "not_found"
    try:
        return parse_competition_task(str(code)), "candidate"
    except ValueError:
        return None, "invalid_task"


def run(args):
    confirmation = TaskCodeConfirmation(args.confirm_frames, args.min_confirm_seconds,
                                        args.max_frame_gap)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        raise ValueError("超时时间须为正有限数")
    detector = QRDetector()
    roi = tuple(args.roi) if args.roi else None
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            raise ValueError(f"无法读取图片: {args.image}")
        code, status = inspect_frame(detector, frame, roi)
        emit({"type": "task_code", "status": "image_ok" if code else status,
              "confirmed": False, "raw": str(code) if code else None})
        return 0 if code else 1

    config = load_config(args.config)
    # 默认沿用项目的扫码相机配置；--camera 则明确覆盖设备来源。
    source = args.camera if args.camera is not None else (
        config.camera.qr_source or config.camera.source)
    camera = Camera(replace(config.camera, source=str(source)))
    print(f"[扫码测试] 相机={source}；请将二维码对准镜头。Ctrl+C 可退出。",
          file=sys.stderr, flush=True)
    try:
        camera.open()
        started_at = time.monotonic()
        deadline = started_at + args.timeout
        frame_count = 0
        while time.monotonic() < deadline:
            ok, frame, _ = camera.read()
            acquired_at = time.monotonic()
            if not ok or frame is None or frame.size == 0:
                raise RuntimeError("相机读取失败；未确认任务码")
            code, status = inspect_frame(detector, frame, roi)
            completed_at = time.monotonic()
            if completed_at >= deadline:
                break
            frame_count += 1
            locked = confirmation.update(code, acquired_at)
            if args.diagnostics:
                emit({"type": "qr_observation", "status": status,
                      "candidate": str(code) if code else None,
                      "matching_frames": confirmation.count,
                      "decode_ms": round((completed_at - acquired_at) * 1000, 1),
                      "confirmed": locked is not None}, sys.stderr)
            if locked is not None:
                emit({"type": "task_code", "status": "ok", "confirmed": True,
                      "raw": str(locked), "matching_frames": confirmation.count,
                      "frames_read": frame_count,
                      "elapsed_s": round(completed_at - started_at, 3)})
                return 0
        emit({"type": "task_code", "status": "timeout", "confirmed": False,
              "raw": None, "frames_read": frame_count})
        return 1
    finally:
        camera.release()


def build_parser():
    parser = argparse.ArgumentParser(description="第1步：初赛任务码稳定读取验收")
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--camera", help="USB 相机编号或设备路径，如 0 或 /dev/video0")
    source.add_argument("--image", help="仅检查单张图片，不作实时确认")
    parser.add_argument("--config", default="config/vision_config.yaml")
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--confirm-frames", type=int, default=3)
    parser.add_argument("--min-confirm-seconds", type=float, default=0.2)
    parser.add_argument("--max-frame-gap", type=float, default=1.0)
    parser.add_argument("--roi", type=int, nargs=4, metavar=("X", "Y", "W", "H"))
    parser.add_argument("--diagnostics", action="store_true")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        return run(args)
    except KeyboardInterrupt:
        emit({"type": "task_code", "status": "cancelled", "confirmed": False,
              "raw": None})
        return 130
    except (OSError, RuntimeError, ValueError, cv2.error) as exc:
        emit({"type": "task_code", "status": "error", "confirmed": False,
              "raw": None, "message": str(exc)})
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
