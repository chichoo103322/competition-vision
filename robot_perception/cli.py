"""Command-line capture, recognition, and safety-stream tools."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time

import cv2

from .color import ColorConfirmation, ColorDetector, DEFAULT_RANGES, load_ranges
from .lidar import SafetyMonitor, Scan
from .task_code import TaskCodeDecoder, TaskConfirmation


def emit(data: dict) -> None:
    print(json.dumps(data, ensure_ascii=False, separators=(",", ":")), flush=True)


def read_image(path: str):
    frame = cv2.imread(path)
    if frame is None:
        raise ValueError(f"cannot read image: {path}")
    return frame


def open_camera(index: int):
    camera = cv2.VideoCapture(index)
    if not camera.isOpened():
        raise RuntimeError(f"cannot open camera {index}")
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return camera


def get_frame(camera):
    ok, frame = camera.read()
    if not ok or frame is None:
        raise RuntimeError("camera did not return an image")
    return frame


def capture(args) -> int:
    camera = open_camera(args.camera)
    try:
        for _ in range(5):
            frame = get_frame(camera)
        if not cv2.imwrite(args.output, frame):
            raise RuntimeError(f"could not save {args.output}")
        emit({"type": "capture", "path": str(Path(args.output).resolve()),
              "width": frame.shape[1], "height": frame.shape[0]})
        return 0
    finally:
        camera.release()


def qr(args) -> int:
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        raise ValueError("timeout must be finite and positive")
    confirmation = TaskConfirmation(args.confirm_frames, args.min_confirm_seconds,
                                    args.max_frame_gap)
    decoder = TaskCodeDecoder()
    roi = tuple(args.roi) if args.roi else None
    if args.image:
        observation = decoder.decode(read_image(args.image), roi)
        if observation.plan is None:
            emit({"type": "task_plan", "status": observation.status, "confirmed": False})
            return 1
        emit({"type": "task_plan", "status": "ok", "confirmed": False,
              **observation.plan.to_dict()})
        return 0

    camera = open_camera(args.camera)
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            frame = get_frame(camera)
            acquired_at = time.monotonic()
            observation = decoder.decode(frame, roi)
            # A late read or decode cannot turn an expired scan into success.
            processed_at = time.monotonic()
            if processed_at >= deadline:
                break
            plan = confirmation.update(observation.plan, acquired_at)
            if args.diagnostics:
                print(json.dumps({"type": "qr_observation", "status": observation.status,
                                  "candidate": observation.plan.raw if observation.plan else None,
                                  "matching_frames": confirmation.count,
                                  "decode_ms": round((processed_at - acquired_at) * 1000, 1),
                                  "confirmed": plan is not None}, ensure_ascii=False),
                      file=sys.stderr, flush=True)
            if plan is not None:
                emit({"type": "task_plan", "status": "ok", "confirmed": True,
                      **plan.to_dict()})
                return 0
        emit({"type": "task_plan", "status": "timeout", "confirmed": False})
        return 1
    finally:
        camera.release()


def color(args) -> int:
    detector = ColorDetector(load_ranges(args.profile), min_area_px=args.min_area)
    roi = tuple(args.roi) if args.roi else None
    if args.image:
        detections = detector.detect(read_image(args.image), roi)
        emit({"type": "material_detections", "confirmed": False,
              "detections": [item.to_dict() for item in detections]})
        return 0

    if args.target is None:
        raise ValueError("--target is required with --camera")
    confirmation = ColorConfirmation(args.confirm_frames)
    camera = open_camera(args.camera)
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            frame = get_frame(camera)
            view = frame if roi is None else frame[roi[1]:roi[1] + roi[3], roi[0]:roi[0] + roi[2]]
            sharpness = cv2.Laplacian(cv2.cvtColor(view, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
            if sharpness < args.min_sharpness:
                confirmation.update([], args.target)
                continue
            confirmed = confirmation.update(detector.detect(frame, roi), args.target)
            if confirmed:
                emit({"type": "material_detection", "confirmed": True, **confirmed.to_dict()})
                return 0
        emit({"type": "material_detection", "confirmed": False, "color": args.target})
        return 1
    finally:
        camera.release()


def scan_reader(messages: queue.Queue) -> None:
    for line in sys.stdin:
        if line.strip():
            try:
                messages.put_nowait(line)
            except queue.Full:
                messages.get_nowait()
                messages.put_nowait(line)
    messages.put(None)


def safety(args) -> int:
    monitor = SafetyMonitor(args.robot_width, args.side_margin, args.stop_distance,
                            args.stale_after, args.blocked_scans)
    serial_port = None
    if args.serial_port:
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("install the serial extra: pip install '.[serial]'") from exc
        serial_port = serial.Serial(args.serial_port, args.baud, timeout=0, write_timeout=0.05)

    messages: queue.Queue[str | None] = queue.Queue(maxsize=2)
    threading.Thread(target=scan_reader, args=(messages,), daemon=True).start()
    ended = False
    last_output = 0.0
    try:
        while True:
            now = time.monotonic()
            try:
                line = messages.get(timeout=0.05)
            except queue.Empty:
                line = ""
            if line is None:
                ended = True
                monitor.status = monitor.status.__class__(
                    "STOP", "scan_stream_ended", True, monitor.status.segment_id, None)
            elif line:
                try:
                    data = json.loads(line)
                    scan = Scan.from_dict(data)
                    monitor.ingest(scan, time.monotonic(), data.get("segment_id", args.segment))
                except (ValueError, KeyError, TypeError):
                    monitor.status = monitor.status.__class__(
                        "STOP", "invalid_scan", True, args.segment, None)
                    monitor.last_scan_at = time.monotonic()
            status = monitor.tick(time.monotonic())
            now = time.monotonic()
            if now - last_output >= 0.1 or ended:
                emit(status.to_dict())
                if serial_port:
                    instruction = "STOP\n" if status.stop_requested else "CLEAR\n"
                    serial_port.write(instruction.encode("ascii"))
                last_output = now
            if ended:
                break
        return 0
    finally:
        if serial_port:
            serial_port.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Competition robot perception tools")
    commands = parser.add_subparsers(dest="command", required=True)

    capture_command = commands.add_parser("capture", help="save one camera frame")
    capture_command.add_argument("--camera", type=int, default=0)
    capture_command.add_argument("--output", required=True)
    capture_command.set_defaults(action=capture)

    qr_command = commands.add_parser("qr", help="decode a preliminary-round task QR code")
    qr_source = qr_command.add_mutually_exclusive_group(required=True)
    qr_source.add_argument("--image")
    qr_source.add_argument("--camera", type=int)
    qr_command.add_argument("--timeout", type=float, default=10.0)
    qr_command.add_argument("--roi", type=int, nargs=4, metavar=("X", "Y", "W", "H"))
    qr_command.add_argument("--confirm-frames", type=int, default=3)
    qr_command.add_argument("--min-confirm-seconds", type=float, default=0.2)
    qr_command.add_argument("--max-frame-gap", type=float, default=1.0)
    qr_command.add_argument("--diagnostics", action="store_true",
                            help="write per-frame confirmation diagnostics to stderr")
    qr_command.set_defaults(action=qr)

    color_command = commands.add_parser("color", help="find a material color")
    color_source = color_command.add_mutually_exclusive_group(required=True)
    color_source.add_argument("--image")
    color_source.add_argument("--camera", type=int)
    color_command.add_argument("--target", choices=DEFAULT_RANGES)
    color_command.add_argument("--roi", type=int, nargs=4, metavar=("X", "Y", "W", "H"))
    color_command.add_argument("--profile")
    color_command.add_argument("--min-area", type=int, default=250)
    color_command.add_argument("--confirm-frames", type=int, default=3)
    color_command.add_argument("--min-sharpness", type=float, default=30.0)
    color_command.add_argument("--timeout", type=float, default=10.0)
    color_command.set_defaults(action=color)

    safety_command = commands.add_parser("safety", help="watch lidar JSON lines from stdin")
    safety_command.add_argument("--robot-width", type=float, default=0.24)
    safety_command.add_argument("--side-margin", type=float, default=0.03)
    safety_command.add_argument("--stop-distance", type=float, default=0.35)
    safety_command.add_argument("--stale-after", type=float, default=0.3)
    safety_command.add_argument("--blocked-scans", type=int, default=3)
    safety_command.add_argument("--segment")
    safety_command.add_argument("--serial-port")
    safety_command.add_argument("--baud", type=int, default=115200)
    safety_command.set_defaults(action=safety)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        return args.action(args)
    except (ValueError, RuntimeError, OSError, cv2.error) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
