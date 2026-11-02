"""Bounded, loopback-only T-mini live viewer. No motion, SLAM or ROS publisher."""
from __future__ import annotations

import argparse
import copy
import fcntl
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid
try:
    from .lidar_navigation import NavigationPreview, local_candidates
    from .lidar_mission import build_mission_plan, mission_status
except ImportError:
    from lidar_navigation import NavigationPreview, local_candidates
    from lidar_mission import build_mission_plan, mission_status


class ScanState:
    def __init__(self, config=None, start_profiles=None):
        self.lock = threading.Lock()
        self.scan = None
        self.received = None
        self.status = "starting"
        self.error = ""
        self.frames = 0
        self.mission_plan = None
        self.session_id = uuid.uuid4().hex
        self.navigation = NavigationPreview(config) if config else None
        self.start_profiles = {}
        for key, profile in (start_profiles or {}).items():
            if key not in ('1','2') or not isinstance(profile,dict) or not profile.get('pose'):
                raise ValueError('invalid start profile')
            validated = NavigationPreview(config or {})
            validated.configure({**profile,'start_zone':int(key)})
            self.start_profiles[key] = copy.deepcopy(validated.settings)

    def ingest(self, line, now=None):
        if not line.startswith("SCAN_JSON "):
            return False
        data = json.loads(line[10:])
        if not isinstance(data.get("seq"), int) or data["seq"] <= 0:
            raise ValueError("bad sequence")
        if not isinstance(data.get("points"), list) or len(data["points"]) > 20000:
            raise ValueError("bad points")
        if not isinstance(data.get("frequency_hz"), (int, float)) or not math.isfinite(data["frequency_hz"]):
            raise ValueError("bad frequency")
        for p in data["points"]:
            if not isinstance(p, list) or len(p) != 3 or not all(isinstance(x, (float, int)) and math.isfinite(x) for x in p):
                raise ValueError("invalid point")
            if not -math.pi - .01 <= p[0] <= math.pi + .01 or not .05 <= p[1] <= 12:
                raise ValueError("point out of bounds")
        with self.lock:
            if self.scan and data["seq"] <= self.scan["seq"]:
                raise ValueError("non-new scan")
            self.scan = data
            self.received = time.monotonic() if now is None else now
            self.status = "live"
            self.error = ""
            self.frames += 1
            if self.navigation:
                self.navigation.ingest(data, self.received)
        return True

    def invalidate(self, status, error=""):
        with self.lock:
            self.status, self.error = status, error
            self.scan = None
            self.received = None
            if self.navigation:
                self.navigation.history.clear()

    def configure(self, command):
        with self.lock:
            if not self.navigation:
                raise ValueError('navigation not loaded')
            if command.get('action')=='plan_mission':
                now=time.monotonic()
                live=self.status=='live' and self.received is not None and 0<=now-self.received<1
                if not live:
                    raise ValueError('雷达数据失效，不能计算实时障碍路线')
                self.mission_plan=build_mission_plan(self.navigation,self.navigation.snapshot(now,live),
                                                     self.scan['seq'],self.session_id)
                return
            if command.get('start_zone') is not None:
                zone=command['start_zone']
                if type(zone) is not int or str(zone) not in self.start_profiles:
                    raise ValueError('start zone profile not configured')
                command={**command,**copy.deepcopy(self.start_profiles[str(zone)])}
            before=copy.deepcopy(self.navigation.settings)
            self.navigation.configure(command)
            if self.navigation.settings!=before:
                self.mission_plan=None

    def snapshot(self, now=None):
        with self.lock:
            age = None if self.received is None else max(0, (time.monotonic() if now is None else now) - self.received)
            valid = self.status == "live" and age is not None and age < 1
            result = dict(status=self.status if age is None or age < 1 else "expired", valid=valid,
                        age_ms=None if age is None else round(age * 1000, 1), max_age_ms=1000,
                        frame=self.scan if valid else None, total_frames=self.frames, error=self.error,
                        frame_id="lidar", angle_unit="radian", range_unit="m",
                        capture_time_semantics="host receipt after SDK processing; not exposure time",
                        map_localization_valid=False)
            result['session_id'] = self.session_id
            if self.navigation:
                result['navigation'] = self.navigation.snapshot(time.monotonic() if now is None else now, valid)
                result['mission_plan'] = mission_status(self.mission_plan,result['navigation'],valid)
                result['candidates'] = local_candidates(self.scan['points']) if valid else []
            return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--capture", required=True, type=Path)
    ap.add_argument("--device", required=True, type=Path)
    ap.add_argument("--port", default=8766, type=int)
    ap.add_argument("--seconds", type=float, default=1800)
    ap.add_argument("--output", required=True, type=Path)
    ap.add_argument("--field-map", type=Path, required=True)
    ap.add_argument("--preview-settings", type=Path, help="Saved stationary preview calibration; no localization")
    ap.add_argument("--scan-frequency",type=float,default=10,choices=(6,8,10))
    args = ap.parse_args()
    if not 1 <= args.seconds <= 3600:
        ap.error("seconds must be 1..3600")
    if not str(args.device).startswith("/dev/serial/by-id/") or not args.device.exists():
        ap.error("explicit existing by-id device required")
    config = json.loads(args.field_map.read_text())
    preview_settings = json.loads(args.preview_settings.read_text()) if args.preview_settings else None
    profiles = preview_settings.get('start_profiles',{}) if preview_settings else {}
    state, stop = ScanState(config,profiles), threading.Event()
    if preview_settings is not None:
        state.configure(preview_settings)
    # Kernel lock plus preflight ownership check; no auto selection of other serial ports.
    holders = subprocess.run(["fuser", str(args.device)], capture_output=True, text=True)
    if holders.returncode not in (0, 1) or holders.stdout.strip():
        ap.error("serial port is busy or ownership check failed")
    device_lock = args.device.open("rb", buffering=0)
    fcntl.flock(device_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    args.output.mkdir(parents=True, exist_ok=True)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *unused):
            pass

        def do_GET(self):
            if self.path == "/state":
                body = json.dumps(state.snapshot(), allow_nan=False).encode()
                mime = "application/json"
            elif self.path in ("/", "/index.html"):
                body, mime = Path(__file__).with_name('lidar_view.html').read_bytes(), "text/html; charset=utf-8"
            elif self.path == '/lidar_display.js':
                body, mime = Path(__file__).with_name('lidar_display.js').read_bytes(), 'text/javascript; charset=utf-8'
            elif self.path == '/field-map':
                body, mime = json.dumps(config, ensure_ascii=False).encode(), 'application/json'
            elif self.path == '/preview-settings':
                body, mime = json.dumps(preview_settings).encode(), 'application/json'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            if self.path != '/configure' or self.headers.get('Sec-Fetch-Site') == 'cross-site':
                self.send_error(403)
                return
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0 < length < 4096:
                    raise ValueError('invalid request size')
                command=json.loads(self.rfile.read(length))
                if not isinstance(command,dict):
                    raise ValueError('invalid command')
                state.configure(command)
            except (ValueError,TypeError) as exc:
                self.send_error(400,str(exc))
                return
            body=b'{"ok":true}'
            self.send_response(200)
            self.send_header('Content-Type','application/json')
            self.send_header('Content-Length',str(len(body)))
            self.send_header('Cache-Control','no-store')
            self.end_headers()
            self.wfile.write(body)

    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    server.daemon_threads = True
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    # Avoid inheriting a pipeline with any motor/chassis process: this child is SDK only.
    child = subprocess.Popen([str(args.capture.resolve()), str(args.device),str(args.scan_frequency)], stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True, bufsize=1)

    def read():
        count = 0
        try:
            with (args.output / "sdk.log").open("w") as log, (args.output / "scans.jsonl").open("w") as scans:
                for line in child.stdout:
                    if line.startswith("SCAN_FAILED"):
                        state.invalidate("scan_error", "扫描失败，等待新一圈")
                    try:
                        accepted = state.ingest(line)
                    except (ValueError, TypeError, json.JSONDecodeError) as exc:
                        state.invalidate("invalid_data", str(exc))
                        accepted = False
                    if accepted:
                        if count < 300:
                            scans.write(json.dumps({"received_unix": time.time(), **json.loads(line[10:])}) + "\n")
                            scans.flush()
                            count += 1
                    elif log.tell() < 2_000_000:
                        log.write(line)
                        log.flush()
        finally:
            state.invalidate("stopped", "雷达采集已停止")

    reader = threading.Thread(target=read, daemon=True)
    reader.start()
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"READY http://127.0.0.1:{args.port} / bounded {args.seconds}s", flush=True)
    deadline = time.monotonic() + args.seconds
    try:
        while not stop.wait(.25):
            if child.poll() is not None or time.monotonic() >= deadline:
                break
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=6)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        reader.join(timeout=2)
        summary = state.snapshot()
        summary["ended_unix"] = time.time()
        summary["child_exit_code"] = child.returncode
        (args.output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2))
        state.invalidate("stopped")
        server.shutdown()
        server.server_close()
        device_lock.close()


if __name__ == "__main__":
    main()
