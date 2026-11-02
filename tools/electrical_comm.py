"""Offline route review and bounded, non-motion electrical queries.

Mac: python3 -m tools.electrical_comm preview STATE.json PROFILE.json OUTPUT.json
Pi (after checking wiring/console/ownership): ... --port DEVICE status
No upload/start/work-done command is exposed by this first-stage CLI.
"""
import argparse
import json
import time
from pathlib import Path

from pi_vision.electrical_link import ElectricalClient
from pi_vision.electrical_protocol import UPLOAD, frame, upload_payload
from pi_vision.electrical_routes import compile_preview
from pi_vision.electrical_coordinates import direction_review


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', help='explicit, verified electrical UART device')
    commands = parser.add_subparsers(dest='command', required=True)
    preview = commands.add_parser('preview')
    for name in ('snapshot', 'profile', 'output'):
        preview.add_argument(name, type=Path)
    commands.add_parser('status')
    commands.add_parser('wait-area')
    watch = commands.add_parser('watch-areas')
    watch.add_argument('--seconds', type=float, default=120)
    parser.add_argument('--timeout', type=float, default=60)
    args = parser.parse_args()
    from tools.electrical_route_test import positive
    positive(args.timeout, 'timeout')
    if args.command == 'watch-areas':
        positive(args.seconds, 'seconds')
    if args.command == 'preview':
        state = json.loads(args.snapshot.read_text())
        profile = json.loads(args.profile.read_text())
        route = compile_preview(state['mission_plan'], state['navigation']['nodes'], **profile)
        # The frame is a review/test artifact. No UART is opened here.
        route['upload_frame_hex_for_review'] = frame(UPLOAD, 1, upload_payload(route)).hex()
        route['direction_review'] = direction_review(upload_payload(route))
        args.output.write_text(json.dumps(route, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(dict(output=str(args.output.resolve()), points=len(route['points']),
                              valid_for_motion=False), ensure_ascii=False))
        return
    if not args.port:
        parser.error('--port must explicitly identify the electrical UART')
    from tools.electrical_route_test import checked_port
    device = checked_port(args.port)
    import serial
    with serial.Serial(str(device), 115200, timeout=0.05, write_timeout=1, exclusive=True) as port:
        client = ElectricalClient(port)
        if args.command == 'status':
            result = dict(status=client.status(), pose=client.pose_snapshot())
        elif args.command == 'wait-area':
            result = client.wait_start_area(args.timeout)
        else:
            deadline = time.monotonic() + args.seconds
            while time.monotonic() < deadline:
                try:
                    selected = client.wait_start_area(min(args.timeout, deadline-time.monotonic()))
                except TimeoutError:
                    continue
                print(json.dumps(selected, ensure_ascii=False), flush=True)
            return
        print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
