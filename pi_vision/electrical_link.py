"""One synchronous UART owner for route V1; no automatic start or reconnect.

Call from a single worker; ROS/vision producers must queue requests to that worker.
The transport is injected, so software tests never open a real device.
"""
from collections import deque
import math
import secrets
import struct
import time

from . import electrical_protocol as p
from .electrical_coordinates import direction_review, ELECTRICAL_FRAME


class Rejected(RuntimeError):
    def __init__(self, status):
        self.status = status
        code = status['code']
        label = p.RESULTS[code] if code < len(p.RESULTS) else str(code)
        super().__init__(f'STM32 rejected request: {label}')


class AreaSuperseded(RuntimeError):
    pass


class ElectricalClient:
    def __init__(self, transport, *, clock=time.monotonic):
        self.transport = transport  # bounded read/write timeouts required
        self.clock = clock
        self.decoder = p.Decoder(clock)
        self.sequence = secrets.randbelow(65536)
        self.events = deque(maxlen=256)
        self._pending = deque()
        self._status = None
        self._received_at = None
        self.start_areas = deque(maxlen=32)
        self.handled_areas = deque(maxlen=64)
        self.latest_area = self.selected_area = None
        self.last_upload_review = None

    def _receive(self, deadline):
        while self.clock() < deadline:
            if self._pending:
                return self._pending.popleft()
            self._pending.extend(self.decoder.feed(self.transport.read(4096)))
        return None

    def _exchange(self, kind, payload=b'', *, reply_kind=p.ACK, timeout=1.0, retries=3):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be positive and finite')
        p.integer(retries, 1, 10, 'retries')
        self.sequence = (self.sequence + 1) & 0xffff
        packet = p.frame(kind, self.sequence, payload)
        rejected_acks = 0
        last_decode_error = None
        try:
            for _ in range(retries):
                if self.transport.write(packet) != len(packet):
                    raise IOError('short UART write; outcome unknown')
                deadline = self.clock() + timeout
                while self.clock() < deadline:
                    packet_reply = self._receive(deadline)
                    if packet_reply is None:
                        break
                    typ, seq, body = packet_reply
                    if typ == p.START_AREA:
                        self._record_area(body)
                        continue
                    if typ in (p.ACK, p.EVENT):
                        try:
                            status = p.status_payload(body)
                        except ValueError as exc:
                            if typ == p.ACK and seq == self.sequence and body[:1] == bytes([kind]):
                                rejected_acks += 1
                                last_decode_error = str(exc)
                            continue
                        if typ == p.EVENT:
                            self.events.append(dict(status, sequence=seq,
                                                    received_monotonic_s=self.clock()))
                            continue  # Events never satisfy an ACK or refresh pose.
                        if seq != self.sequence or status['request'] != kind:
                            continue
                        if status['code']:
                            raise Rejected(status)
                        if reply_kind == p.ACK:
                            return status
            detail = (f' Received {rejected_acks} matching ACK(s) rejected by decoding: {last_decode_error}.'
                      if rejected_acks else '')
            raise TimeoutError('No matching reply; command outcome unknown. Query STATUS.' + detail)
        except (OSError, TimeoutError):
            self._status = self._received_at = None
            self.latest_area = self.selected_area = None
            self.start_areas.clear()
            self._pending.clear()
            self.decoder = p.Decoder(self.clock)
            raise  # No reconnect, START, or fabricated stopped confirmation.

    def status(self):
        result = self._exchange(p.STATUS)
        self._status, self._received_at = result, self.clock()
        return result

    def pose_snapshot(self, max_age_s=0.5):
        if not math.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError('max_age_s must be positive and finite')
        age = None if self._received_at is None else self.clock() - self._received_at
        fresh = age is not None and 0 <= age <= max_age_s
        s = self._status
        available = bool(fresh and s and s['route_id'] and s['state'] != 0)
        return dict(transport_fresh=fresh, received_monotonic_s=self._received_at,
                    age_s=age, frame_id=ELECTRICAL_FRAME, reference_point='chassis_reference_unverified',
                    source='stm32_encoder_imu_subaction_snapshot', continuous=False,
                    valid_for_motion=False, coordinates_available=available,
                    pose=None if not available else dict(x_mm=s['x_mm'], y_mm=s['y_mm'],
                                                         yaw_deg=s['yaw_deg']),
                    controller_tick_ms=None if not fresh else s['tick'])

    @staticmethod
    def area_identity(signal):
        return (signal['request_id'], signal['tick'], signal['area'])

    def _record_area(self, body):
        try:
            signal = p.start_area_payload(body)
        except ValueError:
            return
        if self.latest_area is not None:
            old = self.latest_area
            if signal['request_id'] < old['request_id'] or (signal['request_id'] == old['request_id'] and
                    self.area_identity(signal) != self.area_identity(old)):
                raise RuntimeError('START_AREA identity regressed; recreate client after MCU reboot')
            if self.area_identity(signal) == self.area_identity(old):
                return  # MCU retransmission must not start another scan.
        self.latest_area = signal
        self.start_areas.append(signal)

    def acknowledge_area(self, signal):
        return self._exchange(p.AREA_ACK, p.area_ack_payload(signal))

    def wait_start_area(self, timeout=60.0, cancelled=lambda: False):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('key timeout must be finite and positive')
        deadline = self.clock() + timeout
        while self.clock() < deadline:
            if cancelled():
                raise InterruptedError('key wait cancelled')
            if self.start_areas:
                signal = self.start_areas.pop()  # newest physical selection wins
                self.start_areas.clear()
                identity = self.area_identity(signal)
                if identity in self.handled_areas:
                    continue
                try:
                    self.acknowledge_area(signal)
                except Rejected as exc:
                    if exc.status['code'] in (6, 7):
                        continue  # superseded or cancelled on the MCU
                    raise
                if self.area_identity(self.latest_area) != identity:
                    continue  # new key arrived while waiting for this ACK
                self.handled_areas.append(identity)
                self.selected_area = dict(signal)
                return dict(signal)
            packet = self._receive(min(deadline, self.clock() + .2))
            if packet is None:
                continue
            typ, seq, body = packet
            if typ == p.START_AREA:
                self._record_area(body)
            elif typ == p.EVENT:
                try:
                    self.events.append(dict(p.status_payload(body), sequence=seq))
                except ValueError:
                    pass
        raise TimeoutError('no new KEY1/KEY0 start-area selection')

    def check_start_area(self, signal):
        status = self.status()  # pumps incoming key signals during scanning/planning
        if self.latest_area is None or self.area_identity(self.latest_area) != self.area_identity(signal):
            raise AreaSuperseded('new key selection invalidated the current plan')
        if status['state'] in (5, 6):
            raise RuntimeError('selected scan/route cancelled on controller')
        return status

    def plan_after_key(self, planner, *, timeout=60, cancelled=lambda: False):
        """Common test/production gate. planner(signal, check_current) -> route.

        The planner must call check_current during bounded scan/compute steps.
        New selections discard old work and re-enter the planner with the new area.
        This helper never uploads, starts, or synthesizes work completion.
        """
        while True:
            signal = self.wait_start_area(timeout, cancelled)
            def check_current():
                if cancelled():
                    raise InterruptedError('planning cancelled')
                return self.check_start_area(signal)
            try:
                route = planner(signal, check_current)
                check_current()
                return signal, route
            except AreaSuperseded:
                continue

    def run_after_key(self, planner, on_work=None, *, key_timeout=60, run_timeout=600,
                      sleep=time.sleep, cancelled=lambda: False):
        """Production entry: same key/plan binding, ONLY real work feedback.

        planner(signal, check_current) returns a commissioned route with
        start_zone. on_work(client, status) must be nonblocking: None=pending,
        True=actually completed, False=failed. Without it, remain at WAIT_WORK.
        Use state keyed by route_id/point_index to start each real job only once.
        """
        if not math.isfinite(run_timeout) or run_timeout <= 0:
            raise ValueError('run_timeout must be finite and positive')
        signal, route = self.plan_after_key(planner, timeout=key_timeout, cancelled=cancelled)
        return self.run_selected_route(signal, route, on_work, run_timeout=run_timeout,
                                       sleep=sleep, cancelled=cancelled)

    def run_selected_route(self, signal, route, on_work=None, *, run_timeout=600,
                           sleep=time.sleep, cancelled=lambda: False, before_start=lambda: None):
        """Production execution after the shared prewarmed button gate.

        Does not wait for a second key. No timed/mock work success is generated.
        """
        if not math.isfinite(run_timeout) or run_timeout <= 0:
            raise ValueError('run_timeout must be finite and positive')
        if route.get('test_mode'):
            raise ValueError('test route is forbidden in production execution')
        touched = False
        try:
            self.check_start_area(signal)
            touched = True
            self.upload(route, start_request_id=signal['request_id'])
            before_start()
            self.start_when_ready(route['route_id'], sleep=sleep, cancelled=cancelled)
            deadline = self.clock() + run_timeout
            released = set()
            last_index = -1
            while self.clock() < deadline:
                if cancelled():
                    raise InterruptedError('mission cancelled')
                status = self.status()
                index = status['point_index']
                if status['route_id'] != route['route_id'] or status['point_count'] != len(route['points']):
                    raise RuntimeError('route replaced or MCU rebooted')
                if index < last_index:
                    raise RuntimeError('route progress regressed')
                last_index = index
                if status['state'] in (0, 1, 5, 6) or status['fault'] or status['chassis_fault']:
                    raise RuntimeError(f'controller stopped/fault: {status}')
                gates = {i for i, point in enumerate(route['points']) if point.get('work_id', 0)}
                if status['state'] == 4:
                    if index != len(route['points']) or released != gates:
                        raise RuntimeError('unexpected DONE without all real work feedback')
                    return status
                if any(i < index and i not in released for i in gates):
                    raise RuntimeError('work point skipped by another controller')
                if status['state'] == 3:
                    if (index not in gates or index in released or status['phase'] != 4 or
                            status['work_id'] != route['points'][index]['work_id']):
                        raise RuntimeError('unexpected work point')
                    if on_work is not None:
                        result = on_work(self, dict(status))
                        if result is not None:
                            if type(result) is not bool:
                                raise ValueError('work result must be None/True/False')
                            self.work_done(route['route_id'], index, success=result)
                            if not result:
                                raise RuntimeError('real work failed')
                            released.add(index)
                sleep(.2)
            raise TimeoutError('mission execution timeout')
        except (Exception, KeyboardInterrupt):
            if touched:
                try:
                    self.stop(route['route_id'])
                except Exception:
                    pass  # Original failure is preserved; never claim stop confirmed.
            raise

    def start_when_ready(self, route_id, *, timeout=10, sleep=time.sleep, cancelled=lambda: False):
        deadline = self.clock() + timeout
        while self.clock() < deadline:
            if cancelled():
                raise InterruptedError('start cancelled')
            try:
                return self.start(route_id)
            except Rejected as exc:
                if (exc.status['code'] != 7 or exc.status['state'] != 1 or
                        exc.status['route_id'] != route_id):
                    raise
            sleep(.1)  # READY + BAD_STATE: key still held or IMU not ready
        raise TimeoutError('START still inhibited; release key/check IMU')

    def upload(self, route, *, test_only=False, start_request_id=None):
        # Test-only execution is explicit and still uses the controller's real
        # closed-loop waypoint/settle feedback. It must never be implicit.
        permitted_test = test_only and route.get('test_mode') == 'chassis_route_dry_work'
        if route.get('valid_for_motion') is not True and not permitted_test:
            raise ValueError('preview/uncommissioned route cannot be uploaded')
        payload = p.upload_payload(route)
        self.last_upload_review = direction_review(payload)
        if start_request_id is not None:
            if (self.selected_area is None or self.selected_area['request_id'] != start_request_id or
                    route.get('start_zone') != self.selected_area['area']):
                raise ValueError('route must match acknowledged start-area request')
            self.check_start_area(self.selected_area)
            return self._exchange(p.UPLOAD_FOR_AREA, struct.pack('<I', p.integer(
                start_request_id, 1, 0xffffffff, 'request_id')) + payload)
        return self._exchange(p.UPLOAD, payload)

    def start(self, route_id):
        p.integer(route_id, 1, 0xffffffff, 'route_id')
        status = self.status()
        if status['route_id'] != route_id or status['state'] != 1:
            raise ValueError('explicit START requires matching READY route')
        return self._exchange(p.START, struct.pack('<I', route_id))

    def stop(self, route_id=0):
        p.integer(route_id, 0, 0xffffffff, 'route_id')
        return self._exchange(p.STOP, struct.pack('<I', route_id))

    def work_done(self, route_id, point_index, *, success):
        p.integer(route_id, 1, 0xffffffff, 'route_id')
        p.integer(point_index, 0, 63, 'point_index')
        if type(success) is not bool:
            raise ValueError('success must be an explicit boolean from real work feedback')
        return self._exchange(p.WORK_DONE, struct.pack('<IHBB', route_id, point_index, success, 0))

    def vision(self, route_id, point_index, work_id, *, dx_mm, dy_mm, dyaw_deg, confidence):
        """Only calibrated body offsets: +X forward, +Y left, CCW positive."""
        payload = struct.pack('<IHH iihH',
                              p.integer(route_id, 1, 0xffffffff, 'route_id'),
                              p.integer(point_index, 0, 63, 'point_index'),
                              p.integer(work_id, 1, 65535, 'work_id'),
                              p.scaled(dx_mm, 1, 100000, 'dx_mm'),
                              p.scaled(dy_mm, 1, 100000, 'dy_mm'),
                              p.scaled(dyaw_deg, 100, 180, 'dyaw_deg'),
                              p.integer(confidence, 0, 1000, 'confidence'))
        return self._exchange(p.VISION, payload)
