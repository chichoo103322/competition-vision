"""Bounded ROS perception transport check. No mission scheduler or actuators.

Default: generated images through actual detectors and capture threads.
--camera: explicitly use a physical camera for observation-only diagnostics.
Run in an isolated ROS domain, as documented in MISSION_FRAMEWORK.md.
"""

import argparse
import json
import os
import time
import uuid


def main(argv=None):
    parser = argparse.ArgumentParser(description='有界 ROS 视觉收发检查，不启动任务调度或动作端')
    parser.add_argument('--camera')
    parser.add_argument('--config', default='config/vision_config.yaml')
    parser.add_argument('--material-backend', required=True, choices=['hsv'])
    parser.add_argument('--hsv-profile')
    parser.add_argument('--material-roi', nargs=4, type=int)
    parser.add_argument('--operation', choices=['read_task', 'scan_material', 'locate_position'], default='scan_material')
    parser.add_argument('--position-digits', action='store_true')
    parser.add_argument('--position-orientation', type=int, choices=(0, 90, 180, 270))
    parser.add_argument('--position-id', type=int, choices=(1, 2, 3), default=2)
    parser.add_argument('--position-scene', choices=['normal', 'missing', 'duplicate'], default='normal')
    parser.add_argument('--color', type=int, choices=range(1, 7), default=1)
    parser.add_argument('--seconds', type=float, default=5.)
    args = parser.parse_args(argv)
    if os.environ.get('ROS_DOMAIN_ID') != '74' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        parser.error('probe requires ROS_DOMAIN_ID=74 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    if not 1 <= args.seconds <= 20:
        parser.error('seconds must be in [1, 20]')
    if args.position_digits != (args.position_orientation is not None):
        parser.error('position digits requires explicit orientation, and vice versa')
    if args.camera and args.position_scene != 'normal':
        parser.error('synthetic scene options cannot describe a physical camera scene')
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String
    from pi_vision.vision_ros import create_node, make_source
    from pi_vision.config import default_config
    from pi_vision.pipeline import VisionPipeline
    from pi_vision.stream import FrameStreamer

    if args.camera:
        args.width, args.height = 1280, 720
        args.capture_timeout = args.result_timeout = 1.
        args.qr_timeout = args.seconds
        source = make_source(args)
    else:
        image = np.full((720, 1280, 3), 180, np.uint8)
        cv2.circle(image, (700, 300), 65, (0, 0, 230), -1)
        if args.operation == 'read_task':
            qr = cv2.imread('examples/qr/task_156.png')
            if qr is None:
                raise RuntimeError('missing QR fixture')
            qr = cv2.resize(qr, (450, 450), interpolation=cv2.INTER_NEAREST)
            image[100:550, 500:950] = qr
        elif args.operation == 'locate_position':
            image[:] = 255
            digits = [3, 1, 2] if args.position_scene != 'duplicate' else [3, 1, 1]
            for x, digit in zip((300, 640, 980), digits):
                for radius in (60, 70, 80, 90):
                    cv2.circle(image, (x, 360), radius, (0, 0, 0), 2)
                if args.position_scene == 'missing' and digit == args.position_id:
                    continue
                patch = np.full((110, 110, 3), 255, np.uint8)
                (w, h), _ = cv2.getTextSize(str(digit), cv2.FONT_HERSHEY_SIMPLEX, 1.5, 3)
                cv2.putText(patch, str(digit), (55-w//2, 55+h//2), cv2.FONT_HERSHEY_SIMPLEX,
                            1.5, (0, 0, 0), 3)
                if args.position_digits:
                    patch = np.rot90(patch, args.position_orientation//90).copy()
                image[305:415, x-55:x+55] = patch
        class GeneratedCamera:
            def open(self):
                pass
            def release(self):
                pass
            def read(self):
                time.sleep(.05)
                return True, image.copy(), time.time()
        camera = GeneratedCamera()
        pipeline = VisionPipeline(default_config(), camera=camera, material_backend='hsv')
        pipeline.set_material_roi(args.material_roi)
        if args.position_digits:
            from pi_vision.position_digits import NumberedRingDetector
            pipeline.ring_detector.smooth_alpha = 0.
            pipeline.ring_detector = NumberedRingDetector(pipeline.ring_detector, args.position_orientation)
        source = FrameStreamer(camera, pipeline, enable_qr=False, enable_materials=False,
                               enable_rings=False, preview_enabled=False, qr_timeout=args.seconds)

    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
    rclpy.init(args=[])
    executor = SingleThreadedExecutor()
    vision = client = None
    replies = []
    started = time.monotonic()
    try:
        source.start()
        vision = create_node(source, args.camera or 'generated_test_camera')
        client = Node('vision_observation_probe')
        publisher = client.create_publisher(String, 'mission/command', qos)
        client.create_subscription(String, 'mission/feedback',
                                   lambda msg: replies.append(json.loads(msg.data)), qos)
        executor.add_node(client)
        executor.add_node(vision)
        discovery = time.monotonic()+5.
        while time.monotonic() < discovery and (
                not publisher.get_subscription_count() or not vision.publisher.get_subscription_count()):
            executor.spin_once(timeout_sec=.05)
        if not publisher.get_subscription_count() or not vision.publisher.get_subscription_count():
            raise RuntimeError('ROS discovery timeout')
        command = dict(schema_version=1, simulation=False, run_id=uuid.uuid4().hex,
                       command_id=uuid.uuid4().hex, operation=args.operation,
                       params={} if args.operation == 'read_task' else
                       dict(zone='rough', position_id=args.position_id, layer=0)
                       if args.operation == 'locate_position' else dict(zone='raw', color_id=args.color),
                       issued_at_unix_s=time.time(), timeout_s=args.seconds)
        msg = String()
        msg.data = json.dumps(command)
        publisher.publish(msg)
        deadline = time.monotonic()+args.seconds+1.
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            if any(r['status'] == 'failed' for r in replies):
                break
        # Stop only perception. No whole-robot stopped=true acknowledgement is fabricated.
        command.update(command_id=uuid.uuid4().hex, operation='stop', issued_at_unix_s=time.time())
        msg.data = json.dumps(command)
        publisher.publish(msg)
        for _ in range(5):
            executor.spin_once(timeout_sec=.05)
        state = source.get_state()
        observations = [r['data'] for r in replies if r['status'] == 'observation']
        expected_disabled = args.operation == 'locate_position' and not args.position_digits
        faults = [r for r in replies if r['status'] == 'failed'
                  and r['data'].get('reason') != 'vision_request_timeout'
                  and not (expected_disabled and
                           r['data'].get('reason') == 'position_identity_not_implemented')]
        ok = bool(observations) and not faults and state['capture']['errors'] == 0
        if not args.camera:
            if args.operation == 'locate_position' and not args.position_digits:
                ok = (not observations and not faults and state['capture']['errors'] == 0 and
                      any(r['status'] == 'failed' and
                          r['data'].get('reason') == 'position_identity_not_implemented' for r in replies))
            elif args.operation == 'locate_position' and args.position_scene != 'normal':
                ok = ok and all(o['status'] == 'no_target' and not o['targets'] for o in observations)
            else:
                ok = ok and any(o.get('task_code') == '156+123+516+231' if args.operation == 'read_task'
                                else len(o['targets']) == 1 for o in observations)
        print(json.dumps(dict(evidence='live_camera_ros_observations_only' if args.camera else
                              'generated_images_real_detectors_ros_transport', ok=ok,
                              operation=args.operation, elapsed_s=time.monotonic()-started,
                              replies=replies, capture=state['capture'], preview=state['preview'],
                              detection_after_stop=state['detection']), ensure_ascii=False))
        return 0 if ok else 1
    finally:
        if vision is not None:
            vision.backend.cancel()
            vision.destroy_node()
        if client is not None:
            client.destroy_node()
        source.stop()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
