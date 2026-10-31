"""Headless, single-camera ROS 2 backend for the mission schema 1 contract."""

import argparse
import json
import signal

from .vision_backend import VisionBackend


def create_node(source, camera_id):
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String

    class VisionNode(Node):
        def __init__(self):
            super().__init__('competition_vision_backend')
            self.backend = VisionBackend(source, camera_id)
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            self.publisher = self.create_publisher(String, 'mission/feedback', qos)
            self.create_subscription(String, 'mission/command', self.command, qos)
            self.create_timer(.05, self.update)

        def publish(self, replies):
            for reply in replies:
                msg = String()
                msg.data = json.dumps(reply, ensure_ascii=False, allow_nan=False)
                self.publisher.publish(msg)

        def command(self, msg):
            try:
                if len(msg.data) > 65536:
                    raise ValueError('message too large')
                self.publish(self.backend.command(json.loads(msg.data)))
            except (ValueError, TypeError, KeyError) as exc:
                self.get_logger().warning('拒绝视觉请求: '+str(exc))

        def update(self):
            self.publish(self.backend.poll())

    return VisionNode()


def make_source(args):
    from .camera import Camera
    from .config import load_config
    from .hsv_profile import load_profile
    from .pipeline import VisionPipeline
    from .stream import FrameStreamer

    config = load_config(args.config)
    config.camera.source = args.camera
    config.camera.qr_source = ''  # One physical camera, sequential QR/material roles.
    config.camera.width, config.camera.height = args.width, args.height
    config.camera.undistort = False
    if config.camera.v4l2_controls:
        raise ValueError('此入口暂不应用相机控制；请使用空 v4l2_controls 配置')
    camera = Camera(config.camera)
    pipeline = VisionPipeline(config, camera=camera, material_backend=args.material_backend,
                              hsv_profile=load_profile(args.hsv_profile) if args.hsv_profile else None)
    pipeline.set_material_roi(args.material_roi)
    if getattr(args, 'position_digits', False):
        from .position_digits import NumberedRingDetector
        # Position coordinates must be this frame's candidates, not EMA history.
        pipeline.ring_detector.smooth_alpha = 0.
        pipeline.ring_detector = NumberedRingDetector(pipeline.ring_detector, args.position_orientation)
    return FrameStreamer(camera, pipeline, enable_qr=False, enable_materials=False,
                         enable_rings=False, preview_enabled=False, draw_overlay=False,
                         capture_timeout=args.capture_timeout, result_timeout=args.result_timeout,
                         qr_timeout=args.qr_timeout)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='无网页 ROS 2 真实视觉后端，不开启串口或电机')
    parser.add_argument('--config', default='config/vision_config.yaml')
    parser.add_argument('--camera', required=True)
    parser.add_argument('--material-backend', required=True, choices=['hsv'])
    parser.add_argument('--hsv-profile')
    parser.add_argument('--material-roi', nargs=4, type=int, default=None)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--capture-timeout', type=float, default=1.)
    parser.add_argument('--result-timeout', type=float, default=1.)
    parser.add_argument('--qr-timeout', type=float, default=10.)
    parser.add_argument('--position-digits', action='store_true', help='启用实验性环内数字模板，不按左右编号')
    parser.add_argument('--position-orientation', type=int, choices=(0, 90, 180, 270),
                        help='将数字转正的顺时针角度，必须按观察姿态确认')
    args, ros_args = parser.parse_known_args(argv)
    if min(args.width, args.height) < 1:
        parser.error('width/height must be positive')
    if args.position_digits != (args.position_orientation is not None):
        parser.error('--position-digits and explicit --position-orientation must be supplied together')
    return args, ros_args


def main(argv=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    args, ros_args = parse_args(argv)
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, terminate)
    rclpy.init(args=ros_args)
    source = node = None
    try:
        source = make_source(args)
        source.start()
        node = create_node(source, args.camera)
        node.get_logger().info('真实视觉后端就绪，等待请求；无网页、串口、机械动作')
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.backend.cancel()
            node.destroy_node()
        if source is not None:
            source.stop()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
