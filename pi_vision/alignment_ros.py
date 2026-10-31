"""Isolated alignment controller transport; no capture, serial or motor driver."""

import argparse
import json
import signal
import time
from .alignment import AlignmentCalibration, AlignmentController


def create_node(engine):
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String

    class AlignmentNode(Node):
        def __init__(self):
            super().__init__('competition_alignment')
            self.engine = engine
            self.prefix = 'alignment_sim' if engine.simulation else 'alignment'
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pubs = {kind: self.create_publisher(String, self.prefix+'/'+kind+'_command', qos)
                         for kind in ('vision', 'motion')}
            self.state_pub = self.create_publisher(String, self.prefix+'/status', state_qos)
            for kind in ('vision', 'motion'):
                self.create_subscription(String, self.prefix+'/'+kind+'_feedback',
                    lambda msg, channel=kind: self.feedback(channel, msg), qos)
            self.create_subscription(String, self.prefix+'/control', self.control, qos)
            self.controls = set()
            self.rejection = None
            self.create_timer(.05, self.update)
            self.get_logger().info('局部对准接口就绪，等待 start；主题='+self.prefix+'，无硬件驱动')

        def parse(self, message):
            if len(message.data) > 65536:
                raise ValueError('message too large')
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise ValueError('JSON object required')
            return data

        def control(self, message):
            try:
                data = self.parse(message)
                stamp, key = data.get('issued_at_unix_s'), data.get('request_id')
                if (type(data.get('schema_version')) is not int or data['schema_version'] != 1
                        or data.get('simulation') is not engine.simulation
                        or type(stamp) not in (int, float) or not 0 <= time.time()-stamp < 2
                        or not isinstance(key, str) or not key or len(key) > 256
                        or key in self.controls or len(self.controls) >= 4096):
                    raise ValueError('invalid/stale/duplicate control')
                self.controls.add(key)
                if data.get('operation') == 'start':
                    engine.start(data.get('color_id'), zone=data.get('zone', 'raw'))
                elif data.get('operation') == 'cancel':
                    engine.cancel()
                else:
                    raise ValueError('unsupported control')
                self.rejection = None
            except (ValueError, TypeError) as exc:
                self.rejection = str(exc)
                self.get_logger().warning('拒绝对准控制: '+str(exc))

        def feedback(self, channel, message):
            try:
                engine.feedback(channel, self.parse(message))
            except (ValueError, TypeError, KeyError) as exc:
                self.get_logger().warning('拒绝对准反馈: '+str(exc))

        @staticmethod
        def message(data):
            msg = String()
            msg.data = json.dumps(data, ensure_ascii=False, allow_nan=False)
            return msg

        def update(self):
            engine.tick()
            for item in engine.drain_commands():
                self.pubs[item['channel']].publish(self.message(item['command']))
            self.state_pub.publish(self.message(engine.snapshot() | {'last_control_rejection': self.rejection}))

    return AlignmentNode()


def main(argv=None):
    from pathlib import Path
    import rclpy
    from rclpy.executors import ExternalShutdownException
    parser = argparse.ArgumentParser(description='局部对准调度，不含动作驱动；默认待机')
    parser.add_argument('--simulation', action='store_true')
    parser.add_argument('--calibration', help='本车实测标定 JSON；不提供则拒绝 start')
    args, ros_args = parser.parse_known_args(argv)
    calibration = None
    if args.calibration:
        calibration = AlignmentCalibration(**json.loads(Path(args.calibration).read_text()))
    if args.simulation:
        if calibration is not None:
            parser.error('simulation uses built-in test calibration only')
        from .alignment_demo import simulation_calibration
        calibration = simulation_calibration()
    engine = AlignmentController(calibration, simulation=args.simulation)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_node(engine)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                engine.cancel()
                node.update()  # Best effort; physical watchdog remains backend responsibility.
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
