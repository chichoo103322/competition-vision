"""ROS 2 transport for the task framework. No camera or motor driver is opened.

JSON over std_msgs/String is a versioned prototype contract, NOT the old STM32
byte protocol. Simulation topics are isolated from real command topics.
"""

import json
import time

from .mission import Phase, TaskCoordinator
from .mission_demo import MockBackend


def create_node():
    # Lazy ROS imports keep pure task planning runnable/testable on the Mac.
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String

    class MissionNode(Node):
        def __init__(self):
            super().__init__('competition_mission')
            self.declare_parameter('simulation', False)
            self.declare_parameter('capacity', 1)
            self.declare_parameter('task_code', '156+123+516+231')
            self.declare_parameter('home_id', 'start')
            self.declare_parameter('autostart', False)
            self.declare_parameter('auto_mock', False)
            self.declare_parameter('observation_timeout', 1.)
            for operation, timeout in TaskCoordinator.DEFAULT_TIMEOUTS.items():
                self.declare_parameter('timeouts.'+operation, timeout)
            simulation = self.get_parameter('simulation').value
            self.engine = TaskCoordinator(capacity=self.get_parameter('capacity').value,
                                          simulation=simulation,
                                          observation_timeout=self.get_parameter('observation_timeout').value,
                                          timeouts={op: self.get_parameter('timeouts.'+op).value
                                                    for op in TaskCoordinator.DEFAULT_TIMEOUTS})
            auto_mock = self.get_parameter('auto_mock').value
            if auto_mock and not simulation:
                raise ValueError('auto_mock is restricted to simulation mode')
            self.mock = MockBackend(self.get_parameter('task_code').value) if auto_mock else None
            prefix = 'mission_sim' if simulation else 'mission'
            command_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.VOLATILE)
            status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.command_pub = self.create_publisher(String, prefix+'/command', command_qos)
            self.status_pub = self.create_publisher(String, prefix+'/status', status_qos)
            self.create_subscription(String, prefix+'/control', self.control, command_qos)
            self.create_subscription(String, prefix+'/feedback', self.feedback, command_qos)
            self._controls = set()
            self._last_log_phase = None
            self.create_timer(.1, self.update)
            if self.get_parameter('autostart').value:
                if not simulation:
                    raise ValueError('autostart is restricted to isolated simulation mode')
                self.engine.start(self.get_parameter('home_id').value)
            self.get_logger().info('任务框架就绪；模式=' + ('纯模拟，无硬件' if simulation else '等待外部后端，无硬件驱动'))

        def _parse(self, message):
            if len(message.data) > 65536:
                raise ValueError('message too large')
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise ValueError('JSON object required')
            return data

        def control(self, message):
            try:
                data = self._parse(message)
                stamp = data.get('issued_at_unix_s')
                request = data.get('request_id')
                if (type(data.get('schema_version')) is not int or data['schema_version'] != 1
                        or data.get('simulation') is not self.engine.simulation
                        or type(stamp) not in (int, float) or not 0 <= time.time()-stamp < 2
                        or not isinstance(request, str) or not request or request in self._controls):
                    raise ValueError('stale, duplicate or invalid control')
                if len(self._controls) >= 1024:
                    raise ValueError('control deduplication limit reached; restart idle node')
                if data.get('operation') == 'start':
                    self.engine.start(data.get('home_id', self.get_parameter('home_id').value))
                elif data.get('operation') == 'cancel':
                    self.engine.cancel()
                else:
                    raise ValueError('unsupported control operation')
                self._controls.add(request)
            except (ValueError, TypeError) as error:
                self.get_logger().warning('拒绝控制消息: ' + str(error))

        def feedback(self, message):
            try:
                self.engine.feedback(self._parse(message))
            except (ValueError, TypeError, KeyError) as error:
                self.get_logger().warning('拒绝反馈消息: ' + str(error))

        def update(self):
            self.engine.tick()
            for command in self.engine.drain_commands():
                message = String()
                message.data = json.dumps(command, ensure_ascii=False, allow_nan=False)
                self.command_pub.publish(message)
            if self.mock is not None and self.engine.pending is not None:
                self.engine.feedback(self.mock.reply(self.engine.pending.to_dict(), time.time()))
            state = self.engine.snapshot()
            message = String()
            message.data = json.dumps(state, ensure_ascii=False, allow_nan=False)
            self.status_pub.publish(message)
            if state['phase'] != self._last_log_phase:
                self._last_log_phase = state['phase']
                self.get_logger().info('任务状态: '+message.data)

    return MissionNode()


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = None
    try:
        node = create_node()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.engine.cancel()
                node.update()  # Best effort only; local backend watchdogs remain mandatory.
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
