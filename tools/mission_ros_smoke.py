"""Bounded ROS 2 pub/sub mission simulation, isolated from real command topics."""

import argparse
import json
import time


def main(argv=None):
    parser = argparse.ArgumentParser(description='ROS 2 全流程模拟，不连接相机、串口或电机')
    parser.add_argument('--capacity', type=int, choices=(1, 2, 3), default=3)
    parser.add_argument('--task-code', default='156+123+516+231')
    parser.add_argument('--fail-operation', default=None)
    parser.add_argument('--timeout', type=float, default=30.)
    args = parser.parse_args(argv)
    if not 0 < args.timeout <= 60:
        parser.error('--timeout must be in (0, 60] seconds')

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String
    from pi_vision.mission import Phase
    from pi_vision.mission_demo import MockBackend
    from pi_vision.mission_ros import create_node

    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
    status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)

    class SimulatedRobot(Node):
        def __init__(self):
            super().__init__('mission_simulated_robot')
            self.backend = MockBackend(args.task_code, args.fail_operation)
            self.current = None
            self.commands = []
            self.status = None
            self.pub = self.create_publisher(String, 'mission_sim/feedback', qos)
            self.create_subscription(String, 'mission_sim/command', self.command, qos)
            self.create_subscription(String, 'mission_sim/status', self.state, status_qos)
            self.create_timer(.1, self.update)

        def command(self, message):
            command = json.loads(message.data)
            if command.get('simulation') is not True:
                raise RuntimeError('real command reached simulated backend')
            self.commands.append(command)
            self.current = command

        def state(self, message):
            self.status = json.loads(message.data)

        def update(self):
            if self.current is None:
                return
            response = self.backend.reply(self.current, time.time())
            message = String()
            message.data = json.dumps(response, allow_nan=False)
            self.pub.publish(message)
            if response['status'] != 'observation':
                self.current = None

    rclpy.init(args=['--ros-args', '-p', 'simulation:=true', '-p', f'capacity:={args.capacity}'])
    executor = SingleThreadedExecutor()
    mission = robot = None
    try:
        robot, mission = SimulatedRobot(), create_node()
        executor.add_node(robot)
        executor.add_node(mission)
        deadline = time.monotonic()+args.timeout
        while time.monotonic() < deadline and (
                not mission.command_pub.get_subscription_count() or not robot.pub.get_subscription_count()):
            executor.spin_once(timeout_sec=.05)
        if not mission.command_pub.get_subscription_count() or not robot.pub.get_subscription_count():
            raise RuntimeError('ROS discovery did not connect the simulated nodes')
        mission.engine.start('simulated_drawn_start')
        expected = Phase.FAULT if args.fail_operation else Phase.COMPLETED
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            if robot.status and robot.status['phase'] == expected.value:
                break
        state = mission.engine.snapshot()
        ok = (robot.status is not None and robot.status['phase'] == expected.value
              and state['phase'] == expected.value and state['stop_confirmed'])
        if not args.fail_operation:
            ok = ok and state['counts'] == {'picked': 12, 'placed': 12, 'stacked': 3}
        print(json.dumps({'evidence': 'ros2_pubsub_simulation_only', 'ok': ok,
                          'published_commands': len(robot.commands),
                          'status_received': robot.status is not None, **state}, ensure_ascii=False))
        return 0 if ok else 1
    finally:
        if mission is not None:
            mission.destroy_node()
        if robot is not None:
            robot.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
