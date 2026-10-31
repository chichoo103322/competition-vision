"""Bounded ROS pub/sub alignment simulation. No physical devices or mission nodes."""

import argparse
import json
import os
import time


def main(argv=None):
    parser = argparse.ArgumentParser(description='局部对准 ROS 2 纯模拟检查')
    parser.add_argument('--failure', choices=['target_lost', 'motion_failed', 'motion_timeout', 'stop_timeout'])
    args = parser.parse_args(argv)
    if os.environ.get('ROS_DOMAIN_ID') != '75' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        parser.error('requires ROS_DOMAIN_ID=75 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String
    from pi_vision.alignment import AlignmentController
    from pi_vision.alignment_demo import AlignmentPlant, simulation_calibration
    from pi_vision.alignment_ros import create_node

    rclpy.init(args=[])
    executor = SingleThreadedExecutor()
    node = plant_node = None
    engine = AlignmentController(simulation_calibration(), simulation=True)
    plant = AlignmentPlant(args.failure)
    trace, states = [], []
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
    status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    try:
        node = create_node(engine)
        plant_node = Node('alignment_simulated_plant')
        pubs = {c: plant_node.create_publisher(String, 'alignment_sim/'+c+'_feedback', qos)
                for c in ('vision', 'motion')}
        def receive(channel, msg):
            command = json.loads(msg.data)
            trace.append(dict(channel=channel, command=command))
            plant.command(channel, command)
        for channel in ('vision', 'motion'):
            plant_node.create_subscription(String, 'alignment_sim/'+channel+'_command',
                lambda msg, c=channel: receive(c, msg), qos)
        plant_node.create_subscription(String, 'alignment_sim/status',
            lambda msg: states.append(json.loads(msg.data)), status_qos)
        control_pub = plant_node.create_publisher(String, 'alignment_sim/control', qos)
        def update():
            for channel, reply in plant.update(time.time()):
                pubs[channel].publish(node.message(reply))
        plant_node.create_timer(.1, update)
        executor.add_node(node)
        executor.add_node(plant_node)
        deadline = time.monotonic()+20.
        connected = lambda: (all(p.get_subscription_count() for p in pubs.values())
                              and all(p.get_subscription_count() for p in node.pubs.values())
                              and control_pub.get_subscription_count())
        while time.monotonic() < deadline and not connected():
            executor.spin_once(timeout_sec=.05)
        if not connected():
            raise RuntimeError('ROS discovery timeout')
        control_pub.publish(node.message(dict(schema_version=1, simulation=True, request_id='smoke_start',
            issued_at_unix_s=time.time(), operation='start', color_id=1)))
        expected = 'fault' if args.failure else 'aligned'
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            if states and states[-1]['phase'] == expected:
                break
        state = engine.snapshot()
        ok = (bool(states) and states[-1]['phase'] == expected and state['phase'] == expected
              and state['stop_confirmed'] == (args.failure != 'stop_timeout'))
        # Flush final stop-perception message before shutting down the test nodes.
        for _ in range(5):
            executor.spin_once(timeout_sec=.02)
        print(json.dumps(dict(evidence='ros2_alignment_simulation_only', ok=ok,
            failure=args.failure, state=state, commands=trace,
            status_received=bool(states)), ensure_ascii=False, allow_nan=False))
        return 0 if ok else 1
    finally:
        if node is not None:
            node.destroy_node()
        if plant_node is not None:
            plant_node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
