"""One camera owner + preview + isolated ROS publisher/subscriber timing probe."""
import argparse
import json
import os
from pathlib import Path
import signal
import threading
import time
import uuid


def main():
    parser=argparse.ArgumentParser(description='圆环相机/识别/本机ROS坐标输出计时，不打开串口或动作端')
    parser.add_argument('--camera',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--roi',nargs=4,type=int,default=[160,0,960,720])
    parser.add_argument('--position-id',type=int,choices=(1,2,3),default=2)
    parser.add_argument('--seconds',type=int,default=60)
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='74' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':
        parser.error('requires isolated domain 74 and LOCALHOST discovery')
    if not 5<=args.seconds<=300:parser.error('seconds must be 5..300')
    out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from std_msgs.msg import String
    from pi_vision.vision_ros import make_source,create_node,parse_args
    from pi_vision.stream import StreamHandler,ThreadingHTTPServer
    from pi_vision import stream
    options,_=parse_args(['--camera',args.camera,'--material-backend','hsv',
        '--ring-backend','morphology','--ring-roi',*map(str,args.roi),
        '--position-digits','--position-orientation','0'])
    source=make_source(options)
    # The same FrameStreamer serves preview; never open another camera.
    source.preview_enabled=True
    stream._streamer=source
    server=ThreadingHTTPServer(('0.0.0.0',8090),StreamHandler)
    rclpy.init(args=[])
    executor=SingleThreadedExecutor();events=[];node=client=None;stop=False;server_started=False
    def terminate(*_):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGTERM,terminate);signal.signal(signal.SIGINT,terminate)
    def emit(event):
        events.append(event)
        with (out/'events.jsonl').open('a') as f:f.write(json.dumps(event,ensure_ascii=False)+'\n')
    started=time.monotonic()
    try:
        source.start()
        threading.Thread(target=server.serve_forever,daemon=True).start()
        server_started=True
        node=create_node(source,args.camera,timing_sink=emit)
        client=Node('ring_camera_timing_receiver')
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        publisher=client.create_publisher(String,'mission/command',qos)
        def receive(msg):
            received=time.monotonic();reply=json.loads(msg.data)
            if reply.get('status')=='observation' and reply['data'].get('targets'):
                timing=reply['data'].get('timing',{})
                emit(dict(event='ros_received',at_monotonic_s=received,
                    source_sequence=reply['data']['source_sequence'],command_id=reply['command_id'],
                    request_to_received_ms=(received-request_at)*1000,
                    source_to_received_ms=(received-timing['acquired_monotonic_s'])*1000 if timing else None,
                    detection_to_received_ms=(received-timing['completed_monotonic_s'])*1000 if timing else None,
                    ready_to_received_ms=(received-timing['ready_monotonic_s'])*1000 if timing else None,
                    timing=timing,targets=reply['data']['targets']))
            elif reply.get('status')=='failed':emit(dict(event='failed',reply=reply))
        client.create_subscription(String,'mission/feedback',receive,qos)
        executor.add_node(node);executor.add_node(client)
        deadline=time.monotonic()+5
        while not publisher.get_subscription_count() and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
        if not publisher.get_subscription_count():raise RuntimeError('ROS discovery timeout')
        request_at=time.monotonic()
        command=dict(schema_version=1,simulation=False,run_id=uuid.uuid4().hex,command_id=uuid.uuid4().hex,
            operation='locate_position',params=dict(zone='rough',position_id=args.position_id,layer=0),
            issued_at_unix_s=time.time(),timeout_s=args.seconds)
        emit(dict(event='request',at_monotonic_s=request_at,command=command))
        msg=String();msg.data=json.dumps(command);publisher.publish(msg)
        print('Ring camera timing ready; preview port 8090; no serial/actuators',flush=True)
        deadline=time.monotonic()+args.seconds
        while not stop and time.monotonic()<deadline:executor.spin_once(timeout_sec=.02)
    finally:
        state=source.get_state()
        (out/'final_state.json').write_text(json.dumps(state,ensure_ascii=False,indent=2))
        receipts=[e for e in events if e['event']=='ros_received']
        publications=[e for e in events if e['event']=='ros_publish_returned' and e.get('timing')]
        def stats(values):
            values=sorted(v for v in values if v is not None)
            return dict(count=len(values),min_ms=min(values),median_ms=values[len(values)//2],
                        p95_ms=values[min(len(values)-1,int(len(values)*.95))],max_ms=max(values)) if values else None
        summary=dict(evidence='live_camera_same_host_ros_transport_not_serial',elapsed_s=time.monotonic()-started,
            coordinates_received=len(receipts),first_coordinate_after_request_ms=receipts[0]['request_to_received_ms'] if receipts else None,
            detection_to_publish_return=stats([(e['at_monotonic_s']-e['timing']['completed_monotonic_s'])*1000 for e in publications]),
            detection_to_received=stats([e['detection_to_received_ms'] for e in receipts]),
            source_to_received=stats([e['source_to_received_ms'] for e in receipts]),
            detection_processing=stats([e['timing']['source_to_detection_ms'] for e in receipts if e['timing']]),
            capture=state['capture'])
        (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
        print(json.dumps(summary,ensure_ascii=False),flush=True)
        if node is not None:node.backend.cancel();node.destroy_node()
        if client is not None:client.destroy_node()
        if server_started:server.shutdown()
        server.server_close();source.stop();executor.shutdown()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
