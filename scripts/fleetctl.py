#!/usr/bin/env python3
"""Explicit fleet actions from a terminal in the existing container."""
import argparse
import json
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['select', 'takeoff', 'land', 'hover', 'hover_all', 'reset_map', 'status'])
    parser.add_argument('uav', nargs='?', choices=['uav1', 'uav2'])
    args = parser.parse_args()
    if args.action not in ('hover_all', 'status') and not args.uav:
        parser.error('Specify uav1 or uav2 explicitly')
    rclpy.init()
    node = Node('fleet_terminal_command')
    latest = [None]
    request_id = str(uuid.uuid4())
    acknowledgements = {}
    targets = ['uav1', 'uav2'] if args.action == 'hover_all' else [args.uav]
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = []
    if args.action in ('status', 'select'):
        topic = '/fire_fleet/status' if args.action == 'status' else '/fire_fleet/settings'
        subscriptions.append(node.create_subscription(String, topic, lambda msg: latest.__setitem__(0, msg.data), durable))
    else:
        def receive_ack(msg):
            ack = json.loads(msg.data)
            if ack['request_id'] == request_id:
                acknowledgements[ack['uav']] = ack
        for target in targets:
            subscriptions.append(node.create_subscription(String, f'/fire_fleet/{target}/action_ack', receive_ack, durable))
    publisher = node.create_publisher(String, '/fire_fleet/select_uav' if args.action == 'select' else '/fire_fleet/action', 10)
    try:
        deadline = time.monotonic()+10
        while args.action != 'status' and publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if args.action != 'status':
            if publisher.get_subscription_count() == 0:
                raise RuntimeError('Fleet interface is unavailable; start the fleet scenario first.')
            latest[0] = None
            data = args.uav if args.action == 'select' else json.dumps(
                {'uav': args.uav, 'action': args.action, 'request_id': request_id})
            publisher.publish(String(data=data))
        deadline = time.monotonic()+(60 if args.action == 'takeoff' else 15)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.action not in ('status', 'select'):
                if all(target in acknowledgements for target in targets):
                    for target in targets:
                        ack = acknowledgements[target]
                        print(target+': '+ack['message'])
                    if not all(ack['success'] for ack in acknowledgements.values()):
                        raise RuntimeError('An action was rejected; see the acknowledgement above.')
                    return
                continue
            text = latest[0]
            if text is None:
                continue
            if args.action == 'status':
                print(json.dumps(json.loads(text), indent=2)); return
            if args.action == 'select':
                if json.loads(text)['uav'] == args.uav:
                    print('Selected '+args.uav+' in RViz'); return
                continue
        raise RuntimeError('No final acknowledgement; check fleet/status logs before repeating the command.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
