#!/usr/bin/env python3
"""Configure and explicitly start cooperative missions from a fleet tmux terminal."""
import argparse
import json
import time
import uuid
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('status')
    sub.add_parser('start_all', help='Start cooperative mode on both already airborne drones')
    survey = sub.add_parser('survey', help='Split one shared rectangle; both missions must be paused')
    survey.add_argument('bounds', nargs=4, type=float, metavar='XYXY')
    survey.add_argument('--axis', choices=['auto', 'x', 'y'], default='auto')
    survey.add_argument('--altitude', type=float, default=5.)
    survey.add_argument('--spacing', type=float, default=2.)
    survey.add_argument('--revisit', type=float, default=20.)
    for command in ('start', 'pause', 'resume', 'append', 'clear', 'pop', 'preset', 'route', 'options', 'area', 'assign', 'revisit'):
        p = sub.add_parser(command)
        p.add_argument('uav', choices=['uav1', 'uav2'])
        if command == 'start':
            p.add_argument('--mode', choices=['route', 'cooperative'], default='route')
        elif command == 'append':
            p.add_argument('position', nargs=3, type=float, metavar='XYZ')
            p.add_argument('--dwell', type=float, default=3.)
            p.add_argument('--fires', nargs='*', type=int, default=[])
        elif command == 'route':
            p.add_argument('file', help='YAML list of waypoints, or a mapping containing waypoints')
        elif command == 'options':
            group = p.add_mutually_exclusive_group()
            group.add_argument('--loop', action='store_true')
            group.add_argument('--once', action='store_true')
            p.add_argument('--dwell', type=float)
        elif command == 'area':
            p.add_argument('bounds', nargs=4, type=float, metavar='XYXY')
            p.add_argument('--spacing', type=float, default=1.)
            p.add_argument('--altitude', type=float, default=5.)
        elif command == 'assign':
            p.add_argument('fire_ids', nargs='*', type=int, help='Empty list clears this drone\'s assignments')
        elif command == 'revisit':
            p.add_argument('seconds', type=float)
    args = parser.parse_args()
    actions = dict(start='mission_start', pause='mission_pause', resume='mission_resume', start_all='mission_start_all')
    command = args.command
    data = dict(uav=getattr(args, 'uav', None), request_id=str(uuid.uuid4()))
    if command in actions:
        data.update(action=actions[command], mode=getattr(args, 'mode', 'cooperative'))
        topic = '/fire_fleet/action'
    else:
        topic = '/fire_fleet/mission/config'
        ops = dict(append='append_waypoint', clear='clear_route', pop='pop_waypoint', preset='load_preset',
                   route='set_route', options='set_options', area='set_area', assign='assign_fires', revisit='set_revisit', survey='set_survey')
        data['op'] = ops.get(command)
        if command == 'survey':
            data.update(uav='all', survey=dict(bounds=args.bounds, split_axis=args.axis,
                altitude_m=args.altitude, spacing_m=args.spacing, revisit_sim_s=args.revisit))
        elif command == 'append':
            data['waypoint'] = dict(position=args.position, dwell_sim_s=args.dwell, fire_ids=args.fires)
        elif command == 'route':
            value = yaml.safe_load(Path(args.file).read_text())
            data['waypoints'] = value['waypoints'] if isinstance(value, dict) else value
        elif command == 'options':
            if args.loop or args.once:
                data['loop'] = args.loop
            if args.dwell is not None:
                data['dwell_sim_s'] = args.dwell
        elif command == 'area':
            data['area'] = dict(bounds=args.bounds, spacing_m=args.spacing, altitude_m=args.altitude)
        elif command == 'assign':
            data['fire_ids'] = args.fire_ids
        elif command == 'revisit':
            data['revisit_sim_s'] = args.seconds
    rclpy.init()
    node = Node('mission_terminal_command')
    replies, latest = {}, [None]
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def receive(msg):
        ack = json.loads(msg.data)
        if ack.get('request_id') == data['request_id']:
            replies[ack['uav']] = ack
    if command == 'status':
        node.create_subscription(String, '/fire_fleet/mission/status', lambda msg: latest.__setitem__(0, msg.data), durable)
    elif command in actions:
        for target in (['uav1', 'uav2'] if command == 'start_all' else [args.uav]):
            node.create_subscription(String, f'/fire_fleet/{target}/action_ack', receive, durable)
    else:
        node.create_subscription(String, '/fire_fleet/mission/config_ack', receive, 10)
    publisher = node.create_publisher(String, topic, 10)
    try:
        if command != 'status':
            deadline = time.monotonic()+10
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            if publisher.get_subscription_count() == 0:
                raise RuntimeError('Fleet mission interface unavailable; start the fleet scenario first')
            publisher.publish(String(data=json.dumps(data)))
        deadline = time.monotonic()+30
        count = 2 if command == 'start_all' else 1
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if latest[0]:
                print(json.dumps(json.loads(latest[0]), indent=2)); return
            if len(replies) == count:
                for target, reply in replies.items():
                    print(target+': '+reply['message'])
                if not all(reply['success'] for reply in replies.values()):
                    raise RuntimeError('Mission command rejected')
                return
        raise RuntimeError('No acknowledgement; check mission status before repeating the command')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
