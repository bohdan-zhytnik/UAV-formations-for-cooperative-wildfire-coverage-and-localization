#!/usr/bin/env python3
"""Reproduce the parked-uav1 route bug with uav2 ALREADY airborne in Gazebo.

Runs uav2's preset for one loop, then pauses in hover. No takeoff or landing.
"""
import argparse
import json
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from mission_logic import point_segment_distance, segment_distance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exercise-flight', action='store_true')
    args = parser.parse_args()
    if not args.exercise_flight:
        parser.error('Pass --exercise-flight to explicitly run this simulated flight check')
    rclpy.init()
    node = Node('validate_parked_route')
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    latest, acknowledgements = {}, {}
    report = {'checks': []}
    node.create_subscription(String, '/fire_fleet/mission/status',
        lambda msg: latest.update(json.loads(msg.data)), durable)
    def receive(msg):
        data = json.loads(msg.data)
        acknowledgements[data['request_id']] = data
    node.create_subscription(String, '/fire_fleet/uav2/action_ack', receive, durable)
    node.create_subscription(String, '/fire_fleet/mission/config_ack', receive, 10)
    config = node.create_publisher(String, '/fire_fleet/mission/config', 10)
    actions = node.create_publisher(String, '/fire_fleet/action', 10)
    def state(name):
        return latest.get('uavs', {}).get(name, {})
    def wait(predicate, label, timeout=60.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if predicate():
                print('PASS: '+label, flush=True)
                report['checks'].append(label)
                return
        raise AssertionError(label+': '+json.dumps(latest))
    def command(publisher, **data):
        identifier = str(uuid.uuid4())
        publisher.publish(String(data=json.dumps(dict(uav='uav2', request_id=identifier, **data))))
        wait(lambda: identifier in acknowledgements, str(data)+' acknowledged', 30.)
        assert acknowledgements[identifier]['success'], acknowledgements[identifier]
    try:
        wait(lambda: state('uav2').get('ready') and state('uav2').get('coordinator_fresh')
             and state('uav1').get('hardware_fresh') and state('uav1').get('armed') is False,
             'uav2 is ready; uav1 is confirmed disarmed')
        assert not state('uav2')['active']
        command(config, op='load_preset')
        wait(lambda: len(state('uav2').get('route', [])) == 2, 'Two preset waypoints loaded')
        own, other = state('uav2')['position_m'], state('uav1')['position_m']
        target = state('uav2')['route'][0]['position']
        old_clearance = segment_distance(own, target, other, other)
        new_clearance = point_segment_distance(other, own, target)
        assert old_clearance < 6.5 and new_clearance > 2., (old_clearance, new_clearance)
        report.update(initial=latest.copy(), old_horizontal_clearance_m=old_clearance,
                      planned_ground_clearance_m=new_clearance)
        print('PASS: The old 6.5 m horizontal check would block this exact flight', flush=True)
        report['checks'].append('Reproduced the parked-vehicle blocking geometry')
        initial_cycles, initial_visits = state('uav2')['cycles'], state('uav2')['completed_visits']
        command(actions, action='mission_start', mode='route')
        wait(lambda: state('uav2').get('phase') in ('travelling', 'dwelling'),
             'MRS accepts the route and uav2 starts moving')
        wait(lambda: state('uav2')['completed_visits'] > initial_visits,
             'uav2 reaches and inspects the first waypoint', 480.)
        wait(lambda: state('uav2')['cycles'] > initial_cycles,
             'uav2 reaches the second waypoint and completes a loop', 480.)
        assert state('uav1')['armed'] is False
        command(actions, action='mission_pause')
        wait(lambda: not state('uav2')['active'], 'Mission pauses and uav2 holds position')
        report['success'] = True
    except Exception as error:
        report.update(success=False, error=str(error))
        actions.publish(String(data=json.dumps(dict(uav='uav2', action='hover', request_id=str(uuid.uuid4())))))
        deadline = time.monotonic()+3.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        raise
    finally:
        report['final'] = latest
        (Path(__file__).resolve().parents[1]/'artifacts/preset_route_fix_check.json').write_text(json.dumps(report, indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
