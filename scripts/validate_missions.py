#!/usr/bin/env python3
"""Exercise milestone 4 in the simulated fleet; leaves both drones landed.

Run only with the fire_fleet scenario. Uses its explicit operator command topics.
"""
import json
import math
from pathlib import Path
import time
import uuid

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = Node('validate_fire_missions', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    latest, acks, configs, accepted = {}, {}, {}, {'uav1': 0, 'uav2': 0}
    report = {'checks': []}
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for key, topic in [('mission', '/fire_fleet/mission/status'), ('fleet', '/fire_fleet/status'),
                       ('map', '/fire_fleet/fire_map/status')]:
        node.create_subscription(String, topic, lambda msg, key=key: latest.__setitem__(key, json.loads(msg.data)), durable)
    def ack(msg):
        value = json.loads(msg.data)
        acks[value['request_id'], value['uav']] = value
    def config_ack(msg):
        value = json.loads(msg.data)
        configs[value['request_id']] = value
    node.create_subscription(String, '/fire_fleet/mission/config_ack', config_ack, 10)
    for name in accepted:
        node.create_subscription(String, f'/fire_fleet/{name}/action_ack', ack, durable)
        def goal(msg, name=name):
            accepted[name] += 1
        node.create_subscription(PoseStamped, f'/fire_fleet/{name}/accepted_goal', goal, durable)
    pubs = {name: node.create_publisher(String, '/fire_fleet/'+topic, 10)
            for name, topic in [('action', 'action'), ('goal', 'map_goal'), ('config', 'mission/config')]}

    def wait(predicate, text, timeout=240.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if predicate():
                report['checks'].append(text)
                print('PASS: '+text, flush=True)
                return
        raise AssertionError(text+'; missions: '+json.dumps(latest.get('mission', {})))

    def state(name):
        return latest.get('mission', {}).get('uavs', {}).get(name, {})

    def action(name, operation, **extra):
        request = str(uuid.uuid4())
        pubs['action'].publish(String(data=json.dumps(dict(uav=name, action=operation, request_id=request, **extra))))
        wait(lambda: (request, name) in acks, name+' acknowledges '+operation, 90.)
        if not acks[request, name]['success']:
            raise AssertionError(acks[request, name])

    def configure(name, op, **extra):
        request = str(uuid.uuid4())
        pubs['config'].publish(String(data=json.dumps(dict(uav=name, op=op, request_id=request, **extra))))
        wait(lambda: request in configs, name+' configures '+op, 30.)
        if not configs[request]['success']:
            raise AssertionError(configs[request])

    def goal(name, point):
        before = accepted[name]
        pubs['goal'].publish(String(data=json.dumps(dict(uav=name, enabled=True, frame='common_origin',
            x=point[0], y=point[1], altitude_m=point[2], heading=0.))))
        wait(lambda: accepted[name] > before, name+' accepts manual goal', 30.)
        wait(lambda: state(name).get('position_m') and math.dist(state(name)['position_m'], point) < .4,
             name+' reaches manual destination '+str(point), 480.)

    try:
        wait(lambda: all(state(n).get('position_m') and state(n).get('coordinator_fresh') for n in accepted),
             'Both mission drivers and coordinator connected')
        assert all(not state(n)['active'] for n in accepted), 'Missions must not autostart'
        wait(lambda: all(not latest.get('fleet', {}).get('uavs', {}).get(n, {}).get('armed', True) for n in accepted),
             'Both drones initially disarmed')
        for name in accepted:
            action(name, 'takeoff')
            wait(lambda name=name: state(name).get('ready'), name+' ready for missions')
        for name in accepted:
            action(name, 'mission_start', mode='route')
        wait(lambda: all(state(n).get('cycles', 0) >= 1 for n in accepted),
             'Both independent routes visit two points, inspect with fresh cameras and loop', 900.)
        wait(lambda: len(latest.get('map', {}).get('fires', [])) == 3, 'All three fires detected during patrol')
        report['route_checkpoint'] = latest['mission']
        action('uav1', 'hover')
        wait(lambda: not state('uav1').get('active', True), 'Manual hover pauses uav1 mission')
        first_count, other_visits = accepted['uav1'], state('uav2')['completed_visits']
        wait(lambda: state('uav2')['completed_visits'] > other_visits,
             'uav2 keeps inspecting while uav1 is under manual control')
        assert accepted['uav1'] == first_count and not state('uav1')['active'], 'Old mission resumed after hover'
        goal('uav1', [7., -3., 5.])
        assert not state('uav1')['active']
        action('uav1', 'mission_resume')
        before = state('uav1')['completed_visits']
        wait(lambda: state('uav1')['completed_visits'] > before, 'Explicit resume continues uav1 route', 360.)
        for name in accepted:
            action(name, 'mission_pause')
        wait(lambda: all(not state(n)['active'] for n in accepted), 'Both missions paused before reconfiguration')
        # Revisit uses detected IDs, never configured scene names/positions.
        fires = latest['map']['fires']
        south = [f['id'] for f in fires if f['centre_m'][1] < 3.]
        north = [f['id'] for f in fires if f['centre_m'][1] >= 3.]
        configure('uav1', 'assign_fires', fire_ids=south)
        configure('uav2', 'assign_fires', fire_ids=north)
        for name in accepted:
            configure(name, 'set_revisit', revisit_sim_s=3.)
        goal('uav1', [4., -6., 5.])
        goal('uav2', [4., 6., 5.])
        start_sim = node.get_clock().now().nanoseconds/1e9
        wait(lambda: node.get_clock().now().nanoseconds/1e9-start_sim > 5., 'Out-of-view observations become stale', 60.)
        for name in accepted:
            action(name, 'mission_start', mode='cooperative')
        base_visits = {n: state(n)['completed_visits'] for n in accepted}
        wait(lambda: all(any(h['kind'] == 'fire' and h['stamp_sim_s'] > start_sim
                            for h in state(n)['history']) for n in accepted),
             'Both drones revisit separately assigned stale fires and confirm them with their own cameras', 900.)
        def independent_tasks():
            ids = [set((state(n).get('target') or {}).get('fire_ids', [])) for n in accepted]
            return not ids[0].intersection(ids[1])
        assert independent_tasks()
        report['cooperative_checkpoint'] = latest['mission']
        wait(lambda: all(state(n)['completed_visits'] >= base_visits[n]+2 for n in accepted),
             'Cooperative patrol continues to another search or inspection task', 600.)
        reset = node.create_client(Trigger, '/fire_fleet/fire_map/reset')
        wait(reset.service_is_ready, 'Shared-map reset is available', 30.)
        future = reset.call_async(Trigger.Request())
        wait(future.done, 'Shared-map reset acknowledged', 30.)
        assert future.result().success
        wait(lambda: all(not state(n)['active'] for n in accepted),
             'Fire-map reset pauses both cooperative missions instead of reusing old IDs')
        for name, point in [('uav1', [0., -6., 5.]), ('uav2', [0., 6., 5.])]:
            goal(name, point)
            action(name, 'land')
            wait(lambda name=name: not latest['fleet']['uavs'][name]['armed'], name+' lands and disarms', 480.)
        report['success'] = True
    except Exception as error:
        report.update(success=False, error=str(error))
        # Cancel ownership even if an assertion fails; no further automatic goals.
        for name in accepted:
            pubs['action'].publish(String(data=json.dumps(dict(uav=name, action='hover', request_id=str(uuid.uuid4())))))
        deadline = time.monotonic()+3.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        raise
    finally:
        report['final_state'] = latest
        out = Path(__file__).resolve().parents[1]/'artifacts/milestone4_check.json'
        out.write_text(json.dumps(report, indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
