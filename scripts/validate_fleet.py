#!/usr/bin/env python3
"""Fleet integration check. --exercise-flight moves ONLY this simulated fleet.

Runs through the same addressed command protocol as the RViz fleet tool. Leaves
both vehicles landed and map commands disabled after a successful flight check.
"""
import argparse
import json
import math
from pathlib import Path
import time
import uuid
import yaml

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from mrs_msgs.msg import HwApiStatus, ControlManagerDiagnostics
from nav_msgs.msg import Path as FlightPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exercise-flight', action='store_true')
    parser.add_argument('--timeout-scale', type=float, default=3., help='Wall-time budget multiplier for slow simulation')
    args = parser.parse_args()
    if not math.isfinite(args.timeout_scale) or args.timeout_scale <= 0:
        parser.error('--timeout-scale must be positive and finite')
    rclpy.init()
    node = Node('validate_fire_fleet', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    latest, counts, acks, subscriptions = {}, {}, {}, []
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    names = ['uav1', 'uav2']
    report = {'checks': [], 'flight_exercised': args.exercise_flight}
    project = Path(__file__).resolve().parents[1]
    truth = yaml.safe_load((project/'config/fleet/fires.yaml').read_text())['fires']
    if args.exercise_flight and len(truth) != 3:
        parser.error('This legacy flight exercise requires the three-fire scene; use validate_shared_survey.py --exercise-flight for the current six-fire survey.')
    tf = Buffer()
    listener = TransformListener(tf, node)

    def subscribe(key, topic, kind, qos):
        def receive(msg):
            latest[key] = msg
            counts[key] = counts.get(key, 0)+1
        subscriptions.append(node.create_subscription(kind, topic, receive, qos))

    subscribe('status', '/fire_fleet/status', String, durable)
    subscribe('settings', '/fire_fleet/settings', String, durable)
    subscribe('notice', '/fire_fleet/notice', String, durable)
    subscribe('shared', '/fire_fleet/fire_map/status', String, durable)
    subscribe('shared_markers', '/fire_fleet/fire_map/markers', MarkerArray, durable)
    reset_shared = node.create_client(Trigger, '/fire_fleet/fire_map/reset')
    for name in names:
        prefix = '/fire_fleet/'+name
        for key, topic, kind, qos in [
            ('hw', f'/{name}/hw_api/status', HwApiStatus, qos_profile_sensor_data),
            ('control', f'/{name}/control_manager/diagnostics', ControlManagerDiagnostics, qos_profile_sensor_data),
            ('rgb', f'/{name}/rgb_down/image_raw', Image, qos_profile_sensor_data),
            ('thermal', f'/{name}/thermal_down/image_raw', Image, qos_profile_sensor_data),
            ('image', prefix+'/detection_image', Image, qos_profile_sensor_data),
            ('fire', prefix+'/fire_status', String, durable),
            ('batch', prefix+'/observations', String, 5),
            ('raw_markers', prefix+'/observation_markers', MarkerArray, durable),
            ('command', prefix+'/command_status', String, durable),
            ('accepted', prefix+'/accepted_goal', PoseStamped, durable),
            ('markers', prefix+'/markers', MarkerArray, durable),
            ('path', prefix+'/path', FlightPath, durable)]:
            subscribe(name+'/'+key, topic, kind, qos)
        def ack(msg):
            value = json.loads(msg.data)
            acks[value['request_id'], value['uav']] = value
        subscriptions.append(node.create_subscription(String, prefix+'/action_ack', ack, durable))
    pubs = {key: node.create_publisher(String, '/fire_fleet/'+key, 10)
            for key in ['action', 'map_goal', 'select_uav']}

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)

    def wait(predicate, text, timeout=60):
        end = time.monotonic()+timeout*args.timeout_scale
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if predicate():
                report['checks'].append(text)
                print('PASS: '+text, flush=True)
                return
        raise AssertionError(text+'; fleet: '+str(latest.get('status')))

    def shared_fires():
        return json.loads(latest.get('shared', String(data='{"fires":[]}')).data)['fires']

    def matched_truth(required):
        matches = {}
        for target in truth:
            if target['name'] not in required:
                continue
            candidates = [fire for fire in shared_fires()
                          if np.linalg.norm(np.array(fire['centre_m'])-target['position']) < .4]
            if len(candidates) != 1:
                return {}
            matches[target['name']] = candidates[0]
        if len(matches) != len(required) or len({fire['id'] for fire in matches.values()}) != len(required):
            return {}
        return matches

    def validate_estimates(matches):
        for target in truth:
            if target['name'] not in matches:
                continue
            fire = matches[target['name']]
            expected_area = float(np.prod(target['size']))
            assert abs(fire['area_m2']-expected_area) < .3*expected_area, (target, fire)
            assert abs(fire['mean_kelvin']-target['temperature_kelvin']) < 4, (target, fire)
            assert target['temperature_kelvin']-3 < fire['max_kelvin'] < target['temperature_kelvin']+8
            assert fire['window_samples'] <= 30
            assert np.linalg.eigvalsh(fire['covariance_xy_m2']).min() >= .03**2-1e-9
            report.setdefault('truth_errors_m', {})[target['name']] = float(np.linalg.norm(
                np.array(fire['centre_m'])-target['position']))

    def position(name, frame='common_origin'):
        try:
            t = tf.lookup_transform(frame, name+'/fcu', Time()).transform.translation
            return np.array([t.x, t.y, t.z])
        except TransformException:
            return np.full(3, np.inf)

    def select(name):
        pubs['select_uav'].publish(String(data=name))
        wait(lambda: json.loads(latest.get('settings', String(data='{}')).data).get('uav') == name,
             'RViz selected '+name, 15)
        def camera_selected():
            for topic in [f'/{name}/rgb_down/image_raw', f'/fire_fleet/{name}/detection_image']:
                if not any(info.node_name != node.get_name() and 'rviz' in info.node_name.lower()
                           for info in node.get_subscriptions_info_by_topic(topic)):
                    return False
            return True
        wait(camera_selected, 'Both RViz camera views follow '+name, 15)

    def action(name, operation):
        request_id = str(uuid.uuid4())
        pubs['action'].publish(String(data=json.dumps({'uav': name, 'action': operation, 'request_id': request_id})))
        wait(lambda: (request_id, name) in acks, name+' acknowledged '+operation, 70)
        assert acks[request_id, name]['success'], acks[request_id, name]

    def send_goal(name, point, enabled=True, frame='common_origin'):
        pubs['map_goal'].publish(String(data=json.dumps({'uav': name, 'x': point[0], 'y': point[1],
            'altitude_m': point[2], 'heading': 0., 'enabled': enabled, 'frame': frame})))

    def fly(name, point, switch_to=None):
        before = counts.get(name+'/accepted', 0)
        other = 'uav2' if name == 'uav1' else 'uav1'
        other_before = counts.get(other+'/accepted', 0)
        send_goal(name, point)
        if switch_to:
            select(switch_to)
        wait(lambda: counts.get(name+'/accepted', 0) > before, name+' alone accepted its map goal', 20)
        received = latest[name+'/accepted']
        p = received.pose.position
        assert received.header.frame_id == name+'/world_origin'
        np.testing.assert_allclose([p.x, p.y, p.z], point, atol=0.08)
        assert counts.get(other+'/accepted', 0) == other_before, 'Goal leaked to other drone'
        wait(lambda: np.linalg.norm(position(name)-point) < 0.35, name+' reached '+str(point), 180)
        spin(2)

    def wait_flying(name):
        wait(lambda: position(name)[2] > 1 and name+'/control' in latest
             and latest[name+'/control'].flying_normally
             and latest[name+'/control'].active_tracker == 'MpcTracker',
             name+' completed takeoff and switched to MpcTracker', 120)

    try:
        wait(lambda: all(name+'/hw' in latest and name+'/markers' in latest for name in names),
             'Both independent vehicle and mapper nodes connected', 100)
        wait(lambda: 'shared' in latest and 'shared_markers' in latest,
             'Shared fire map and RViz marker layer connected', 60)
        wait(lambda: all(np.all(np.isfinite(position(name))) for name in names),
             'Both drone transforms connected to common_origin', 90)
        spin(2)
        report['initial_positions_m'] = {name: position(name).tolist() for name in names}
        for name, y in [('uav1', -6), ('uav2', 6)]:
            if args.exercise_flight:
                np.testing.assert_allclose(position(name)[:2], [0, y], atol=0.25)
            transform = tf.lookup_transform('common_origin', name+'/world_origin', Time()).transform
            assert np.linalg.norm([transform.translation.x, transform.translation.y, transform.translation.z]) < 0.01
            assert abs(transform.rotation.w) > 0.999
            assert all(marker.header.frame_id == 'common_origin' for marker in latest[name+'/markers'].markers)
        report['checks'].append('Shared GNSS world origins coincide')
        if args.exercise_flight:
            report['checks'].append('Launch pads are 12 m apart')
        for name in names:
            wait(lambda name=name: all(name+'/'+key in latest for key in ['rgb', 'thermal', 'image']),
                 'RGB, raw thermal and annotated image received from '+name, 60)
            assert latest[name+'/thermal'].encoding == 'mono16'
        select('uav2')
        select('uav1')
        if args.exercise_flight:
            wait(lambda: all(not latest[name+'/hw'].armed for name in names),
                 'Both drones disarmed before the flight check', 180)
            for name in names:
                action(name, 'reset_map')
            wait(lambda: all(json.loads(latest[name+'/fire'].data)['observations'] == 0 for name in names),
                 'Both camera estimates reset for a fresh observation check', 15)
            wait(reset_shared.service_is_ready, 'Shared-map reset service available', 10)
            future = reset_shared.call_async(Trigger.Request())
            wait(future.done, 'Shared map reset acknowledged', 15)
            assert future.result().success
            wait(lambda: len(shared_fires()) == 0, 'Shared map starts empty; scene truth is not preloaded', 15)
            before = counts.get('uav1/accepted', 0)
            send_goal('uav1', [4, -6, 4], enabled=False)
            wait(lambda: 'enable map commands' in latest.get('uav1/command', String()).data,
                 'Disabled map command rejected', 10)
            assert counts.get('uav1/accepted', 0) == before
            send_goal('unknown_uav', [4, -6, 4])
            wait(lambda: 'Unknown drone' in latest.get('notice', String()).data, 'Unknown vehicle rejected', 10)
            action('uav1', 'takeoff')
            wait_flying('uav1')
            assert not latest['uav2/hw'].armed
            np.testing.assert_allclose(position('uav2')[:2], [0, 6], atol=0.25)
            spin(4)
            fly('uav1', [8, -.8, 5], switch_to='uav2')
            wait(lambda: json.loads(latest['uav1/fire'].data).get('complete_count', 0) >= 2,
                 'One actual thermal camera detects two complete fires in one image', 50)
            wait(lambda: len(matched_truth(['patch_a', 'patch_b'])) == 2,
                 'Two separate shared tracks match the first two fire locations', 40)
            first = matched_truth(['patch_a', 'patch_b'])
            initial_ids = {name: fire['id'] for name, fire in first.items()}
            validate_estimates(first)
            report['first_uav_tracks'] = first
            fly('uav1', [4, -6, 4])
            wait(lambda: len(shared_fires()) == 2 and all(not fire['visible'] for fire in shared_fires()),
                 'Both fire estimates retained outside all current camera views', 30)
            action('uav2', 'takeoff')
            wait_flying('uav2')
            spin(4)
            fly('uav2', [4, 6, 5], switch_to='uav1')
            np.testing.assert_allclose(position('uav1'), [4, -6, 4], atol=0.45)
            action('uav1', 'hover')
            np.testing.assert_allclose(position('uav2'), [4, 6, 5], atol=0.45)
            fly('uav1', [0, -6, 4])
            action('uav1', 'land')
            wait(lambda: not latest['uav1/hw'].armed, 'uav1 landed while uav2 remained airborne', 150)
            assert latest['uav2/hw'].armed and position('uav2')[2] > 4
            fly('uav2', [8, -.8, 5])
            wait(lambda: json.loads(latest['uav2/fire'].data).get('complete_count', 0) >= 2,
                 'Second camera also detects both nearby fires', 50)
            def both_contribute():
                matches = matched_truth(['patch_a', 'patch_b'])
                return len(matches) == 2 and all(fire['contributing_uavs'] == names for fire in matches.values())
            wait(both_contribute, 'Both drones contribute to the same two global fire IDs', 40)
            combined = matched_truth(['patch_a', 'patch_b'])
            assert {name: fire['id'] for name, fire in combined.items()} == initial_ids
            assert len(shared_fires()) == 2, 'Second observer created duplicate fire tracks'
            validate_estimates(combined)
            report['combined_tracks'] = combined
            fly('uav2', [12, 6, 5])
            wait(lambda: len(matched_truth(['patch_a', 'patch_b', 'patch_c'])) == 3,
                 'Third fire detected while earlier map entries persist', 50)
            all_fires = matched_truth(['patch_a', 'patch_b', 'patch_c'])
            assert len(shared_fires()) == 3
            validate_estimates(all_fires)
            report['all_three_tracks'] = all_fires
            wait(lambda: all(name+'/raw_markers' in latest for name in names),
                 'Individual drone observation marker streams remain available', 20)
            kinds = {marker.type for marker in latest['shared_markers'].markers}
            assert {Marker.SPHERE, Marker.LINE_STRIP, Marker.TEXT_VIEW_FACING}.issubset(kinds)
            fly('uav2', [0, 6, 5])
            wait(lambda: len(shared_fires()) == 3 and all(not fire['visible'] for fire in shared_fires()),
                 'All three shared IDs retained after cameras leave the fires', 40)
            assert {fire['id'] for fire in shared_fires()} == {fire['id'] for fire in all_fires.values()}
            action('uav2', 'land')
            wait(lambda: not latest['uav2/hw'].armed, 'Both drones landed and disarmed', 150)
            select('uav2')
            select('uav1')
        report['success'] = True
    except Exception as error:
        report['success'] = False
        report['error'] = repr(error)
        raise
    finally:
        report['final_positions_m'] = {name: position(name).tolist() for name in names}
        out = project/'artifacts'
        out.mkdir(exist_ok=True)
        report['shared_map'] = json.loads(latest.get('shared', String(data='{}')).data)
        (out/('milestone2_check.json' if args.exercise_flight else 'fleet_runtime_check.json')).write_text(json.dumps(report, indent=2)+'\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
