#!/usr/bin/env python3
"""Inspect demo outputs; --exercise-flight also takes off, commands goals, and lands.

Run only against this simulated one-drone scenario. The flight option uses the
same /goal_pose input as RViz and the existing takeoff/landing helpers.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PolygonStamped, PoseStamped
from mrs_msgs.msg import HwApiStatus
from nav_msgs.msg import Path as FlightPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from fire_geometry import rotation_matrix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exercise-flight', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = Node('validate_fire_demo', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    latest = {}
    counts = {}
    subscriptions = []
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for key, topic, kind, qos in [
            ('fire', '/fire_demo/fire_status', String, durable),
            ('command', '/fire_demo/command_status', String, durable),
            ('markers', '/fire_demo/markers', MarkerArray, durable),
            ('goal_markers', '/fire_demo/goal_markers', MarkerArray, durable),
            ('accepted', '/fire_demo/accepted_goal', PoseStamped, durable),
            ('boundary', '/fire_demo/fire_boundary', PolygonStamped, durable),
            ('path', '/fire_demo/path/uav1', FlightPath, durable),
            ('hardware', '/uav1/hw_api/status', HwApiStatus, qos_profile_sensor_data),
            ('image', '/fire_demo/detection_image', Image, qos_profile_sensor_data)]:
        def receive(msg, key=key):
            latest[key] = json.loads(msg.data) if key == 'fire' else msg
            counts[key] = counts.get(key, 0)+1
        subscriptions.append(node.create_subscription(kind, topic, receive, qos))
    settings_pub = node.create_publisher(String, '/fire_demo/goal_settings', durable)
    goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)
    action_pub = node.create_publisher(String, '/fire_demo/action', 10)
    tf = Buffer()
    listener = TransformListener(tf, node)
    project = Path(__file__).resolve().parents[1]
    report = {'checks': [], 'flight_exercised': args.exercise_flight}

    def spin_for(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

    def wait(predicate, description, timeout=40):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if predicate():
                print('PASS:', description, flush=True)
                report['checks'].append(description)
                return
        raise AssertionError(description + '; latest status: ' + str(latest.get('fire')))

    def configure(enabled=True, altitude=5.0, uav='uav1'):
        settings_pub.publish(String(data=json.dumps({'uav': uav, 'enabled': enabled, 'altitude_m': altitude})))
        spin_for(0.5)

    def goal(x=8., y=0., frame='uav1/world_origin'):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.w = 1.0
        return msg

    def reject(msg, expected):
        old = counts.get('accepted', 0)
        latest.pop('command', None)
        goal_pub.publish(msg)
        wait(lambda: expected in latest.get('command', String()).data, 'Rejected goal: ' + expected, 8)
        assert counts.get('accepted', 0) == old, 'Rejected goal reached MRS'

    def position():
        try:
            t = tf.lookup_transform('uav1/world_origin', 'uav1/fcu', Time()).transform.translation
            return np.array([t.x, t.y, t.z])
        except TransformException:
            return np.full(3, np.inf)

    def fly(msg, expected):
        before = counts.get('accepted', 0)
        goal_pub.publish(msg)
        wait(lambda: counts.get('accepted', 0) > before, 'Goal accepted through RViz topic')
        accepted = latest['accepted'].pose.position
        np.testing.assert_allclose([accepted.x, accepted.y, accepted.z], expected, atol=0.03)
        wait(lambda: np.linalg.norm(position()-expected) < 0.3, 'Drone reached ' + str(expected), 80)
        spin_for(3)

    try:
        wait(lambda: 'hardware' in latest and 'markers' in latest and settings_pub.get_subscription_count() > 0,
             'Demo nodes and hardware status discovered', 65)
        if args.exercise_flight:
            wait(lambda: any(info.node_name == 'fire_demo_controls'
                             for info in node.get_publishers_info_by_topic('/fire_demo/goal_settings')),
                 'RViz controls connected to the command settings topic', 60)
            spin_for(0.5)
            configure(False)
            reject(goal(), 'enable map commands')
            configure(True)
            if not latest['hardware'].armed:
                reject(goal(), 'Take off first')
            configure(True, altitude=50)
            assert 'Settings rejected' in latest['command'].data
            configure(True, uav='uav2')
            assert 'only supports uav1' in latest['command'].data
            configure(True)
            if not latest['hardware'].armed:
                subprocess.run([str(project/'tmux/takeoff.sh')], check=True, timeout=100)
            wait(lambda: position()[2] > 1.0 and latest['hardware'].armed and latest['hardware'].offboard,
                 'Takeoff completed', 60)
            configure(True)
            reject(goal(x=100), 'outside the demo square')
            reject(goal(frame='unknown_demo_frame'), 'no transform')
            # Exercise a real frame conversion: world target -> fixed_origin -> bridge -> world.
            transform = tf.lookup_transform('uav1/fixed_origin', 'uav1/world_origin', Time()).transform
            q, t = transform.rotation, transform.translation
            rotation = rotation_matrix([q.x, q.y, q.z, q.w])
            local = rotation @ np.array([8., 0., 0.]) + np.array([t.x, t.y, t.z])
            msg = goal(local[0], local[1], 'uav1/fixed_origin')
            msg.pose.position.z = float(local[2])
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
            msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            fly(msg, np.array([8., 0., 5.]))
        wait(lambda: latest.get('fire', {}).get('visible') and latest['fire']['observations'] >= 8,
             'Complete hot patch detected repeatedly', 50)
        centre = np.asarray(latest['fire']['centre_m'])
        assert np.linalg.norm(centre-[8, 0]) < 0.2, ('Fire position error', centre)
        report['fire'] = latest['fire']
        report['position_error_m'] = float(np.linalg.norm(centre-[8, 0]))
        wait(lambda: 'boundary' in latest and 'path' in latest and 'image' in latest,
             'Boundary, flight path, and annotated thermal image received')
        polygon = np.array([[p.x, p.y] for p in latest['boundary'].polygon.points])
        area = abs(np.dot(polygon[:, 0], np.roll(polygon[:, 1], 1))-
                   np.dot(polygon[:, 1], np.roll(polygon[:, 0], 1)))/2
        assert 2.5 < area < 5.5, ('Unexpected fire area', area)
        footprints = [m for m in latest['markers'].markers if m.ns == 'footprints']
        assert len(footprints) == 2, 'Both camera footprints must be visible'
        assert any(m.ns == 'fire' and m.id == 1 for m in latest['markers'].markers), 'Uncertainty ellipse missing'
        centre_marker = next(m for m in latest['markers'].markers if m.ns == 'fire' and m.id == 0)
        ellipse_marker = next(m for m in latest['markers'].markers if m.ns == 'fire' and m.id == 1)
        assert min(p.z for p in ellipse_marker.points) >= centre_marker.pose.position.z + centre_marker.scale.z/2 - 1e-6, \
            'The centre sphere must not cover the small uncertainty ellipse'
        assert any(m.ns == 'drones' and 'uav1' in m.text for m in latest['markers'].markers), 'Drone label missing'
        report['boundary_area_m2'] = float(area)
        report['path_points'] = len(latest['path'].poses)
        report['camera_footprints'] = len(footprints)
        report['ellipse_clears_centre_marker'] = True
        out = project/'artifacts'
        out.mkdir(exist_ok=True)
        image = CvBridge().imgmsg_to_cv2(latest['image'], desired_encoding='bgr8')
        cv2.imwrite(str(out/'fire_detection.png'), image)
        if args.exercise_flight:
            fly(goal(8.5, 0.4), np.array([8.5, 0.4, 5.]))
            wait(lambda: latest['fire']['visible'] and latest['fire']['observations'] > report['fire']['observations']+5,
                 'Estimate updated from another viewpoint', 40)
            assert np.linalg.norm(np.asarray(latest['fire']['centre_m'])-[8, 0]) < 0.2
            report['second_view_fire'] = latest['fire']
            action_pub.publish(String(data='hover'))
            wait(lambda: 'hover: accepted' in latest.get('command', String()).data,
                 'RViz hover action accepted by MRS', 15)
            wait(lambda: any(m.action == 3 for m in latest.get('goal_markers', MarkerArray()).markers),
                 'Hover cleared the previous destination marker', 10)
            fly(goal(0, 0), np.array([0., 0., 5.]))
            wait(lambda: not latest['fire']['visible'] and latest['fire']['last_seen_age_s'] > 2,
                 'Out-of-view fire estimate retained with last-seen age', 30)
            assert latest['fire']['centre_m'] is not None
            report['retained_fire'] = latest['fire']
            configure(False)
            subprocess.run([str(project/'tmux/land.sh')], check=True, timeout=30)
            wait(lambda: not latest['hardware'].armed, 'Landed and disarmed', 90)
            wait(lambda: any(m.action == 3 for m in latest.get('goal_markers', MarkerArray()).markers),
                 'Landing cleared the previous destination marker', 10)
        report['success'] = True
        (out/'rviz_demo_check.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2), flush=True)
    finally:
        if args.exercise_flight:
            configure(False)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
