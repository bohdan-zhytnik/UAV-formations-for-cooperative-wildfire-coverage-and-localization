#!/usr/bin/env python3
"""Translate explicitly enabled RViz goals to stamped MRS position references."""
import json
import copy
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from mrs_msgs.msg import HwApiStatus
from mrs_msgs.srv import ReferenceStampedSrv
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from fire_geometry import rotation_matrix


class GoalBridge(Node):
    def __init__(self):
        super().__init__('goal_bridge')
        defaults = {'uav_name': 'uav1', 'map_frame': 'uav1/world_origin', 'default_altitude_m': 5.0,
                    'minimum_altitude_m': 1.0, 'maximum_altitude_m': 9.0, 'horizontal_limit_m': 20.0,
                    'goal_marker_color': [0.0, 0.7, 0.4, 1.0]}
        self.settings = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        self.uav, self.frame = self.settings['uav_name'], self.settings['map_frame']
        self.altitude = self.settings['default_altitude_m']
        self.enabled = False
        self.hardware = None
        self.hardware_received = -math.inf
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/fire_demo/command_status', durable)
        self.marker_pub = self.create_publisher(MarkerArray, '/fire_demo/goal_markers', durable)
        self.accepted_pub = self.create_publisher(PoseStamped, '/fire_demo/accepted_goal', durable)
        self.create_subscription(String, '/fire_demo/goal_settings', self.on_settings, durable)
        self.create_subscription(String, '/fire_demo/action', self.on_action, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(HwApiStatus, f'/{self.uav}/hw_api/status', self.on_hardware, qos_profile_sensor_data)
        self.reference = self.create_client(ReferenceStampedSrv, f'/{self.uav}/control_manager/reference')
        self.hover = self.create_client(Trigger, f'/{self.uav}/control_manager/hover')
        self.reset = self.create_client(Trigger, '/fire_demo/reset_map')
        self.pending = None
        self.inflight = None
        self.create_timer(0.1, self.process_goal)
        self.report('Map commands disabled. Take off with ./takeoff.sh, then enable them in the RViz panel.')

    def report(self, message):
        self.status_pub.publish(String(data=message))
        self.get_logger().info(message)

    def on_hardware(self, msg):
        was_armed = self.hardware is not None and self.hardware.armed
        self.hardware, self.hardware_received = msg, time.monotonic()
        if was_armed and not msg.armed:
            self.pending = None
            self.marker_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
            self.report('Drone disarmed; previous flight target cleared. Take off before sending another goal.')

    def on_settings(self, msg):
        try:
            settings = json.loads(msg.data)
            if settings['uav'] != self.uav:
                raise ValueError('This demo only supports ' + self.uav)
            altitude = float(settings['altitude_m'])
            if not math.isfinite(altitude) or not self.settings['minimum_altitude_m'] <= altitude <= self.settings['maximum_altitude_m']:
                raise ValueError('Flight altitude must be between '
                                 f'{self.settings["minimum_altitude_m"]:g} and '
                                 f'{self.settings["maximum_altitude_m"]:g} m')
            if type(settings['enabled']) is not bool:
                raise ValueError('enabled must be a boolean')
            previous = (self.enabled, self.altitude)
            self.enabled, self.altitude = settings['enabled'], altitude
            if not self.enabled:
                self.pending = None
            if previous != (self.enabled, self.altitude):
                self.report(f'Map commands {"enabled" if self.enabled else "disabled"}; {self.uav}, altitude {self.altitude:.1f} m. '
                            'Changing altitude alone does not move the drone.')
        except (ValueError, KeyError, TypeError) as error:
            self.enabled = False
            self.pending = None
            self.report('Settings rejected: ' + str(error))

    def ready_to_fly(self):
        return (self.hardware is not None and time.monotonic()-self.hardware_received < 5
                and self.hardware.connected and self.hardware.armed and self.hardware.offboard)

    def on_goal(self, msg):
        if not self.enabled:
            self.report('Goal ignored: enable map commands in the RViz panel first.')
            return
        if not self.ready_to_fly():
            self.report('Goal ignored: fresh connected/armed/offboard status required. Take off first.')
            return
        if self.inflight is not None and not getattr(self, 'queue_manual_goals', False):
            self.report('Goal ignored: waiting for the previous MRS response.')
            return
        position, q = msg.pose.position, msg.pose.orientation
        values = [position.x, position.y, position.z, q.x, q.y, q.z, q.w]
        if not msg.header.frame_id or not all(math.isfinite(value) for value in values):
            self.report('Goal rejected: a valid frame and finite coordinates are required.')
            return
        if math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w) < 1e-6:
            self.report('Goal rejected: invalid heading quaternion.')
            return
        self.pending = (msg, self.altitude, time.monotonic())

    def process_goal(self):
        if self.inflight is not None:
            future, sent = self.inflight
            if not future.done() and time.monotonic()-sent > 10:
                # A service call may have executed even when its reply was lost.
                self.inflight = None
                self.report('MRS response timed out; outcome unknown. Check the drone before sending another goal.')
            return
        if self.pending is None:
            return
        msg, altitude, received = self.pending
        if not self.enabled or not self.ready_to_fly():
            self.pending = None
            self.report('Goal cancelled: map commands disabled or hardware status is no longer ready.')
            return
        try:
            input_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            q = msg.pose.orientation
            heading = rotation_matrix([q.x, q.y, q.z, q.w])[:, 0]
            if msg.header.frame_id != self.frame:
                transform = self.tf.lookup_transform(self.frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
                t, q = transform.transform.translation, transform.transform.rotation
                rotation = rotation_matrix([q.x, q.y, q.z, q.w])
                input_position = rotation @ input_position + np.array([t.x, t.y, t.z])
                heading = rotation @ heading
        except TransformException:
            if time.monotonic()-received > 2:
                self.pending = None
                self.report('Goal rejected: no transform from ' + msg.header.frame_id + ' to ' + self.frame)
            return
        except ValueError as error:
            self.pending = None
            self.report('Goal rejected: ' + str(error))
            return
        self.pending = None
        if np.max(np.abs(input_position[:2])) > self.settings['horizontal_limit_m']:
            self.report('Goal rejected: outside the demo square x/y = -20 to +20 m.')
            return
        if not self.reference.service_is_ready():
            self.report('Goal rejected: MRS reference service is unavailable.')
            return
        request = ReferenceStampedSrv.Request()
        request.header.frame_id = self.frame
        # Stamp zero asks MRS for the latest transform of this world-fixed reference.
        request.reference.position = Point(x=float(input_position[0]), y=float(input_position[1]), z=altitude)
        request.reference.heading = math.atan2(heading[1], heading[0])
        future = self.reference.call_async(request)
        self.inflight = (future, time.monotonic())
        self.on_reference_sent(future, request)
        self.report(f'Sending {self.uav} to ({input_position[0]:.2f}, {input_position[1]:.2f}, {altitude:.1f}) m in {self.frame}')
        future.add_done_callback(lambda result: self.reference_result(result, request))

    def on_reference_sent(self, future, request):
        """Extension point for the fleet's command-ownership bookkeeping."""
        pass

    def reference_result(self, future, request):
        if self.inflight is not None and self.inflight[0] is future:
            self.inflight = None
        try:
            response = future.result()
            if not response.success:
                self.report('MRS rejected the goal: ' + response.message)
                return
            self.publish_goal(request)
            self.report('MRS accepted the goal. The drone will hold it until another command or safety action. ' + response.message)
        except Exception as error:
            self.report('MRS service failed: ' + str(error))

    def publish_goal(self, request):
        pose = PoseStamped()
        pose.header.frame_id = self.frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = request.reference.position
        pose.pose.orientation.z = math.sin(request.reference.heading/2)
        pose.pose.orientation.w = math.cos(request.reference.heading/2)
        self.accepted_pub.publish(pose)
        markers = []
        for identifier, kind in [(0, Marker.SPHERE), (1, Marker.LINE_STRIP), (2, Marker.TEXT_VIEW_FACING)]:
            marker = Marker()
            marker.header = pose.header
            marker.ns, marker.id, marker.type = 'accepted_goal', identifier, kind
            marker.pose.orientation.w = 1.0
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = self.settings['goal_marker_color']
            marker.scale.x = marker.scale.y = marker.scale.z = 0.3
            if kind == Marker.LINE_STRIP:
                marker.scale.x = 0.03
                marker.points = [Point(x=pose.pose.position.x, y=pose.pose.position.y, z=0.0), pose.pose.position]
            else:
                marker.pose = copy.deepcopy(pose.pose)
                if kind == Marker.TEXT_VIEW_FACING:
                    marker.pose.position.z += 0.5
                    marker.text = f'{self.uav} target | {request.reference.position.z:.1f} m'
            markers.append(marker)
        self.marker_pub.publish(MarkerArray(markers=markers))

    def on_action(self, msg):
        clients = {'hover': self.hover, 'reset_map': self.reset}
        client = clients.get(msg.data)
        if client is None:
            self.report('Unknown demo action: ' + msg.data)
            return
        if not client.service_is_ready():
            self.report('Action service unavailable: ' + msg.data)
            return
        if msg.data == 'hover':
            self.pending = None
        future = client.call_async(Trigger.Request())
        def complete(result):
            try:
                response = result.result()
                if response.success and msg.data == 'hover':
                    self.marker_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
                self.report(f'{msg.data}: {"accepted" if response.success else "rejected"}: {response.message}')
            except Exception as error:
                self.report(msg.data + ': ' + str(error))
        future.add_done_callback(complete)


def main():
    rclpy.init()
    node = GoalBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as error:
            # The installed Jazzy context can race its SIGINT handler.
            if 'rcl_shutdown already called' not in str(error):
                raise


if __name__ == '__main__':
    main()
