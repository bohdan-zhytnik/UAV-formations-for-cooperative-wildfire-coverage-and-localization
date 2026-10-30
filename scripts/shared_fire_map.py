#!/usr/bin/env python3
"""Shared persistent fire map built only from timestamped camera observations."""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from fire_geometry import ellipse_points
from multi_fire import SharedFireMap


class SharedMapNode(Node):
    def __init__(self):
        super().__init__('shared_fire_map')
        self.frame = self.declare_parameter('map_frame', 'common_origin').value
        uavs = self.declare_parameter('uav_names', ['uav1', 'uav2']).value
        defaults = {'association_distance_m': .9, 'window_per_uav': 15, 'uncertainty_floor_m': .03,
                    'visible_age_s': 2., 'max_tracks': 64}
        settings = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        self.map = SharedFireMap(uavs=uavs, **settings)
        self.last_clock, self.reset_after, self.generation = 0., -1., 0
        self.rejected_frames, self.duplicate_frames = 0, 0
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        prefix = '/fire_fleet/fire_map'
        self.status_pub = self.create_publisher(String, prefix+'/status', durable)
        self.markers_pub = self.create_publisher(MarkerArray, prefix+'/markers', durable)
        self.owned_subscriptions = [self.create_subscription(String, '/fire_fleet/'+uav+'/observations',
            lambda msg, source=uav: self.receive(source, msg), 5) for uav in uavs]
        self.create_service(Trigger, prefix+'/reset', self.reset_service)
        self.create_timer(.5, self.publish)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds/1e9

    def check_clock(self):
        now = self.now_seconds()
        if now < self.last_clock:
            self.clear(-1.)
        self.last_clock = now
        return now

    def clear(self, after):
        self.map.reset()
        self.reset_after = after
        self.generation += 1
        self.markers_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))

    def reset_service(self, _request, response):
        self.clear(self.now_seconds())
        self.publish()
        response.success = True
        response.message = 'Shared fire map cleared; new complete camera observations will rebuild it.'
        return response

    def receive(self, source, msg):
        now = self.check_clock()
        try:
            batch = json.loads(msg.data)
            if batch['schema_version'] != 1 or batch['source'] != source or batch['frame'] != self.frame:
                raise ValueError('Wrong observation schema, source or frame')
            stamp = batch['stamp_ns']/1e9
            if not math.isfinite(stamp) or stamp > now+.1 or now-stamp > 3.:
                raise ValueError('Image timestamp outside the current simulation-time window')
            if stamp < self.reset_after:
                return
            if self.map.update(source, stamp, batch['observations']) is None:
                self.duplicate_frames += 1
        except (ValueError, KeyError, TypeError, IndexError) as error:
            self.rejected_frames += 1
            self.get_logger().warning('Observation frame rejected: '+str(error))

    def marker(self, fire, offset, kind, colour, scale):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id, marker.type = 'shared_fires', fire['id']*10+offset, kind
        marker.pose.orientation.w = 1.
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = colour
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        return marker

    def publish(self):
        now = self.check_clock()
        fires = self.map.snapshot(now)
        self.status_pub.publish(String(data=json.dumps({
            'schema_version': 1, 'frame': self.frame, 'stamp_ns': self.get_clock().now().nanoseconds,
            'generation': self.generation, 'fires': fires,
            'rejected_frames': self.rejected_frames, 'duplicate_frames': self.duplicate_frames,
            'dropped_new_tracks': self.map.dropped_new_tracks,
            'method': 'equal UAV weight; bounded rolling mean; illustrative 2-sigma repeatability, not calibrated accuracy',
            'boundary_method': 'latest complete projected outline recentered on the rolling estimate'}, allow_nan=False)))
        markers = [Marker(action=Marker.DELETEALL)]
        for fire in fires:
            colour = (1., .12, .03, 1.) if fire['visible'] else (.7, .25, .12, .9)
            centre = self.marker(fire, 0, Marker.SPHERE, colour, .1)
            centre.pose.position = Point(x=fire['centre_m'][0], y=fire['centre_m'][1], z=.08)
            markers.append(centre)
            ellipse = self.marker(fire, 1, Marker.LINE_STRIP, (1., .85, 0., 1.), .035)
            ellipse.points = [Point(x=float(x), y=float(y), z=.15)
                              for x, y in ellipse_points(fire['centre_m'], fire['covariance_xy_m2'])]
            markers.append(ellipse)
            boundary = self.marker(fire, 2, Marker.LINE_STRIP, colour, .05)
            closed = fire['boundary_m']+[fire['boundary_m'][0]]
            boundary.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])+.04) for p in closed]
            markers.append(boundary)
            label = self.marker(fire, 3, Marker.TEXT_VIEW_FACING, colour, .28)
            label.pose.position = Point(x=fire['centre_m'][0], y=fire['centre_m'][1], z=.75)
            label.text = (f"Fire {fire['id']} ({fire['centre_m'][0]:.1f}, {fire['centre_m'][1]:.1f}) m\n"
                          f"{fire['area_m2']:.2f} m^2 | mean/max {fire['mean_kelvin']:.0f}/{fire['max_kelvin']:.0f} K\n"
                          f"{', '.join(fire['contributing_uavs'])} | last {fire['last_seen_age_s']:.1f} s")
            markers.append(label)
        self.markers_pub.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = SharedMapNode()
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
            if 'rcl_shutdown already called' not in str(error):
                raise


if __name__ == '__main__':
    main()
