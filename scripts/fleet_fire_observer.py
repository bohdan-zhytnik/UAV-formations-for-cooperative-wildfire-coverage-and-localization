#!/usr/bin/env python3
"""All hot regions from one camera; no world truth and no persistent fire IDs."""
import json
import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from fire_mapper import FireMapper
from multi_fire import hot_regions


class FleetFireObserver(FireMapper):
    def __init__(self):
        super().__init__()
        prefix = '/fire_fleet/'+self.uav
        self.observations_pub = self.create_publisher(String, prefix+'/observations', 5)
        self.raw_markers_pub = self.create_publisher(MarkerArray, prefix+'/observation_markers',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def reset_state(self):
        super().reset_state()
        self.regions, self.sequence, self.processed_stamp = [], 0, None
        self.note = 'Waiting for multi-fire camera observations'

    def reset_service(self, request, response):
        super().reset_service(request, response)
        self.raw_markers_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
        response.message = 'Selected observer and trail cleared; shared fire map is retained.'
        return response

    def detect(self, msg, info, rotation, translation):
        if msg.encoding not in ('mono16', '16UC1'):
            raise ValueError('Thermal detector expects mono16 raw counts')
        if info.distortion_model not in ('', 'plumb_bob', 'rational_polynomial'):
            raise ValueError('Unsupported distortion model')
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if raw.shape != (info.height, info.width):
            raise ValueError('Image/calibration dimensions differ')
        self.regions = hot_regions(raw, info.k, info.d, rotation, translation, self.ground,
            self.settings['threshold_kelvin'], self.settings['kelvin_per_count'], self.settings['minimum_hot_pixels'])
        self.sequence += 1
        self.processed_stamp = Time.from_msg(msg.header.stamp)
        complete = [value for value in self.regions if not value['partial']]
        self.visible, self.partial = bool(complete), any(value['partial'] for value in self.regions)
        self.observations += len(complete)
        if complete:
            self.last_seen = self.processed_stamp
        self.note = f'{len(complete)} complete, {len(self.regions)-len(complete)} partial regions; frame-local region numbers'
        self.observations_pub.publish(String(data=json.dumps({
            'schema_version': 1, 'source': self.uav, 'frame': self.frame,
            'stamp_ns': self.processed_stamp.nanoseconds, 'sequence': self.sequence,
            'observations': [{key: value for key, value in region.items()
                              if key not in ('pixel_contour', 'centre_pixel')} for region in self.regions]}, allow_nan=False)))
        kelvin = raw.astype(np.float32)*self.settings['kelvin_per_count']
        preview = cv2.applyColorMap(np.clip((kelvin-290)/360*255, 0, 255).astype(np.uint8), cv2.COLORMAP_INFERNO)
        for region in self.regions:
            contour = np.asarray(region['pixel_contour'], dtype=np.int32)
            colour = (0, 180, 255) if region['partial'] else (0, 255, 0)
            cv2.drawContours(preview, [contour], -1, colour, 2)
            x, y = np.rint(region['centre_pixel']).astype(int)
            label = f"r{region['region_id']} {region['mean_kelvin']:.0f} K"
            cv2.putText(preview, label, (max(1, x-25), max(35, y)), cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 255, 255), 1)
        cv2.putText(preview, f'{self.uav}: {len(complete)} complete, {len(self.regions)-len(complete)} partial',
                    (5, 18), cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 255, 255), 1)
        annotated = self.bridge.cv2_to_imgmsg(preview, encoding='bgr8')
        annotated.header = msg.header
        self.image_pub.publish(annotated)
        self.publish_raw_markers()

    def publish_raw_markers(self):
        markers = [Marker(action=Marker.DELETEALL)]
        for index, region in enumerate(self.regions):
            colour = (1., .65, .05, .9) if region['partial'] else self.drone_color
            boundary = np.asarray(region['boundary_m'])
            closed = np.vstack((boundary, boundary[0])).copy()
            closed[:, 2] += .07
            outline = self.line('raw_observations', 3*index, closed, colour, .025)
            dot = self.marker('raw_observations', 3*index+1, Marker.SPHERE, colour, .09)
            dot.pose.position = Point(x=region['centre_m'][0], y=region['centre_m'][1], z=self.ground+.1)
            text = self.marker('raw_observations', 3*index+2, Marker.TEXT_VIEW_FACING, colour, .22)
            text.pose.position = Point(x=region['centre_m'][0], y=region['centre_m'][1], z=self.ground+.4)
            text.text = f"{self.uav} r{region['region_id']} | {'partial' if region['partial'] else 'complete'}"
            for marker in [outline, dot, text]:
                marker.header.stamp = self.processed_stamp.to_msg()
                marker.lifetime.sec = 2
                markers.append(marker)
        self.raw_markers_pub.publish(MarkerArray(markers=markers))

    def publish_fire_status(self, age, recent):
        frame_age = None if self.processed_stamp is None else (self.get_clock().now()-self.processed_stamp).nanoseconds/1e9
        fresh = frame_age is not None and frame_age < 2
        complete = [value for value in self.regions if not value['partial']]
        self.status_pub.publish(String(data=json.dumps({
            'schema_version': 2, 'source': self.uav, 'frame': self.frame,
            'visible': bool(fresh and complete), 'partial': bool(fresh and self.partial),
            'observations': self.observations, 'complete_count': len(complete) if fresh else 0,
            'partial_count': len(self.regions)-len(complete) if fresh else 0,
            'last_frame_age_s': frame_age, 'note': self.note,
            'regions': [{key: value for key, value in region.items()
                         if key not in ('pixel_contour', 'centre_pixel')} for region in self.regions]}, allow_nan=False)))


def main():
    rclpy.init()
    node = FleetFireObserver()
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
