#!/usr/bin/env python3
"""One ideal thermal camera, one static fire, flat ground, bounded rolling mean."""
import json
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PolygonStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from fire_geometry import RollingCentre, ellipse_points, project_ground, rotation_matrix


class FireMapper(Node):
    def __init__(self):
        super().__init__('fire_mapper')
        defaults = {'uav_name': 'uav1', 'map_frame': 'uav1/world_origin', 'ground_z': 0.0,
                    'threshold_kelvin': 450.0, 'kelvin_per_count': 0.02,
                    'minimum_hot_pixels': 50, 'rolling_window': 30,
                    'uncertainty_floor_m': 0.03, 'update_period_s': 0.4,
                    'drone_color': [0.1, 0.4, 1.0, 1.0], 'show_ground': True,
                    'start_position': [0.0, 0.0]}
        self.settings = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        self.uav = self.settings['uav_name']
        self.frame = self.settings['map_frame']
        self.ground = self.settings['ground_z']
        self.drone_color = self.settings['drone_color']
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.bridge = CvBridge()
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers_pub = self.create_publisher(MarkerArray, '/fire_demo/markers', durable)
        self.path_pub = self.create_publisher(Path, '/fire_demo/path/' + self.uav, durable)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/fire_demo/fire_pose', durable)
        self.boundary_pub = self.create_publisher(PolygonStamped, '/fire_demo/fire_boundary', durable)
        self.status_pub = self.create_publisher(String, '/fire_demo/fire_status', durable)
        self.image_pub = self.create_publisher(Image, '/fire_demo/detection_image', qos_profile_sensor_data)
        self.infos = {}
        self.subscriptions_owned = []
        for camera in ('rgb_down', 'thermal_down'):
            self.subscriptions_owned.append(self.create_subscription(
                CameraInfo, f'/{self.uav}/{camera}/camera_info',
                lambda msg, camera=camera: self.infos.__setitem__(camera, msg), qos_profile_sensor_data))
        self.subscriptions_owned.append(self.create_subscription(
            Image, f'/{self.uav}/thermal_down/image_raw', self.on_image, qos_profile_sensor_data))
        self.create_service(Trigger, '/fire_demo/reset_map', self.reset_service)
        self.pending = None
        self.last_process = -math.inf
        self.last_clock = 0
        self.last_note = ''
        self.note = 'Waiting for thermal images and camera transforms'
        self.reset_state()
        self.create_timer(0.05, self.process_pending)
        self.create_timer(0.2, self.publish_scene)

    def reset_state(self):
        self.rolling = RollingCentre(self.settings['rolling_window'], self.settings['uncertainty_floor_m'])
        self.centre = self.covariance = self.boundary = None
        self.observations = 0
        self.last_seen = None
        self.path = Path()
        self.path.header.frame_id = self.frame
        self.visible = False
        self.partial = False
        self.note = 'No complete fire observation yet'

    def reset_service(self, _request, response):
        self.reset_state()
        clear = Marker(action=Marker.DELETEALL)
        self.markers_pub.publish(MarkerArray(markers=[clear]))
        response.success = True
        response.message = 'Fire estimate and flight trail cleared; new observations will rebuild them.'
        return response

    def on_image(self, msg):
        # Keep one pending image until its historical TF arrives. Replacing it
        # on every frame can starve processing when TF lags the image stream.
        if self.pending is None:
            self.pending = (msg, time.monotonic())

    def camera_transform(self, frame, stamp=None):
        transform = self.tf.lookup_transform(self.frame, frame, Time.from_msg(stamp) if stamp else Time())
        q, t = transform.transform.rotation, transform.transform.translation
        return rotation_matrix([q.x, q.y, q.z, q.w]), np.array([t.x, t.y, t.z])

    def project(self, pixels, info, rotation, translation):
        if info.distortion_model not in ('', 'plumb_bob', 'rational_polynomial'):
            raise ValueError('Unsupported camera distortion model: ' + info.distortion_model)
        if info.k[0] <= 0 or info.k[4] <= 0:
            raise ValueError('Camera calibration is unavailable')
        return project_ground(pixels, np.asarray(info.k).reshape(3, 3), info.d,
                              rotation, translation, self.ground)

    def process_pending(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if now < self.last_clock:
            self.reset_state()
            self.pending = None
            self.last_process = -math.inf
            self.tf.clear()
        self.last_clock = now
        if self.pending is None or now - self.last_process < self.settings['update_period_s']:
            return
        msg, received = self.pending
        info = self.infos.get('thermal_down')
        if info is None:
            return
        try:
            rotation, translation = self.camera_transform(msg.header.frame_id, msg.header.stamp)
        except TransformException:
            if time.monotonic() - received > 2:
                self.pending = None
                self.note = 'Waiting for TF at image timestamp; no latest-pose substitution'
            return
        self.pending = None
        self.last_process = now
        try:
            self.detect(msg, info, rotation, translation)
        except (ValueError, cv2.error) as error:
            self.visible = False
            self.note = str(error)

    def detect(self, msg, info, rotation, translation):
        if msg.encoding not in ('mono16', '16UC1'):
            raise ValueError('Thermal detector expects mono16 raw counts')
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if raw.shape != (info.height, info.width):
            raise ValueError('Thermal image and calibration sizes differ')
        kelvin = raw.astype(np.float32) * self.settings['kelvin_per_count']
        mask = (kelvin > self.settings['threshold_kelvin']).astype(np.uint8)
        preview = cv2.applyColorMap(np.clip((kelvin-290)/360*255, 0, 255).astype(np.uint8), cv2.COLORMAP_INFERNO)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        self.visible = False
        self.partial = False
        self.note = 'No fire in current thermal view; retaining last estimate'
        if count > 1:
            component = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
            x, y, width, height, area = stats[component]
            if area >= self.settings['minimum_hot_pixels']:
                selected = (labels == component).astype(np.uint8)
                contours, _ = cv2.findContours(selected, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contour = max(contours, key=cv2.contourArea)
                self.partial = x <= 1 or y <= 1 or x+width >= info.width-1 or y+height >= info.height-1
                cv2.drawContours(preview, [contour], -1, (0, 255, 0) if not self.partial else (0, 180, 255), 2)
                if self.partial:
                    self.note = 'Partial hot patch at image edge; centre update skipped'
                else:
                    # Project all selected pixel centres, not a hard-coded fire location.
                    rows, columns = np.nonzero(selected)
                    pixels = np.column_stack((columns, rows)).astype(float)
                    # Pixel indices describe pixel centres; CameraInfo K uses the same convention.
                    points = self.project(pixels, info, rotation, translation)
                    measured = points[:, :2].mean(axis=0)
                    centre_pixel = pixels.mean(axis=0)
                    offsets = self.project([centre_pixel, centre_pixel+[1, 0], centre_pixel+[0, 1]],
                                           info, rotation, translation)[:, :2]
                    jacobian = (offsets[1:] - offsets[0]).T
                    self.centre, self.covariance = self.rolling.add(measured, jacobian @ jacobian.T / 12)
                    outline = cv2.approxPolyDP(contour, 1.5, True).reshape(-1, 2)
                    self.boundary = self.project(outline, info, rotation, translation)
                    self.observations += 1
                    self.last_seen = self.get_clock().now()
                    self.visible = True
                    self.note = 'Fire 1 measured by ' + self.uav + '; rolling mean, no Kalman filter'
                    self.publish_estimate(msg.header.stamp)
                    cv2.circle(preview, tuple(np.rint(centre_pixel).astype(int)), 4, (255, 255, 255), -1)
        text = 'Fire 1' if self.visible else ('Partial: no update' if self.partial else 'No complete fire')
        cv2.putText(preview, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
        annotated = self.bridge.cv2_to_imgmsg(preview, encoding='bgr8')
        annotated.header = msg.header
        self.image_pub.publish(annotated)

    def publish_estimate(self, stamp):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id, pose.header.stamp = self.frame, stamp
        pose.pose.pose.position = Point(x=float(self.centre[0]), y=float(self.centre[1]), z=self.ground)
        pose.pose.pose.orientation.w = 1.0
        for row in range(2):
            for column in range(2):
                pose.pose.covariance[row*6+column] = float(self.covariance[row, column])
        # Ground z is assumed, and fire orientation is undefined; neither is estimated.
        self.pose_pub.publish(pose)
        boundary = PolygonStamped()
        boundary.header = pose.header
        from geometry_msgs.msg import Point32
        boundary.polygon.points = [Point32(x=float(p[0]), y=float(p[1]), z=self.ground) for p in self.boundary]
        self.boundary_pub.publish(boundary)

    def marker(self, namespace, identifier, kind, color, scale=0.05):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id, marker.type = namespace, identifier, kind
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def line(self, namespace, identifier, points, color, width=0.04):
        marker = self.marker(namespace, identifier, Marker.LINE_STRIP, color, width)
        marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points]
        return marker

    def publish_scene(self):
        stamp = self.get_clock().now().to_msg()
        markers = []
        ground = self.marker('terrain', 0, Marker.CUBE, (0.28, 0.40, 0.25, 0.6))
        ground.pose.position.z = self.ground - 0.03
        ground.scale.x = ground.scale.y = 40.0
        ground.scale.z = 0.02
        if self.settings['show_ground']:
            markers.append(ground)
        start = self.marker('terrain', 1, Marker.CUBE, self.drone_color)
        start.pose.position.x, start.pose.position.y = self.settings['start_position']
        start.scale.x = start.scale.y = 0.6
        start.scale.z = 0.01
        start.pose.position.z = self.ground
        markers.append(start)
        for index, (camera, color) in enumerate([('rgb_down', (0.1, 0.7, 1.0, 0.9)),
                                                ('thermal_down', (1.0, 0.4, 0.05, 0.9))]):
            info = self.infos.get(camera)
            try:
                if info is None:
                    continue
                rotation, translation = self.camera_transform(info.header.frame_id)
                corners = [[0, 0], [info.width-1, 0], [info.width-1, info.height-1], [0, info.height-1], [0, 0]]
                footprint = self.project(corners, info, rotation, translation)
                footprint[:, 2] += 0.04 + index*0.02
                marker = self.line('footprints', index, footprint, color)
                marker.lifetime.sec = 1
                markers.append(marker)
            except (TransformException, ValueError, cv2.error):
                pass
        try:
            transform = self.tf.lookup_transform(self.frame, self.uav+'/fcu', Time())
            position = transform.transform.translation
            robot = self.marker('drones', 0, Marker.ARROW, self.drone_color)
            robot.pose.position = Point(x=position.x, y=position.y, z=position.z)
            robot.pose.orientation = transform.transform.rotation
            robot.scale.x, robot.scale.y, robot.scale.z = 0.8, 0.13, 0.13
            markers.append(robot)
            label = self.marker('drones', 1, Marker.TEXT_VIEW_FACING, self.drone_color, 0.3)
            label.pose.position = Point(x=position.x, y=position.y, z=position.z+0.65)
            label.text = self.uav + ' | RGB + thermal'
            markers.append(label)
            last = self.path.poses[-1].pose.position if self.path.poses else None
            if last is None or math.dist([last.x, last.y, last.z], [position.x, position.y, position.z]) > 0.1:
                pose = PoseStamped()
                pose.header.frame_id, pose.header.stamp = self.frame, stamp
                pose.pose = robot.pose
                self.path.poses.append(pose)
                self.path.poses = self.path.poses[-1500:]
            self.path.header.stamp = stamp
            self.path_pub.publish(self.path)
        except TransformException:
            pass
        age = None if self.last_seen is None else (self.get_clock().now()-self.last_seen).nanoseconds/1e9
        recent = self.visible and age is not None and age < 2.0
        if self.centre is not None:
            color = (1.0, 0.1, 0.03, 1.0) if recent else (0.75, 0.2, 0.08, 0.9)
            centre = self.marker('fire', 0, Marker.SPHERE, color, 0.08)
            centre.pose.position = Point(x=float(self.centre[0]), y=float(self.centre[1]), z=self.ground+0.08)
            markers.append(centre)
            ellipse = ellipse_points(self.centre, self.covariance)
            markers.append(self.line('fire', 1, np.column_stack((ellipse, np.full(len(ellipse), self.ground+0.12))),
                                     (1.0, 0.8, 0.0, 1.0), 0.035))
            closed = np.vstack((self.boundary, self.boundary[0])).copy()
            closed[:, 2] += 0.035
            markers.append(self.line('fire', 2, closed, color, 0.06))
            label = self.marker('fire', 3, Marker.TEXT_VIEW_FACING, color, 0.4)
            label.pose.position = Point(x=float(self.centre[0]), y=float(self.centre[1]), z=self.ground+0.65)
            label.text = (f'Fire 1 | {self.uav}\n({self.centre[0]:.2f}, {self.centre[1]:.2f}) m\n'
                          f'2-sigma repeatability, not calibrated\nlast seen {age:.1f} s ago')
            markers.append(label)
        self.markers_pub.publish(MarkerArray(markers=markers))
        self.publish_fire_status(age, recent)

    def publish_fire_status(self, age, recent):
        status = {'fire_id': 'fire_1', 'source': self.uav, 'frame': self.frame,
                  'visible': bool(recent), 'partial': bool(self.partial), 'observations': self.observations,
                  'window_samples': len(self.rolling.samples), 'last_seen_age_s': age,
                  'centre_m': None if self.centre is None else self.centre.tolist(),
                  'covariance_xy_m2': None if self.covariance is None else self.covariance.tolist(),
                  'method': 'rolling mean; illustrative repeatability with 3 cm floor', 'note': self.note}
        self.status_pub.publish(String(data=json.dumps(status)))
        if self.note != self.last_note:
            self.get_logger().info(self.note)
            self.last_note = self.note


def main():
    rclpy.init()
    node = FireMapper()
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
            # The installed Jazzy context can race its SIGINT handler even
            # after ok() succeeds. Suppress only an already-completed shutdown.
            if 'rcl_shutdown already called' not in str(error):
                raise


if __name__ == '__main__':
    main()
