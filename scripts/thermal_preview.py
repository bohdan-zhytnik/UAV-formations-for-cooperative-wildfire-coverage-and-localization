#!/usr/bin/env python3
"""Display-only colour preview; quantitative measurements stay on image_raw.
Raw L16 values have resolution 0.02 kelvin/count. Preview range: 290..650 K.
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ThermalPreview(Node):
    def __init__(self):
        super().__init__('fire_thermal_preview')
        uav = self.declare_parameter('uav_name', 'uav1').value
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, f'/{uav}/thermal_down/image_color', qos_profile_sensor_data)
        self.sub = self.create_subscription(Image, f'/{uav}/thermal_down/image_raw', self.image, qos_profile_sensor_data)
        self.reported = False

    def image(self, msg):
        if msg.encoding not in ('mono16', '16UC1'):
            if not self.reported:
                self.get_logger().error('Expected mono16/16UC1 thermal data, received ' + msg.encoding)
                self.reported = True
            return
        kelvin = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32) * 0.02
        preview = np.clip((kelvin - 290.0) * (255.0 / 360.0), 0, 255).astype(np.uint8)
        output = self.bridge.cv2_to_imgmsg(cv2.applyColorMap(preview, cv2.COLORMAP_INFERNO), encoding='bgr8')
        output.header = msg.header
        self.pub.publish(output)
        if not self.reported:
            self.get_logger().info(f'Thermal preview active: {msg.encoding}, {msg.width}x{msg.height}, 0.02 K/count')
            self.reported = True

def main():
    rclpy.init()
    node = ThermalPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
