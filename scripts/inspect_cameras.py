#!/usr/bin/env python3
"""Read real ROS camera messages, save images, and optionally verify the hot patch."""
import argparse
import json
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

parser=argparse.ArgumentParser()
parser.add_argument('--require-fire', action='store_true')
args=parser.parse_args()
rclpy.init()
node=Node('fire_camera_inspection')
latest={}; counts={}
bridge=CvBridge()
subs=[]
for key,topic,typ in [
 ('rgb','/uav1/rgb_down/image_raw',Image),
 ('thermal','/uav1/thermal_down/image_raw',Image),
 ('preview','/uav1/thermal_down/image_color',Image),
 ('rgb_info','/uav1/rgb_down/camera_info',CameraInfo),
 ('thermal_info','/uav1/thermal_down/camera_info',CameraInfo)]:
 def receive(msg,key=key):
  latest[key]=msg; counts[key]=counts.get(key,0)+1
 subs.append(node.create_subscription(typ,topic,receive,qos_profile_sensor_data))
start=time.monotonic()
while time.monotonic()-start<35:
 rclpy.spin_once(node,timeout_sec=0.1)
 if len(latest)==5 and time.monotonic()-start>4: break
try:
 assert len(latest)==5, f'Missing camera streams: {set(["rgb","thermal","preview","rgb_info","thermal_info"])-latest.keys()}'
 rgb=bridge.imgmsg_to_cv2(latest['rgb'],desired_encoding='bgr8')
 raw=bridge.imgmsg_to_cv2(latest['thermal'],desired_encoding='passthrough')
 kelvin=raw.astype(np.float32)*0.02
 preview=bridge.imgmsg_to_cv2(latest['preview'],desired_encoding='bgr8')
 info={'samples':counts,'rgb_encoding':latest['rgb'].encoding,
       'thermal_encoding':latest['thermal'].encoding,'size':[latest['thermal'].width,latest['thermal'].height],
       'thermal_min_K':float(kelvin.min()),'thermal_max_K':float(kelvin.max()),
       'hot_pixels_above_500_K':int((kelvin>500).sum()),
       'rgb_frame':latest['rgb'].header.frame_id,'thermal_frame':latest['thermal'].header.frame_id,
       'rgb_info_frame':latest['rgb_info'].header.frame_id,'thermal_info_frame':latest['thermal_info'].header.frame_id,
       'rgb_fx':latest['rgb_info'].k[0],'thermal_fx':latest['thermal_info'].k[0]}
 assert info['rgb_fx']>0 and info['thermal_fx']>0, 'Camera calibration is missing'
 assert info['rgb_frame']=='uav1/rgb_down/optical'
 assert info['thermal_frame']=='uav1/thermal_down/optical'
 if args.require_fire:
  assert info['hot_pixels_above_500_K']>100, 'Hot patch not visible in thermal camera'
  assert 570<info['thermal_max_K']<630, 'Unexpected hot-patch temperature'
 out=Path(__file__).resolve().parents[1]/'artifacts'; out.mkdir(exist_ok=True)
 cv2.imwrite(str(out/'rgb.png'),rgb)
 cv2.imwrite(str(out/'thermal_raw.png'),raw)
 cv2.imwrite(str(out/'thermal_preview.png'),preview)
 cv2.imwrite(str(out/'camera_pair.png'),np.hstack([rgb,preview]))
 (out/'camera_check.json').write_text(json.dumps(info,indent=2)+'\n')
 print(json.dumps(info,indent=2),flush=True)
finally:
 node.destroy_node()
 rclpy.try_shutdown()
