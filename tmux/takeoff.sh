#!/usr/bin/env python3
"""Repeatable takeoff after landing. Stop immediately on any rejected request."""
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from mrs_msgs.msg import HwApiStatus
from rclpy.qos import qos_profile_sensor_data

def main():
    rclpy.init()
    n = Node('fire_demo_takeoff')
    status = [None]
    subscription = n.create_subscription(HwApiStatus, '/uav1/hw_api/status', lambda msg: status.__setitem__(0, msg), qos_profile_sensor_data)
    steps = [('/uav1/control_manager/toggle_output', SetBool, SetBool.Request(data=True)),
             ('/uav1/hw_api/offboard', Trigger, Trigger.Request()),
             ('/uav1/hw_api/arming', SetBool, SetBool.Request(data=True)),
             ('/uav1/uav_manager/takeoff', Trigger, Trigger.Request())]
    try:
        for name, typ, req in steps:
            client = n.create_client(typ, name)
            if not client.wait_for_service(timeout_sec=15):
                raise RuntimeError('Service unavailable: ' + name)
            future = client.call_async(req)
            rclpy.spin_until_future_complete(n, future, timeout_sec=15)
            if not future.done() or future.result() is None:
                raise RuntimeError('No response: ' + name)
            result = future.result()
            print(name, result.success, result.message, flush=True)
            if not result.success:
                raise RuntimeError('Request rejected; later steps were not sent.')
            deadline = time.monotonic() + 15
            if name.endswith('/arming') or name.endswith('/offboard'):
                flag = 'armed' if name.endswith('/arming') else 'offboard'
                while time.monotonic() < deadline:
                    rclpy.spin_once(n, timeout_sec=0.1)
                    if status[0] is not None and getattr(status[0], flag):
                        break
                else:
                    raise RuntimeError('Hardware status did not confirm ' + flag)
            elif name.endswith('/toggle_output'):
                # Allow offboard setpoints to arrive before switching modes.
                warmup = time.monotonic() + 2
                while time.monotonic() < warmup:
                    rclpy.spin_once(n, timeout_sec=0.1)
    finally:
        n.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
