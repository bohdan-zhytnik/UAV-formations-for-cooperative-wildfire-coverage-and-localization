#!/usr/bin/env python3
import os
import rclpy
import yaml
from rclpy.node import Node
from mrs_msgs.srv import String


def main():
    with open(os.environ['FIRE_FLEET_SCENARIO_FILE']) as stream:
        scenario = yaml.safe_load(stream)
    rclpy.init()
    node = Node('spawn_fire_fleet')
    client = node.create_client(String, '/mrs_drone_spawner/spawn')
    try:
        if not client.wait_for_service(timeout_sec=45):
            raise RuntimeError('Gazebo spawner service unavailable')
        for vehicle in scenario['uavs']:
            position = ' '.join(str(value) for value in vehicle['spawn'])
            value = f"{vehicle['id']} --{scenario['model']} --enable-rangefinder --pos {position}"
            future = client.call_async(String.Request(value=value))
            rclpy.spin_until_future_complete(node, future, timeout_sec=45)
            if not future.done() or future.result() is None:
                raise RuntimeError('No spawner response for '+vehicle['name'])
            result = future.result()
            print(vehicle['name'], result.success, result.message, flush=True)
            if not result.success:
                raise RuntimeError('Spawn rejected: '+result.message)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
