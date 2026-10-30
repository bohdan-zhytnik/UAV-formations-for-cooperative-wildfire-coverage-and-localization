#!/usr/bin/env python3
"""Explicitly addressed operator commands and fleet status; no autonomous flight."""
import json
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from mrs_msgs.msg import HwApiStatus, ControlManagerDiagnostics
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class FleetRouter(Node):
    def __init__(self):
        super().__init__('fleet_router')
        path = self.declare_parameter('scenario_file', '').value
        with open(path) as stream:
            self.config = yaml.safe_load(stream)
        self.names = [vehicle['name'] for vehicle in self.config['uavs']]
        self.selected = self.names[0]
        self.hardware = {}
        self.control = {}
        self.messages = {name: 'Waiting for hardware and MRS' for name in self.names}
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, '/fire_fleet/status', durable)
        self.notice = self.create_publisher(String, '/fire_fleet/notice', durable)
        self.goal_pubs, self.action_pubs, self.settings_pubs = {}, {}, {}
        self.owned_subscriptions = []
        for name in self.names:
            prefix = '/fire_fleet/'+name
            self.goal_pubs[name] = self.create_publisher(String, prefix+'/map_goal', 10)
            self.action_pubs[name] = self.create_publisher(String, prefix+'/action', 10)
            self.settings_pubs[name] = self.create_publisher(String, prefix+'/goal_settings', durable)
            self.owned_subscriptions += [
                self.create_subscription(HwApiStatus, f'/{name}/hw_api/status',
                    lambda msg, name=name: self.hardware.__setitem__(name, (msg, time.monotonic())), qos_profile_sensor_data),
                self.create_subscription(String, prefix+'/command_status',
                    lambda msg, name=name: self.messages.__setitem__(name, msg.data), durable),
                self.create_subscription(ControlManagerDiagnostics, f'/{name}/control_manager/diagnostics',
                    lambda msg, name=name: self.control.__setitem__(name, msg), qos_profile_sensor_data),
            ]
        self.create_subscription(String, '/fire_fleet/settings', self.on_settings, durable)
        self.create_subscription(String, '/fire_fleet/map_goal', self.on_goal, 10)
        self.create_subscription(String, '/fire_fleet/action', self.on_action, 10)
        self.create_timer(0.5, self.publish_status)

    def reject(self, error):
        message = 'Command rejected: ' + str(error)
        self.notice.publish(String(data=message))
        self.get_logger().warning(message)

    def addressed(self, msg):
        command = json.loads(msg.data)
        if command['uav'] not in self.names:
            raise ValueError('Unknown drone: ' + str(command['uav']))
        return command

    def on_settings(self, msg):
        try:
            command = self.addressed(msg)
            self.selected = command['uav']
            self.settings_pubs[self.selected].publish(msg)
        except (ValueError, KeyError, TypeError) as error:
            self.reject(error)

    def on_goal(self, msg):
        try:
            command = self.addressed(msg)
            # The selected UAV was captured by the RViz tool at mouse release.
            self.goal_pubs[command['uav']].publish(msg)
        except (ValueError, KeyError, TypeError) as error:
            self.reject(error)

    def on_action(self, msg):
        try:
            command = json.loads(msg.data)
            action = command['action']
            if not isinstance(command.get('request_id'), str) or not command['request_id']:
                raise ValueError('Missing request ID')
            if action in ('hover_all', 'mission_start_all'):
                for name, publisher in self.action_pubs.items():
                    publisher.publish(String(data=json.dumps(dict(command, uav=name,
                        action='hover' if action == 'hover_all' else 'mission_start', mode='cooperative'))))
                return
            command = self.addressed(msg)
            if action not in ('takeoff', 'land', 'hover', 'reset_map', 'mission_start', 'mission_resume', 'mission_pause'):
                raise ValueError('Unknown action: ' + str(action))
            self.action_pubs[command['uav']].publish(msg)
        except (ValueError, KeyError, TypeError) as error:
            self.reject(error)

    def publish_status(self):
        states = {}
        for name in self.names:
            hw, received = self.hardware.get(name, (None, 0))
            age = time.monotonic()-received
            position = None
            try:
                t = self.tf.lookup_transform(self.config['frame'], name+'/fcu', Time()).transform.translation
                position = [t.x, t.y, t.z]
            except TransformException:
                pass
            states[name] = {'connected': bool(hw and hw.connected and age < 5),
                            'armed': bool(hw and hw.armed), 'offboard': bool(hw and hw.offboard),
                            'status_age_s': age if hw else None, 'position_m': position,
                            'message': self.messages[name]}
            control = self.control.get(name)
            states[name]['tracker'] = control.active_tracker if control else None
            states[name]['flying_normally'] = bool(control and control.flying_normally)
        self.status.publish(String(data=json.dumps({'selected': self.selected, 'frame': self.config['frame'], 'uavs': states})))


def main():
    rclpy.init()
    node = FleetRouter()
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
