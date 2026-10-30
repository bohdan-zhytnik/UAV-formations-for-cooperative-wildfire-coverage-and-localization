#!/usr/bin/env python3
"""One vehicle's independent MRS goal adapter and asynchronous flight actions."""
import json
import math
import time
import uuid
import yaml
import numpy as np

import rclpy
from geometry_msgs.msg import PoseStamped, Point
from mrs_msgs.msg import ControlManagerDiagnostics
from mrs_msgs.srv import ReferenceStampedSrv
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformException

from goal_bridge import GoalBridge
from fire_geometry import rotation_matrix
from mission_logic import VehicleMission, waypoint


class FleetVehicle(GoalBridge):
    queue_manual_goals = True

    def __init__(self):
        super().__init__()
        self.routine = None
        self.control_state = None
        self.control_received = -math.inf
        self.command_epoch = 0
        self.reference_calls = {}
        self.pending_action = self.action_future = self.auto_pending = None
        self.command_goal_common = None
        self.coordinator_id, self.coordinator_received = None, -math.inf
        self.latest_image_stamp, self.fire_map = None, {}
        self.map_generation, self.last_mission_clock = None, 0.
        self.mission_started = False
        path = self.declare_parameter('mission_file', '').value
        config = yaml.safe_load(open(path)) if path else {'settings': {}, 'uavs': {}}
        self.mission_settings = config['settings']
        preset = config['uavs'].get(self.uav, {})
        self.preset = preset
        self.mission = VehicleMission(preset.get('route', []), preset.get('loop', True),
            self.mission_settings.get('dwell_sim_s', 3.),
            self.mission_settings.get('arrival_tolerance_m', .4),
            self.mission_settings.get('arrival_hold_sim_s', 1.),
            self.mission_settings.get('travel_timeout_sim_s', 90.),
            self.mission_settings.get('observation_timeout_sim_s', 12.))
        self.mission_pub = self.create_publisher(String, f'/fire_fleet/{self.uav}/mission_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.config_ack = self.create_publisher(String, '/fire_fleet/mission/config_ack', 10)
        self.ack_pub = self.create_publisher(String, f'/fire_fleet/{self.uav}/action_ack',
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.operation_clients = {
            'output': self.create_client(SetBool, f'/{self.uav}/control_manager/toggle_output'),
            'offboard': self.create_client(Trigger, f'/{self.uav}/hw_api/offboard'),
            'arming': self.create_client(SetBool, f'/{self.uav}/hw_api/arming'),
            'takeoff': self.create_client(Trigger, f'/{self.uav}/uav_manager/takeoff'),
            'land': self.create_client(Trigger, f'/{self.uav}/uav_manager/land'),
        }
        self.create_subscription(String, f'/fire_fleet/{self.uav}/map_goal', self.map_goal, 10)
        self.create_subscription(ControlManagerDiagnostics, f'/{self.uav}/control_manager/diagnostics',
                                 self.on_control_state, qos_profile_sensor_data)
        self.create_timer(0.1, self.advance_takeoff)
        self.create_subscription(String, '/fire_fleet/mission/heartbeat', self.on_coordinator, 10)
        self.create_subscription(String, f'/fire_fleet/{self.uav}/mission_task', self.on_mission_task, 10)
        self.create_subscription(String, f'/fire_fleet/{self.uav}/mission_config', self.on_mission_config, 10)
        self.create_subscription(String, f'/fire_fleet/{self.uav}/observations', self.on_observations, 5)
        self.create_subscription(String, '/fire_fleet/fire_map/status', self.on_fire_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.2, self.mission_tick)
        self.create_timer(.25, self.mission_watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.report('Map commands disabled. Use Take off selected, wait for MpcTracker, then enable map goals.')

    def on_control_state(self, msg):
        self.control_state, self.control_received = msg, time.monotonic()

    def map_goal(self, msg):
        """The GUI binds vehicle, altitude, enable state and pose in ONE message.

        Selecting another vehicle while this message is in transit cannot retarget it.
        A vehicle keeps its own service future and pending transform independently.
        """
        try:
            command = json.loads(msg.data)
            if command['uav'] != self.uav:
                raise ValueError('Wrong vehicle in addressed goal')
            x, y, heading = [float(command[key]) for key in ('x', 'y', 'heading')]
            altitude = float(command['altitude_m'])
            if not all(math.isfinite(value) for value in [x, y, heading, altitude]):
                raise ValueError('Goal coordinates must be finite')
            if type(command['enabled']) is not bool:
                raise ValueError('enabled must be boolean')
            if not self.settings['minimum_altitude_m'] <= altitude <= self.settings['maximum_altitude_m']:
                raise ValueError('Altitude outside the fleet limits')
            if not command['frame']:
                raise ValueError('Missing goal coordinate frame')
            self.on_settings(String(data=json.dumps({
                'uav': self.uav, 'altitude_m': altitude, 'enabled': command['enabled']})))
            goal = PoseStamped()
            goal.header.frame_id = command['frame']
            goal.pose.position.x, goal.pose.position.y = x, y
            goal.pose.orientation.z, goal.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
            self.on_goal(goal)
        except (ValueError, KeyError, TypeError) as error:
            self.report('Goal rejected: ' + str(error))

    def on_goal(self, msg):
        if self.pending_action is not None or self.action_future is not None:
            self.report('Goal ignored: wait for the pending hover or landing response.')
            return
        if self.routine is not None:
            self.report('Goal ignored: takeoff preparation is still in progress.')
            return
        if self.enabled and self.ready_to_fly() and (
                self.control_state is None or time.monotonic()-self.control_received > 5
                or not self.control_state.flying_normally or self.control_state.active_tracker != 'MpcTracker'):
            self.report('Goal ignored: wait for normal flight with MpcTracker after takeoff; map goals are unavailable during landing.')
            return
        if self.enabled and self.ready_to_fly():
            self.take_manual_control('Manual map goal')
        super().on_goal(msg)

    def on_action(self, msg):
        try:
            command = json.loads(msg.data)
            if command['uav'] != self.uav:
                raise ValueError('Wrong drone in addressed action')
            action, request_id = command['action'], command['request_id']
            if not isinstance(request_id, str) or not request_id:
                raise ValueError('Missing request ID')
        except (ValueError, KeyError, TypeError) as error:
            self.report('Action rejected: ' + str(error))
            return
        if action in ('mission_start', 'mission_resume'):
            try:
                if not self.mission_ready() or self.reference_calls or self.pending_action or self.action_future:
                    raise ValueError('Wait for normal MpcTracker flight and any pending command response')
                if not self.coordinator_fresh():
                    raise ValueError('Mission coordinator is unavailable')
                resume = action == 'mission_resume'
                if resume and not self.mission_started:
                    raise ValueError('No previous mission to resume; use Start first')
                self.mission.start(str(uuid.uuid4()), self.mission.mode if resume else command.get('mode', 'route'), resume)
                self.command_epoch += 1
                self.pending = None
                self.mission_started = True
                self.publish_mission_status()
                self.finish_action(action, request_id, True, 'Mission requested; take over with a manual goal, hover or land.')
            except ValueError as error:
                self.finish_action(action, request_id, False, str(error))
        elif action == 'mission_pause':
            self.take_manual_control('Paused by operator')
            self.queue_action(action, request_id, self.hover)
        elif action == 'takeoff':
            if self.routine is not None:
                self.finish_action(action, request_id, False, 'Another takeoff sequence is already running.')
                return
            if self.hardware is None or time.monotonic()-self.hardware_received > 5 or not self.hardware.connected:
                self.finish_action(action, request_id, False, 'Fresh connected hardware status is required.')
                return
            if self.hardware.armed:
                self.finish_action(action, request_id, False, 'Vehicle is already armed.')
                return
            self.routine = {'step': 0, 'phase': 'send', 'request_id': request_id}
            self.report('takeoff: preparing control output, offboard and arming.')
        elif action in ('land', 'hover', 'reset_map'):
            if action in ('land', 'hover'):
                self.take_manual_control('Manual '+action)
                if self.routine is not None:
                    self.fail_takeoff('Cancelled by ' + action)
                self.pending = None
            client = {'land': self.operation_clients['land'], 'hover': self.hover, 'reset_map': self.reset}[action]
            if action in ('land', 'hover'):
                self.queue_action(action, request_id, client)
            else:
                self.trigger_action(action, request_id, client)
        else:
            self.finish_action(action, request_id, False, 'Unknown action')

    def finish_action(self, action, request_id, success, message):
        self.report(f'{action} {"completed" if success else "failed"}: {message}')
        self.ack_pub.publish(String(data=json.dumps({'uav': self.uav, 'request_id': request_id,
            'action': action, 'success': success, 'message': message})))

    def fail_takeoff(self, reason):
        request_id = self.routine['request_id']
        self.routine = None
        self.finish_action('takeoff', request_id, False, reason + '; later steps were not sent.')

    def advance_takeoff(self):
        routine = self.routine
        if routine is None:
            return
        operations = ['output', 'offboard', 'arming', 'takeoff']
        operation = operations[routine['step']]
        now = time.monotonic()
        phase = routine['phase']
        if phase == 'send':
            client = self.operation_clients[operation]
            if not client.service_is_ready():
                self.fail_takeoff('Service unavailable: ' + operation)
                return
            request = SetBool.Request(data=True) if operation in ('output', 'arming') else Trigger.Request()
            routine.update(phase='reply', future=client.call_async(request), deadline=now+15)
        elif phase == 'reply':
            future = routine['future']
            if not future.done():
                if now > routine['deadline']:
                    self.fail_takeoff('No response from ' + operation + '; outcome unknown')
                return
            try:
                response = future.result()
                if response is None or not response.success:
                    self.fail_takeoff(operation + ': ' + (response.message if response else 'no response'))
                    return
                self.report('takeoff: ' + operation + ': ' + response.message)
            except Exception as error:
                self.fail_takeoff(str(error))
                return
            if operation == 'output':
                routine.update(phase='warmup', warmup_sim_ns=self.get_clock().now().nanoseconds+2_000_000_000)
            elif operation in ('offboard', 'arming'):
                routine.update(phase='confirm', deadline=now+15)
            else:
                self.routine = None
                self.finish_action('takeoff', routine['request_id'], True,
                    'MRS takeoff initiated; wait for the climb to finish.')
        elif phase == 'warmup':
            if self.get_clock().now().nanoseconds >= routine['warmup_sim_ns']:
                routine.update(step=routine['step']+1, phase='send')
        elif phase == 'confirm':
            flag = 'offboard' if operation == 'offboard' else 'armed'
            if self.hardware is not None and now-self.hardware_received < 5 and getattr(self.hardware, flag):
                routine.update(step=routine['step']+1, phase='send')
            elif now > routine['deadline']:
                self.fail_takeoff('Hardware status did not confirm ' + flag)

    def trigger_action(self, action, request_id, client):
        if not client.service_is_ready():
            self.finish_action(action, request_id, False, 'MRS service is unavailable.')
            return
        future = client.call_async(Trigger.Request())
        if action in ('hover', 'land', 'mission_pause'):
            self.action_future = future
        def complete(result):
            if self.action_future is result:
                self.action_future = None
            try:
                response = result.result()
                if response.success and action in ('hover', 'land', 'mission_pause'):
                    self.command_goal_common = None
                if response.success and action in ('hover', 'mission_pause'):
                    self.marker_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
                self.finish_action(action, request_id, response.success, response.message)
            except Exception as error:
                self.finish_action(action, request_id, False, str(error))
        future.add_done_callback(complete)

    def take_manual_control(self, reason):
        self.command_epoch += 1
        self.auto_pending = self.pending = None
        self.mission.pause(reason)
        self.publish_mission_status()

    def queue_action(self, action, request_id, client):
        if self.pending_action:
            previous, previous_id, _ = self.pending_action
            if action != 'land':
                self.finish_action(action, request_id, False, 'Another manual action is pending')
                return
            self.finish_action(previous, previous_id, False, 'Superseded by landing')
        self.pending_action = (action, request_id, client)
        # A sent MRS reference cannot be unsent. Drain its reply before sending
        # hover/land, so a late autonomous reference cannot override the operator.
        if self.reference_calls:
            self.report(action+': waiting for the outstanding reference response; mission ownership is already cancelled.')

    def position_common(self):
        try:
            transform = self.tf.lookup_transform('common_origin', self.uav+'/fcu', Time())
            if abs((self.get_clock().now()-Time.from_msg(transform.header.stamp)).nanoseconds/1e9) > 2.:
                return None
            p = transform.transform.translation
            return [p.x, p.y, p.z]
        except TransformException:
            return None

    def mission_ready(self):
        return (self.ready_to_fly() and self.routine is None and self.control_state is not None
                and time.monotonic()-self.control_received < 5 and self.control_state.flying_normally
                and self.control_state.active_tracker == 'MpcTracker' and self.position_common() is not None)

    def coordinator_fresh(self):
        return self.coordinator_id and time.monotonic()-self.coordinator_received < self.mission_settings.get('coordinator_timeout_wall_s', 5.)

    def on_coordinator(self, msg):
        try:
            identifier = json.loads(msg.data)['coordinator_id']
            if not isinstance(identifier, str) or not identifier:
                return
            if self.coordinator_id not in (None, identifier) and self.mission.active:
                self.pause_and_hover('Mission coordinator restarted; explicit resume required')
            self.coordinator_id, self.coordinator_received = identifier, time.monotonic()
        except (ValueError, KeyError, TypeError):
            pass

    def on_observations(self, msg):
        try:
            batch = json.loads(msg.data)
            stamp = batch['stamp_ns']/1e9
            now = self.get_clock().now().nanoseconds/1e9
            if batch['source'] == self.uav and batch['frame'] == 'common_origin' and now-3 <= stamp <= now+.1:
                self.latest_image_stamp = stamp
        except (ValueError, KeyError, TypeError):
            pass

    def on_fire_map(self, msg):
        try:
            data = json.loads(msg.data)
            if self.map_generation is not None and data['generation'] != self.map_generation and self.mission.active:
                if self.mission.mode == 'cooperative' or self.mission.target and self.mission.target['fire_ids']:
                    self.pause_and_hover('Shared fire map reset; recheck IDs and explicitly resume')
            self.map_generation, self.fire_map = data['generation'], data
        except (ValueError, KeyError, TypeError):
            pass

    def on_mission_task(self, msg):
        try:
            data = json.loads(msg.data)
            if data['uav'] != self.uav or data['coordinator_id'] != self.coordinator_id or not self.coordinator_fresh():
                return
            if data['op'] == 'task':
                self.mission.accept_task(data['session'], data['request'], data['target'])
            elif data['op'] == 'grant':
                if self.mission.grant(data['session'], data['request']):
                    self.auto_pending = (self.mission.session, self.mission.request, dict(self.mission.target))
            elif data['op'] == 'wait' and self.mission.active:
                if (data['session'], data['request']) == (self.mission.session, self.mission.request) and self.mission.phase == 'waiting_reservation':
                    if self.mission.reason != data['reason']:
                        self.report('Mission waiting: '+data['reason'])
                    self.mission.reason = data['reason']
        except (ValueError, KeyError, TypeError) as error:
            self.report('Mission proposal rejected: '+str(error))

    def on_mission_config(self, msg):
        request_id = ''
        try:
            data = json.loads(msg.data)
            request_id = data['request_id']
            if data['uav'] != self.uav or data['coordinator_id'] != self.coordinator_id:
                raise ValueError('Wrong configuration target or coordinator')
            if self.mission.active:
                raise ValueError('Pause the mission before editing its route')
            route = list(self.mission.route)
            op = data['op']
            if op == 'append_waypoint':
                if len(route) >= 100:
                    raise ValueError('Route is limited to 100 points')
                route.append(waypoint(data['waypoint'], self.mission.dwell))
            elif op == 'set_route':
                if len(data['waypoints']) > 100:
                    raise ValueError('Route is limited to 100 points')
                route = [waypoint(value, self.mission.dwell) for value in data['waypoints']]
            elif op == 'clear_route':
                route = []
            elif op == 'pop_waypoint':
                route = route[:-1]
            elif op == 'load_preset':
                route = [waypoint(value, self.mission.dwell) for value in self.preset.get('route', [])]
            elif op == 'set_options':
                pass
            else:
                raise ValueError('Unknown route configuration operation')
            dwell = float(data.get('dwell_sim_s', self.mission.dwell))
            if not math.isfinite(dwell) or not 1 <= dwell <= 60 or type(data.get('loop', self.mission.loop)) is not bool:
                raise ValueError('Invalid dwell or loop setting')
            if 'dwell_sim_s' in data:
                route = [dict(value, dwell_sim_s=dwell) for value in route]
            self.mission.route, self.mission.dwell = route, dwell
            self.mission.loop = data.get('loop', self.mission.loop)
            self.mission.route_index = 0
            self.mission.pause('Route updated; explicit Start or Resume required')
            self.config_ack.publish(String(data=json.dumps({'request_id': request_id, 'uav': self.uav,
                'success': True, 'message': f'Route updated: {len(route)} points'})))
        except (ValueError, KeyError, TypeError) as error:
            self.config_ack.publish(String(data=json.dumps({'request_id': request_id, 'uav': self.uav,
                'success': False, 'message': str(error)})))
        self.publish_mission_status()

    def mission_watchdog(self):
        if self.mission.active and (not self.coordinator_fresh() or not self.ready_to_fly()):
            self.pause_and_hover('Coordinator or hardware heartbeat lost; explicit resume required')

    def pause_and_hover(self, reason):
        self.take_manual_control(reason)
        if self.hardware and self.hardware.armed and not self.pending_action and not self.action_future:
            self.queue_action('hover', str(uuid.uuid4()), self.hover)

    def mission_tick(self):
        now = self.get_clock().now().nanoseconds/1e9
        if now < self.last_mission_clock and self.mission.active:
            self.pause_and_hover('Simulation time rewound; explicit resume required')
        self.last_mission_clock = now
        if self.mission.active:
            if not self.mission_ready():
                self.pause_and_hover('Normal flight or position unavailable; explicit resume required')
            else:
                stamp = self.fire_map.get('stamp_ns', 0)/1e9
                seen = {fire['id']: stamp-fire['last_seen_by_uav_s'][self.uav]
                        for fire in self.fire_map.get('fires', []) if self.uav in fire['last_seen_by_uav_s']}
                event = self.mission.tick(now, self.position_common(), self.latest_image_stamp, seen)
                if event == 'paused':
                    self.pause_and_hover(self.mission.reason)
        self.publish_mission_status()

    def publish_mission_status(self):
        if not hasattr(self, 'mission_pub'):
            return
        data = dict(self.mission.snapshot(), uav=self.uav, position_m=self.position_common(),
                    armed=bool(self.hardware and self.hardware.armed),
                    hardware_fresh=bool(self.hardware and self.hardware.connected and time.monotonic()-self.hardware_received < 5),
                    command_goal_common=self.command_goal_common, reference_busy=bool(self.reference_calls),
                    pending_goal_common=getattr(self, 'pending_goal_common', None),
                    ready=self.mission_ready(), coordinator_fresh=bool(self.coordinator_fresh()))
        self.mission_pub.publish(String(data=json.dumps(data, allow_nan=False)))

    def request_to_common(self, request):
        point = request.reference.position
        xyz = np.array([point.x, point.y, point.z])
        if request.header.frame_id != 'common_origin':
            transform = self.tf.lookup_transform('common_origin', request.header.frame_id, Time()).transform
            q, t = transform.rotation, transform.translation
            xyz = rotation_matrix([q.x, q.y, q.z, q.w]) @ xyz+np.array([t.x, t.y, t.z])
        return xyz.tolist()

    def on_reference_sent(self, future, request):
        self.reference_calls[future] = {'kind': 'manual', 'epoch': self.command_epoch, 'sent': time.monotonic()}
        try:
            self.pending_goal_common = self.request_to_common(request)
        except TransformException:
            self.pending_goal_common = None

    def reference_result(self, future, request):
        meta = self.reference_calls.pop(future, None)
        self.pending_goal_common = None
        if self.inflight is not None and self.inflight[0] is future:
            self.inflight = None
        if meta is None:
            return
        try:
            response = future.result()
            if response.success:
                self.command_goal_common = self.request_to_common(request)
            if meta['epoch'] != self.command_epoch:
                return  # A late reply cannot restore ownership or an old target marker.
            if meta['kind'] == 'manual':
                super().reference_result(future, request)
            else:
                self.mission.reference_reply(meta['session'], meta['request'], response.success,
                    self.get_clock().now().nanoseconds/1e9, response.message)
                if response.success and self.mission.active:
                    self.publish_goal(request)
                    self.report('Mission goal accepted by MRS')
                elif not response.success:
                    self.pause_and_hover('MRS rejected mission goal: '+response.message)
        except Exception as error:
            if self.mission.active:
                self.pause_and_hover('Mission reference failed: '+str(error))
            self.report('Reference response failed: '+str(error))

    def process_goal(self):
        if not hasattr(self, 'reference_calls'):
            return
        if self.reference_calls:
            if any(time.monotonic()-meta['sent'] > 15 for meta in self.reference_calls.values()):
                if self.mission.active:
                    self.pause_and_hover('MRS reference response timed out; waiting before manual command')
            return
        if self.action_future is not None:
            return
        if self.pending_action is not None:
            action, identifier, client = self.pending_action
            self.pending_action = None
            self.trigger_action(action, identifier, client)
            return
        if self.auto_pending is not None:
            session, identifier, target = self.auto_pending
            self.auto_pending = None
            if not self.mission.active or (session, identifier) != (self.mission.session, self.mission.request):
                return
            if not self.mission_ready():
                self.pause_and_hover('Mission flight readiness lost')
                return
            try:
                transform = self.tf.lookup_transform(self.frame, 'common_origin', Time()).transform
                q, t = transform.rotation, transform.translation
                rotation = rotation_matrix([q.x, q.y, q.z, q.w])
                xyz = rotation @ np.asarray(target['position'])+np.array([t.x, t.y, t.z])
                direction = rotation @ np.array([math.cos(target['heading']), math.sin(target['heading']), 0.])
                request = ReferenceStampedSrv.Request()
                request.header.frame_id = self.frame
                request.reference.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
                request.reference.heading = math.atan2(direction[1], direction[0])
                if not self.reference.service_is_ready():
                    raise ValueError('MRS reference service unavailable')
                future = self.reference.call_async(request)
                self.inflight = (future, time.monotonic())
                self.reference_calls[future] = {'kind': 'mission', 'epoch': self.command_epoch,
                    'session': session, 'request': identifier, 'sent': time.monotonic()}
                self.pending_goal_common = target['position']
                future.add_done_callback(lambda result: self.reference_result(result, request))
            except (ValueError, TransformException) as error:
                self.pause_and_hover('Mission goal unavailable: '+str(error))
            return
        if not self.mission.active:
            super().process_goal()


def main():
    rclpy.init()
    node = FleetVehicle()
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
