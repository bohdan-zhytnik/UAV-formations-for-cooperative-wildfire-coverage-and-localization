#!/usr/bin/env python3
"""Assign image-derived fire tasks and reserve conservative horizontal corridors."""
import json
import math
import time
import uuid

import rclpy
import yaml
from geometry_msgs.msg import Point
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from mission_logic import area_points, can_reserve, choose_task
from survey_logic import make_survey, claim_discoveries, choose_survey_task, owned_age


class MissionCoordinator(Node):
    def __init__(self):
        super().__init__('mission_coordinator')
        with open(self.declare_parameter('mission_file', '').value) as stream:
            config = yaml.safe_load(stream)
        with open(self.declare_parameter('scenario_file', '').value) as stream:
            scenario = yaml.safe_load(stream)
        self.names = [uav['name'] for uav in scenario['uavs']]
        self.colours = {uav['name']: uav['color'] for uav in scenario['uavs']}
        self.identifier = str(uuid.uuid4())
        self.settings = config['settings']
        self.areas = {name: config['uavs'][name]['area'] for name in self.names}
        for area in self.areas.values():
            area_points(area)
        self.revisit = {name: self.settings['revisit_sim_s'] for name in self.names}
        self.survey = None
        self.survey_started = 0.
        self.coverage = {name: set() for name in self.names}
        if config.get('survey', {}).get('enabled', False):
            self.survey = make_survey(config['survey'], self.names, self.settings['minimum_separation_m'])
            self.areas = self.survey['areas']
            self.revisit = {name: self.survey['revisit_sim_s'] for name in self.names}
        self.ownership_mode = {}
        self.assignments, self.states, self.received = {}, {}, {}
        self.reservations, self.proposals = {}, {}
        self.fire_map, self.map_generation = {}, None
        self.notice = 'Missions idle. Take off manually, then explicitly Start.'
        self.config_ids = set()
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.heartbeat = self.create_publisher(String, '/fire_fleet/mission/heartbeat', 10)
        self.status = self.create_publisher(String, '/fire_fleet/mission/status', durable)
        self.markers = self.create_publisher(MarkerArray, '/fire_fleet/mission/markers', durable)
        self.acks = self.create_publisher(String, '/fire_fleet/mission/config_ack', 10)
        self.tasks, self.configs = {}, {}
        for name in self.names:
            self.tasks[name] = self.create_publisher(String, f'/fire_fleet/{name}/mission_task', 10)
            self.configs[name] = self.create_publisher(String, f'/fire_fleet/{name}/mission_config', 10)
            self.create_subscription(String, f'/fire_fleet/{name}/mission_status',
                lambda msg, name=name: self.on_state(name, msg), durable)
        self.create_subscription(String, '/fire_fleet/fire_map/status', self.on_map, durable)
        self.create_subscription(String, '/fire_fleet/mission/config', self.on_config, 10)
        self.create_timer(.5, self.tick)
        self.create_timer(1., self.send_heartbeat, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.send_heartbeat()

    def send_heartbeat(self):
        self.heartbeat.publish(String(data=json.dumps({'coordinator_id': self.identifier})))

    def on_state(self, name, msg):
        try:
            data = json.loads(msg.data)
            if data['uav'] == name:
                self.states[name], self.received[name] = data, time.monotonic()
        except (ValueError, KeyError, TypeError):
            pass

    def on_map(self, msg):
        try:
            data = json.loads(msg.data)
            if self.map_generation not in (None, data['generation']):
                self.assignments.clear()
                self.ownership_mode.clear()
                self.notice = 'Fire map reset: assignments cleared. Recheck IDs before resuming.'
            self.map_generation, self.fire_map = data['generation'], data
            self.claim_new_fires()
        except (ValueError, KeyError, TypeError):
            pass

    def on_config(self, msg):
        request_id, name = '', ''
        try:
            data = json.loads(msg.data)
            request_id, name = data['request_id'], data.get('uav', 'all')
            shared = data.get('op') == 'set_survey'
            if not isinstance(request_id, str) or not request_id or (not shared and name not in self.names):
                raise ValueError('Configuration needs a request_id and explicit uav1 or uav2')
            if request_id in self.config_ids:
                return
            self.config_ids.add(request_id)
            if len(self.config_ids) > 2000:
                self.config_ids = {request_id}
            op = data['op']
            if op in ('append_waypoint', 'set_route', 'clear_route', 'pop_waypoint', 'load_preset', 'set_options'):
                if data.get('frame', 'common_origin') != 'common_origin':
                    raise ValueError('Route editing requires RViz Fixed Frame common_origin')
                self.configs[name].publish(String(data=json.dumps(dict(data, coordinator_id=self.identifier))))
                return  # The vehicle acknowledges the edit after validating its own state.
            if op == 'set_survey':
                if data.get('frame', 'common_origin') != 'common_origin':
                    raise ValueError('Survey selection requires Fixed Frame common_origin')
                if any(s.get('active') or s.get('reference_busy') for s in self.states.values()):
                    raise ValueError('Pause both missions and wait for pending references before replacing the survey')
                survey = make_survey(data['survey'], self.names, self.settings['minimum_separation_m'])
                self.survey, self.areas = survey, survey['areas']
                self.revisit = {uav: survey['revisit_sim_s'] for uav in self.names}
                self.coverage = {uav: set() for uav in self.names}
                self.survey_started = self.get_clock().now().nanoseconds/1e9
                self.assignments.clear()
                self.ownership_mode.clear()
                self.proposals.clear()
                self.claim_new_fires()
                name = 'all'
                message = 'Survey split equally; discovery claims enabled. Take off and Start both: cooperative.'
            elif op == 'set_area':
                area_points(data['area'])
                # Store normalized numbers; validation alone must not retain text coordinates.
                self.areas[name] = {'bounds': [float(v) for v in data['area']['bounds']],
                    'spacing_m': float(data['area'].get('spacing_m', 1.)),
                    'altitude_m': float(data['area'].get('altitude_m', 5.))}
                self.survey = None  # Explicit per-drone areas restore legacy operation.
                message = 'Search area updated; takes effect at the next task'
            elif op == 'assign_fires':
                ids = data['fire_ids']
                available = {fire['id'] for fire in self.fire_map.get('fires', [])}
                if not isinstance(ids, list) or any(type(i) is not int or i not in available for i in ids):
                    raise ValueError('Use existing shared-map fire IDs; an empty list clears assignments')
                for other, state in self.states.items():
                    if other != name and state['active'] and set(ids).intersection((state.get('target') or {}).get('fire_ids', [])):
                        raise ValueError('A requested fire is currently being inspected by '+other)
                self.assignments = {i: owner for i, owner in self.assignments.items() if owner != name}
                self.assignments.update({i: name for i in ids})
                self.ownership_mode = {i: mode for i, mode in self.ownership_mode.items() if i in self.assignments}
                self.ownership_mode.update({i: 'manual' for i in ids})
                message = 'Fire assignments updated; takes effect at the next task'
            elif op == 'set_revisit':
                interval = float(data['revisit_sim_s'])
                if not math.isfinite(interval) or not 3 <= interval <= 600:
                    raise ValueError('Revisit threshold must be 3..600 simulated seconds')
                self.revisit[name] = interval
                message = 'Stale-observation threshold updated'
            else:
                raise ValueError('Unknown mission configuration operation')
            self.notice = message
            self.ack(request_id, name, True, message)
        except (ValueError, KeyError, TypeError) as error:
            self.ack(request_id, name, False, str(error))

    def claim_new_fires(self):
        if self.survey:
            claims = claim_discoveries(self.fire_map.get('fires', []), self.assignments, self.survey)
            self.assignments.update(claims)
            originals = {fire['id']: fire.get('first_observer') for fire in self.fire_map.get('fires', [])}
            self.ownership_mode.update({identifier: 'first_observer' if owner == originals[identifier] else 'area_observer'
                                       for identifier, owner in claims.items()})

    def ack(self, request_id, name, success, message):
        self.acks.publish(String(data=json.dumps(dict(request_id=request_id, uav=name, success=success, message=message))))

    def send(self, name, state, op, **extra):
        self.tasks[name].publish(String(data=json.dumps(dict(uav=name, coordinator_id=self.identifier,
            session=state['session'], request=state['request'], op=op, **extra))))

    def tick(self):
        now_wall = time.monotonic()
        states = {name: dict(self.states.get(name, {}),
            stale=now_wall-self.received.get(name, -math.inf) > 5.) for name in self.names}
        for collection in (self.reservations, self.proposals):
            for name, item in list(collection.items()):
                state = states[name]
                if not state.get('active') or (item['session'], item['request']) != (state.get('session'), state.get('request')):
                    del collection[name]
        # Account for age between map publications, using simulation time.
        now = self.get_clock().now().nanoseconds/1e9
        extra_age = max(0., now-self.fire_map.get('stamp_ns', 0)/1e9)
        fires = [dict(fire, last_seen_age_s=fire['last_seen_age_s']+extra_age,
                      last_seen_by_uav_s={name: age+extra_age for name, age in fire['last_seen_by_uav_s'].items()})
                 for fire in self.fire_map.get('fires', [])]
        for name, state in states.items():
            if state['stale'] or not state.get('active') or not state.get('coordinator_fresh'):
                continue
            if state['phase'] == 'requesting_task':
                claimed = set()
                for other, other_state in states.items():
                    if other != name and other_state.get('active'):
                        claimed.update((other_state.get('target') or {}).get('fire_ids', []))
                for other, proposal in self.proposals.items():
                    if other != name:
                        claimed.update(proposal['target'].get('fire_ids', []))
                if name not in self.proposals:
                    if self.survey:
                        target = choose_survey_task(name, state, fires, self.assignments, claimed,
                            dict(self.survey, revisit_sim_s=self.revisit[name]))
                    else:
                        target = choose_task(name, state, fires, self.areas[name], self.assignments, claimed,
                            state['dwell_sim_s'], self.revisit[name])
                    self.proposals[name] = dict(session=state['session'], request=state['request'], target=target)
                self.send(name, state, 'task', target=self.proposals[name]['target'])
            elif state['phase'] == 'waiting_reservation':
                allowed, reason = can_reserve(name, state['target']['position'], states, self.reservations,
                    self.settings['minimum_separation_m'], self.settings.get('grounded_separation_m', 2.))
                if allowed:
                    self.reservations[name] = dict(session=state['session'], request=state['request'], target=state['target']['position'])
                    self.send(name, state, 'grant')
                else:
                    self.send(name, state, 'wait', reason=reason)
        fire_checks = []
        for fire in fires:
            owner = self.assignments.get(fire['id'])
            if owner:
                age = owned_age(fire, owner)
                fire_checks.append(dict(id=fire['id'], owner=owner, age_sim_s=age,
                    due_in_sim_s=self.revisit[owner]-age, overdue=age > self.revisit[owner],
                    assignment=self.ownership_mode.get(fire['id'], 'manual')))
        survey_status = None
        if self.survey:
            coverage = {}
            for name in self.names:
                points = area_points(self.areas[name])
                for visit in states[name].get('history', []):
                    if visit['kind'] == 'search' and visit['stamp_sim_s'] >= self.survey_started:
                        for index, point in enumerate(points):
                            if math.dist(point, visit['position']) < .01:
                                self.coverage[name].add(index)
                coverage[name] = dict(visited_cells=len(self.coverage[name]), total_cells=len(points),
                    points=points, owned_fire_ids=[f['id'] for f in fire_checks if f['owner'] == name])
            survey_status = dict(self.survey, coverage=coverage)
        self.status.publish(String(data=json.dumps(dict(coordinator_id=self.identifier, frame='common_origin',
            stamp_ns=self.get_clock().now().nanoseconds, uavs=states, areas=self.areas,
            assignments=self.assignments, revisit_sim_s=self.revisit,
            reservations=self.reservations, survey=survey_status, fire_checks=fire_checks,
            ownership_mode=self.ownership_mode, notice=self.notice), allow_nan=False)))
        self.draw(states)

    def draw(self, states):
        markers = [Marker(action=Marker.DELETEALL)]
        for name in self.names:
            colour = self.colours[name]
            def marker(identifier, kind):
                item = Marker()
                item.header.frame_id = 'common_origin'
                item.header.stamp = self.get_clock().now().to_msg()
                item.ns, item.id, item.type = name+'_mission', identifier, kind
                item.pose.orientation.w = 1.
                item.color.r, item.color.g, item.color.b, item.color.a = *colour[:3], .85
                markers.append(item)
                return item
            route = states[name].get('route', [])
            if states[name].get('active') and states[name].get('mode') == 'cooperative':
                route = []
            line = marker(0, Marker.LINE_STRIP)
            line.scale.x = .06
            line.points = [Point(x=p['position'][0], y=p['position'][1], z=p['position'][2]) for p in route]
            if route and states[name].get('loop'):
                line.points.append(line.points[0])
            for index, point in enumerate(route):
                item = marker(index+1, Marker.TEXT_VIEW_FACING)
                item.scale.z = .3
                p = point['position']
                item.pose.position = Point(x=p[0], y=p[1], z=p[2]+.4)
                item.text = f'{name} WP {index+1}'
            area = marker(500, Marker.LINE_STRIP)
            area.scale.x = .08
            xmin, ymin, xmax, ymax = self.areas[name]['bounds']
            area.points = [Point(x=float(x), y=float(y), z=.06) for x, y in
                [(xmin,ymin),(xmax,ymin),(xmax,ymax),(xmin,ymax),(xmin,ymin)]]
            if self.survey:
                for index, p in enumerate(area_points(self.areas[name])):
                    item = marker(600+index, Marker.SPHERE)
                    item.scale.x = item.scale.y = item.scale.z = .12
                    item.color.a = .25 if index in self.coverage[name] else .9
                    item.pose.position = Point(x=p[0], y=p[1], z=p[2])
                for fire in self.fire_map.get('fires', []):
                    if self.assignments.get(fire['id']) == name:
                        item = marker(1000+fire['id'], Marker.TEXT_VIEW_FACING)
                        item.scale.z = .3
                        item.pose.position = Point(x=fire['centre_m'][0], y=fire['centre_m'][1], z=1.3)
                        age = owned_age(fire, name)
                        item.text = f"Fire {fire['id']} owner: {name} | own view {age:.1f} s ago"
                        if age > self.revisit[name]:
                            item.text += ' OVERDUE'
                            item.color.r, item.color.g, item.color.b = 1., 0., 0.
            state = states[name]
            if state.get('active') and state.get('target') and state.get('position_m'):
                task = marker(501, Marker.LINE_STRIP)
                task.scale.x = .12
                task.points = [Point(x=p[0], y=p[1], z=p[2]) for p in [state['position_m'], state['target']['position']]]
        self.markers.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = MissionCoordinator()
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
