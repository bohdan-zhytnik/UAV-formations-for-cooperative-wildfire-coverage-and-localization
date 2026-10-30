"""Small mission state machines and conservative reservations, independent of ROS."""
from collections import deque
import math


def waypoint(value, default_dwell=3.):
    point = [float(v) for v in value['position']]
    dwell = float(value.get('dwell_sim_s', default_dwell))
    heading = float(value.get('heading', 0.))
    if len(point) != 3 or not all(math.isfinite(v) for v in [*point, dwell, heading]):
        raise ValueError('A waypoint needs finite x, y, altitude, heading and dwell time')
    if max(abs(point[0]), abs(point[1])) > 20 or not 1 <= point[2] <= 6:
        raise ValueError('Waypoint outside x/y +/-20 m or altitude 1..6 m')
    if not 1 <= dwell <= 60:
        raise ValueError('Inspection dwell must be 1..60 simulated seconds')
    ids = value.get('fire_ids', [])
    if not isinstance(ids, list) or any(type(v) is not int or v < 1 for v in ids):
        raise ValueError('fire_ids must be a list of positive integers')
    return {'position': point, 'heading': heading, 'dwell_sim_s': dwell, 'fire_ids': sorted(set(ids))}


def area_points(area):
    bounds = [float(v) for v in area['bounds']]
    spacing, altitude = float(area.get('spacing_m', 1.)), float(area.get('altitude_m', 5.))
    if len(bounds) != 4 or not all(math.isfinite(v) for v in [*bounds, spacing, altitude]):
        raise ValueError('Area requires finite [xmin, ymin, xmax, ymax], spacing and altitude')
    xmin, ymin, xmax, ymax = bounds
    if not -20 <= xmin < xmax <= 20 or not -20 <= ymin < ymax <= 20 or not .5 <= spacing <= 10:
        raise ValueError('Invalid search bounds or spacing (0.5..10 m)')
    if not 1 <= altitude <= 6:
        raise ValueError('Search altitude must be 1..6 m')
    inset = float(area.get('inset_m', 0.))
    if not math.isfinite(inset) or inset < 0 or 2*inset > min(xmax-xmin, ymax-ymin)+1e-9:
        raise ValueError('Search inset does not fit inside the area')
    xmin, ymin, xmax, ymax = xmin+inset, ymin+inset, xmax-inset, ymax-inset
    nx, ny = math.ceil(max(0., xmax-xmin)/spacing), math.ceil(max(0., ymax-ymin)/spacing)
    if (nx+1)*(ny+1) > 200:
        raise ValueError('Search grid exceeds 200 points; increase spacing or reduce the area')
    points = []
    for row in range(ny+1):
        columns = range(nx+1) if row % 2 == 0 else range(nx, -1, -1)
        for column in columns:
            points.append([xmin+(xmax-xmin)*column/nx if nx else xmin,
                           ymin+(ymax-ymin)*row/ny if ny else ymin, altitude])
    return points


def contains(area, point):
    xmin, ymin, xmax, ymax = area['bounds']
    return xmin <= point[0] <= xmax and ymin <= point[1] <= ymax


def segment_distance(a, b, c, d):
    """Minimum horizontal distance between two line segments, including points."""
    a, b, c, d = [tuple(p[:2]) for p in (a, b, c, d)]
    def cross(p, q, r):
        return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    def point_distance(p, q, r):
        vx, vy = r[0]-q[0], r[1]-q[1]
        length = vx*vx+vy*vy
        t = max(0., min(1., ((p[0]-q[0])*vx+(p[1]-q[1])*vy)/length)) if length else 0.
        return math.hypot(p[0]-q[0]-t*vx, p[1]-q[1]-t*vy)
    ab_c, ab_d, cd_a, cd_b = cross(a, b, c), cross(a, b, d), cross(c, d, a), cross(c, d, b)
    if ab_c*ab_d < 0 and cd_a*cd_b < 0:
        return 0.
    return min(point_distance(a, c, d), point_distance(b, c, d),
               point_distance(c, a, b), point_distance(d, a, b))


def point_segment_distance(point, start, end):
    """Euclidean point-to-segment clearance, including the vertical dimension."""
    delta = [b-a for a, b in zip(start, end)]
    length = sum(v*v for v in delta)
    t = max(0., min(1., sum((p-a)*v for p, a, v in zip(point, start, delta))/length)) if length else 0.
    return math.sqrt(sum((p-a-t*v)**2 for p, a, v in zip(point, start, delta)))


def can_reserve(uav, target, states, reservations, separation=6.5, grounded_separation=2.):
    own = states.get(uav, {}).get('position_m')
    if own is None:
        return False, 'Waiting for own position'
    for other, state in states.items():
        if other == uav:
            continue
        position = state.get('position_m')
        if position is None or state.get('stale', False):
            return False, 'Waiting for fresh '+other+' position'
        # A confirmed, disarmed vehicle on this flat ground is a stationary
        # obstacle, not an airborne corridor. Unknown/armed states retain the
        # full traffic margin. Never grant a line through a parked vehicle.
        grounded = (state.get('armed') is False and state.get('hardware_fresh') is True
                    and abs(position[2]) <= .5 and not state.get('reference_busy')
                    and other not in reservations)
        if grounded:
            if point_segment_distance(position, own, target) < grounded_separation:
                return False, f'Waiting for {grounded_separation:g} m clearance around landed '+other
            continue
        endpoints = [position]
        if state.get('command_goal_common') is not None:
            endpoints.append(state['command_goal_common'])
        if state.get('pending_goal_common') is not None:
            endpoints.append(state['pending_goal_common'])
        if other in reservations:
            endpoints.append(reservations[other]['target'])
        if any(segment_distance(own, target, position, end) < separation for end in endpoints):
            return False, 'Waiting for separation from '+other
    return True, 'Reserved'


def choose_task(uav, state, fires, area, assignments, claimed, dwell=3., revisit=20.):
    """Choose stale mapped fires, otherwise the next search cell. No scene truth."""
    candidates = []
    for fire in fires:
        identifier = fire['id']
        owner = assignments.get(identifier)
        if identifier in claimed or (owner is not None and owner != uav):
            continue
        if owner != uav and not contains(area, fire['centre_m']):
            continue
        if fire['last_seen_age_s'] < revisit:
            continue
        position = state['position_m']
        distance = math.dist(position[:2], fire['centre_m']) if position else 0.
        candidates.append((-fire['last_seen_age_s'], distance, identifier, fire))
    if candidates:
        fire = min(candidates, key=lambda value: value[:3])[3]
        return dict(waypoint({'position': [*fire['centre_m'], area.get('altitude_m', 5.)],
                              'dwell_sim_s': dwell, 'fire_ids': [fire['id']]}), kind='fire')
    points = area_points(area)
    index = state.get('survey_index', 0) % len(points)
    return dict(waypoint({'position': points[index], 'dwell_sim_s': dwell}), kind='search')


class VehicleMission:
    """Local execution; only an explicit operator start/resume creates a session."""
    def __init__(self, route=(), loop=True, dwell=3., arrival_tolerance=.4, arrival_hold=1.,
                 travel_timeout=90., observation_timeout=12.):
        self.route = [waypoint(value, dwell) for value in route]
        self.loop, self.dwell = loop, dwell
        self.tolerance, self.hold = arrival_tolerance, arrival_hold
        self.travel_timeout, self.observation_timeout = travel_timeout, observation_timeout
        self.mode, self.phase, self.reason = 'route', 'idle', 'No mission started'
        self.active, self.session, self.target = False, None, None
        self.route_index = self.survey_index = self.request_seq = 0
        self.cycles = self.completed_visits = 0
        self.history = deque(maxlen=50)
        self.near_since = self.arrived_at = self.travel_started = None

    @property
    def request(self):
        return f'{self.session}:{self.request_seq}' if self.session else None

    def start(self, session, mode='route', resume=False):
        if self.active:
            raise ValueError(f'Mission already active: {self.phase}: {self.reason}. Pause before starting another')
        if mode not in ('route', 'cooperative'):
            raise ValueError('Mission mode must be route or cooperative')
        if mode == 'route' and not self.route:
            raise ValueError('The selected route is empty; add waypoints first')
        if not resume:
            self.route_index, self.survey_index = 0, 0
        self.mode, self.session, self.active = mode, session, True
        self.request_seq += 1
        self._next()

    def _next(self):
        self.target = None
        self.near_since = self.arrived_at = self.travel_started = None
        self.reason = 'Waiting for a task' if self.mode == 'cooperative' else 'Waiting for path reservation'
        if self.mode == 'cooperative':
            self.phase = 'requesting_task'
        else:
            self.target = dict(self.route[self.route_index], kind='route')
            self.phase = 'waiting_reservation'

    def pause(self, reason='Paused by operator'):
        self.active, self.session, self.target = False, None, None
        self.phase, self.reason = 'paused', reason
        self.near_since = self.arrived_at = None

    def accept_task(self, session, request, target):
        if not self.active or self.phase != 'requesting_task' or (session, request) != (self.session, self.request):
            return False
        if target.get('kind') not in ('fire', 'search'):
            raise ValueError('Invalid cooperative task kind')
        checked = waypoint(target, self.dwell)
        if target['kind'] == 'fire' and not checked['fire_ids']:
            raise ValueError('Fire task must name a mapped fire')
        self.target = dict(checked, kind=target['kind'])
        self.phase, self.reason = 'waiting_reservation', 'Waiting for path reservation'
        return True

    def grant(self, session, request):
        if not self.active or self.phase != 'waiting_reservation' or (session, request) != (self.session, self.request):
            return False
        self.phase, self.reason = 'waiting_reference', 'Waiting for MRS to accept the reference'
        return True

    def reference_reply(self, session, request, success, now, reason=''):
        if not self.active or self.phase != 'waiting_reference' or (session, request) != (self.session, self.request):
            return False
        if not success:
            self.pause('MRS rejected mission goal: '+reason)
            return False
        self.phase, self.reason, self.travel_started = 'travelling', 'Flying to inspection point', now
        return True

    def tick(self, now, position, image_stamp, fire_seen):
        if not self.active or self.phase not in ('travelling', 'dwelling'):
            return None
        if self.phase == 'travelling' and now-self.travel_started > self.travel_timeout:
            self.pause('Inspection travel timed out in simulation time')
            return 'paused'
        distance = math.dist(position, self.target['position'])
        if distance > self.tolerance:
            self.near_since = self.arrived_at = None
            self.phase, self.reason = 'travelling', 'Flying to inspection point'
            return None
        if self.near_since is None:
            self.near_since = now
        if self.arrived_at is None:
            if now-self.near_since < self.hold:
                return None
            self.arrived_at, self.phase, self.reason = now, 'dwelling', 'Inspecting with a fresh camera frame'
        elapsed = now-self.arrived_at
        fresh = image_stamp is not None and image_stamp >= self.arrived_at and now-image_stamp <= 2.
        observed = [identifier for identifier, stamp in fire_seen.items() if stamp >= self.arrived_at]
        expected = set(self.target['fire_ids'])
        if elapsed >= self.target['dwell_sim_s'] and fresh and expected.issubset(observed):
            target = self.target
            self.completed_visits += 1
            self.history.append({'stamp_sim_s': now, 'kind': target['kind'], 'position': target['position'],
                                 'expected_fire_ids': target['fire_ids'], 'observed_fire_ids': sorted(observed)})
            if target['kind'] == 'search':
                self.survey_index += 1
            if self.mode == 'route':
                self.route_index += 1
                if self.route_index >= len(self.route):
                    self.cycles += 1
                    self.route_index = 0
                    if not self.loop:
                        self.active, self.session, self.target = False, None, None
                        self.phase, self.reason = 'completed', 'Route complete; holding the final position'
                        return 'completed'
            self.request_seq += 1
            self._next()
            return 'visited'
        if elapsed > self.target['dwell_sim_s']+self.observation_timeout:
            self.pause('Assigned fire not fully observed' if expected else 'No fresh camera frame at inspection point')
            return 'paused'
        return None

    def snapshot(self):
        return {'active': self.active, 'session': self.session, 'mode': self.mode, 'phase': self.phase,
                'reason': self.reason, 'request': self.request, 'target': self.target,
                'route': self.route, 'route_index': self.route_index, 'survey_index': self.survey_index,
                'loop': self.loop, 'dwell_sim_s': self.dwell, 'cycles': self.cycles,
                'completed_visits': self.completed_visits, 'history': list(self.history)}
