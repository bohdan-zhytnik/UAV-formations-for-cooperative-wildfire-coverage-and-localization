"""Shared rectangular surveys, first-observer ownership and deadline-aware tasks.

Only operator bounds and measured shared-map records enter these functions.
"""
import math
from mission_logic import area_points, contains, waypoint


def make_survey(value, names=('uav1', 'uav2'), separation=3.5):
    if len(names) != 2:
        raise ValueError('This survey divides work between exactly two drones')
    bounds = [float(v) for v in value['bounds']]
    if len(bounds) != 4 or not all(math.isfinite(v) for v in bounds):
        raise ValueError('Survey needs finite xmin, ymin, xmax, ymax')
    xmin, ymin, xmax, ymax = bounds
    if not -20 <= xmin < xmax <= 20 or not -20 <= ymin < ymax <= 20:
        raise ValueError('Survey bounds must be ordered and within +/-20 m')
    altitude = float(value.get('altitude_m', 5.))
    spacing = float(value.get('spacing_m', 2.))
    revisit = float(value.get('revisit_sim_s', 20.))
    dwell = float(value.get('dwell_sim_s', 1.))
    speed = float(value.get('planning_speed_mps', 1.))
    margin = float(value.get('return_margin_sim_s', 2.))
    if not all(math.isfinite(v) for v in [altitude, spacing, revisit, dwell, speed, margin]):
        raise ValueError('Survey settings must be finite')
    if not 1 <= altitude <= 6 or not .5 <= spacing <= 3 or not 3 <= revisit <= 600:
        raise ValueError('Survey altitude 1..6 m, spacing 0.5..3 m, revisit 3..600 sim s')
    if not 1 <= dwell <= 10 or not .2 <= speed <= 2 or not 0 <= margin <= 10:
        raise ValueError('Invalid survey dwell, planning speed or return margin')
    # Camera calibration in this demo: 60-degree HFOV, 4:3, flat ground,
    # downward pointing at heading zero. Use the smaller ground half-width
    # with mounting/edge margin, and keep centres apart across the split.
    camera_half = (altitude-.1)*math.tan(math.pi/6)*.75*.95
    inset = separation/2+.15
    if camera_half < inset or spacing > 2*camera_half:
        raise ValueError('Camera footprint cannot cover the split at this altitude/separation; increase altitude or reduce separation')
    axis = value.get('split_axis', 'auto')
    if axis == 'auto':
        axis = 'x' if xmax-xmin > ymax-ymin else 'y'
    if axis not in ('x', 'y'):
        raise ValueError('Split axis must be auto, x or y')
    low, high = list(bounds), list(bounds)
    index = 0 if axis == 'x' else 1
    middle = (bounds[index]+bounds[index+2])/2
    low[index+2], high[index] = middle, middle
    areas = {name: dict(bounds=b, spacing_m=spacing, altitude_m=altitude, inset_m=inset)
             for name, b in zip(names, [low, high])}
    for area in areas.values():
        area_points(area)
    return dict(bounds=bounds, split_axis=axis, altitude_m=altitude, spacing_m=spacing,
                revisit_sim_s=revisit, dwell_sim_s=dwell, planning_speed_mps=speed,
                return_margin_sim_s=margin, inset_m=inset, areas=areas)


def claim_discoveries(fires, assignments, survey):
    """First eligible complete observer owns a track; existing ownership stays.

    A transit sighting or old map must not assign an unobservable fire to a
    drone on the other side of a new partition. Leave it unclaimed until a
    camera observer capable of inspecting it from its own half is available.
    """
    claims = {}
    for fire in fires:
        if fire['id'] in assignments or not contains(survey, fire['centre_m']):
            continue
        sources = list(dict.fromkeys([fire.get('first_observer'), *fire.get('last_seen_by_uav_s', {})]))
        for source in sources:
            if source not in survey['areas']:
                continue
            area = survey['areas'][source]
            target = fire_viewpoint(fire, area)
            half = (area['altitude_m']-.1)*math.tan(math.pi/6)*.75*.98
            outline = fire.get('boundary_m', [[*fire['centre_m'], 0.]])
            if all(max(abs(p[0]-target[0]), abs(p[1]-target[1])) <= half for p in outline):
                claims[fire['id']] = source
                break
    return claims


def owned_age(fire, uav):
    return fire.get('last_seen_by_uav_s', {}).get(uav, 1e6)


def search_point(state, area):
    points = area_points(area)
    # Reverse the sweep at its ends, rather than jumping diagonally back.
    cycle = max(1, 2*(len(points)-1))
    index = state.get('survey_index', 0) % cycle
    index = index if index < len(points) else cycle-index
    return points[index], index, len(points)


def fire_viewpoint(fire, area):
    inset = area.get('inset_m', 0.)
    b = area['bounds']
    return [max(b[0]+inset, min(b[2]-inset, fire['centre_m'][0])),
            max(b[1]+inset, min(b[3]-inset, fire['centre_m'][1])), area['altitude_m']]


def inspection_view(focus, owned, area):
    """Combine nearby owned fires only if their measured outlines fit one view."""
    # Use the smaller image extent on BOTH world axes: the optical mounting
    # swaps the long/short image directions relative to the vehicle frame.
    half_x = (area['altitude_m']-.1)*math.tan(math.pi/6)*.75*.98
    half_y = half_x
    b, inset = area['bounds'], area.get('inset_m', 0.)
    group = [focus]
    def fit(fires):
        outlines = [p for fire in fires for p in fire.get('boundary_m', [[*fire['centre_m'], 0.]])]
        lo_x, hi_x = min(p[0] for p in outlines), max(p[0] for p in outlines)
        lo_y, hi_y = min(p[1] for p in outlines), max(p[1] for p in outlines)
        xmin, xmax = max(b[0]+inset, hi_x-half_x), min(b[2]-inset, lo_x+half_x)
        ymin, ymax = max(b[1]+inset, hi_y-half_y), min(b[3]-inset, lo_y+half_y)
        if xmin > xmax or ymin > ymax:
            return None
        return [max(xmin, min(xmax, (lo_x+hi_x)/2)),
                max(ymin, min(ymax, (lo_y+hi_y)/2)), area['altitude_m']]
    target = fit(group) or fire_viewpoint(focus, area)
    for candidate in sorted(owned, key=lambda fire: math.dist(fire['centre_m'], focus['centre_m'])):
        if candidate['id'] == focus['id']:
            continue
        combined = fit(group+[candidate])
        if combined is not None:
            group.append(candidate)
            target = combined
    return target, sorted(fire['id'] for fire in group)


def choose_survey_task(uav, state, fires, assignments, claimed, survey):
    area = survey['areas'][uav]
    cell, index, _ = search_point(state, area)
    current = state.get('position_m') or cell
    speed, dwell = survey['planning_speed_mps'], survey['dwell_sim_s']
    search_time = math.dist(current, cell)/speed+1.+dwell
    owned = [fire for fire in fires if assignments.get(fire['id']) == uav]
    recent = state.get('history', [])[-max(1, len(owned)):]
    coverage_turn = bool(owned and len(recent) >= len(owned) and all(v['kind'] == 'fire' for v in recent))
    due = []
    for fire in fires:
        if assignments.get(fire['id']) != uav or fire['id'] in claimed:
            continue
        target = fire_viewpoint(fire, area)
        age = owned_age(fire, uav)
        # In an infeasible large region, repeatedly "revisiting" the same
        # just-seen fire would freeze coverage forever. Bound inspection bursts
        # to one round of owned fires, and expose missed deadlines in status.
        if coverage_turn or age < min(3., survey['revisit_sim_s']*.2):
            continue
        # If another coverage stop would leave too little time to return, go
        # now. Own camera timestamps matter even when the other drone sees it.
        return_time = math.dist(cell, target)/speed+1.
        if age+search_time+return_time+survey['return_margin_sim_s'] >= survey['revisit_sim_s']:
            due.append((-(age+math.dist(current, target)/speed), fire['id'], target))
    if due:
        _, identifier, target = min(due)
        focus = next(fire for fire in owned if fire['id'] == identifier)
        target, identifiers = inspection_view(focus, [f for f in owned if f['id'] not in claimed], area)
        return dict(waypoint(dict(position=target, fire_ids=identifiers, dwell_sim_s=dwell)), kind='fire')
    return dict(waypoint(dict(position=cell, dwell_sim_s=dwell)), kind='search')
