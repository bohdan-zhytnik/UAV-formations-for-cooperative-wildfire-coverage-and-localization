#!/usr/bin/env python3
"""Mission behavior checks without ROS or Gazebo."""
import unittest
from mission_logic import VehicleMission, waypoint, area_points, choose_task, can_reserve, segment_distance


class Missions(unittest.TestCase):
    def engine(self, loop=True, **kwargs):
        return VehicleMission([{'position': [8., -1., 5.]}, {'position': [8., -2., 5.]}], loop=loop, **kwargs)

    def depart(self, mission):
        self.assertTrue(mission.grant(mission.session, mission.request))
        self.assertTrue(mission.reference_reply(mission.session, mission.request, True, 0.))

    def visit(self, mission, now=0.):
        p = mission.target['position']
        mission.tick(now, p, now, {})
        mission.tick(now+1., p, now+1., {})
        return mission.tick(now+4., p, now+4., {})

    def test_route_order_and_loop(self):
        m = self.engine()
        m.start('a')
        self.depart(m)
        self.assertEqual(self.visit(m), 'visited')
        self.assertEqual(m.route_index, 1)
        self.depart(m)
        self.visit(m, 5.)
        self.assertEqual((m.route_index, m.cycles, m.completed_visits), (0, 1, 2))
        self.assertTrue(m.active)

    def test_once_holds_final_point(self):
        m = self.engine(loop=False)
        m.start('a')
        for i in range(2):
            self.depart(m)
            self.visit(m, i*5.)
        self.assertFalse(m.active)
        self.assertEqual(m.phase, 'completed')

    def test_waits_for_arrival_and_new_camera(self):
        m = self.engine()
        m.start('a')
        self.depart(m)
        m.tick(1., [0., 0., 5.], 1., {})
        self.assertEqual(m.phase, 'travelling')
        p = m.target['position']
        m.tick(2., p, 1., {})
        m.tick(3., p, 1., {})
        m.tick(6., p, 1., {})
        self.assertEqual(m.completed_visits, 0)
        m.tick(6.2, p, 6.2, {})
        self.assertEqual(m.completed_visits, 1)

    def test_fire_inspection_requires_own_recent_fire(self):
        m = self.engine()
        m.start('a', 'cooperative')
        target = dict(waypoint({'position': [8., 0., 5.], 'fire_ids': [2]}), kind='fire')
        self.assertTrue(m.accept_task(m.session, m.request, target))
        self.depart(m)
        p = target['position']
        for t in [0., 1., 4.]:
            m.tick(t, p, t, {2: .5})
        self.assertEqual(m.completed_visits, 0)
        m.tick(5., p, 5., {2: 5.})
        self.assertEqual(m.completed_visits, 1)
        self.assertEqual(m.history[-1]['expected_fire_ids'], [2])

    def test_missing_camera_pauses(self):
        m = self.engine()
        m.start('a'); self.depart(m)
        p = m.target['position']
        m.tick(0., p, None, {}); m.tick(1., p, None, {})
        self.assertEqual(m.tick(17., p, None, {}), 'paused')
        self.assertFalse(m.active)

    def test_manual_cancel_invalidates_late_grant_and_reply(self):
        for before_reply in (True, False):
            m = self.engine()
            m.start('old')
            session, request = m.session, m.request
            if before_reply:
                m.grant(session, request)
            m.pause('manual hover')
            self.assertFalse(m.grant(session, request))
            self.assertFalse(m.reference_reply(session, request, True, 2.))
            self.assertFalse(m.active)
            m.start('new', resume=True)
            self.assertFalse(m.grant(session, request))
            self.assertEqual(m.session, 'new')

    def test_resume_preserves_next_point_start_restarts(self):
        m = self.engine(); m.start('a'); self.depart(m); self.visit(m)
        m.pause(); m.start('b', resume=True)
        self.assertEqual(m.route_index, 1)
        m.pause(); m.start('c')
        self.assertEqual(m.route_index, 0)

    def test_travel_timeout_and_rejected_reference(self):
        m = self.engine(travel_timeout=2.); m.start('a'); self.depart(m)
        self.assertEqual(m.tick(3., [0., 0., 5.], 3., {}), 'paused')
        m.start('b'); m.grant(m.session, m.request)
        m.reference_reply(m.session, m.request, False, 3., 'denied')
        self.assertFalse(m.active)

    def test_long_dwell_does_not_count_as_travel(self):
        m = VehicleMission([{'position': [0., 0., 5.], 'dwell_sim_s': 60.}], travel_timeout=10.)
        m.start('a'); self.depart(m)
        m.tick(0., [0., 0., 5.], 0., {}); m.tick(1., [0., 0., 5.], 1., {})
        self.assertEqual(m.tick(61., [0., 0., 5.], 61., {}), 'visited')

    def test_input_limits_and_empty_route(self):
        for value in ([21, 0, 5], [0, 0, .5], [0, 0, float('nan')], [0, 0]):
            with self.assertRaises(ValueError):
                waypoint({'position': value})
        with self.assertRaises(ValueError):
            VehicleMission().start('a')
        with self.assertRaises(ValueError):
            area_points({'bounds': [-20, -20, 20, 20], 'spacing_m': .5})

    def test_boustrophedon_coverage(self):
        points = area_points({'bounds': [0, 0, 2, 1], 'spacing_m': 1.})
        self.assertEqual(points, [[0., 0., 5.], [1., 0., 5.], [2., 0., 5.],
                                  [2., 1., 5.], [1., 1., 5.], [0., 1., 5.]])

    def test_stale_fire_assignment_and_exclusion(self):
        area = {'bounds': [0, -3, 10, 3], 'altitude_m': 5.}
        fires = [dict(id=1, centre_m=[8., 0.], last_seen_age_s=30.),
                 dict(id=2, centre_m=[8., 2.], last_seen_age_s=40.)]
        state = {'position_m': [0., 0., 5.]}
        self.assertEqual(choose_task('uav1', state, fires, area, {}, set())['fire_ids'], [2])
        self.assertEqual(choose_task('uav1', state, fires, area, {2: 'uav2'}, set())['fire_ids'], [1])
        self.assertEqual(choose_task('uav1', state, fires, area, {}, {1, 2})['kind'], 'search')
        self.assertEqual(choose_task('uav1', state, fires, area, {}, set(), revisit=60.)['kind'], 'search')

    def test_crossing_routes_blocked_pending_manual_goal_considered(self):
        states = {'uav1': {'position_m': [0., -8., 5.]}, 'uav2': {'position_m': [0., 8., 5.]}}
        self.assertTrue(can_reserve('uav1', [8., -8., 5.], states, {})[0])
        states['uav2']['pending_goal_common'] = [8., -8., 5.]
        self.assertFalse(can_reserve('uav1', [8., -8., 5.], states, {})[0])
        states['uav2']['pending_goal_common'] = None
        self.assertFalse(can_reserve('uav1', [8., -8., 5.], states, {'uav2': {'target': [8., -8., 5.]}})[0])
        states['uav2']['stale'] = True
        self.assertFalse(can_reserve('uav1', [8., -8., 5.], states, {})[0])
        self.assertEqual(segment_distance([0, 0], [2, 2], [0, 2], [2, 0]), 0.)

    def test_parked_drone_does_not_block_a_clear_preset_route(self):
        states = {'uav2': {'position_m': [0., 6., 1.6]},
                  'uav1': {'position_m': [3.3, 1.07, .15], 'armed': False, 'hardware_fresh': True}}
        self.assertTrue(can_reserve('uav2', [12., 6., 5.], states, {})[0])
        self.assertTrue(can_reserve('uav2', [11., 5., 5.], states, {})[0])
        states['uav1']['armed'] = True
        self.assertFalse(can_reserve('uav2', [12., 6., 5.], states, {})[0])

    def test_parked_vehicle_remains_an_obstacle_and_unknown_states_block(self):
        states = {'uav2': {'position_m': [-3., 0., 1.]},
                  'uav1': {'position_m': [0., 0., .15], 'armed': False, 'hardware_fresh': True}}
        self.assertFalse(can_reserve('uav2', [3., 0., 1.], states, {})[0])
        states['uav2']['position_m'] = [-3., 0., 5.]
        self.assertTrue(can_reserve('uav2', [3., 0., 5.], states, {})[0])
        states['uav1']['hardware_fresh'] = False
        self.assertFalse(can_reserve('uav2', [3., 0., 5.], states, {})[0])
        states['uav1']['hardware_fresh'] = True
        states['uav1']['reference_busy'] = True
        self.assertFalse(can_reserve('uav2', [3., 0., 5.], states, {})[0])
        states['uav1']['reference_busy'] = False
        states['uav1']['stale'] = True
        self.assertFalse(can_reserve('uav2', [3., 0., 5.], states, {})[0])


if __name__ == '__main__':
    unittest.main()
