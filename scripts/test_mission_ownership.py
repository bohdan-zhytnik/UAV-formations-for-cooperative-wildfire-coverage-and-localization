#!/usr/bin/env python3
"""Exercise the real vehicle adapter's cancellation paths without flying.

Requires the MRS Python environment, but no ROS node, services or simulation.
The fake future deliberately completes after manual ownership was cancelled.
"""
import time
import json
import unittest
from types import SimpleNamespace
from fleet_vehicle import FleetVehicle
from mission_logic import VehicleMission
from std_msgs.msg import String


class Ownership(unittest.TestCase):
    def vehicle(self):
        vehicle = FleetVehicle.__new__(FleetVehicle)
        vehicle.mission = VehicleMission([{'position': [8., 0., 5.]}])
        vehicle.mission.start('old')
        vehicle.command_epoch = 1
        vehicle.auto_pending = vehicle.pending = object()
        vehicle.pending_action = vehicle.action_future = None
        vehicle.command_goal_common = None
        vehicle.events = []
        vehicle.publish_mission_status = lambda: None
        vehicle.request_to_common = lambda request: [8., 0., 5.]
        vehicle.publish_goal = lambda request: vehicle.events.append('published old goal')
        vehicle.report = lambda text: vehicle.events.append(text)
        vehicle.hover = object()
        return vehicle

    def test_cancel_drains_late_reference_before_hover(self):
        v = self.vehicle()
        class LateFuture:
            def result(self):
                return SimpleNamespace(success=True, message='accepted before cancellation')
        future = LateFuture()
        session, request = v.mission.session, v.mission.request
        v.reference_calls = {future: dict(kind='mission', epoch=1, session=session, request=request, sent=time.monotonic())}
        v.inflight = (future, time.monotonic())
        v.take_manual_control('hover')
        v.queue_action('hover', 'manual', v.hover)
        self.assertFalse(v.mission.active)
        self.assertIsNone(v.auto_pending)
        v.process_goal()  # Reference still outstanding: must not send hover yet.
        self.assertIsNotNone(v.pending_action)
        v.reference_result(future, None)  # Old success cannot restore ownership/markers.
        self.assertFalse(v.mission.active)
        self.assertNotIn('published old goal', v.events)
        self.assertEqual(v.command_goal_common, [8., 0., 5.])
        sent = []
        v.trigger_action = lambda *args: sent.append(args[0])
        v.process_goal()
        self.assertEqual(sent, ['hover'])
        self.assertFalse(v.mission.reference_reply(session, request, True, 0.))

    def test_reference_timeout_keeps_barrier_and_pauses(self):
        v = self.vehicle()
        future = object()
        v.reference_calls = {future: {'sent': time.monotonic()-16}}
        v.hardware = SimpleNamespace(armed=True)
        v.process_goal()
        self.assertFalse(v.mission.active)
        self.assertIn(future, v.reference_calls)
        self.assertEqual(v.pending_action[0], 'hover')

    def test_lost_coordinator_requires_explicit_resume(self):
        v = self.vehicle()
        v.hardware = SimpleNamespace(armed=True)
        v.reference_calls = {}
        v.coordinator_fresh = lambda: False
        v.ready_to_fly = lambda: True
        v.mission_watchdog()
        self.assertFalse(v.mission.active)
        self.assertEqual(v.pending_action[0], 'hover')
        v.coordinator_fresh = lambda: True
        v.mission_watchdog()
        self.assertFalse(v.mission.active)

    def test_route_edits_require_pause_and_acknowledge_the_actual_result(self):
        v = self.vehicle()
        v.uav, v.coordinator_id = 'uav1', 'coordinator'
        replies = []
        v.config_ack = SimpleNamespace(publish=lambda msg: replies.append(json.loads(msg.data)))
        def edit(op, **extra):
            v.on_mission_config(String(data=json.dumps(dict(uav=v.uav, coordinator_id=v.coordinator_id,
                request_id='configuration', op=op, **extra))))
        edit('clear_route')
        self.assertFalse(replies[-1]['success'])
        self.assertEqual(len(v.mission.route), 1)
        v.mission.pause()
        edit('clear_route')
        self.assertTrue(replies[-1]['success'])
        self.assertEqual(v.mission.route, [])
        edit('append_waypoint', waypoint={'position': [8., -1., 5.], 'fire_ids': [1]})
        edit('append_waypoint', waypoint={'position': [8., -2., 5.]})
        self.assertEqual(len(v.mission.route), 2)
        edit('set_options', loop=False, dwell_sim_s=5.)
        self.assertTrue(replies[-1]['success'])
        self.assertFalse(v.mission.loop)
        self.assertTrue(all(p['dwell_sim_s'] == 5. for p in v.mission.route))
        self.assertFalse(v.mission.active)


if __name__ == '__main__':
    unittest.main()
