#!/usr/bin/env python3
"""Regressions for detection, association, source weighting and scene generation."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from fire_scene import generate_world
from multi_fire import SharedFireMap, hot_regions, polygon_area


def observation(x=8., y=0., width=1., height=1., temperature=480., partial=False):
    return {'region_id': 1, 'partial': partial, 'centre_m': [x, y],
            'boundary_m': [[x-width/2, y-height/2, 0.], [x+width/2, y-height/2, 0.],
                           [x+width/2, y+height/2, 0.], [x-width/2, y+height/2, 0.]],
            'pixel_covariance_xy_m2': [[.0001, 0.], [0., .0001]],
            'mean_kelvin': temperature, 'max_kelvin': temperature+1.}


class DetectionTests(unittest.TestCase):
    def detect(self, raw):
        return hot_regions(raw, [[100, 0, 80], [0, 100, 60], [0, 0, 1]], [],
                           np.diag([1., -1., -1.]), [0, 0, 5], minimum_hot_pixels=20)

    def test_all_regions_and_temperatures(self):
        raw = np.full((120, 160), 14658, dtype=np.uint16)
        raw[30:50, 25:45] = 24000
        raw[55:85, 100:130] = 28000
        regions = self.detect(raw)
        self.assertEqual(len(regions), 2)
        self.assertTrue(all(not region['partial'] for region in regions))
        self.assertEqual(sorted(region['mean_kelvin'] for region in regions), [480., 560.])
        np.testing.assert_allclose(regions[0]['centre_m'], [-2.275, 1.025], atol=1e-6)
        self.assertAlmostEqual(regions[0]['area_m2'], .95*.95)
        self.assertAlmostEqual(regions[1]['area_m2'], 1.45*1.45)

    def test_partial_and_small_noise(self):
        raw = np.full((120, 160), 14658, dtype=np.uint16)
        raw[0:12, 10:30] = 32000
        raw[60, 60] = 32000
        regions = self.detect(raw)
        self.assertEqual(len(regions), 1)
        self.assertTrue(regions[0]['partial'])
        self.assertEqual(regions[0]['mean_kelvin'], 640.)

    def test_no_hot_regions_and_invalid_calibration(self):
        self.assertEqual(self.detect(np.zeros((120, 160), dtype=np.uint16)), [])
        with self.assertRaises(ValueError):
            hot_regions(np.ones((20, 20)), np.zeros((3, 3)), [], np.eye(3), [0, 0, 5])


class MapTests(unittest.TestCase):
    def test_first_complete_observer_is_stable(self):
        mapping=SharedFireMap()
        mapping.update('uav2',1.,[observation(partial=True)])
        self.assertEqual(mapping.snapshot(1.),[])
        mapping.update('uav1',2.,[observation()])
        mapping.update('uav2',3.,[observation()])
        fire=mapping.snapshot(3.)[0]
        self.assertEqual(fire['first_observer'],'uav1')
        self.assertEqual(fire['first_observed_sim_s'],2.)

    def test_two_drones_share_ids_regardless_of_region_order(self):
        mapping = SharedFireMap()
        first = mapping.update('uav1', 1., [observation(y=-2.5), observation(width=2., height=2., temperature=560.)])
        second = mapping.update('uav2', 2., [observation(x=8.1, width=2., height=2., temperature=560.),
                                           observation(x=8.1, y=-2.5)])
        self.assertEqual(first, {0: 1, 1: 2})
        self.assertEqual(second, {0: 2, 1: 1})
        fires = mapping.snapshot(2.)
        self.assertEqual(len(fires), 2)
        self.assertTrue(all(fire['contributing_uavs'] == ['uav1', 'uav2'] for fire in fires))
        np.testing.assert_allclose(fires[0]['centre_m'], [8.05, -2.5])

    def test_same_image_regions_never_merge(self):
        mapping = SharedFireMap()
        mapping.update('uav1', 1., [observation(x=0.), observation(x=.6)])
        assignments = mapping.update('uav2', 2., [observation(x=.65), observation(x=.05)])
        self.assertEqual(assignments, {1: 1, 0: 2})
        self.assertEqual(len(mapping.tracks), 2)

    def test_partial_does_not_create_or_move_tracks(self):
        mapping = SharedFireMap()
        mapping.update('uav1', 1., [observation(partial=True)])
        self.assertEqual(mapping.snapshot(1.), [])
        mapping.update('uav1', 2., [observation()])
        mapping.update('uav1', 3., [observation(x=8.5, partial=True)])
        fire = mapping.snapshot(3.)[0]
        self.assertEqual(fire['observations'], 1)
        self.assertEqual(fire['centre_m'], [8., 0.])
        self.assertFalse(fire['visible'])

    def test_replayed_and_out_of_order_frames_are_not_evidence(self):
        mapping = SharedFireMap()
        mapping.update('uav1', 10., [observation()])
        self.assertIsNone(mapping.update('uav1', 10., [observation(x=8.1)]))
        self.assertIsNone(mapping.update('uav1', 9., [observation(x=8.2)]))
        self.assertEqual(mapping.snapshot(10.)[0]['observations'], 1)

    def test_out_of_view_retention_and_stable_reacquisition(self):
        mapping = SharedFireMap()
        mapping.update('uav1', 1., [observation()])
        mapping.update('uav1', 2., [])
        retained = mapping.snapshot(30.)[0]
        self.assertEqual(retained['id'], 1)
        self.assertFalse(retained['visible'])
        self.assertEqual(retained['last_seen_age_s'], 29.)
        self.assertEqual(mapping.update('uav2', 31., [observation(x=8.15)]), {0: 1})

    def test_bounded_history_and_equal_source_weight(self):
        mapping = SharedFireMap(window_per_uav=5)
        for stamp in range(1, 31):
            mapping.update('uav1', float(stamp), [observation(x=0.)])
        mapping.update('uav2', 31., [observation(x=.6)])
        fire = mapping.snapshot(31.)[0]
        self.assertEqual(fire['window_samples'], 6)
        self.assertAlmostEqual(fire['centre_m'][0], .3)
        self.assertGreaterEqual(np.linalg.eigvalsh(fire['covariance_xy_m2']).min(), .03**2)
        self.assertAlmostEqual(polygon_area(fire['boundary_m']), fire['area_m2'])
        json.dumps(fire, allow_nan=False)

    def test_invalid_batch_is_atomic_and_source_checked(self):
        mapping = SharedFireMap()
        invalid = observation()
        invalid['centre_m'][0] = float('nan')
        with self.assertRaises(ValueError):
            mapping.update('uav1', 1., [observation(), invalid])
        self.assertEqual(len(mapping.tracks), 0)
        with self.assertRaises(ValueError):
            mapping.update('unknown', 2., [observation()])

    def test_distinct_sizes_and_memory_limit(self):
        mapping = SharedFireMap(max_tracks=2)
        mapping.update('uav1', 1., [observation()])
        mapping.update('uav1', 2., [observation(x=8.2, width=3., height=2.)])
        mapping.update('uav1', 3., [observation(x=15.)])
        self.assertEqual(len(mapping.tracks), 2)
        self.assertEqual(mapping.dropped_new_tracks, 1)
        mapping.reset()
        self.assertEqual(mapping.snapshot(4.), [])


class SceneTests(unittest.TestCase):
    def test_configured_world_preserves_physics_and_has_three_temperatures(self):
        root = Path(__file__).resolve().parents[1]
        fires = yaml.safe_load((root/'config/fleet/fires.yaml').read_text())['fires']
        fires = copy.deepcopy(fires)
        # Exercise edited configuration, not only the shipped example values.
        fires[0]['temperature_kelvin'] = 505.
        fires[0]['position'] = [6., -3.]
        fires[0]['size'] = [1.2, .8]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'world.sdf'
            generate_world(root/'worlds/fire_fleet.sdf', fires, output)
            world = ET.parse(output).getroot().find('world')
            template = ET.parse(root/'worlds/fire_fleet.sdf').getroot().find('world')
            self.assertEqual(world.findtext('physics/real_time_factor'), template.findtext('physics/real_time_factor'))
            self.assertEqual(world.findtext('physics/max_step_size'), template.findtext('physics/max_step_size'))
            models = [m for m in world.findall('model') if m.attrib['name'].startswith('configured_fire_')]
            self.assertEqual(len(models), len(fires))
            for model, fire in zip(models, fires):
                self.assertEqual(float(model.findtext('.//plugin/temperature')), fire['temperature_kelvin'])
                self.assertEqual([float(v) for v in model.findtext('pose').split()[:2]], fire['position'])
                self.assertEqual([float(v) for v in model.findtext('.//box/size').split()[:2]], fire['size'])
            self.assertTrue(all(m.findtext('static') == 'true' for m in models))


if __name__ == '__main__':
    unittest.main()
