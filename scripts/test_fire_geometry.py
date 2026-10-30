#!/usr/bin/env python3
"""Geometry regressions: optical convention, tilt, invalid rays, rolling memory."""
import unittest
import numpy as np
from fire_geometry import RollingCentre, ellipse_points, project_ground, rotation_matrix


class GeometryTests(unittest.TestCase):
    def test_downward_projection_and_pixel_axes(self):
        intrinsic = np.array([[100, 0, 50], [0, 100, 50], [0, 0, 1]], dtype=float)
        points = project_ground([[50, 50], [60, 50], [50, 60]], intrinsic, [],
                                np.diag([1., -1., -1.]), [2, 3, 10])
        np.testing.assert_allclose(points, [[2, 3, 0], [3, 3, 0], [2, 2, 0]], atol=1e-9)

    def test_tilted_camera_recovers_known_ground(self):
        intrinsic = np.array([[277, 0, 160], [0, 277, 120], [0, 0, 1]], dtype=float)
        rotation = rotation_matrix([0.96, 0.12, -0.09, 0.18])
        translation = np.array([7., 2., 5.])
        ground = np.array([[7., 2., 0.], [8., 1., 0.], [6.5, 2.3, 0.]])
        optical = (ground-translation) @ rotation
        pixels_h = optical @ intrinsic.T
        pixels = pixels_h[:, :2] / pixels_h[:, 2:]
        recovered = project_ground(pixels, intrinsic, [], rotation, translation)
        np.testing.assert_allclose(recovered, ground, atol=1e-8)

    def test_sky_ray_is_rejected(self):
        with self.assertRaises(ValueError):
            project_ground([[0, 0]], np.eye(3), [], np.eye(3), [0, 0, 5])

    def test_moving_target_forgets_old_measurements_and_keeps_floor(self):
        estimate = RollingCentre(window=3, floor_m=0.03)
        for centre in [[0, 0], [10, 2], [10, 2], [10, 2]]:
            position, covariance = estimate.add(centre, np.zeros((2, 2)))
        np.testing.assert_allclose(position, [10, 2])
        np.testing.assert_allclose(covariance, np.eye(2)*0.03**2)
        ellipse = ellipse_points(position, covariance)
        np.testing.assert_allclose(np.linalg.norm(ellipse-position, axis=1), 0.06)


if __name__ == '__main__':
    unittest.main()
