"""Flat-ground geometry and a bounded rolling estimate; no ROS dependencies."""
from collections import deque

import cv2
import numpy as np


def rotation_matrix(quaternion):
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid quaternion')
    x, y, z, w = q / norm
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])


def project_ground(pixels, intrinsic, distortion, rotation, translation,
                   ground_z=0.0, max_range=60.0):
    """Intersect optical rays with a known horizontal plane. Reject sky rays."""
    pixels = np.asarray(pixels, dtype=float).reshape(-1, 1, 2)
    normalized = cv2.undistortPoints(pixels, np.asarray(intrinsic, dtype=float),
                                   np.asarray(distortion, dtype=float) if len(distortion) else None)
    rays = np.column_stack((normalized.reshape(-1, 2), np.ones(len(pixels)))) @ rotation.T
    translation = np.asarray(translation, dtype=float)
    with np.errstate(divide='ignore', invalid='ignore'):
        distance = (ground_z - translation[2]) / rays[:, 2]
        points = translation + rays * distance[:, None]
    valid = (rays[:, 2] < -1e-6) & (distance > 0) & np.isfinite(points).all(axis=1)
    valid &= np.linalg.norm(points - translation, axis=1) < max_range
    if not valid.all():
        raise ValueError('Some camera rays do not meet nearby ground')
    return points


class RollingCentre:
    """Rolling mean and illustrative repeatability ellipse, not a Kalman filter.

    The spread between observations is not the physical size of the hot patch.
    Limit effective sample count and retain a floor instead of implying unlimited
    precision from many correlated video frames. This is not calibrated accuracy.
    """
    def __init__(self, window=30, floor_m=0.03):
        self.samples = deque(maxlen=window)
        self.floor_m = floor_m

    def add(self, centre, pixel_covariance):
        self.samples.append(np.asarray(centre, dtype=float))
        samples = np.asarray(self.samples)
        centre = samples.mean(axis=0)
        scatter = np.cov(samples.T) if len(samples) > 1 else np.zeros((2, 2))
        covariance = scatter / min(len(samples), 5)
        covariance += pixel_covariance / min(len(samples), 5)
        covariance += np.eye(2) * self.floor_m**2
        return centre, covariance


def ellipse_points(centre, covariance, sigma=2.0, count=65):
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    angles = np.linspace(0, 2*np.pi, count)
    unit = np.array([np.cos(angles), np.sin(angles)])
    return (eigenvectors @ (np.sqrt(np.maximum(eigenvalues, 0))[:, None] * unit) * sigma).T + centre
