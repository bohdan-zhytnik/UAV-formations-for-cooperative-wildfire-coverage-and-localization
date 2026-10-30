"""Image-derived observations and a small static-fire map, independent of ROS.

No scene truth, Kalman filter, SLAM or motion model enters this module.
"""
from collections import deque
import math

import cv2
import numpy as np

from fire_geometry import project_ground


def polygon_area(points):
    xy = np.asarray(points, dtype=float)[:, :2]
    return float(abs(np.dot(xy[:, 0], np.roll(xy[:, 1], 1)) -
                     np.dot(xy[:, 1], np.roll(xy[:, 0], 1))) / 2)


def hot_regions(raw, intrinsic, distortion, rotation, translation, ground_z=0.,
                threshold_kelvin=450., kelvin_per_count=.02, minimum_hot_pixels=50):
    """Project EVERY connected hot component, explicitly flagging clipped ones.

    Boundary and area describe the visible pixel contour on the assumed ground.
    A clipped region is useful for display, but must not update a fire centre.
    """
    if raw.ndim != 2 or not np.isfinite(raw).all():
        raise ValueError('Expected a finite single-channel thermal image')
    intrinsic = np.asarray(intrinsic, dtype=float).reshape(3, 3)
    if not np.isfinite(intrinsic).all() or min(intrinsic[0, 0], intrinsic[1, 1]) <= 0:
        raise ValueError('Invalid camera calibration')
    kelvin = raw.astype(np.float32) * kelvin_per_count
    count, labels, stats, _ = cv2.connectedComponentsWithStats((kelvin > threshold_kelvin).astype(np.uint8), 8)
    observations = []
    height, width = raw.shape
    for component in range(1, count):
        x, y, w, h, pixels_count = stats[component]
        if pixels_count < minimum_hot_pixels:
            continue
        selected = (labels == component).astype(np.uint8)
        contours, _ = cv2.findContours(selected, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour = cv2.approxPolyDP(max(contours, key=cv2.contourArea), 1.5, True).reshape(-1, 2)
        if len(contour) < 3:
            continue
        rows, columns = np.nonzero(selected)
        pixels = np.column_stack((columns, rows)).astype(float)
        points = project_ground(pixels, intrinsic, distortion, rotation, translation, ground_z)
        centre_pixel = pixels.mean(axis=0)
        offsets = project_ground([centre_pixel, centre_pixel+[1, 0], centre_pixel+[0, 1]],
                                 intrinsic, distortion, rotation, translation, ground_z)[:, :2]
        jacobian = (offsets[1:]-offsets[0]).T
        boundary = project_ground(contour, intrinsic, distortion, rotation, translation, ground_z)
        temperatures = kelvin[rows, columns]
        observations.append({
            'region_id': len(observations)+1,
            'partial': bool(x <= 1 or y <= 1 or x+w >= width-1 or y+h >= height-1),
            'centre_m': points[:, :2].mean(axis=0).tolist(),
            'pixel_covariance_xy_m2': (jacobian @ jacobian.T / 12).tolist(),
            'boundary_m': boundary.tolist(), 'area_m2': polygon_area(boundary),
            'mean_kelvin': float(temperatures.mean(dtype=np.float64)), 'max_kelvin': float(temperatures.max()),
            'hot_pixels': int(pixels_count), 'pixel_contour': contour.tolist(),
            'centre_pixel': centre_pixel.tolist()})
    return observations


def checked_observation(value):
    """Validate before mutating any map state; never silently accept NaNs."""
    centre = np.asarray(value['centre_m'], dtype=float)
    boundary = np.asarray(value['boundary_m'], dtype=float)
    covariance = np.asarray(value['pixel_covariance_xy_m2'], dtype=float)
    if centre.shape != (2,) or boundary.ndim != 2 or boundary.shape[1] != 3 or len(boundary) < 3:
        raise ValueError('Invalid observation geometry')
    if covariance.shape != (2, 2) or not all(np.isfinite(x).all() for x in [centre, boundary, covariance]):
        raise ValueError('Observation geometry must be finite')
    if not np.allclose(covariance, covariance.T) or np.linalg.eigvalsh(covariance).min() < -1e-10:
        raise ValueError('Invalid pixel covariance')
    area = polygon_area(boundary)
    mean, maximum = float(value['mean_kelvin']), float(value['max_kelvin'])
    if area <= 0 or not all(math.isfinite(x) for x in [mean, maximum]) or not 0 < mean <= maximum:
        raise ValueError('Invalid area or temperature')
    if type(value['partial']) is not bool:
        raise ValueError('partial must be boolean')
    return dict(value, centre_m=centre, boundary_m=boundary, pixel_covariance_xy_m2=covariance,
                area_m2=area, mean_kelvin=mean, max_kelvin=maximum)


class FireTrack:
    def __init__(self, identifier, window_per_uav, floor_m):
        self.identifier, self.window, self.floor = identifier, window_per_uav, floor_m
        self.samples, self.last_seen_by = {}, {}
        self.observations = 0

    def add(self, source, stamp, observation):
        if self.observations == 0:
            self.first_observer, self.first_observed_sim_s = source, stamp
        self.samples.setdefault(source, deque(maxlen=self.window)).append(observation)
        self.last_seen_by[source] = stamp
        self.observations += 1
        samples, weights = [], []
        for source_samples in self.samples.values():
            samples.extend(source_samples)
            weights.extend([1/(len(self.samples)*len(source_samples))]*len(source_samples))
        weights = np.asarray(weights)
        centres = np.array([sample['centre_m'] for sample in samples])
        self.centre = np.sum(centres*weights[:, None], axis=0)
        residual = centres-self.centre
        effective_count = min(len(samples), 5)
        self.covariance = (residual.T @ (residual*weights[:, None])) / effective_count
        self.covariance += sum(w*s['pixel_covariance_xy_m2'] for w, s in zip(weights, samples)) / effective_count
        self.covariance += np.eye(2)*self.floor**2
        # Keep the latest complete outline, translated onto the rolling centre.
        # Its shape/area is measured, not a box read from the scene configuration.
        self.boundary = observation['boundary_m'].copy()
        self.boundary[:, :2] += self.centre-observation['centre_m']
        self.area = observation['area_m2']
        self.mean_kelvin = float(sum(w*s['mean_kelvin'] for w, s in zip(weights, samples)))
        self.max_kelvin = max(s['max_kelvin'] for s in samples)

    def snapshot(self, now, visible_sources):
        return {'id': self.identifier, 'fire_id': f'fire_{self.identifier}',
                'first_observer': self.first_observer, 'first_observed_sim_s': self.first_observed_sim_s,
                'centre_m': self.centre.tolist(), 'boundary_m': self.boundary.tolist(),
                'covariance_xy_m2': self.covariance.tolist(), 'area_m2': self.area,
                'mean_kelvin': self.mean_kelvin, 'max_kelvin': self.max_kelvin,
                'observations': self.observations,
                'window_samples': sum(len(values) for values in self.samples.values()),
                'contributing_uavs': sorted(self.samples), 'visible_from': sorted(visible_sources),
                'visible': bool(visible_sources),
                'last_seen_age_s': max(0., now-max(self.last_seen_by.values())),
                'last_seen_by_uav_s': {key: max(0., now-stamp) for key, stamp in self.last_seen_by.items()}}


class SharedFireMap:
    """Gated nearest-centre matching, one-to-one within each camera frame.

    No tracks are deleted just because they leave view. IDs last until reset.
    This intentionally assumes distinct, stationary, separated hot regions.
    """
    def __init__(self, uavs=('uav1', 'uav2'), association_distance_m=.9,
                 window_per_uav=15, uncertainty_floor_m=.03, visible_age_s=2., max_tracks=64):
        if association_distance_m <= 0 or window_per_uav < 1 or max_tracks < 1:
            raise ValueError('Invalid map limits')
        self.uavs, self.distance = tuple(uavs), association_distance_m
        self.window, self.floor, self.visible_age = window_per_uav, uncertainty_floor_m, visible_age_s
        self.max_tracks = max_tracks
        self.reset()

    def reset(self):
        self.tracks, self.last_stamps, self.latest_ids = {}, {}, {}
        self.next_id, self.dropped_new_tracks = 1, 0

    def update(self, source, stamp, observations):
        if source not in self.uavs or not math.isfinite(stamp) or stamp < 0:
            raise ValueError('Unknown observer or invalid image timestamp')
        if stamp <= self.last_stamps.get(source, -math.inf):
            return None  # Duplicate/replayed or out-of-order frame, not new evidence.
        checked = [checked_observation(value) for value in observations]
        complete = [(i, value) for i, value in enumerate(checked) if not value['partial']]
        candidates = []
        for index, observation in complete:
            for identifier, track in self.tracks.items():
                distance = float(np.linalg.norm(observation['centre_m']-track.centre))
                ratio = observation['area_m2']/track.area
                if distance <= self.distance and .4 <= ratio <= 2.5:
                    candidates.append((distance, identifier, index))
        assigned, used_tracks = {}, set()
        for _, identifier, index in sorted(candidates):
            if index not in assigned and identifier not in used_tracks:
                assigned[index] = identifier
                used_tracks.add(identifier)
        # New tracks are deliberately not candidates for other regions in this
        # same image: two simultaneous detections must never collapse into one.
        for index, observation in complete:
            if index not in assigned:
                if len(self.tracks) >= self.max_tracks:
                    self.dropped_new_tracks += 1
                    continue
                identifier = self.next_id
                self.next_id += 1
                self.tracks[identifier] = FireTrack(identifier, self.window, self.floor)
                assigned[index] = identifier
            self.tracks[assigned[index]].add(source, stamp, observation)
        self.last_stamps[source], self.latest_ids[source] = stamp, set(assigned.values())
        return assigned

    def snapshot(self, now):
        return [track.snapshot(now, [source for source, ids in self.latest_ids.items()
                                    if identifier in ids and now-self.last_stamps[source] < self.visible_age])
                for identifier, track in sorted(self.tracks.items())]
