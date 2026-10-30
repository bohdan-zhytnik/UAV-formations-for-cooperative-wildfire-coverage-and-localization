#!/usr/bin/env python3
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import yaml
from fire_scene import generate_world

share = Path(get_package_share_directory('fire_observation_demo'))
runtime = Path(__file__).resolve().parents[1]/'tmux_fleet/runtime'
runtime.mkdir(exist_ok=True)
params = {'mrs_drone_spawner': {'ros__parameters': {
    'extra_resource_paths': [str(share/'models')],
    'tf_static_publisher': {'base_frame': 'fcu', 'ignored_sensor_frames': [
        'air_pressure_sensor', 'magnetometer_sensor', 'navsat_sensor', 'imu_sensor',
        'rgb_down/color', 'thermal_down']}}}}
(runtime/'spawner.yaml').write_text(yaml.safe_dump(params))
fires = yaml.safe_load((share/'config/fleet/fires.yaml').read_text())['fires']
generate_world(share/'worlds/fire_fleet.sdf', fires, runtime/'fire_fleet.sdf')
print('Fleet resources:', share)
print('Generated world:', runtime/'fire_fleet.sdf', '| hot patches:', len(fires))
