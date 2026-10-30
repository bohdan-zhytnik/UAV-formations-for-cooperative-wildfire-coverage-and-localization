#!/usr/bin/env python3
"""Render using the installed MRS templates and check the physical sensor assembly."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import jinja2
from ament_index_python.packages import get_package_share_directory

p=Path(__file__).resolve().parents[1]
models=Path(get_package_share_directory('mrs_uav_gazebo_simulator'))/'models'
env=jinja2.Environment(loader=jinja2.FileSystemLoader([str(p/'models'),str(models)]))
env.globals['math']=math
s=env.get_template('x500_fire.sdf.jinja').render(spawner_args={'name':'uav1','enable-rangefinder':{}})
root=ET.fromstring(s)
sensors={x.attrib['name']:x for x in root.findall('.//sensor')}
assert sensors['thermal_down'].attrib['type']=='thermal'
assert sensors['rgb_down/color'].attrib['type']=='camera'
assert any(x.attrib['name']=='lidar' for x in root.findall('.//sensor'))
mass=sum(float(x.text) for x in root.findall('.//inertial/mass'))
assert 2.20<mass<2.23,mass
out=p/'artifacts'; out.mkdir(exist_ok=True)
(out/'rendered_x500_fire.sdf').write_text(s)
print(f'Rendered model: {len(sensors)} sensors, total link mass {mass:.6f} kg')
