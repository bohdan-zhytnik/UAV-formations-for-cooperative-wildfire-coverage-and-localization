"""Generate hot patches from scene truth, independently of the perception nodes."""
import math
import re
import xml.etree.ElementTree as ET


def add_text(parent, tag, text):
    node = ET.SubElement(parent, tag)
    node.text = str(text)
    return node


def generate_world(template, fires, destination):
    names = set()
    for fire in fires:
        name = fire['name']
        values = [*fire['position'], *fire['size'], fire['temperature_kelvin'], *fire['color']]
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name) or name in names:
            raise ValueError('Fire model names must be unique identifiers')
        names.add(name)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError('Fire parameters must be finite')
        if len(fire['position']) != 2 or len(fire['size']) != 2 or len(fire['color']) != 4:
            raise ValueError('Expected position [x,y], size [width,height], color [r,g,b,a]')
        if min(fire['size']) <= 0 or fire['temperature_kelvin'] <= 0 or not all(0 <= c <= 1 for c in fire['color']):
            raise ValueError('Invalid fire dimensions, temperature or colour')
    root = ET.parse(template, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    world = root.getroot().find('world')
    for model in list(world.findall('model')):
        if model.attrib['name'] == 'fire_patch' or model.attrib['name'].startswith('configured_fire_'):
            world.remove(model)
    for fire in fires:
        model = ET.SubElement(world, 'model', name='configured_fire_'+fire['name'])
        add_text(model, 'pose', f"{fire['position'][0]} {fire['position'][1]} 0.015 0 0 0")
        add_text(model, 'static', 'true')
        visual = ET.SubElement(ET.SubElement(model, 'link', name='link'), 'visual', name='hot_patch')
        box = ET.SubElement(ET.SubElement(visual, 'geometry'), 'box')
        add_text(box, 'size', f"{fire['size'][0]} {fire['size'][1]} 0.02")
        material = ET.SubElement(visual, 'material')
        colour = ' '.join(str(value) for value in fire['color'])
        add_text(material, 'ambient', colour)
        add_text(material, 'diffuse', colour)
        add_text(material, 'emissive', '0.2 0.02 0 1')
        plugin = ET.SubElement(visual, 'plugin', filename='gz-sim-thermal-system', name='gz::sim::systems::Thermal')
        add_text(plugin, 'temperature', fire['temperature_kelvin'])
    ET.indent(root, space='  ')
    root.write(destination, encoding='utf-8', xml_declaration=True)
