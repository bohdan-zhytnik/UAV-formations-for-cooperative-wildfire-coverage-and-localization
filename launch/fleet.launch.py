from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import yaml


def generate_launch_description():
    share = Path(get_package_share_directory('fire_observation_demo'))
    config = yaml.safe_load((share/'config/fleet/scenario.yaml').read_text())
    mapping = yaml.safe_load((share/'config/fleet/fire_mapping.yaml').read_text())
    nodes = [ComposableNodeContainer(
        name='fire_fleet_tf_container', namespace='', package='rclcpp_components',
        executable='component_container_mt', output='screen', composable_node_descriptions=[ComposableNode(
            package='mrs_tf_connector', plugin='mrs_tf_connector::TFConnector', name='fire_fleet_tf_connector',
            parameters=[{'use_sim_time': True, 'public_config': str(share/'config/fleet/tf_connector.yaml')}],
            remappings=[('~/tf_in', '/tf'), ('~/tf_out', '/tf')])]),
        Node(package='fire_observation_demo', executable='fleet_router.py', name='fleet_router',
             parameters=[{'use_sim_time': True, 'scenario_file': str(share/'config/fleet/scenario.yaml')}], output='screen'),
        Node(package='fire_observation_demo', executable='mission_coordinator.py', name='mission_coordinator',
             parameters=[{'use_sim_time': True, 'scenario_file': str(share/'config/fleet/scenario.yaml'),
                          'mission_file': str(share/'config/fleet/missions.yaml')}], output='screen'),
        Node(package='fire_observation_demo', executable='shared_fire_map.py', name='shared_fire_map',
             parameters=[dict(mapping['map'], use_sim_time=True, map_frame=config['frame'],
                              uav_names=[vehicle['name'] for vehicle in config['uavs']])], output='screen')]
    for index, vehicle in enumerate(config['uavs']):
        uav = vehicle['name']
        prefix = '/fire_fleet/'+uav
        remaps = [('/fire_demo/'+name, prefix+'/'+name) for name in [
            'markers', 'fire_pose', 'fire_boundary', 'fire_status', 'detection_image', 'reset_map',
            'command_status', 'goal_markers', 'accepted_goal', 'goal_settings', 'action']]
        remaps += [('/fire_demo/path/'+uav, prefix+'/path'), ('/goal_pose', prefix+'/goal')]
        nodes += [
            Node(package='fire_observation_demo', executable='fleet_vehicle.py', name=uav+'_fleet_vehicle',
                 parameters=[{'use_sim_time': True, 'uav_name': uav, 'map_frame': uav+'/world_origin',
                              'maximum_altitude_m': config['maximum_command_altitude'],
                              'mission_file': str(share/'config/fleet/missions.yaml'),
                              'goal_marker_color': vehicle['color']}], remappings=remaps, output='screen'),
            Node(package='fire_observation_demo', executable='fleet_fire_observer.py', name=uav+'_fire_observer',
                 parameters=[dict(mapping['observer'], **{'use_sim_time': True, 'uav_name': uav, 'map_frame': config['frame'],
                              'drone_color': vehicle['color'], 'show_ground': index == 0,
                              'start_position': vehicle['spawn'][:2]})],
                 remappings=remaps, output='screen'),
        ]
    return LaunchDescription(nodes)
