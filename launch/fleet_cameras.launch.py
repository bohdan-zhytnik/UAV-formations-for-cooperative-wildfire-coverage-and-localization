from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import yaml


def generate_launch_description():
    share = Path(get_package_share_directory('fire_observation_demo'))
    scenario = yaml.safe_load((share/'config/fleet/scenario.yaml').read_text())
    nodes = []
    for vehicle in scenario['uavs']:
        uav = vehicle['name']
        nodes += [
            Node(package='ros_gz_image', executable='image_bridge', name=uav+'_thermal_bridge',
                 arguments=[f'/{uav}/thermal_down/image_raw'], parameters=[{'use_sim_time': True}]),
            Node(package='ros_gz_bridge', executable='parameter_bridge', name=uav+'_thermal_info_bridge',
                 arguments=[f'/{uav}/thermal_down/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
                 parameters=[{'use_sim_time': True}]),
            Node(package='fire_observation_demo', executable='thermal_preview.py', name=uav+'_thermal_preview',
                 parameters=[{'use_sim_time': True, 'uav_name': uav}]),
        ]
        for camera, x in [('rgb_down', '0.04'), ('thermal_down', '-0.04')]:
            nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
                name=uav+'_'+camera+'_tf', parameters=[{'use_sim_time': True}], arguments=[
                    '--x', x, '--y', '0', '--z', '-0.09', '--roll', '3.141592653589793',
                    '--pitch', '0', '--yaw', '-1.5707963267948966', '--frame-id', uav+'/fcu',
                    '--child-frame-id', uav+'/'+camera+'/optical']))
    return LaunchDescription(nodes)
