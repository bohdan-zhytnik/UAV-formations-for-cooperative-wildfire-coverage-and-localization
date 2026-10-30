from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    settings = os.path.join(get_package_share_directory('fire_observation_demo'), 'config', 'demo.yaml')
    return LaunchDescription([
        Node(package='fire_observation_demo', executable=executable, name=name,
             parameters=[settings, {'use_sim_time': True}], output='screen')
        for executable, name in [('fire_mapper.py', 'fire_mapper'), ('goal_bridge.py', 'goal_bridge')]
    ])
