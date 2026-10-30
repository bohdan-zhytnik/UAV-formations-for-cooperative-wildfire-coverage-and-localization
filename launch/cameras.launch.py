from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        Node(package='ros_gz_image', executable='image_bridge', name='fire_thermal_image_bridge',
             arguments=['/uav1/thermal_down/image_raw'],
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='fire_thermal_info_bridge',
             arguments=['/uav1/thermal_down/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='fire_observation_demo', executable='thermal_preview.py',
             name='fire_thermal_preview', parameters=[{'use_sim_time': True}], output='screen'),
    ]
    for camera, x in [('rgb_down', '0.04'), ('thermal_down', '-0.04')]:
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
            name='fire_' + camera + '_tf', arguments=[
                '--x', x, '--y', '0', '--z', '-0.09',
                '--roll', '3.141592653589793', '--pitch', '0', '--yaw', '-1.5707963267948966',
                '--frame-id', 'uav1/fcu', '--child-frame-id', 'uav1/' + camera + '/optical'],
            parameters=[{'use_sim_time': True}], output='screen'))
    return LaunchDescription(nodes)
