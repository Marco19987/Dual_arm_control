from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.425', '0.05', '0.00', '3.14', '0', '0', 'camera_1_link', 'camera_2_link']
        ),
    ])