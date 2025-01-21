from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    robot1_camera = Node(
            package='dual_arm_control',  # Replace with the actual package name
            executable='tf_publisher_from_topic',
            name='tf_publisher_from_topic',
            output='screen',
            parameters=[{'source_frame': 'world', 'target_frame' : 'robot1_color_optical_frame'}] ,
            remappings=[('/pose_topic', '/robot1/fkine_camera')]
        )
    
    robot2_camera = Node(
            package='dual_arm_control',  # Replace with the actual package name
            executable='tf_publisher_from_topic',
            name='tf_publisher_from_topic',
            output='screen',
            parameters=[{'source_frame': 'yaskawa_base_link', 'target_frame' : 'robot2_color_optical_frame'}] ,
            remappings=[('/pose_topic', '/robot2/fkine_camera')]
        )
    
    camera1_camera2_breadboard = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.425', '0.05', '0.00', '3.14', '0', '0', 'camera_1_link', 'camera_2_link']
        )


    return LaunchDescription([
        # camera1_camera2_breadboard,
        robot1_camera,
        robot2_camera
        
    ])