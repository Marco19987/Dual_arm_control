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
    # Transform between robots
    b1Tb2_pub =  Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['1.63626', '-0.006', '0.013', '1.5282496', '-0.0356366', '0.0205613', 'world', 'yaskawa_base_link']
        )
    
    b1Tb2_estimated = Node(
            package='dual_arm_control',  # Replace with the actual package name
            executable='tf_publisher_from_topic',
            name='tf_publisher_from_topic',
            output='screen',
            parameters=[{'source_frame': 'world', 'target_frame' : 'yaskawa_base_link'}] ,
            remappings=[('/pose_topic', '/ekf/b1Tb2_filtered')]
        )
    
    object_pose = Node(
            package='dual_arm_control',  # Replace with the actual package name
            executable='tf_publisher_from_topic',
            name='tf_publisher_from_topic',
            output='screen',
            parameters=[{'source_frame': 'world', 'target_frame' : 'object_pose'}] ,
            remappings=[('/pose_topic', '/ekf/object_pose')]
        )



    return LaunchDescription([
        # camera1_camera2_breadboard,
        #b1Tb2_pub,
        robot1_camera,
        robot2_camera,
        b1Tb2_estimated,
        object_pose
        
    ])