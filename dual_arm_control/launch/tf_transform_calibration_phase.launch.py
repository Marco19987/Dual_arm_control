from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.actions import GroupAction
from launch_ros.actions import SetRemap
from launch_ros.actions import PushRosNamespace

def generate_launch_description():

    namespace_1 = 'camera_1'

    usb_port_id_camera_1 = '2-3'
    # usb_port_id_camera_2 = '2-4'

    camera_1_node = Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense_camera',
            output='screen',
            namespace=namespace_1,
            parameters=[
                {'usb_port_id': usb_port_id_camera_1, 'camera_name': namespace_1, 'align_depth.enable': True, 'depth_module.profile': '640x480x30', 'rgb_module.profile': '640x480x30'}
            ],
    )


    # aruco parameters
    marker_size = '0.03'
    camera_frame_1 = namespace_1 + '_color_optical_frame'
    reference_frame_1 = namespace_1 + '_color_optical_frame'
    min_marker_size = '0.03'
    detection_mode = 'DM_NORMAL'

    # aruco detection nodes
    aruco_detection_1 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('aruco_ros'),
                    'launch',
                    'marker_publisher_general.launch.py'
                ])
            ]),
            launch_arguments={
                'marker_size': marker_size,
                'camera_frame': camera_frame_1,
                'reference_frame': reference_frame_1,
                'min_marker_size': min_marker_size,
                'detection_mode': detection_mode
            }.items()
            )

    aruco_detection_1_namespace = GroupAction(
            actions=[
                PushRosNamespace(namespace_1),
                aruco_detection_1,
      ])
    
    # transform aruco poses server nodes
    pose_conversion_1_node  = Node( package='uclv_aruco_detection',
                                    executable='pose_conversion_server_node',
                                    namespace=namespace_1,
                                    name='pose_conversion_server',
                                    remappings=[
                                        ('/aruco_marker_poses', 'marker_publisher/markers'),     
                                    ],
                                    # parameters=[{'additional_transformation_topic': '/robot1/fkine_camera','frame_id' : 'world'}],
                                    output='log'
                                )
    
    aruco_pose = Node(
            package='dual_arm_control',  # Replace with the actual package name
            executable='tf_publisher_from_topic',
            name='tf_publisher_from_topic',
            output='screen',
            parameters=[{'source_frame': 'camera_1_color_optical_frame', 'target_frame' : 'camera_1_object_pose'}] ,
            remappings=[('/pose_topic', '/camera_1/resin_block_1/side1_T_object/pose')]
        )
    

    object_pose = Node(
            package='dual_arm_control',  # Replace with the actual package name
            executable='tf_publisher_from_topic',
            name='tf_publisher_from_topic',
            output='screen',
            parameters=[{'source_frame': 'world', 'target_frame' : 'robot1_fkine_prepivoting'}] ,
            remappings=[('/pose_topic', '/robot1/fkine_pre_pivoting_joint')]
        )



    return LaunchDescription([
        camera_1_node,
        aruco_detection_1_namespace,
        pose_conversion_1_node,
        aruco_pose,
        object_pose
        
    ])