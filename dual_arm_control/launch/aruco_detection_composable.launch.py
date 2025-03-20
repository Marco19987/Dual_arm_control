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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    namespace_1 = 'robot1'
    namespace_2 = 'robot2'
    container_name = 'aruco_container'

    # camera parameters
    # usb_port_id_camera_1 = '2-10.1'
    # usb_port_id_camera_2 = '1-4'

    # usb_port_id_camera_1 = '2-10.1'
    # usb_port_id_camera_2 = '2-4'

    usb_port_id_camera_1 = '2-3'
    usb_port_id_camera_2 = '2-4'


    container_aruco = ComposableNodeContainer(
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='realsense_camera',
                namespace=namespace_1,
                parameters=[
                {'usb_port_id': usb_port_id_camera_1, 'camera_name': namespace_1, 'align_depth.enable': True, 'depth_module.profile': '640x480x30', 'rgb_module.profile': '640x480x30'
                 ,'publish_tf': False}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='realsense_camera',
                namespace=namespace_2,
                parameters=[
                {'usb_port_id': usb_port_id_camera_2, 'camera_name': namespace_2, 'align_depth.enable': True, 'depth_module.profile': '640x480x30', 'rgb_module.profile': '640x480x30'
                 ,'publish_tf': False}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ComposableNode(
                package='uclv_aruco_detection',
                plugin='uclv_aruco_detection::PoseConversionServer',
                name='pose_conversion_server',
                namespace=namespace_1,
                remappings=[('/aruco_marker_poses', 'aruco_marker_publisher/markers')],
                parameters=[{'additional_transformation_topic': '/robot1/fkine_camera','frame_id' : 'world'}],
                extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ComposableNode(
                package='uclv_aruco_detection',
                plugin='uclv_aruco_detection::PoseConversionServer',
                name='pose_conversion_server',
                namespace=namespace_2,
                remappings=[('/aruco_marker_poses', 'aruco_marker_publisher/markers')],
                parameters=[{'additional_transformation_topic': '/robot2/fkine_camera','frame_id' : 'yaskawa_base_link'}],
                extra_arguments=[{'use_intra_process_comms': True}],
                )
            ]
        )
    

    
    
    # aruco parameters
    marker_size = '0.0325'
    camera_frame_1 = namespace_1 + '_color_optical_frame'
    reference_frame_1 = namespace_1 + '_color_optical_frame'
    min_marker_size = '0.01'
    detection_mode = 'DM_FAST'
    camera_frame_2 = namespace_2 + '_color_optical_frame'
    reference_frame_2 = namespace_2 + '_color_optical_frame'

    # aruco detection nodes
    aruco_detection_1 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('aruco_ros'),
                    'launch',
                    'marker_publisher_general_composable.launch.py'
                ])
            ]),
            launch_arguments={
                'marker_size': marker_size,
                'camera_frame': camera_frame_1,
                'reference_frame': reference_frame_1,
                'min_marker_size': min_marker_size,
                'detection_mode': detection_mode,
                'container_name_to_attach': container_name
            }.items()
            )

    aruco_detection_1_namespace = GroupAction(
            actions=[
                PushRosNamespace(namespace_1),
                aruco_detection_1,
      ])
    aruco_detection_2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('aruco_ros'),
                    'launch',
                    'marker_publisher_general_composable.launch.py'
                ])
            ]),
            launch_arguments={
                'marker_size': marker_size,
                'camera_frame': camera_frame_2,
                'reference_frame': reference_frame_2,
                'min_marker_size': min_marker_size,
                'detection_mode': detection_mode,
                'container_name_to_attach': container_name
            }.items(),
            #add rempaing to change the topic name
         
            )
    aruco_detection_2_namespace = GroupAction(
            actions=[

                PushRosNamespace(namespace_2),
                aruco_detection_2,
      ])
    


    


    
    return LaunchDescription([
        container_aruco,
        aruco_detection_1_namespace,
        aruco_detection_2_namespace,
    ])