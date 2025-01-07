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
    namespace_2 = 'camera_2'

    # camera parameters
    # usb_port_id_camera_1 = '2-10.1'
    # usb_port_id_camera_2 = '1-4'

    usb_port_id_camera_1 = '2-10.1'
    usb_port_id_camera_2 = '2-4'


    # launch camera nodes
    realsense_cameras = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('dual_arm_control'),
                    'launch',
                    'two_realsense.launch.py'
                ])
            ]),
            launch_arguments={
                'namespace_camera_1': namespace_1,
                'namespace_camera_2': namespace_2,
                'usb_port_id_camera_1': usb_port_id_camera_1,
                'usb_port_id_camera_2': usb_port_id_camera_2
            }.items()
            )
    
    # aruco parameters
    marker_size = '0.04'
    camera_frame_1 = namespace_1 + '_color_optical_frame'
    reference_frame_1 = namespace_1 + '_color_optical_frame'
    min_marker_size = '0.035'
    detection_mode = 'DM_NORMAL'
    camera_frame_2 = namespace_2 + '_color_optical_frame'
    reference_frame_2 = namespace_2 + '_color_optical_frame'

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
    aruco_detection_2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('aruco_ros'),
                    'launch',
                    'marker_publisher_general.launch.py'
                ])
            ]),
            launch_arguments={
                'marker_size': marker_size,
                'camera_frame': camera_frame_2,
                'reference_frame': reference_frame_2,
                'min_marker_size': min_marker_size,
                'detection_mode': detection_mode
            }.items(),
            #add rempaing to change the topic name
         
            )
    aruco_detection_2_namespace = GroupAction(
            actions=[

                PushRosNamespace(namespace_2),
                aruco_detection_2,
      ])
    
    # transform aruco poses server nodes
    pose_conversion_1_node  = Node( package='uclv_aruco_detection',
                                    executable='pose_conversion_server_node',
                                    namespace=namespace_1,
                                    name='pose_conversion_server',
                                    remappings=[
                                        ('/aruco_marker_poses', 'marker_publisher/markers'),     
                                    ],
                                    output='log'
                                )
    pose_conversion_2_node  = Node(   package='uclv_aruco_detection',
                                    executable='pose_conversion_server_node',
                                    namespace=namespace_2,
                                    name='pose_conversion_server',
                                    remappings=[
                                        ('/aruco_marker_poses', 'marker_publisher/markers'),     
                                    ],
                                    output='log'
                                )

    
    return LaunchDescription([
        realsense_cameras,
        aruco_detection_1_namespace,
        aruco_detection_2_namespace,
        pose_conversion_1_node,
        pose_conversion_2_node
    ])