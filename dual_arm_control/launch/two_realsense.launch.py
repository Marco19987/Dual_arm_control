from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    namespace_camera_1 = LaunchConfiguration('namespace_camera_1')
    namespace_camera_2 = LaunchConfiguration('namespace_camera_2')
    usb_port_id_camera_1 = LaunchConfiguration('usb_port_id_camera_1')
    usb_port_id_camera_2 = LaunchConfiguration('usb_port_id_camera_2')
    

    namespace_camera_1_launch_arg = DeclareLaunchArgument(
        'namespace_camera_1',
        default_value='camera_1',
        description='Namespace for the camera 1 node'
    )

    namespace_camera_2_launch_arg = DeclareLaunchArgument(
        'namespace_camera_2',
        default_value='camera_2',
        description='Namespace for the camera 2 node'
    )

    usb_port_id_camera_1_launch_arg = DeclareLaunchArgument(
        'usb_port_id_camera_1',
        default_value='2-10.1',
        description='USB port ID for the camera 1'
    )

    usb_port_id_camera_2_launch_arg = DeclareLaunchArgument(
        'usb_port_id_camera_2',
        default_value='1-4',
        description='USB port ID for the camera 2'
    )



    # get string from launch configuration
    
    camera_1_node = Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense_camera',
            output='screen',
            namespace=namespace_camera_1,
            parameters=[
                {'usb_port_id': usb_port_id_camera_1, 'camera_name': namespace_camera_1, 'align_depth.enable': True}
            ],
            # remappings=[
            #     ('/' + namespace_camera_1 + '/realsense_camera/color/image_raw', '/' + namespace_camera_1 + '/aruco/image'),
            #     ('/' + namespace_camera_1 + '/realsense_camera/color/camera_info','/' + namespace_camera_1 + '/aruco/camera_info')
                
            # ]
    )
    camera_2_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense_camera',
        output='screen',
        namespace=namespace_camera_2,
        parameters=[
            {'usb_port_id': usb_port_id_camera_2, 'camera_name': namespace_camera_2, 'align_depth.enable': True}
        ],
        # remappings=[
        #     ('/' + namespace_camera_2 + '/realsense_camera/color/image_raw', '/' + namespace_camera_2 + '/aruco/image'),
        #     ('/' + namespace_camera_2 + '/realsense_camera/color/camera_info','/' + namespace_camera_2 + '/aruco/camera_info')
        # ]
    )
    
    return LaunchDescription([
        namespace_camera_1_launch_arg,
        namespace_camera_2_launch_arg,
        usb_port_id_camera_1_launch_arg,
        usb_port_id_camera_2_launch_arg,
        camera_1_node,
        camera_2_node
    ])