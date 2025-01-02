from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.conditions import IfCondition, UnlessCondition


from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import SetRemap
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    
    robot1_namespace = 'robot1'
    robot2_namespace = 'robot2'
    default_rviz_config_path = PathJoinSubstitution([FindPackageShare('dual_arm_control'), 'rviz', 'two_robots.rviz'])
    
    ld = LaunchDescription()
    
    # Declare launch arguments
    
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    # robot 1 launch
    robot1_launch_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('dual_arm_control'),
                    'launch',
                    'sim_yaskawa.launch.py'
                ])
            ]),
            launch_arguments={
                'gui': 'true',
                'display_rviz': 'false',
            }.items()
            )
    robot1_launch_node = GroupAction(
            actions=[
                PushRosNamespace(robot1_namespace),
                robot1_launch_node,
      ])
    ld.add_action(robot1_launch_node)
    
    # robot 2 launch
    robot2_launch_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('dual_arm_control'),
                    'launch',
                    'sim_iiwa.launch.py'
                ])
            ]),
            launch_arguments={
                'gui': 'true',
                'display_rviz': 'false',
            }.items()
            )
    robot2_launch_node = GroupAction(
            actions=[
                PushRosNamespace(robot2_namespace),
                robot2_launch_node,
        ])
    ld.add_action(robot2_launch_node)
    

    # rviz
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    ))
    
    # Transform between robots
    ld.add_action(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['1.6', '0.0', '0.0', '3.14', '0', '0.0', 'world', 'yaskawa_base_link']
        )
    )

    return ld