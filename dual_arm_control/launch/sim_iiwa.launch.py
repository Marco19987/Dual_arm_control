from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    ld = LaunchDescription()

    uclv_robot_ros_path = FindPackageShare('dual_arm_control')
    default_model_path = PathJoinSubstitution([uclv_robot_ros_path, 'urdf/', 'iiwa_ibr_pivoting.xacro'])
    default_rviz_config_path = PathJoinSubstitution([uclv_robot_ros_path, 'rviz', 'rviz_iiwa.rviz'])

    # These parameters are maintained for backwards compatibility
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    ld.add_action(gui_arg)
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    display_rviz_arg = DeclareLaunchArgument(name='display_rviz', default_value='true', choices=['true', 'false'],
                                     description='Flag to enable rviz display')
    ld.add_action(rviz_arg)
    ld.add_action(display_rviz_arg)

    

    # This parameter has changed its meaning slightly from previous versions
    ld.add_action(DeclareLaunchArgument(name='urdf_model', default_value=default_model_path,
                                        description='Path to robot urdf file'))
    
    urdf_path = PathJoinSubstitution(['', LaunchConfiguration('urdf_model')])

    robot_description_content = ParameterValue(Command(['xacro ', LaunchConfiguration('urdf_model')]), value_type=str)

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                      }])

    ld.add_action(robot_state_publisher_node)

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    ld.add_action(DeclareLaunchArgument(name='use_joint_state_publisher', default_value='true', choices=['true', 'false'],
                                        description='Flag to enable joint_state_publisher'))
    
    # ld.add_action(Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     condition=IfCondition(LaunchConfiguration('use_joint_state_publisher')),
    #     parameters=[{
    #         'source_list': ['command/joint_states'],
    #     }],
    # ))

    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')),
        parameters=[{
            'source_list': ['command/joint_states'],
        }],
    ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('display_rviz')),
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    ))

    return ld