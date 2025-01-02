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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

configurable_fkine_parameters_robot1 = [
    {'name': 'publish_jacobian',  'default': 'true',
        'description': 'publish the jacobian'},
    {'name': 'robot_library_name', 'default': 'dual_arm_control',
        'description': 'package name of the robot library'},
    {'name': 'robot_plugin_name', 'default': 'uclv::robot::LBRiiwa7Ext',
        'description': 'plugin name of the robot'}
]
configurable_fkine_parameters_robot2 = [
    {'name': 'publish_jacobian',  'default': 'true',
        'description': 'publish the jacobian'},
    {'name': 'robot_library_name', 'default': 'dual_arm_control',
        'description': 'package name of the robot library'},
    {'name': 'robot_plugin_name', 'default': 'uclv::robot::MotomanSIA5FExt',
        'description': 'plugin name of the robot'}
]

configurable_inv_diffkine_parameters = [
    {'name': 'joint_names',  'default': "['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7']",
        'description': 'joint_names'}
]

configurable_integrator_diffkine_parameters = [
    {'name': 'integrator.sampling_time',  'default': "0.02",
        'description': 'sampling_time'}
]

configurable_joint_traj_diffkine_parameters = [
    {'name': 'trajectory.rate',  'default': str(1.0/0.002),
        'description': 'rate'}
]


def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]


def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


def generate_launch_description():
    
    robot1_namespace = 'robot1'
    robot2_namespace = 'robot2'
    default_rviz_config_path = PathJoinSubstitution([FindPackageShare('dual_arm_control'), 'rviz', 'two_robots.rviz'])
    
        
    ld = LaunchDescription()
    
    # Declare launch arguments
    
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)
    
    for param in declare_configurable_parameters(configurable_fkine_parameters_robot2):
        ld.add_action(param)
    

    # robot 1 urdf launch
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
    
    # robot 2 urdf launch
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
    
    # robots fkine nodes
    ld.add_action(ComposableNodeContainer(
        name='robot_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=[
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot1_namespace,
                    plugin='uclv_robot_ros::FKineNode',
                    name="",
                    parameters=[set_configurable_parameters(
                        configurable_fkine_parameters_robot2)],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                # ComposableNode(
                #     package='uclv_robot_ros',
                #     namespace='lbr',
                #     plugin='uclv_robot_ros::JointTrajectoryNode',
                #     name="",
                #     # parameters=[
                #     # ],
                #     extra_arguments=[{'use_intra_process_comms': True}]),
                # ComposableNode(
                #     package='uclv_robot_ros',
                #     namespace='lbr',
                #     plugin='uclv_robot_ros::JointIntegrator',
                #     name="",
                #     remappings=[('joint_vel_states', 'command/joint_vel_states'),
                #                 ('integrator/joint_states', 'command/joint_states')
                #                 ],
                #     # parameters=[
                #     # ],
                #     extra_arguments=[{'use_intra_process_comms': True}]),
                # ComposableNode(
                #     package='uclv_robot_ros',
                #     namespace='lbr',
                #     plugin='uclv_robot_ros::InverseDifferentialKinematics',
                #     name="",
                #     parameters=[
                #         set_configurable_parameters(configurable_inv_diffkine_parameters)
                #     ],
                #     extra_arguments=[{'use_intra_process_comms': True}]),
                
        ],
        output='both',
    ))

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