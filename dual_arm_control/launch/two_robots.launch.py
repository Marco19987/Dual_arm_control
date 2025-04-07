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
    {'name': 'publish_jacobian',  'default': True,
        'description': 'publish the jacobian'},
    {'name': 'robot_library_name', 'default': 'dual_arm_control',
        'description': 'package name of the robot library'},
    {'name': 'robot_plugin_name', 'default': 'uclv::robot::LBRiiwa7Ext',
        'description': 'plugin name of the robot'}
]
configurable_fkine_parameters_robot2 = [
    {'name': 'publish_jacobian',  'default': True,
        'description': 'publish the jacobian'},
    {'name': 'robot_library_name', 'default': 'dual_arm_control',
        'description': 'package name of the robot library'},
    {'name': 'robot_plugin_name', 'default': 'uclv::robot::MotomanSIA5FExt',
        'description': 'plugin name of the robot'}
]

configurable_fkine_parameters_robot1_tactile_sensor = [
    {'name': 'publish_jacobian',  'default': False,
        'description': 'publish the jacobian'},
    {'name': 'robot_library_name', 'default': 'dual_arm_control',
        'description': 'package name of the robot library'},
    {'name': 'robot_plugin_name', 'default': 'uclv::robot::LBRiiwa7Ext',
        'description': 'plugin name of the robot'},
    {'name': 'n_joint', 'default': 7, 'description': 'number of joints until which the fkine is calculated'}
]

configurable_fkine_parameters_robot2_tactile_sensor = [
    {'name': 'publish_jacobian',  'default': False,
        'description': 'publish the jacobian'},
    {'name': 'robot_library_name', 'default': 'dual_arm_control',
        'description': 'package name of the robot library'},
    {'name': 'robot_plugin_name', 'default': 'uclv::robot::MotomanSIA5FExt',
        'description': 'plugin name of the robot'},
    {'name': 'n_joint', 'default': 7, 'description': 'number of joints until which the fkine is calculated'}
]
configurable_fkine_parameters_robot1_camera = [
    {'name': 'publish_jacobian',  'default': False,
        'description': 'publish the jacobian'},
    {'name': 'robot_library_name', 'default': 'dual_arm_control',
        'description': 'package name of the robot library'},
    {'name': 'robot_plugin_name', 'default': 'uclv::robot::LBRiiwa7Ext',
        'description': 'plugin name of the robot'},
    {'name': 'n_joint', 'default': 7, 'description': 'number of joints until which the fkine is calculated'}
]
configurable_fkine_parameters_robot2_camera = [
    {'name': 'publish_jacobian',  'default': False,
        'description': 'publish the jacobian'},
    {'name': 'robot_library_name', 'default': 'dual_arm_control',
        'description': 'package name of the robot library'},
    {'name': 'robot_plugin_name', 'default': 'uclv::robot::MotomanSIA5FExt',
        'description': 'plugin name of the robot'},
    {'name': 'n_joint', 'default': 7, 'description': 'number of joints until which the fkine is calculated'}
]
configurable_fkine_parameters_robot1_nopivoting = [
    {'name': 'publish_jacobian',  'default': False,
        'description': 'publish the jacobian'},
    {'name': 'robot_library_name', 'default': 'dual_arm_control',
        'description': 'package name of the robot library'},
    {'name': 'robot_plugin_name', 'default': 'uclv::robot::LBRiiwa7Ext',
        'description': 'plugin name of the robot'},
    {'name': 'n_joint', 'default': 7, 'description': 'number of joints until which the fkine is calculated'}
]
configurable_fkine_parameters_robot2_nopivoting = [
    {'name': 'publish_jacobian',  'default': False,
        'description': 'publish the jacobian'},
    {'name': 'robot_library_name', 'default': 'dual_arm_control',
        'description': 'package name of the robot library'},
    {'name': 'robot_plugin_name', 'default': 'uclv::robot::MotomanSIA5FExt',
        'description': 'plugin name of the robot'},
    {'name': 'n_joint', 'default': 7, 'description': 'number of joints until which the fkine is calculated'}
]
configurable_inv_diffkine_parameters_robot1 = [
    {'name': 'joint_names',  'default': ['iiwa_joint1', 'iiwa_joint2', 'iiwa_joint3', 'iiwa_joint4', 'iiwa_joint5', 'iiwa_joint6', 'iiwa_joint7','iiwa_pivoting_joint'],
        'description': 'joint_names'},
    {'name': 'joint_vel_limits', 'default':[1.7104, 1.7104, 1.7453, 2.2689, 2.4435, 3.1416, 3.1416,100.0], 'description': 'robot 1 joint vel limits'},
    {'name': 'joints_to_exclude', 'default' : [7], 'description': 'joints to exclude from the inverse differential kinematics, provide as list of integers'}
]


configurable_inv_diffkine_parameters_robot2 = [
    {'name': 'joint_names',  'default': ['yaskawa_joint_s', 'yaskawa_joint_l','yaskawa_joint_e','yaskawa_joint_u', 'yaskawa_joint_r', 'yaskawa_joint_b', 'yaskawa_joint_t','yaskawa_pivoting_joint'],
        'description': 'joint_names'},
    {'name': 'joint_vel_limits', 'default': [3.50, 3.50, 3.50, 3.50, 3.50, 4.015, 6.11, 100.0], 'description': 'robot 2 joint vel limits'},
    {'name': 'joints_to_exclude', 'default' : [7], 'description': 'joints to exclude from the inverse differential kinematics, provide as list of integers'}
]

configurable_integrator_diffkine_parameters = [
    {'name': 'integrator.sampling_time',  'default': "0.02",
        'description': 'sampling_time'}
]

configurable_joint_traj_diffkine_parameters = [
    {'name': 'trajectory.rate',  'default': str(1.0/0.002),
        'description': 'rate'}
]
configurable_cooperative_robots_parameters = [
    {'name': 'joint_names_robot1',  'default': ['iiwa_joint1', 'iiwa_joint2', 'iiwa_joint3', 'iiwa_joint4', 'iiwa_joint5', 'iiwa_joint6', 'iiwa_joint7','iiwa_pivoting_joint'],
        'description': 'joint_names robot 1'},
    {'name': 'joint_names_robot2',  'default': ['yaskawa_joint_s', 'yaskawa_joint_l','yaskawa_joint_e','yaskawa_joint_u', 'yaskawa_joint_r', 'yaskawa_joint_b', 'yaskawa_joint_t','yaskawa_pivoting_joint'],
        'description': 'joint_names'},
    {'name': 'realtime_priority',  'default': 0, 'description': 'realtime priority'},
    {'name': 'joint_vel_limits_robot1', 'default': [1.7104, 1.7104, 1.7453, 2.2689, 2.4435, 3.1416, 3.1416,100.0], 'description': 'robot 1 joint vel limits'},
    {'name': 'joint_vel_limits_robot2', 'default': [3.50, 3.50, 3.50, 3.50, 3.50, 4.015, 6.11, 100.0], 'description': 'robot 2 joint vel limits'},
    {'name': 'b1Tb2', 'default': [1.63, 0.0, 0.0,0.7071,0,0,0.7071],
        'description': 'transformation between robot1 base and robot2 base'}, # x y z qw qx qy qz
    {'name': 'bTb1', 'default': [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        'description': 'transformation between base frame and robot1 base frame'}, # x y z qw qx qy qz
    {'name': 'robot1_prefix', 'default': 'robot1', 'description': 'robot1_prefix'},
    {'name': 'robot2_prefix', 'default': 'robot2', 'description': 'robot2_prefix'},
    {'name': 'hold_robots_relative_pose', 'default': True, 'description': 'True if you want to hold the robots orientation'},
    {'name': 'base_frame_name', 'default': 'world', 'description': 'base_frame_name'},
    {'name': 'q1_desired', 'default': [1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'description': '(secondary task) desired joint positions for robot 1'},
    {'name': 'q2_desired', 'default': [-1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'description': '(secondary task) desired joint positions for robot 2'},
    {'name': 'secondary_task_weight', 'default': 0.0001, 'description': 'secondary_task_weight'}
]

configurable_object_pose_control_node_parameters = [
    {'name': 'sample_time',  'default': 0.02, 'description': 'sample_time'},
    {'name': 'control_gain_diag_vector', 'default': [0.5,0.5,0.5,0.5,0.5,0.5], 'description': 'elements of the control gain diagonal matrix'},
]

configurable_internal_force_control_node_parameters = [
    {'name': 'sample_time',  'default': 0.02, 'description': 'sample_time'},
    {'name': 'force_control_gain_diag_vector', 'default': [0.0002,0.0002,0.0002,0.0002,0.0002,0.0002], 'description': 'elements of the control gain diagonal matrix'},
    {'name': 'selection_matrix_diag_vector', 'default': [1,1,1,1,1,0], 'description': 'elements of the selection matrix diagonal matrix'}
]

#[0.1,0.1,0.1,0.1,0.1,0.1]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]


def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def convert_parameters(parameters):
    return {param['name']: param['default'] for param in parameters}


def generate_launch_description():
    
    robot1_namespace = 'robot1'
    robot2_namespace = 'robot2'
    default_rviz_config_path = PathJoinSubstitution([FindPackageShare('dual_arm_control'), 'rviz', 'two_robots.rviz'])

    simulation = 'true'
    robot1_sample_time = 0.005
    robot2_sample_time = 0.02

    if simulation == 'true':
        simulation_not = 'false'
    else:
        simulation_not = 'true'
    
        
    ld = LaunchDescription()
    
    # Declare launch arguments
    
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)
    
    

    # robot 1 urdf launch
    robot1_launch_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('dual_arm_control'),
                    'launch',
                    'sim_iiwa.launch.py'
                ])
            ]),
            launch_arguments={
                'gui': simulation,
                'use_joint_state_publisher': simulation,
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
                    'sim_yaskawa.launch.py'
                ])
            ]),
            launch_arguments={
                'gui': simulation,
                'use_joint_state_publisher': simulation,
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
                    name="fkine",
                    parameters=[convert_parameters(configurable_fkine_parameters_robot1)],
                    extra_arguments=[{'use_intra_process_comms': True}]
                    ),
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot2_namespace,
                    plugin='uclv_robot_ros::FKineNode',
                    name="fkine",
                    parameters=[convert_parameters(configurable_fkine_parameters_robot2)],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot1_namespace,
                    plugin='uclv_robot_ros::JointTrajectoryNode',
                    name="joint_trajectory",
                    condition=UnlessCondition(simulation_not),
                    # parameters=[
                    # ],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot2_namespace,
                    plugin='uclv_robot_ros::JointTrajectoryNode',
                    name="joint_trajectory",
                    # parameters=[
                    # ],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot1_namespace,
                    plugin='uclv_robot_ros::JointIntegrator',
                    name="joint_integrator",
                    remappings=[('joint_vel_states', 'command/joint_vel_states'),
                                ('integrator/joint_states', 'command/joint_states')
                                ],
                    parameters=[
                        {'integrator.sampling_time': robot1_sample_time}
                    ],
                    condition=UnlessCondition(simulation_not),
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot2_namespace,
                    plugin='uclv_robot_ros::JointIntegrator',
                    name="joint_integrator",
                    remappings=[('joint_vel_states', 'command/joint_vel_states'),
                                ('integrator/joint_states', 'command/joint_states')
                                ],
                    parameters=[
                        # {'realtime_priority': 97},
                        {'integrator.sampling_time': robot2_sample_time}
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot1_namespace,
                    parameters=[convert_parameters(configurable_inv_diffkine_parameters_robot1)],
                    plugin='uclv_robot_ros::InverseDifferentialKinematics',
                    name="clik",
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot2_namespace,
                    parameters=[convert_parameters(configurable_inv_diffkine_parameters_robot2)],
                    plugin='uclv_robot_ros::InverseDifferentialKinematics',
                    name="clik",
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot1_namespace,
                    plugin='uclv_robot_ros::CartesianTrajectoryNode',
                    name="cartesian_trajectory",
                    # parameters=[
                    # ],
                    remappings=[('cartesian_traj/twist', 'twist')],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot2_namespace,
                    plugin='uclv_robot_ros::CartesianTrajectoryNode',
                    name="cartesian_trajectory",
                    # parameters=[
                    # ],
                    remappings=[('cartesian_traj/twist', 'twist')],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='uclv_robot_ros',
                    plugin='uclv_robot_ros::CartesianTrajectoryNode',
                    name="cartesian_trajectory_cooperative",
                    # parameters=[
                    # ],
                    remappings=[('cartesian_traj/pose', 'desired_object_pose')],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode( # add another tracjectory generator to publush directly on the absolute twist
                    package='uclv_robot_ros',
                    namespace="cooperative_utils",
                    plugin='uclv_robot_ros::CartesianTrajectoryNode',
                    name="cartesian_trajectory_cooperative",
                    # parameters=[
                    # ],
                    remappings=[('cartesian_traj/twist', '/absolute_twist')],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot1_namespace,
                    plugin='uclv_robot_ros::FKineNode',
                    name="fkine_camera_node",
                    parameters=[convert_parameters(configurable_fkine_parameters_robot1_camera)],
                    remappings=[('fkine', 'fkine_camera'), ('set_end_effector', 'camera/set_end_effector')],
                    extra_arguments=[{'use_intra_process_comms': True}]
                    ),
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot2_namespace,
                    plugin='uclv_robot_ros::FKineNode',
                    name="fkine_camera_node",
                    parameters=[convert_parameters(configurable_fkine_parameters_robot2_camera)],
                    remappings=[('fkine', 'fkine_camera'),('set_end_effector', 'camera/set_end_effector')],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot1_namespace,
                    plugin='uclv_robot_ros::FKineNode',
                    name="fkine_pre_pivoting_joint_node",
                    parameters=[convert_parameters(configurable_fkine_parameters_robot1_nopivoting)],
                    remappings=[('fkine', 'fkine_pre_pivoting_joint'),('set_end_effector', 'pre_pivoting/set_end_effector')],
                    extra_arguments=[{'use_intra_process_comms': True}]
                    ),
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot2_namespace,
                    plugin='uclv_robot_ros::FKineNode',
                    name="fkine_pre_pivoting_joint_node",
                    parameters=[convert_parameters(configurable_fkine_parameters_robot2_nopivoting)],
                    remappings=[('fkine', 'fkine_pre_pivoting_joint'),('set_end_effector', 'pre_pivoting/set_end_effector')],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot1_namespace,
                    plugin='uclv_robot_ros::FKineNode',
                    name="fkine_tactile_sensor_node",
                    parameters=[convert_parameters(configurable_fkine_parameters_robot1_tactile_sensor)],
                    remappings=[('fkine', 'fkine_tactile_sensor'),('set_end_effector', 'tactile_sensor/set_end_effector')],
                    extra_arguments=[{'use_intra_process_comms': True}]
                    ),
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot2_namespace,
                    plugin='uclv_robot_ros::FKineNode',
                    name="fkine_tactile_sensor_node",
                    parameters=[convert_parameters(configurable_fkine_parameters_robot2_tactile_sensor)],
                    remappings=[('fkine', 'fkine_tactile_sensor'),('set_end_effector', 'tactile_sensor/set_end_effector')],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                #  ComposableNode(
                #     package='uclv_systems_ros',
                #     namespace=robot1_namespace,
                #     plugin='uclv_systems_ros::FilterJointStateNode',
                #     name='joint_state_filter_1',
                #     remappings=[
                #         ('msg_in', 'joint_states'),
                #         ('msg_out', 'joint_states_filtered'),
                #     ],
                #     parameters=[
                #         {
                #             'sample_time': robot1_sample_time, 
                #             'cut_frequency': 1.0
                #         }
                #     ],
                #     extra_arguments=[{'use_intra_process_comms': True}],
                #     ),
                # ComposableNode(
                #     package='uclv_systems_ros',
                #     namespace=robot2_namespace,
                #     plugin='uclv_systems_ros::FilterJointStateNode',
                #     name='joint_state_filter_2',
                #     remappings=[
                #         ('msg_in', 'joint_states'),
                #         ('msg_out', 'joint_states_filtered'),
                #     ],
                #     parameters=[
                #         {
                #             'sample_time': robot2_sample_time, 
                #             'cut_frequency': 1.0
                #         }
                #     ],
                #     extra_arguments=[{'use_intra_process_comms': True}],
                #     ),
                # ComposableNode(
                #     package='uclv_robot_ros',
                #     namespace=robot1_namespace,
                #     plugin='uclv_robot_ros::FkineTwist',
                #     name="fkine_twist_1",
                #     remappings=[('joint_states', 'joint_states_filtered')],
                #     extra_arguments=[{'use_intra_process_comms': True}]
                # ),
                # ComposableNode(
                #     package='uclv_robot_ros',
                #     namespace=robot2_namespace,
                #     plugin='uclv_robot_ros::FkineTwist',
                #     name="fkine_twist_2",
                #     remappings=[('joint_states', 'joint_states_filtered')],
                #     extra_arguments=[{'use_intra_process_comms': True}]
                # ),
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot1_namespace,
                    plugin='uclv_robot_ros::FKineNode',
                    name="fkine_command_1",
                    parameters=[convert_parameters(configurable_fkine_parameters_robot1)],
                    remappings=[('joint_states', 'command/joint_states'), ('fkine', 'command/fkine'),('jacobian', 'command/jacobian')],
                    extra_arguments=[{'use_intra_process_comms': True}]
                    ),
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot2_namespace,
                    plugin='uclv_robot_ros::FKineNode',
                    name="fkine_command_2",
                    parameters=[convert_parameters(configurable_fkine_parameters_robot2)],
                    remappings=[('joint_states', 'command/joint_states'), ('fkine', 'command/fkine'),('jacobian', 'command/jacobian')],
                    extra_arguments=[{'use_intra_process_comms': True}]),  
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot1_namespace,
                    plugin='uclv_robot_ros::FkineTwist',
                    name="fkine_twist_command_1",
                    remappings=[('joint_states', 'command/joint_vel_states'), ('fkine_twist', 'command/fkine_twist'),('jacobian', 'command/jacobian')],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),   
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot2_namespace,
                    plugin='uclv_robot_ros::FkineTwist',
                    name="fkine_twist_command_2",
                    remappings=[('joint_states', 'command/joint_vel_states'), ('fkine_twist', 'command/fkine_twist'),('jacobian', 'command/jacobian')],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),                      
        ],
        output='both',
    ))
    
    # launch cooperative robots node - to make as a composable node
    ld.add_action(Node(
        package='dual_arm_control',
        executable='cooperative_robots_server',
        output='screen',
        parameters=[convert_parameters(configurable_cooperative_robots_parameters)]
    ))

    ld.add_action(Node(
        package='dual_arm_control',
        executable='object_pose_control_node',
        output='screen',
        parameters=[convert_parameters(configurable_object_pose_control_node_parameters)],
        remappings=[('/object_pose', '/ekf/object_pose')]
    ))
    ld.add_action(Node(
        package='dual_arm_control',
        executable='internal_force_control_node',
        output='screen',
        parameters=[convert_parameters(configurable_internal_force_control_node_parameters)],
        remappings=[('/object_pose', '/ekf/object_pose'),
                    ('robot1/wrench','/iiwa/wsg50/wrench_rotated_after_pivoting'),
                    ('robot2/wrench','/yaskawa/wsg32/wrench_rotated_after_pivoting')]
    ))

    ld.add_action(Node(
        package='dual_arm_control',
        namespace=robot2_namespace,
        executable='joint_mux',
        name='joint_mux_robot2',
        output='screen',
        remappings=[('joint_states_1','/motoman/joint_states'), ('joint_states_2','pivoting_joint')],
        parameters=[{'joint_names': ['yaskawa_joint_s', 'yaskawa_joint_l','yaskawa_joint_e','yaskawa_joint_u', 'yaskawa_joint_r', 'yaskawa_joint_b', 'yaskawa_joint_t','yaskawa_pivoting_joint']}],
        condition=UnlessCondition(simulation)
    ))  
    ld.add_action(Node(
        package='dual_arm_control',
        namespace=robot1_namespace,
        executable='joint_mux',
        name='joint_mux_robot1',
        output='screen',
        remappings=[('joint_states_1','/lbr/joint_states'), ('joint_states_2','pivoting_joint')],
        parameters=[{'joint_names': ['iiwa_joint1', 'iiwa_joint2', 'iiwa_joint3', 'iiwa_joint4', 'iiwa_joint5', 'iiwa_joint6', 'iiwa_joint7','iiwa_pivoting_joint']}],
        condition=UnlessCondition(simulation)
    ))  
    ld.add_action(Node(
        package='dual_arm_control',
        namespace=robot2_namespace,
        executable='joint_demux',
        name='joint_demux_robot2',
        output='screen',
        parameters=[{'split_index': 7}],
        remappings=[('joint_states_1','/motoman/joint_ll_control'), ('joint_states_2','pivoting_joint'), ('joint_states','command/joint_states')],
        condition=UnlessCondition(simulation)
    ))  
    ld.add_action(Node(
        package='dual_arm_control',
        namespace=robot1_namespace,
        executable='joint_demux',
        name='joint_demux_robot1',
        output='screen',
        parameters=[{'split_index': 7}],
        remappings=[('joint_states_1','/lbr/command/joint_states'), ('joint_states_2','pivoting_joint'),('joint_states','command/joint_states')],
        condition=UnlessCondition(simulation)
    ))  


    # rviz
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    ))



    # # Transform between robots
    # ld.add_action(
    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         arguments = ['1.63626', '-0.006', '0.013', '1.5282496', '-0.0356366', '0.0205613', 'world', 'yaskawa_base_link']
    #     )
    # )
    

    return ld
