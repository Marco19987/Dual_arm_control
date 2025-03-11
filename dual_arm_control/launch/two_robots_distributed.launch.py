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
    {'name': 'joints_to_exclude', 'default' : [7], 'description': 'joints to exclude from the inverse differential kinematics, provide as list of integers'},
    {'name': 'realtime_priority', 'default' : 0, 'description': 'realtime priority'}
]


configurable_inv_diffkine_parameters_robot2 = [
    {'name': 'joint_names',  'default': ['yaskawa_joint_s', 'yaskawa_joint_l','yaskawa_joint_e','yaskawa_joint_u', 'yaskawa_joint_r', 'yaskawa_joint_b', 'yaskawa_joint_t','yaskawa_pivoting_joint'],
        'description': 'joint_names'},
    {'name': 'joint_vel_limits', 'default': [3.50, 3.50, 3.50, 3.50, 3.50, 4.015, 6.11, 100.0], 'description': 'robot 2 joint vel limits'},
    {'name': 'joints_to_exclude', 'default' : [7], 'description': 'joints to exclude from the inverse differential kinematics, provide as list of integers'},
    {'name': 'realtime_priority', 'default' : 0, 'description': 'realtime priority'}
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


def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]


def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def convert_parameters(parameters):
    return {param['name']: param['default'] for param in parameters}


def generate_launch_description():
    
    robot1_namespace = 'robot1'
    robot2_namespace = 'robot2'

    sample_time_robot1 = 0.005
    sample_time_robot2 = 0.02
    sample_time_cooperative_traj = 0.005


    ld = LaunchDescription()
    
    # robots fkine nodes
    ld.add_action(ComposableNodeContainer(
        name='robot_container',
        namespace='',
        package='uclv_realtime_ros2',
        executable='component_container_isolated',
        composable_node_descriptions=[
                # ComposableNode(
                #     package='uclv_iiwa_fri',
                #     namespace='lbr',
                #     plugin='uclv_iiwa_fri::IIWAFRIDriverNode',
                #     name="fri_app",
                #     parameters=[
                #         {'fri.realtime_priority': 98},
                #         {'cmd_filter.cut_time': 0.010 },
                #         {'joint_names': ['iiwa_joint1', 'iiwa_joint2', 'iiwa_joint3', 'iiwa_joint4', 'iiwa_joint5', 'iiwa_joint6', 'iiwa_joint7']}
                #     ],
                #  extra_arguments=[{'use_intra_process_comms': True}]),
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
                    parameters=[
                        {"trajectory.rate": 1.0/sample_time_robot1},
                        {"trajectory.realtime_priority": 0}
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot2_namespace,
                    plugin='uclv_robot_ros::JointTrajectoryNode',
                    name="joint_trajectory",
                    parameters=[
                        {"trajectory.rate": 1.0/sample_time_robot2},
                        {"trajectory.realtime_priority": 0}
                    ],
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
                        {'realtime_priority': 97},
                        {'integrator.sampling_time': sample_time_robot1}
                    ],
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
                        {'realtime_priority': 97},
                        {'integrator.sampling_time': sample_time_robot2}
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
                    parameters=[
                        {"trajectory.rate": 1.0/sample_time_robot1},
                        {"trajectory.realtime_priority": 0}
                    ],
                    remappings=[('cartesian_traj/twist', 'twist')],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace=robot2_namespace,
                    plugin='uclv_robot_ros::CartesianTrajectoryNode',
                    name="cartesian_trajectory",
                    parameters=[
                        {"trajectory.rate": 1.0/sample_time_robot2},
                        {"trajectory.realtime_priority": 0}
                    ],
                    remappings=[('cartesian_traj/twist', 'twist')],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='uclv_robot_ros',
                    plugin='uclv_robot_ros::CartesianTrajectoryNode',
                    name="cartesian_trajectory_cooperative",
                    parameters=[
                        {"trajectory.rate": 1.0/sample_time_cooperative_traj},
                        {"trajectory.realtime_priority": 0}
                    ],
                    remappings=[('cartesian_traj/pose', 'desired_object_pose')],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode( # add another tracjectory generator to publush directly on the absolute twist
                    package='uclv_robot_ros',
                    namespace="cooperative_utils",
                    plugin='uclv_robot_ros::CartesianTrajectoryNode',
                    name="cartesian_trajectory_cooperative",
                    parameters=[
                        {"trajectory.rate": 1.0/sample_time_cooperative_traj},
                        {"trajectory.realtime_priority": 0}
                    ],
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
                
        ],
        output='both',
        arguments=["--use-realtime --middleware-priority 95"]
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
        namespace=robot2_namespace,
        executable='joint_mux',
        name='joint_mux_robot2',
        output='screen',
        remappings=[('joint_states_1','/motoman/joint_states'), ('joint_states_2','pivoting_joint')],
        parameters=[{'joint_names': ['yaskawa_joint_s', 'yaskawa_joint_l','yaskawa_joint_e','yaskawa_joint_u', 'yaskawa_joint_r', 'yaskawa_joint_b', 'yaskawa_joint_t','yaskawa_pivoting_joint']}],
    ))  
    ld.add_action(Node(
        package='dual_arm_control',
        namespace=robot1_namespace,
        executable='joint_mux',
        name='joint_mux_robot1',
        output='screen',
        remappings=[('joint_states_1','/lbr/joint_states'), ('joint_states_2','pivoting_joint')],
        parameters=[{'joint_names': ['iiwa_joint1', 'iiwa_joint2', 'iiwa_joint3', 'iiwa_joint4', 'iiwa_joint5', 'iiwa_joint6', 'iiwa_joint7','iiwa_pivoting_joint']}],
    ))  
    ld.add_action(Node(
        package='dual_arm_control',
        namespace=robot2_namespace,
        executable='joint_demux',
        name='joint_demux_robot2',
        output='screen',
        parameters=[{'split_index': 7}],
        remappings=[('joint_states_1','/motoman/joint_ll_control'), ('joint_states_2','pivoting_joint'), ('joint_states','command/joint_states')],
    ))  
    ld.add_action(Node(
        package='dual_arm_control',
        namespace=robot1_namespace,
        executable='joint_demux',
        name='joint_demux_robot1',
        output='screen',
        parameters=[{'split_index': 7}],
        remappings=[('joint_states_1','/lbr/command/joint_states'), ('joint_states_2','pivoting_joint'),('joint_states','command/joint_states')],
    ))  


    
    return ld
