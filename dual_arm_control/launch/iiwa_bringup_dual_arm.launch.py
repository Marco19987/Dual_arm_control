from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnIncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node


from launch.actions import DeclareLaunchArgument

configurable_fkine_parameters_pusher = [
    {'name': 'publish_jacobian',  'default': 'True',
        'description': 'publish the jacobian'}
]

configurable_fkine_parameters_camera = [
    {'name': 'publish_jacobian',  'default': 'False',
        'description': 'publish the jacobian'}
]

configurable_inv_diffkine_parameters = [
    {'name': 'joint_names',  'default': "['iiwa_joint1', 'iiwa_joint2', 'iiwa_joint3', 'iiwa_joint4', 'iiwa_joint5', 'iiwa_joint6', 'iiwa_joint7']",
        'description': 'joint_names'},
    {'name': 'joint_vel_limits',  'default': "[1.7104, 1.7104, 1.7453, 2.2689, 2.4435, 3.1416, 3.1416]",
        'description': 'joint_vel_limits'}
]

configurable_integrator_diffkine_parameters = [
    {'name': 'integrator.sampling_time',  'default': "0.001",
        'description': 'sampling_time'}
]

configurable_joint_traj_diffkine_parameters = [
    {'name': 'trajectory.rate',  'default': str(1.0/0.001),
        'description': 'rate'}
]


def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]


def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    sample_time = 0.005

    twist_overall_topic = "/twist_controller"
    p_v_ee_topic = "/pusher_velocity_ee"

    for param in declare_configurable_parameters(configurable_fkine_parameters_pusher):
        ld.add_action(param)
    for param in declare_configurable_parameters(configurable_fkine_parameters_camera):
        ld.add_action(param)
    for param in declare_configurable_parameters(configurable_inv_diffkine_parameters):
        ld.add_action(param)

    ld.add_action(Node(
        package='dual_arm_control',
        namespace='robot1',
        executable='joint_demux',
        name='joint_demux_robot1',
        output='screen',
        parameters=[{'split_index': 7}],
        remappings=[('joint_states_1','/lbr/command/joint_states'), ('joint_states_2','pivoting_joint'),('joint_states','command/joint_states')],
    ))  


    ld.add_action(ComposableNodeContainer(
        name='lbr_container',
        namespace='',
        package='uclv_realtime_ros2',
        executable='component_container_isolated',
        composable_node_descriptions=[
                ComposableNode(
                    package='uclv_iiwa_fri',
                    namespace='lbr',
                    plugin='uclv_iiwa_fri::IIWAFRIDriverNode',
                    name="fri_app",
                    parameters=[
                        {'fri.realtime_priority': 98},
                        {'cmd_filter.cut_time': 0.010 },
                        {'joint_names': ['iiwa_joint1', 'iiwa_joint2', 'iiwa_joint3', 'iiwa_joint4', 'iiwa_joint5', 'iiwa_joint6', 'iiwa_joint7']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                 ComposableNode(
                    package='uclv_robot_ros',
                    namespace="robot1",
                    plugin='uclv_robot_ros::JointIntegrator',
                    name="joint_integrator",
                    remappings=[('joint_vel_states', 'command/joint_vel_states'),
                                ('integrator/joint_states', 'command/joint_states')
                                ],
                    parameters=[
                        {'realtime_priority': 97},
                        {'integrator.sampling_time': sample_time}
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='uclv_robot_ros',
                    namespace='robot1',
                    plugin='uclv_robot_ros::JointTrajectoryNode',
                    name="joint_trajectory",
                     parameters=[
                        {"trajectory.rate": 1.0/sample_time},
                        {"trajectory.realtime_priority": 97}
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                # ComposableNode(
                #     package='uclv_robot_ros',
                #     namespace='lbr',
                #     plugin='uclv_robot_ros::FKineNode',
                #     name="fkine_pusher",
                #     parameters=[set_configurable_parameters(
                #         configurable_fkine_parameters_pusher)],
                #     remappings=[
                #             ('joint_states', 'commanded_joint_states'),
                #             ('/lbr/fkine', '/lbr/world_pusher'),
                #             ('/lbr/set_end_effector', '/lbr/set_end_effector_pusher'),
                #         ], 
                #     extra_arguments=[{'use_intra_process_comms': True}]),
                # ComposableNode(
                #     package='uclv_robot_ros',
                #     namespace='lbr',
                #     plugin='uclv_robot_ros::FKineNode',
                #     name="fkine_camera",
                #     parameters=[set_configurable_parameters(
                #         configurable_fkine_parameters_camera)],
                #     remappings=[
                #             ('joint_states', 'commanded_joint_states'),
                #             ('/lbr/fkine', '/lbr/world_camera'),
                #             ('/lbr/jacobian', '/lbr/jacobian_camera'),
                #             ('/lbr/set_end_effector', '/lbr/set_end_effector_camera'),
                #         ], 
                #     extra_arguments=[{'use_intra_process_comms': True}]),
                # ComposableNode(
                #     package='uclv_robot_ros',
                #     namespace='lbr',
                #     plugin='uclv_robot_ros::JointTrajectoryNode',
                #     name="",
                #     parameters=[
                #         {"trajectory.rate": 1.0/sample_time},
                #         {"trajectory.realtime_priority": 97}
                #     ],
                #     extra_arguments=[{'use_intra_process_comms': True}]),
                # ComposableNode(
                #     package='uclv_robot_ros',
                #     namespace='lbr',
                #     plugin='uclv_robot_ros::JointIntegrator',
                #     name="",
                #     remappings=[('joint_vel_states', 'command/joint_vel_states'),
                #                 ('integrator/joint_states', 'command/joint_states')
                #                 ],
                #     parameters=[
                #         {'realtime_priority': 97},
                #         {'integrator.sampling_time': sample_time}
                #     ],
                #     extra_arguments=[{'use_intra_process_comms': True}]),
                # ComposableNode(
                #     package='uclv_robot_ros',
                #     namespace='lbr',
                #     plugin='uclv_robot_ros::InverseDifferentialKinematics',
                #     name="",
                #     parameters=[
                #         {'realtime_priority': 97},
                #         set_configurable_parameters(configurable_inv_diffkine_parameters)
                #     ],
                #     extra_arguments=[{'use_intra_process_comms': True}]),
                # ComposableNode(
                #     package='uclv_msgs_transform',
                #     namespace='lbr',
                #     plugin='uclv_msgs_transform::RotateTwistStampedNode',
                #     name="contact2pusher_twist_node",
                #     remappings=[
                #         ('msg_in', twist_overall_topic),
                #         ('msg_out', p_v_ee_topic),
                #         ("pose","/contact_pusher_topic")
                #     ],
                #     parameters=[
                #         {"inverse_rotation": True}
                #     ],
                #     extra_arguments=[{'use_intra_process_comms': True}]),
                # ComposableNode(
                #     package='uclv_msgs_transform',
                #     namespace='lbr',
                #     plugin='uclv_msgs_transform::RotateTwistStampedNode',
                #     name="pusher2world_twist_node",
                #     remappings=[
                #         ('msg_in', p_v_ee_topic),
                #         ('msg_out', '/lbr/twist'),
                #         ("pose","/lbr/world_pusher")
                #     ],
                #     parameters=[
                #         {"inverse_rotation": False}
                #     ],
                #     extra_arguments=[{'use_intra_process_comms': True}]),
                # ComposableNode(
                #     package='uclv_msgs_transform',
                #     namespace='lbr',
                #     plugin='uclv_msgs_transform::RotateTwistStampedNode',
                #     name="slider2contact_planar_twist",
                #     remappings=[
                #         ('msg_in', '/slider_frame_planar_twist'),
                #         ('msg_out', '/contact_frame_planar_twist'),
                #         ("pose","/contact_slider_topic")
                #     ],
                #     parameters=[
                #         {"inverse_rotation": False}
                #     ],
                #     extra_arguments=[{'use_intra_process_comms': True}]),
                # ComposableNode(
                #     package='uclv_msgs_transform',
                #     namespace='lbr',
                #     plugin='uclv_msgs_transform::RotateTwistStampedNode',
                #     name="slider02slider_planar_twist",
                #     remappings=[
                #         ('msg_in', '/planar_twist'),
                #         ('msg_out', '/slider_frame_planar_twist'),
                #         ("pose","/planar_pose_stamped")
                #     ],
                #     parameters=[
                #         {"inverse_rotation": True}
                #     ],
                #     extra_arguments=[{'use_intra_process_comms': True}]),
                # ComposableNode(
                #     package='uclv_msgs_transform',
                #     namespace='lbr',
                #     plugin='uclv_msgs_transform::RotateWrenchStampedNode',
                #     name="contact2pusher_wrench_node",
                #     remappings=[
                #         ('msg_in', "/fixed_wrench"),
                #         ('msg_out', "/force_measure"),
                #         ("pose","/contact_pusher_topic")
                #     ],
                #     parameters=[
                #         {"inverse_rotation": False}
                #     ],
                #     extra_arguments=[{'use_intra_process_comms': True}]),
        ],
        output='both',
        arguments=["--use-realtime --middleware-priority 95"]
    ))
    print('end')

    return ld
