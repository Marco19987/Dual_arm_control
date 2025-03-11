from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes, Node

def generate_launch_description():

    rotation_nodes_container = ComposableNodeContainer(
        name='rotation_wrench_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=[
            ComposableNode(
                package='uclv_msgs_transform',
                plugin='uclv_msgs_transform::TransformPoseStampedNode',
                name='pivoting_T_tactile_sensor_node',
                namespace='robot1',
                remappings=[
                    ('msg_in', 'fkine_tactile_sensor'),
                    ('msg_out', 'pivoting_T_tactile_sensor'),
                    ('pose', 'fkine')
                ],
                parameters=[
                    {'inverse_transform': True , 
                     'pose_frame_id': 'iiwa_pivoting_link'
                     }
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='uclv_msgs_transform',
                plugin='uclv_msgs_transform::RotateWrenchStampedNode',
                name='rotation_wrench_node',
                namespace='robot1',
                remappings=[
                    ('msg_in', '/iiwa/wsg50/wrench'),
                    ('msg_out', '/iiwa/wsg50/wrench_rotated_after_pivoting'),
                    ('pose', 'pivoting_T_tactile_sensor')
                ],
                parameters=[
                    {'inverse_transform': True}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='uclv_msgs_transform',
                plugin='uclv_msgs_transform::TransformPoseStampedNode',
                name='pivoting_T_tactile_sensor_node',
                namespace='robot2',
                remappings=[
                    ('msg_in', 'fkine_tactile_sensor'),
                    ('msg_out', 'pivoting_T_tactile_sensor'),
                    ('pose', 'fkine')
                ],
                parameters=[
                    {'inverse_transform': True , 
                     'pose_frame_id': 'yaskawa_pivoting_link'
                     }
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='uclv_msgs_transform',
                plugin='uclv_msgs_transform::RotateWrenchStampedNode',
                name='rotation_wrench_node',
                namespace='robot2',
                remappings=[
                    ('msg_in', '/yaskawa/wsg32/wrench'),
                    ('msg_out', '/yaskawa/wsg32/wrench_rotated_after_pivoting'),
                    ('pose', 'pivoting_T_tactile_sensor')
                ],
                parameters=[
                    {'inverse_transform': True}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ]
    )



    return LaunchDescription([
        rotation_nodes_container
        
    ])