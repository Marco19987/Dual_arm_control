from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes, Node

def generate_launch_description():

    robot1_camera = Node(
            package='dual_arm_control',  # Replace with the actual package name
            executable='tf_publisher_from_topic',
            name='tf_publisher_from_topic_robot1_camera',
            output='screen',
            parameters=[{'source_frame': 'world', 'target_frame' : 'robot1_color_optical_frame'}] ,
            remappings=[('/pose_topic', '/robot1/fkine_camera')]
        )
    
    robot2_camera = Node(
            package='dual_arm_control',  # Replace with the actual package name
            executable='tf_publisher_from_topic',
            name='tf_publisher_from_topic_robot2_camera',
            output='screen',
            parameters=[{'source_frame': 'yaskawa_base_link', 'target_frame' : 'robot2_color_optical_frame'}] ,
            remappings=[('/pose_topic', '/robot2/fkine_camera')]
        )
    
    camera1_camera2_breadboard = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.425', '0.05', '0.00', '3.14', '0', '0', 'camera_1_link', 'camera_2_link']
        )
    # Transform between robots
    b1Tb2_pub =  Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['1.6', '-0.006', '0.013', '1.57', '-0.0', '0.0', 'world', 'yaskawa_base_link']
        )
    
    b1Tb2_estimated = Node(
            package='dual_arm_control',  # Replace with the actual package name
            executable='tf_publisher_from_topic',
            name='tf_publisher_from_topic_b1Tb2_estimated',
            output='screen',
            parameters=[{'source_frame': 'world', 'target_frame' : 'yaskawa_base_link'}] ,
            remappings=[('/pose_topic', '/ekf/b1Tb2_filtered')]
        )
    
    object_pose = Node(
            package='dual_arm_control',  # Replace with the actual package name
            executable='tf_publisher_from_topic',
            name='tf_publisher_from_topic_object_pose',
            output='screen',
            parameters=[{'source_frame': 'world', 'target_frame' : 'object_pose'}] ,
            remappings=[('/pose_topic', '/ekf/object_pose')]
        )
    
    robot1_tactile = Node(
            package='dual_arm_control',  # Replace with the actual package name
            executable='tf_publisher_from_topic',
            name='tf_publisher_from_topic_robot1_camera',
            output='screen',
            parameters=[{'source_frame': 'world', 'target_frame' : 'tactile_sensor_robot1'}] ,
            remappings=[('/pose_topic', '/robot1/fkine_tactile_sensor')]
        )
    
    robot2_tactile = Node(
            package='dual_arm_control',  # Replace with the actual package name
            executable='tf_publisher_from_topic',
            name='tf_publisher_from_topic_robot2_camera',
            output='screen',
            parameters=[{'source_frame': 'yaskawa_base_link', 'target_frame' : 'tactile_sensor_robot2'}] ,
            remappings=[('/pose_topic', '/robot2/fkine_tactile_sensor')]
        )
    
    # pivoting_T_tactile_sensor_container = ComposableNodeContainer(
    #     name='pivoting_T_tactile_sensor_container',
    #     namespace='rotation',
    #     package='rclcpp_components',
    #     executable='component_container',
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package='uclv_msgs_transform',
    #             plugin='uclv_msgs_transform::RotatePoseStampedNode',
    #             name='pivoting_T_tactile_sensor_node',
    #             namespace='robot1',
    #             remappings=[
    #                 ('msg_in', 'fkine_tactile_sensor'),
    #                 ('msg_out', 'pivoting_T_tactile_sensor'),
    #                 ('pose', 'fkine')
    #             ],
    #             parameters=[
    #                 {'inverse_rotation': True}
    #             ]
    #         )
    #     ],
    #     output='screen'
    # )


    rotation_nodes_container = LoadComposableNodes(
        target_container='robot_container',
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
                     #'msg_frame_id': 'iiwa_pivoting_link',
                     'pose_frame_id': 'tactile_sensor_robot1_after_pivoting'
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
                     #'msg_frame_id': 'iiwa_pivoting_link',
                     'pose_frame_id': 'tactile_sensor_robot2_after_pivoting'
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


    # composable_node = Node(
    #     package='uclv_msgs_transform',
    #     executable='RotateWrenchStampedNode',
    #     name="contact2pusher_wrench_node",
    #     output='screen',
    #     remappings=[
    #         ('msg_in', "/fixed_wrench"),
    #         ('msg_out', "/force_measure"),
    #         ("pose", "/contact_pusher_topic")
    #     ],
    #     parameters=[
    #         {"inverse_rotation": False}
    #     ],
    #     extra_arguments=[{'use_intra_process_comms': True}]
    # )

    return LaunchDescription([
        # camera1_camera2_breadboard,
        #b1Tb2_pub,
        robot1_camera,
        robot2_camera,
        b1Tb2_estimated,
        object_pose,
        robot1_tactile,
        robot2_tactile,
        rotation_nodes_container
        
    ])