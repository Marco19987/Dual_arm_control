from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes, Node

def generate_launch_description():

    ld = LaunchDescription()

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

    

    return ld