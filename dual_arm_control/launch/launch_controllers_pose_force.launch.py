from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes, Node


configurable_object_pose_control_node_parameters = [
    {'name': 'sample_time',  'default': 0.02, 'description': 'sample_time'},
    {'name': 'control_gain_diag_vector', 'default': [0.5,0.5,0.5,0.5,0.5,0.5], 'description': 'elements of the control gain diagonal matrix'},
]

configurable_internal_force_control_node_parameters = [
    {'name': 'sample_time',  'default': 0.01, 'description': 'sample_time'},
    {'name': 'force_control_gain_diag_vector', 'default': [0.0004,0.0004,0.0004,0.0004,0.0004,0.0004], 'description': 'elements of the control gain diagonal matrix'},
    {'name': 'selection_matrix_diag_vector', 'default': [1,1,1,1,1,0], 'description': 'elements of the selection matrix diagonal matrix'}
]

def convert_parameters(parameters):
    return {param['name']: param['default'] for param in parameters}


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