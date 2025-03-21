from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dual_arm_control',
            executable='ekf_server',
            name='ekf_server',
            output='screen',
            parameters=[
                {'sample_time': 0.03},
                {'robot_1_prefix': 'robot1'},
                {'robot_2_prefix': 'robot2'},
                {'base_frame_name': 'world'},
                {'alpha_occlusion': 1.15},
                {'saturation_occlusion': 15.0},
                {'covariance_state_diagonal': [0.000001, 0.0000001, 0.00001, 0.00001, 0.0000000001, 0.00000000000001,0.001,0.001]},
                {'covariance_measure_diagonal': [0.0000001, 0.0000001, 0.0000001, 0.000000001, 0.0000000001, 0.000000001, 0.000000001,0.000001,0.000001]}
            ],
            remappings=[
                ('/robot1/wrench', '/iiwa/wsg50/wrench_rotated_after_pivoting'),
                ('/robot2/wrench', '/yaskawa/wsg32/wrench_rotated_after_pivoting')
            ]
        
        )
    ])