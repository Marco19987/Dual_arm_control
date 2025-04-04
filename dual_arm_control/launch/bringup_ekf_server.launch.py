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
                {'sample_time': 0.002},
                {'publishing_sample_time': 0.01},
                {'robot_1_prefix': 'robot1'},
                {'robot_2_prefix': 'robot2'},
                {'base_frame_name': 'world'},
                {'covariance_state_diagonal': [0.000000000001, 0.000000000001, 0.00001, 0.00001, 0.00000001, 0.00000001,
                                               0.00000001,0.00000001,0.000000000001,0.000000000001]},
                {'covariance_measure_diagonal': [0.0000001, 0.0000001, 0.0000001, 0.000000001, 0.0000000001, 0.000000001, 0.000000001,
                                                 0.001, 0.001,0.0000001,0.0000001,0.0000001,0.0000001]}
            ],
            remappings=[
                # ('/robot1/wrench', '/iiwa/wsg50/wrench_rotated_after_pivoting'),
                # ('/robot2/wrench', '/yaskawa/wsg32/wrench_rotated_after_pivoting'),
                ('/robot1/wrench', '/robot1/wrench'),
                ('/robot2/wrench', '/robot2/wrench'),
                ('/robot1/fkine', '/robot1/fkine'),
                ('/robot2/fkine', '/robot2/fkine'),
                ('/robot1/twist_fkine', '/robot1/cmd_twist'),
                ('/robot2/twist_fkine', '/robot2/cmd_twist'),
            ]
        
        )
    ])