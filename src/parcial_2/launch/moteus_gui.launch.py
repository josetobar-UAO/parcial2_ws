from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    motor_ids = [1, 2, 3]
    joint_names = ['cadera', 'rodilla', 'rueda']   # cámbialos por los tuyos

    return LaunchDescription([
        Node(
            package='moteus_control',
            executable='moteus_node',
            namespace='moteus',
            output='screen',
            parameters=[{
                'ids': motor_ids,
                'query_period': 0.02,
                'initial_stop': True,
                'args': ['--force-transport', 'fdcanusb'],
            }]
        ),

        Node(
            package='parcial_2',
            executable='moteus_joint_bridge',
            output='screen',
            parameters=[{
                'motor_ids': motor_ids,
                'joint_names': joint_names,
                'gear_ratios': [1.0, 1.0, 1.0],   # ajusta si hay reducción
                'signs': [1.0, 1.0, 1.0],         # usa -1.0 si algún eje va invertido
                'zero_deg': [0.0, 0.0, 0.0],      # offsets mecánicos
                'moteus_ns': 'moteus',
                'publish_rate': 50.0,
            }]
        ),

        Node(
            package='parcial_2',
            executable='gui_moteus_angles',
            output='screen',
            parameters=[{
                'joint_names': joint_names,
            }]
        ),
    ])

