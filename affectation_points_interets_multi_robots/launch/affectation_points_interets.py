from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='affectation_points_interets_multi_robots',
            executable='affectation_points_interets',
            name='affectation_points_interets',
            parameters=[{'distance': 0.2}],
        ),
    ])

