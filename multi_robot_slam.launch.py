#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # SLAM Toolbox pour TB3_1
    slam_tb3_1 = GroupAction([
        PushRosNamespace('TB3_1'),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'odom_frame': 'TB3_1/odom',
                    'base_frame': 'TB3_1/base_footprint',
                    'map_frame': 'TB3_1/map',
                    'scan_topic': '/TB3_1/scan',
                }
            ],
            remappings=[
                ('/scan', '/TB3_1/scan'),
                ('/map', '/TB3_1/map'),
                ('/map_metadata', '/TB3_1/map_metadata'),
            ]
        )
    ])
    
    # SLAM Toolbox pour TB3_2
    slam_tb3_2 = GroupAction([
        PushRosNamespace('TB3_2'),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'odom_frame': 'TB3_2/odom',
                    'base_frame': 'TB3_2/base_footprint',
                    'map_frame': 'TB3_2/map',
                    'scan_topic': '/TB3_2/scan',
                }
            ],
            remappings=[
                ('/scan', '/TB3_2/scan'),
                ('/map', '/TB3_2/map'),
                ('/map_metadata', '/TB3_2/map_metadata'),
            ]
        )
    ])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        slam_tb3_1,
        slam_tb3_2,
    ])
