#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    slam_tb3_1 = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        name='slam_gmapping',
        namespace='TB3_1',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'odom_frame': 'TB3_1/odom',
                'base_frame': 'TB3_1/base_footprint',
                'map_frame': 'TB3_1/map',
                # GMapping specific parameters
                'maxUrange': 3.5,  # Ajuste à la portée réelle du lidar (ex: 3.5m)
                'maxRange': 8.0,
                'particles': 80,   # Plus robuste (ex: 80 ou 100)
                'linearUpdate': 0.2,  # Correction plus fréquente (ex: 0.2m)
                'angularUpdate': 0.1,  # Correction plus fréquente (ex: 0.1 rad)
                'temporalUpdate': 0.5,
                'xmin': -10.0,
                'ymin': -10.0,
                'xmax': 10.0,
                'ymax': 10.0,
                'delta': 0.02,  # Carte plus fine (ex: 0.02m)
                'transform_publish_period': 0.05,
                'map_update_interval': 0.5,
                'minimumScore': 200,  # Évite les fausses associations de scans
            }
        ],
        remappings=[
            # Remap relatif scan vers absolu
            ('scan', 'scan_filtered'),
            # Garder tf global (pas de remap = utilise /tf)
        ]
    )
    
    # SLAM GMapping pour TB3_2
    slam_tb3_2 = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        name='slam_gmapping',
        namespace='TB3_2',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'odom_frame': 'TB3_2/odom',
                'base_frame': 'TB3_2/base_footprint',
                'map_frame': 'TB3_2/map',
                # GMapping specific parameters
                'maxUrange': 3.5,
                'maxRange': 8.0,
                'particles': 80,
                'linearUpdate': 0.2,
                'angularUpdate': 0.1,
                'temporalUpdate': 0.5,
                'xmin': -10.0,
                'ymin': -10.0,
                'xmax': 10.0,
                'ymax': 10.0,
                'delta': 0.02,
                'transform_publish_period': 0.05,
                'map_update_interval': 0.5,
                'minimumScore': 200,
            }
        ],
        remappings=[
            ('scan', 'scan_filtered'),
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        slam_tb3_1,
        slam_tb3_2,
    ])
