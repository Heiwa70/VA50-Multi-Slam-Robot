#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value='true', description='Use simulator time'
    )

    package_dir = get_package_share_directory('multi_robot_exploration')
    nav_launch_dir = os.path.join(package_dir, 'launch', 'nav2_bringup')
    
    default_bt_xml = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees', 'navigate_w_replanning_and_recovery.xml'
    )

    robots = {
        'TB3_1': os.path.join(package_dir, 'params', 'nav2_params_tb3_1.yaml'),
        'TB3_2': os.path.join(package_dir, 'params', 'nav2_params_tb3_2.yaml'),
    }

    ld.add_action(declare_use_sim_time)

    for robot_name, params_file in robots.items():
        bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'slam': 'False',
                'loc' : 'False',
                'namespace': robot_name,
                'use_namespace': 'True',
                'map': '',
                'map_server': 'False',
                'params_file': params_file,
                'default_bt_xml_filename': default_bt_xml,
                'autostart': 'true',
                'use_sim_time': 'true',
            }.items()
        )
        ld.add_action(bringup_cmd)

    return ld
