#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    yaw_pose = LaunchConfiguration('yaw_pose', default='0.0')
    robot_name = LaunchConfiguration('robot_name', default=TURTLEBOT3_MODEL)
    namespace = LaunchConfiguration('namespace', default='')
    sdf_path = LaunchConfiguration('sdf_path', default='')

    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='X position of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Y position of the robot')

    declare_yaw_position_cmd = DeclareLaunchArgument(
        'yaw_pose', default_value='0.0',
        description='Yaw orientation of the robot (radians)')

    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-file', sdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-Y', yaw_pose,
            '-robot_namespace', namespace
        ],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_yaw_position_cmd)
    ld.add_action(start_gazebo_ros_spawner_cmd)
    return ld
