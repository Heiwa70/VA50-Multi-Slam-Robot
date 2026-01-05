import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()
    default_config = os.path.join(
        get_package_share_directory("multirobot_map_merge"), "config", "params.yaml"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    known_init_poses = LaunchConfiguration("known_init_poses")
    config_file = LaunchConfiguration("config_file")

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )
    declare_namespace_argument = DeclareLaunchArgument(
        "namespace",
        default_value="/",
        description="Namespace for the explore node",
    )
    declare_known_init_poses_argument = DeclareLaunchArgument(
        "known_init_poses",
        default_value="True",
        description="Known initial poses of the robots. If so don't forget to declare them in the params.yaml file",
    )
    declare_config_file_argument = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to custom config file (overrides default params.yaml)",
    )

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    node = Node(
        package="multirobot_map_merge",
        name="map_merge",
        namespace=namespace,
        executable="map_merge",
        parameters=[
            config_file,
            {"use_sim_time": use_sim_time},
            {"known_init_poses": known_init_poses},
        ],
        output="screen",
        remappings=remappings,
    )
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_known_init_poses_argument)
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_config_file_argument)
    ld.add_action(node)
    return ld
