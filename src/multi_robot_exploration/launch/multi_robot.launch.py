#!/usr/bin/env python3

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
import xml.etree.ElementTree as ET_local
from launch_ros.actions import Node
from launch.substitutions import PythonExpression
import subprocess


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    number_of_robots = 2
    namespace = 'TB3'
    #pose = [[-3, 4.0, 2], [-4.5, 3.0, 2]]
    pose = [[-3.7, 4.0, 2], [-4.0, 4.0, 2]]
    model_folder = 'turtlebot3_burger_rgbd'
    urdf_path = os.path.join(
        os.path.expanduser('~/ros2_humble/models'),
        model_folder,
        'model.sdf'
    )
    save_path = os.path.expanduser('~/ros2_humble/tmp')
    os.makedirs(save_path, exist_ok=True)
    save_path = os.path.join(save_path, 'tmp')

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    world = LaunchConfiguration('world', default=os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_house.world'
    ))

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd_list = []

    # Évaluer le xacro pour obtenir le URDF complet avec la caméra
    urdf_file_name = 'turtlebot3_burgerv2.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        urdf_file_name)
    
    # Exécuter xacro pour évaluer les macros
    try:
        xacro_result = subprocess.run(
            ['xacro', urdf_path],
            capture_output=True,
            text=True,
            check=True
        )
        robot_desc = xacro_result.stdout
    except subprocess.CalledProcessError as e:
        print(f"Erreur lors de l'évaluation du xacro: {e}")
        print(f"Utilisation du URDF brut sans évaluation xacro")
        with open(urdf_path, 'r') as infp:
            robot_desc = infp.read()
    except FileNotFoundError:
        print("xacro non trouvé, utilisation du URDF brut")
        with open(urdf_path, 'r') as infp:
            robot_desc = infp.read()

    for count in range(number_of_robots):
        
        robot_state_publisher_cmd_list.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=f'{namespace}_{count+1}',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_description': robot_desc,
                    'frame_prefix': PythonExpression([f"'{namespace}_{count+1}/'"])
                }]
            )
        )

    spawn_turtlebot_cmd_list = []

    # Pour Gazebo, utiliser le modèle SDF officiel (contient plugins de base)
    # et ajouter manuellement les plugins de la caméra RealSense D435i
    sdf_model_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        'turtlebot3_burger',
        'model.sdf'
    )
    
    for count in range(number_of_robots):
        # Lire le SDF officiel
        with open(sdf_model_path, 'r') as f:
            sdf_content = f.read()
        
        # Ajouter le namespace aux topics ROS
        sdf_content = sdf_content.replace('<namespace></namespace>', f'<namespace>{namespace}_{count+1}</namespace>')
        
        # Préfixer tous les noms de frames avec le namespace
        # Parse XML pour modifier proprement
        root = ET_local.fromstring(sdf_content)
        
        # Modifier tous les frame_name pour ajouter le namespace
        for frame_name_tag in root.iter('frame_name'):
            if frame_name_tag.text and not frame_name_tag.text.startswith(f'{namespace}_{count+1}/'):
                frame_name_tag.text = f'{namespace}_{count+1}/{frame_name_tag.text}'
        
        # Modifier odometry_frame et robot_base_frame pour les plugins
        for tag_name in ['odometry_frame', 'robot_base_frame']:
            for tag in root.iter(tag_name):
                if tag.text and not tag.text.startswith(f'{namespace}_{count+1}/'):
                    tag.text = f'{namespace}_{count+1}/{tag.text}'
        
        # Reconvertir en string
        sdf_content = ET_local.tostring(root, encoding='unicode')
        
        # Ajouter le plugin de la caméra RealSense D435i avant la balise </model>
        camera_plugin = f'''
    <link name="camera_link">
      <pose>0.073 0 0.094 0 0 0</pose>
      <inertial>
        <mass>0.072</mass>
        <inertia>
          <ixx>0.000001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.000001</iyy>
          <iyz>0</iyz>
          <izz>0.000001</izz>
        </inertia>
      </inertial>
      <sensor name="camera_depth" type="depth">
        <update_rate>30</update_rate>
        <camera>
          <horizontal_fov>1.5184</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>10</far>
          </clip>
          <depth_camera>
            <clip>
              <near>0.1</near>
              <far>10</far>
            </clip>
          </depth_camera>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace>{namespace}_{count+1}</namespace>
            <remapping>camera_depth/image_raw:=camera_depth/image_raw</remapping>
            <remapping>camera_depth/image_depth:=camera_depth/image_rect_raw</remapping>
            <remapping>camera_depth/camera_info:=camera_depth/camera_info</remapping>
            <remapping>camera_depth/points:=camera_depth/points</remapping>
          </ros>
          <camera_name>camera_depth</camera_name>
          <frame_name>{namespace}_{count+1}/camera_rgb_optical_frame_depth_optical_frame</frame_name>
          <hack_baseline>0.07</hack_baseline>
          <min_depth>0.1</min_depth>
          <max_depth>10.0</max_depth>
        </plugin>
      </sensor>
    </link>
    <joint name="camera_joint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
    </joint>
'''
        
        # Insérer le plugin caméra avant </model>
        sdf_content = sdf_content.replace('</model>', camera_plugin + '\n  </model>')
        
        # Sauvegarder le SDF modifié
        with open(f'{save_path}{count+1}.sdf', 'w') as f:
            f.write(sdf_content)

        spawn_turtlebot_cmd_list.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('multi_robot_exploration'), 'launch', 'multi_spawn_turtlebot3_with_yaw.launch.py')
                ),
                launch_arguments={
                        'x_pose': str(pose[count][0]),
                        'y_pose': str(pose[count][1]),
                        'yaw_pose': str(pose[count][2]),
                        'robot_name': f'{TURTLEBOT3_MODEL}_{count+1}',
                        'namespace': f'{namespace}_{count+1}',
                        'sdf_path': f'{save_path}{count+1}.sdf'
                }.items()
            )
        )

    ld = LaunchDescription()
    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(RegisterEventHandler(
        OnShutdown(
            on_shutdown=lambda event,
            context: [os.remove(f'{save_path}{count+1}.sdf') for count in range(number_of_robots)]
        )
    ))
    for count, spawn_turtlebot_cmd in enumerate(spawn_turtlebot_cmd_list, start=1):
        ld.add_action(GroupAction([PushRosNamespace(f'{namespace}_{count}'),
                                  robot_state_publisher_cmd_list[count-1],
                                  spawn_turtlebot_cmd]))

    return ld
