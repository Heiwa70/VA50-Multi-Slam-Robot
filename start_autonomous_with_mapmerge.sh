#!/bin/bash

echo "========================================================="
echo "🤖 Multi-Robot Exploration with Map Merge 🗺️"
echo "========================================================="
echo ""
echo "Configuration:"
echo "  • Exploration: autonomous_exploration (proven working)"
echo "  • SLAM: slam_toolbox pour TB3_1 et TB3_2"
echo "  • Map Merge: multirobot_map_merge (de m-explore-ros2)"
echo "  • Résultat: Carte fusionnée en temps réel sur /map"
echo ""
echo "========================================================="

echo ""
echo "===== 1️⃣ Setup ROS 2 Humble ====="
source /opt/ros/humble/setup.bash
source "$HOME/ros2_humble/install/setup.bash"
export ROS_DISTRO=humble

export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models

echo ""
echo "===== 2️⃣ Launch Gazebo with 2 robots ====="
gnome-terminal -- bash -c "ros2 launch turtlebot3_gazebo multi_robot.launch.py; exec bash"

sleep 10

echo ""
echo "===== 3️⃣ Launch SLAM Toolbox for both robots ====="
gnome-terminal -- bash -c "ros2 launch autonomous_exploration multi_robot_slam.launch.py; exec bash"

sleep 3


echo ""
echo "===== 5️⃣ Publish Static TF (map → TB3_1/map et map → TB3_2/map) ====="
# Transformer la position TB3_1: (-2, -0.5) → transform map vers TB3_1/map
gnome-terminal -- bash -c "ros2 run tf2_ros static_transform_publisher -2.0 -0.5 0 0 0 0 map TB3_1/map; exec bash"

# Transformer la position TB3_2: (0.5, -2) → transform map vers TB3_2/map  
gnome-terminal -- bash -c "ros2 run tf2_ros static_transform_publisher 0.5 -2.0 0 0 0 0 map TB3_2/map; exec bash"

sleep 2

echo ""
echo "===== 6️⃣ Launch Map Merge (multirobot_map_merge) ====="
echo "Fusion automatique des cartes /TB3_1/map + /TB3_2/map → /map"
gnome-terminal -- bash -c "ros2 run multirobot_map_merge map_merge --ros-args \
  --params-file $HOME/ros2_humble/map_merge_params.yaml \
  -p world_frame:=map \
  -p /TB3_1/map_merge/init_pose_x:=-2.0 \
  -p /TB3_1/map_merge/init_pose_y:=-0.5 \
  -p /TB3_1/map_merge/init_pose_z:=0.0 \
  -p /TB3_1/map_merge/init_pose_yaw:=0.0 \
  -p /TB3_2/map_merge/init_pose_x:=0.5 \
  -p /TB3_2/map_merge/init_pose_y:=-2.0 \
  -p /TB3_2/map_merge/init_pose_z:=0.0 \
  -p /TB3_2/map_merge/init_pose_yaw:=0.0; exec bash"

sleep 3

echo ""
echo "===== 7️⃣ Launch autonomous exploration for TB3_1 ====="
PARAMS_FILE="$HOME/ros2_humble/src/ROS2-FrontierBaseExplorationForAutonomousRobot/autonomous_exploration/config/params.yaml"
gnome-terminal -- bash -c "ros2 run autonomous_exploration control --ros-args --params-file $PARAMS_FILE -r __ns:=/TB3_1 -r scan:=/TB3_1/scan -r cmd_vel:=/TB3_1/cmd_vel -r odom:=/TB3_1/odom -r map:=/TB3_1/map; exec bash"

echo ""
echo "===== 8️⃣ Launch autonomous exploration for TB3_2 ====="
gnome-terminal -- bash -c "ros2 run autonomous_exploration control --ros-args --params-file $PARAMS_FILE -r __ns:=/TB3_2 -r scan:=/TB3_2/scan -r cmd_vel:=/TB3_2/cmd_vel -r odom:=/TB3_2/odom -r map:=/TB3_2/map; exec bash"

echo ""
echo "✅ Système complet lancé!"
echo ""
echo "========================================================="
echo "📊 Visualisation dans RViz - Ajoutez ces topics:"
echo "========================================================="
echo ""
echo "🗺️  CARTE FUSIONNÉE (Map Merge):"
echo "   - Map → Topic: /map"
echo "   - Fixed Frame: map"
echo ""
echo "🤖 CARTES INDIVIDUELLES:"
echo "   - Map → Topic: /TB3_1/map (Robot 1)"
echo "   - Map → Topic: /TB3_2/map (Robot 2)"
echo ""
echo "📡 LASER SCANS:"
echo "   - LaserScan → Topic: /TB3_1/scan"
echo "   - LaserScan → Topic: /TB3_2/scan"
echo ""
echo "========================================================="
echo "💾 Pour sauvegarder les cartes:"
echo "========================================================="
echo ""
echo "Carte fusionnée:"
echo "  ros2 run nav2_map_server map_saver_cli -f merged_map --ros-args -r map:=/map"
echo ""
echo "Carte TB3_1:"
echo "  ros2 service call /TB3_1/slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{name: {data: 'TB3_1_map'}}\""
echo ""
echo "Carte TB3_2:"
echo "  ros2 service call /TB3_2/slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{name: {data: 'TB3_2_map'}}\""
echo ""
echo "========================================================="
echo "✨ Les robots explorent et les cartes fusionnent automatiquement!"
echo "========================================================="
