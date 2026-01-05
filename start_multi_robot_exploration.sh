#!/bin/bash

echo "========================================================="
echo "Exploration Multi-Robot"
echo "========================================================="



# --- Configuration de l'environnement ---
export GAZEBO_MODEL_PATH=$HOME/ros2_humble/models:$GAZEBO_MODEL_PATH
export TURTLEBOT3_MODEL=burger
source ~/ros2_humble/install/setup.bash



# --- Nettoyage des processus existants ---
echo ""
echo "Arret des processus existants..."
pkill -9 gazebo
pkill -9 gzclient
pkill -9 -f "async_slam_toolbox_node"
pkill -9 -f "sync_slam_toolbox_node"
pkill -9 -f "slam_gmapping"
pkill -9 -f "map_merge"
pkill -9 -f "static_transform_publisher"
pkill -9 rviz2
pkill -9 -f "teleop"
pkill -9 -f "explore"
pkill -9 -f "icp_alignment"
sleep 2



# --- Etape 1 : Lancement de la simulation ---
echo ""
echo "===== 1. Lancement Gazebo + Robots ====="

gnome-terminal -- bash -c "cd ~/ros2_humble && source install/setup.bash && ros2 launch  multi_robot_exploration multi_robot.launch.py; exec bash" &

sleep 10



# --- Etape 2 : Filtrage LaserScan ---
echo ""
echo "===== 2. Lancement du filtre LaserScan (TB3_1 et TB3_2) ====="

# Filtre pour le Robot 1
gnome-terminal -- bash -c "cd ~/ros2_humble && source install/setup.bash && ros2 run multi_robot_exploration dynamic_laser_filter --ros-args -p scan_topic:=/TB3_1/scan -p filtered_scan_topic:=/TB3_1/scan_filtered -p self_robot:=TB3_1 -p other_robots:=[TB3_1,TB3_2] -p base_frame:=base_footprint -p world_frame:=TB3_1/map; exec bash" &

# Filtre pour le Robot 2
gnome-terminal -- bash -c "cd ~/ros2_humble && source install/setup.bash && ros2 run multi_robot_exploration dynamic_laser_filter --ros-args -p scan_topic:=/TB3_2/scan -p filtered_scan_topic:=/TB3_2/scan_filtered -p self_robot:=TB3_2 -p other_robots:=[TB3_1,TB3_2] -p base_frame:=base_footprint -p world_frame:=TB3_2/map; exec bash" &

sleep 5



# --- Etape 3 : Verification de la disponibilite des donnees ---
echo ""
echo ""
echo "===== 3. Attente des scans filtres ====="

for robot in TB3_1 TB3_2; do
    echo "  Attente du topic ${robot}/scan_filtered..."
    TIMEOUT=30
    ELAPSED=0
    
    while ! ros2 topic list | grep -q "/${robot}/scan_filtered"; do
        sleep 1
        ELAPSED=$((ELAPSED + 1))
        
        if [ $ELAPSED -ge $TIMEOUT ]; then
            echo "ERREUR : TIMEOUT - Topic /${robot}/scan_filtered non disponible"
            exit 1
        fi
    done
    
    echo "  OK : Topic /${robot}/scan_filtered disponible"
done

sleep 2



# --- Etape 4 : Cartographie (SLAM) ---
echo ""
echo "===== 4. Lancement SLAM gmapping ====="

gnome-terminal -- bash -c "cd ~/ros2_humble && source install/setup.bash && ros2 launch multi_robot_exploration multi_robot_slam_gmapping.launch.py; exec bash" &

sleep 15



# --- Etape 5 : Lien Base_Link / Camera_rgb ---
echo ""
echo "===== 5. Publication des transformations statiques (TF) ====="

gnome-terminal --title="Base Link Camera" -- bash -c "cd ~/ros2_humble && source install/setup.bash && ros2 run tf2_ros static_transform_publisher 0 0 0 -1.57 0 -1.57 TB3_1/base_link TB3_1/camera_rgb_optical_frame_depth_optical_frame & ros2 run tf2_ros static_transform_publisher 0 0 0 -1.57 0 -1.57 TB3_2/base_link TB3_2/camera_rgb_optical_frame_depth_optical_frame; exec bash"

sleep 2



# --- Etape 6 : Verification des transformations (TF) ---
echo ""
echo "===== 6. Attente des TF map ====="

for robot in TB3_1 TB3_2; do
    echo "  Vérification TF pour $robot..."
    TIMEOUT=60
    ELAPSED=0
    
    while ! ros2 run tf2_ros tf2_echo ${robot}/map ${robot}/base_footprint 2>&1 | grep -q "Translation"; do
        sleep 2
        ELAPSED=$((ELAPSED + 2))
        
        if [ $ELAPSED -ge $TIMEOUT ]; then
            echo "ERREUR : TIMEOUT - TF ${robot}/map -> ${robot}/base_footprint non disponible"
            exit 1
        fi
    done
    
    echo "  OK : TF disponible pour $robot"
done



# --- Etape 7 : Verification des capteurs RGBD ---
echo ""
echo "===== 7. Attente des caméras RGBD ====="

for robot in TB3_1 TB3_2; do
    echo "  Vérification caméra depth pour $robot..."
    TIMEOUT=30
    ELAPSED=0
    
    while ! ros2 topic list | grep -q "${robot}/camera_depth/points"; do
        sleep 1
        ELAPSED=$((ELAPSED + 1))
        
        if [ $ELAPSED -ge $TIMEOUT ]; then
            echo "ERREUR : TIMEOUT - Topic ${robot}/camera_depth/points non disponible"
            exit 1
        fi
    done
    
    echo "  OK : Caméra depth disponible pour $robot"
done



# --- Etape 8 : Calcul des poses par ICP ---
echo ""
echo "===== 8. Lancement ICP pour calcul des poses ====="
echo "Attente de quelques secondes pour que les cameras publient..."
sleep 3

echo ""
echo "Calcul du recalage ICP (les robots observent la scene actuelle)..."

ros2 run multi_robot_exploration icp_alignment
ICP_EXIT_CODE=$?

if [ $ICP_EXIT_CODE -ne 0 ]; then
    echo "ERREUR : Le calcul ICP a echoue"
    exit 1
fi

if [ ! -f ~/ros2_humble/src/multi_robot_exploration/config/map_merge_icp_poses.yaml ]; then
    echo "ERREUR : Fichier map_merge_icp_poses.yaml non genere"
    exit 1
fi

echo ""
echo "OK : Poses calculees avec succes !"
echo "    Fichier : ~/ros2_humble/src/multi_robot_exploration/config/map_merge_icp_poses.yaml"



# --- Etape 9 : Publication des transformations mondiales (TF) ---
echo ""
echo "===== 9. Publication des transformations mondiales (TF) ====="

gnome-terminal --title="TF world" -- bash -c "cd ~/ros2_humble && source install/setup.bash && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world TB3_1/map & ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world TB3_2/map; exec bash" &

sleep 2



# --- Etape 10 : Relance de la cartographie (SLAM) ---
echo ""
echo "===== 10. Relance SLAM gmapping pour map propre (init filtre) ====="
pkill -9 -f slam_gmapping

sleep 2

gnome-terminal -- bash -c "cd ~/ros2_humble && source install/setup.bash && ros2 launch multi_robot_exploration multi_robot_slam_gmapping.launch.py; exec bash" &

sleep 8


# --- Etape 11 : Fusion des cartes (Map Merge) ---
echo ""
echo "===== 11. Lancement Map Merge avec poses ICP ====="

gnome-terminal -- bash -c "cd ~/ros2_humble && source install/setup.bash && ros2 launch multirobot_map_merge map_merge.launch.py known_init_poses:=true config_file:=$HOME/ros2_humble/src/multi_robot_exploration/config/map_merge_icp_poses.yaml; exec bash" &

sleep 3



# --- Etape 12 : Visualisation (RViz2) ---
echo ""
echo "===== 12. Lancement RViz ====="


RVIZ_CONFIG="$HOME/ros2_humble/src/multi_robot_exploration/rviz/exploration.rviz"

gnome-terminal -- bash -c "cd ~/ros2_humble && source install/setup.bash && rviz2 -d $RVIZ_CONFIG; exec bash" &

sleep 3



# --- Etape 13 : Navigation Autonome (Nav2) ---
echo ""
echo "===== 13. Lancement de la Navigation (Nav2) pour tous les robots ====="
echo "Lancement Nav2 multi-robot..."

gnome-terminal --title="Nav2 Multi-Robot" -- bash -c "cd ~/ros2_humble && source install/setup.bash && ros2 launch multi_robot_exploration multi_robot_navigation.launch.py use_sim_time:=true; exec bash" &

sleep 10



# --- Etape 14 : Exploration par frontieres ---
echo ""
echo "===== 14. Lancement du calcul des frontieres et envoi des goals ====="
echo "Lancement de l'exploration autonome..."

ros2 run multi_robot_exploration multi_robot_frontier_explorer --ros-args -p debug_mode:=true -p min_frontier_size:=0.1



echo ""
echo "========================================================="
echo "Systeme pret !"
echo "========================================================="
