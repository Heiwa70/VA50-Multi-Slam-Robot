#!/bin/bash

echo "=========================================="
echo "💾 Sauvegarde des Cartes Multi-Robot"
echo "=========================================="
echo ""

# Créer dossier maps
mkdir -p ~/ros2_humble/maps
cd ~/ros2_humble/maps

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

echo "📅 Timestamp: $TIMESTAMP"
echo ""

echo "1️⃣  Sauvegarde de la carte FUSIONNÉE (/map)..."
ros2 run nav2_map_server map_saver_cli -f "merged_map_$TIMESTAMP" --ros-args -r map:=/map
if [ $? -eq 0 ]; then
    echo "   ✅ merged_map_${TIMESTAMP}.pgm"
    echo "   ✅ merged_map_${TIMESTAMP}.yaml"
else
    echo "   ❌ Erreur: Topic /map non disponible"
fi

echo ""
echo "2️⃣  Sauvegarde de la carte TB3_1 (/TB3_1/map)..."
ros2 run nav2_map_server map_saver_cli -f "TB3_1_map_$TIMESTAMP" --ros-args -r map:=/TB3_1/map
if [ $? -eq 0 ]; then
    echo "   ✅ TB3_1_map_${TIMESTAMP}.pgm"
    echo "   ✅ TB3_1_map_${TIMESTAMP}.yaml"
else
    echo "   ❌ Erreur: Topic /TB3_1/map non disponible"
fi

echo ""
echo "3️⃣  Sauvegarde de la carte TB3_2 (/TB3_2/map)..."
ros2 run nav2_map_server map_saver_cli -f "TB3_2_map_$TIMESTAMP" --ros-args -r map:=/TB3_2/map
if [ $? -eq 0 ]; then
    echo "   ✅ TB3_2_map_${TIMESTAMP}.pgm"
    echo "   ✅ TB3_2_map_${TIMESTAMP}.yaml"
else
    echo "   ❌ Erreur: Topic /TB3_2/map non disponible"
fi

echo ""
echo "=========================================="
echo "✅ Sauvegarde terminée!"
echo "=========================================="
echo ""
echo "📁 Emplacement: ~/ros2_humble/maps/"
ls -lh ~/ros2_humble/maps/*.pgm ~/ros2_humble/maps/*.yaml 2>/dev/null

echo ""
echo "🖼️  Pour voir les images:"
echo "   eog ~/ros2_humble/maps/merged_map_${TIMESTAMP}.pgm"
echo "   eog ~/ros2_humble/maps/TB3_1_map_${TIMESTAMP}.pgm"
echo "   eog ~/ros2_humble/maps/TB3_2_map_${TIMESTAMP}.pgm"
echo ""
