#!/bin/bash

# Script pour générer la documentation de tous les packages ROS2

echo "Génération de la documentation pour tous les packages..."

# Générer la doc Python avec Sphinx
echo "Génération de la documentation Python..."
cd src/multi_robot_exploration/docs
rm -rf _build
sphinx-build -b html . _build
cd ../../..

# Liste des packages C++ à documenter avec rosdoc2
PACKAGES=(
    "src/m-explore-ros2/map_merge"
    "src/slam_gmapping/slam_gmapping"
    "src/slam_gmapping/openslam_gmapping"
)

# Générer la doc C++ pour chaque package
for package in "${PACKAGES[@]}"; do
    if [ -d "$package" ]; then
        echo "Génération de la documentation pour: $package"
        rosdoc2 build --package-path "$package"
    else
        echo "Package non trouvé: $package"
    fi
done

# Copier la doc Python dans docs_output
echo "Copie de la documentation Python..."
mkdir -p docs_output/multi_robot_exploration
cp -r src/multi_robot_exploration/docs/_build/* docs_output/multi_robot_exploration/

echo "✅ Documentation générée dans docs_output/"
