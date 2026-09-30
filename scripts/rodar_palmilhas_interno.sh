#!/bin/bash
# Script para rodar DIRETO dentro do container Docker
echo "========================================================"
echo " Carregando ambiente ROS 2 Humble..."
echo "========================================================"
source /ros_entrypoint.sh
source /ros2_ws/install/setup.bash

echo "========================================================"
echo " Iniciando pressure_node (C++)..."
echo "========================================================"
/ros2_ws/install/pressure_node/lib/pressure_node/pressure_node
