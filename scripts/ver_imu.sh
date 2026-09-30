#!/bin/bash
# ==============================================================================
# Script para visualizar a orientação da IMU em tempo real via ROS 2
# ==============================================================================
source /ros_entrypoint.sh
source /ros2_ws/install/setup.bash

echo "=========================================================="
echo " Monitorando tópico /imu/state (Ctrl+C para sair)..."
echo "=========================================================="
exec ros2 topic echo /imu/state
