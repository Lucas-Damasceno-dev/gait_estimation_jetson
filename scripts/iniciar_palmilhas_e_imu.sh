#!/bin/bash
# ==============================================================================
# Script Oficial para Executar Palmilhas e IMU Simultaneamente via ROS 2 C++
# Plataforma: Nvidia Jetson Nano
# ==============================================================================

echo "=========================================================="
echo " 1. Carregando Ambiente ROS 2 Humble e Workspace..."
echo "=========================================================="
source /ros_entrypoint.sh
source /ros2_ws/install/setup.bash

echo "=========================================================="
echo " 2. Verificando Hardware Conectado..."
echo "=========================================================="
echo "▶ Portas seriais USB (Palmilhas):"
ls -la /dev/ttyACM* 2>/dev/null || echo "Nenhuma porta /dev/ttyACM* detectada!"

echo "▶ Barramento I2C (IMU):"
ls -la /dev/i2c-1 2>/dev/null || echo "/dev/i2c-1 não detectado!"

echo "=========================================================="
echo " 3. Iniciando Sensores (IMU + Palmilhas)..."
echo "=========================================================="
exec ros2 launch bringup sensors_only.launch.py
