#!/bin/bash
# ==============================================================================
# Script Oficial para Executar a Leitura da IMU (BNO055) via ROS 2 C++
# Plataforma: Nvidia Jetson Nano (Barramento I2C nativo /dev/i2c-1)
# ==============================================================================

echo "=========================================================="
echo " 1. Carregando Ambiente ROS 2 Humble e Workspace..."
echo "=========================================================="
source /ros_entrypoint.sh
source /ros2_ws/install/setup.bash

echo "=========================================================="
echo " 2. Verificando barramento I2C /dev/i2c-1..."
echo "=========================================================="
if [ -e /dev/i2c-1 ]; then
    echo "✅ Barramento /dev/i2c-1 detectado!"
    i2cdetect -y -r 1
else
    echo "⚠️ AVISO: /dev/i2c-1 não encontrado dentro do Docker!"
    echo "Certifique-se de iniciar o Docker com: --privileged -v /dev:/dev"
fi

echo "=========================================================="
echo " 3. Iniciando nó da IMU (imu_node C++ nativo)..."
echo "    Publicando em: /imu/state e /imu/time"
echo "=========================================================="

CONFIG_FILE="/ros2_ws/install/bringup/share/bringup/config/imu.yaml"
if [ ! -f "$CONFIG_FILE" ]; then
    CONFIG_FILE="/ros2_ws/src/launch/bringup/config/imu.yaml"
fi

exec /ros2_ws/install/imu_node/lib/imu_node/imu_node --ros-args --params-file "$CONFIG_FILE"
