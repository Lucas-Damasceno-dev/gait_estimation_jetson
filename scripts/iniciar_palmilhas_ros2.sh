#!/bin/bash
# ==============================================================================
# Script Oficial para Executar a Leitura das Palmilhas via ROS 2 C++
# Deve ser executado DENTRO do container Docker com ROS 2
# ==============================================================================

echo "=========================================================="
echo " 1. Carregando Ambiente ROS 2 Humble e Workspace..."
echo "=========================================================="
source /ros_entrypoint.sh
source /ros2_ws/install/setup.bash

echo "=========================================================="
echo " 2. Verificando portas seriais /dev/ttyACM*..."
echo "=========================================================="
if ls /dev/ttyACM* 1> /dev/null 2>&1; then
    echo "✅ Portas detectadas:"
    ls -la /dev/ttyACM*
else
    echo "⚠️ AVISO: Nenhuma porta /dev/ttyACM* encontrada dentro do Docker!"
    echo "Certifique-se de iniciar o Docker com: --privileged -v /dev:/dev"
fi

echo "=========================================================="
echo " 3. Iniciando nó de pressão (pressure_node C++)..."
echo "    Configuração: 2 Palmilhas (/dev/ttyACM0 e /dev/ttyACM1)"
echo "    Publicando em: /pressure/state"
echo "=========================================================="

CONFIG_FILE="/ros2_ws/install/bringup/share/bringup/config/pressure.yaml"
if [ ! -f "$CONFIG_FILE" ]; then
    CONFIG_FILE="/ros2_ws/src/launch/bringup/config/pressure.yaml"
fi

/ros2_ws/install/pressure_node/lib/pressure_node/pressure_node --ros-args --params-file "$CONFIG_FILE"
