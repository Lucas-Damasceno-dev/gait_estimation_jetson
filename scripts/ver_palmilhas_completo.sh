#!/bin/bash
# Script para visualizar as DUAS palmilhas e todos os 16 sensores de cada uma em C++
source /ros_entrypoint.sh
source /ros2_ws/install/setup.bash
/ros2_ws/install/pressure_node/lib/pressure_node/monitor_palmilhas_completo_node
