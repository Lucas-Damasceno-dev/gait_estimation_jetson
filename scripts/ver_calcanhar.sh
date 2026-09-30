#!/bin/bash
# Executa o monitor nativo em C++ (livre de problemas de Python)
source /ros_entrypoint.sh
source /ros2_ws/install/setup.bash
/ros2_ws/install/pressure_node/lib/pressure_node/monitor_calcanhar_node
