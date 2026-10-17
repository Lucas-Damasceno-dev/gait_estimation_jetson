#!/bin/bash
# ==============================================================================
# Script Oficial para Abrir Interface RViz2 de Monitoramento de Palmilhas e IMUs
# Plataforma: Nvidia Jetson Nano / Host PC com ROS 2 Humble
# ==============================================================================

if [ -f "/ros_entrypoint.sh" ]; then
    source /ros_entrypoint.sh
fi

if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
elif [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi

echo "=========================================================="
echo " 🦶🧭 INTERFACE RVIZ2 - MONITOR DE PALMILHAS & IMUS"
echo "=========================================================="
echo " Tópicos inscritos:"
echo "   • Palmilhas: /pressure/state"
echo "   • IMU:       /imu/state"
echo " Tópicos publicados:"
echo "   • Visualização 3D: /sensor_monitor/markers"
echo "   • Taxas em Hz    : /sensor_monitor/pressure_hz e /imu_hz"
echo "=========================================================="

# Passa argumentos extras para o launch (ex: start_sensors:=true)
exec ros2 launch bringup rviz_sensors.launch.py "$@"
