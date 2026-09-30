#!/usr/bin/env python3
"""
Leitor Visual de Calcanhar via Tópico Nativo do ROS 2 (/pressure/state)
Inscrito no tópico oficial do pressure_node e exibe os 4 sensores do calcanhar em tempo real.
"""

import sys
import os

# Garante inclusão dos módulos do ROS 2 Humble e do Workspace
ros_paths = [
    "/opt/ros/humble/install/lib/python3.6/site-packages",
    "/ros2_ws/install/interfaces/lib/python3.6/site-packages"
]
for p in ros_paths:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from interfaces.msg import PressureState

class HeelROS2Monitor(Node):
    def __init__(self):
        super().__init__('heel_ros2_monitor')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        self.sub = self.create_subscription(
            PressureState,
            '/pressure/state',
            self.pressure_callback,
            qos
        )

        print("\n" + "=" * 65)
        print(" 🦶 MONITOR DE CALCANHAR VIA ROS 2 (Tópico: /pressure/state)")
        print(" Aguardando pacotes do pressure_node... Pressione o calcanhar!")
        print("=" * 65 + "\n")

    def pressure_callback(self, msg: PressureState):
        for idx, p_data in enumerate(msg.pressures):
            nome = msg.names[idx] if idx < len(msg.names) else f"Insole_{idx}"
            
            # Sensores 0 a 3 são os 4 sensores mais próximos do calcanhar (LEGACT FS-INS-16Z)
            calcanhar = [p.pressure for p in p_data.pressures[:4]]
            
            # Se a mensagem tiver menos de 4 sensores
            while len(calcanhar) < 4:
                calcanhar.append(0)

            c1, c2, c3, c4 = calcanhar[0], calcanhar[1], calcanhar[2], calcanhar[3]
            total_calcanhar = c1 + c2 + c3 + c4

            # Barra gráfica para visualização rápida no terminal
            nivel_barra = min(15, total_calcanhar // 350)
            barra = "█" * nivel_barra + "-" * (15 - nivel_barra)

            print(f"[{nome:12s}] Calcanhar: S1={c1:4d} | S2={c2:4d} | S3={c3:4d} | S4={c4:4d} | Total={total_calcanhar:5d} | [{barra}]")

def main():
    rclpy.init()
    node = HeelROS2Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nMonitor de calcanhar encerrado.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
