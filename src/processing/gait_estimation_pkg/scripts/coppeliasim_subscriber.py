#!/usr/bin/env python3
"""
Exemplo de receptor ZeroMQ no CoppeliaSim (Lua / Python) para renderizar a órtese e o esqueleto humano em tempo real.
"""
import zmq
import json
import time

def main():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://127.0.0.1:23000")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    print("Conectado ao VisualizationNode via ZeroMQ (tcp://127.0.0.1:23000)... Aguardando ângulos articulares...")

    try:
        while True:
            msg_str = socket.recv_string()
            try:
                joints = json.loads(msg_str)
                print(f"[CoppeliaSim Stream] Quadril: {joints.get('hip_joint_r', 0.0):+.3f} rad | "
                      f"Joelho: {joints.get('knee_joint_r', 0.0):+.3f} rad | "
                      f"Tornozelo: {joints.get('ankle_joint_r', 0.0):+.3f} rad")
            except Exception as e:
                print(f"Frame bruto: {msg_str}")
    except KeyboardInterrupt:
        print("Encerrando subscriber CoppeliaSim.")

if __name__ == '__main__':
    main()
