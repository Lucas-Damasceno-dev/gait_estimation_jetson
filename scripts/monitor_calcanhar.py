#!/usr/bin/env python3
"""
Monitor Direto de Calcanhar para as Palmilhas LEGACT (16 canais)
Lê diretamente as portas seriais /dev/ttyACM0 e /dev/ttyACM1 (ou via ROS 2 se disponível)
e exibe em tempo real apenas os 4 sensores mais próximos do calcanhar (canais 0 a 3).
"""

import sys
import time
import glob

try:
    import serial
except ImportError:
    print("Instalando pyserial ou certifique-se de que está disponível...")

def parse_line_values(raw_line):
    # Trata separadores comuns (espaço, vírgula, ponto e vírgula, tabulação)
    cleaned = raw_line.replace(',', ' ').replace(';', ' ').replace('\t', ' ')
    tokens = cleaned.strip().split()
    values = []
    for t in tokens:
        try:
            values.append(int(float(t)))
        except ValueError:
            pass
    return values

def monitor_port(port_name, baud=115200):
    print(f"\n=======================================================")
    print(f" Conectando à palmilha em {port_name} a {baud} bps...")
    print(f" Mostrando apenas os sensores do CALCANHAR (Canais 1 a 4)")
    print(f"=======================================================\n")
    try:
        ser = serial.Serial(port_name, baud, timeout=1.0)
        # Limpa o buffer de entrada inicial
        ser.reset_input_buffer()
        
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            
            vals = parse_line_values(line)
            if len(vals) >= 4:
                # Sensores 0 a 3 são os sensores do calcanhar (LEGACT FS-INS-16Z)
                c1, c2, c3, c4 = vals[0], vals[1], vals[2], vals[3]
                total_calcanhar = c1 + c2 + c3 + c4
                
                # Barra gráfica simples para visualização rápida no terminal
                nivel_pressao = min(20, total_calcanhar // 400)
                barra = "█" * nivel_pressao + "-" * (20 - nivel_pressao)
                
                print(f"[{port_name}] Calcanhar: S1={c1:4d} | S2={c2:4d} | S3={c3:4d} | S4={c4:4d} | Total={total_calcanhar:5d} | [{barra}]")
            else:
                print(f"[{port_name}] Dados: {line}")
            
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nMonitoramento encerrado pelo usuário.")
    except PermissionError:
        print(f"\n[ERRO DE PERMISSÃO] Execute no terminal do Ubuntu (fora do Docker):")
        print(f"   sudo chmod 666 {port_name}")
    except Exception as e:
        print(f"\nErro na porta {port_name}: {e}")

if __name__ == '__main__':
    porta = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    monitor_port(porta)
