# Estimação da Marcha Antropomórfica em ROS 2 (STM32 + Nvidia Jetson Nano)

Projeto de Iniciação Científica (PROBIC/UEFS) e Trabalho de Conclusão de Curso (TCC 1 - Engenharia de Computação).

* **Autor:** Lucas da Conceição Damasceno ([@Lucas-Damasceno-dev](https://github.com/Lucas-Damasceno-dev))
* **Orientador:** Prof. Dr. Armando Sanca Sanca
* **Instituição:** Universidade Estadual de Feira de Santana (UEFS)

---

## 🎯 Visão Geral do Sistema

Arquitetura embarcada de alta performance desenvolvida em **ROS 2 C++ (`rclcpp`)** na plataforma **Nvidia Jetson Nano** acoplada a um microcontrolador **STM32H743**:
* **Aquisição Síncrona**: Leitura determinística de 2 Palmilhas de pressão plantar (16 canais cada - Legact FS-INS-16Z) e até 3 IMUs 9-DOF via protocolo binário estruturado (`0xAA 0x55` + CRC16).
* **Sincronização Multissensorial**: Filtragem e sincronização temporal via `message_filters::ApproximateTime` (`sync_node`).
* **Motor de IA & Estimação Cinemática**: Classificação de eventos de marcha (*Heel-Strike*, *Mid-Stance*, *Toe-Off*, *Swing*) e predição angular contínua de quadril, joelho e tornozelo com **latência $< 10\text{ ms}$** (`gait_estimation_node`).
* **Visualização em Tempo Real**: Publicação em `/joint_states` para RViz2 e ponte ZeroMQ de alta taxa com o simulador CoppeliaSim (`visualization_node`).

---

## 📁 Estrutura do Repositório

```text
├── docs_contexto/                   # Documentos oficiais do projeto
│   ├── plano_ic_tcc_6meses.md       # Plano diretor vivo de 6 meses
│   ├── Pre-projeto-LucasDamasceno.pdf # Pré-projeto de TCC 1
│   ├── Proposta_Bolsa_IC_2026.pdf   # Proposta de Bolsa de Iniciação Científica
│   └── Planejamento_atividades.xlsx # Cronograma detalhado
├── src/                             # Pacotes ROS 2 C++ nativos
│   ├── interfaces/                  # Mensagens customizadas (SyncedSensorData, etc.)
│   ├── drivers/common_serial/       # Camada de transporte e protocolo binário STM32
│   ├── nodes/
│   │   ├── serial_driver_node/      # Driver serial STM32 e emulador offline
│   │   ├── sync_node/               # Nó C++ de sincronização temporal
│   │   ├── imu_node/                # Drivers de baixo nível
│   │   └── pressure_node/           # Drivers de pressão
│   ├── processing/
│   │   └── gait_estimation_pkg/     # Motores de IA, cinemática e visualização ZeroMQ
│   └── launch/bringup/              # Launch files e configurações .yaml
└── README.md
```

---

## 🚀 Como Compilar e Executar

```bash
# 1. Compilar todo o workspace em C++
colcon build --symlink-install

# 2. Carregar o ambiente
source install/setup.bash

# 3. Executar o pipeline completo na Jetson Nano
ros2 launch bringup jetson_bringup.launch.py
```
