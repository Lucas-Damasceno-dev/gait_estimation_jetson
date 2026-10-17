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
├── scripts/                         # Scripts utilitários de execução rápida
│   └── iniciar_rviz_sensores.sh     # Inicializador direto da interface RViz2
├── src/                             # Pacotes ROS 2 C++ nativos
│   ├── interfaces/                  # Mensagens customizadas (SyncedSensorData, etc.)
│   ├── drivers/common_serial/       # Camada de transporte e protocolo binário STM32
│   ├── nodes/
│   │   ├── serial_driver_node/      # Driver serial STM32 e emulador offline
│   │   ├── sync_node/               # Sincronização temporal (1 kHz Insole Fast-Sync + ZOH IMU)
│   │   ├── imu_node/                # Drivers I2C nativos com multiplexação binária
│   │   ├── pressure_node/           # Drivers de pressão plantar (16 canais)
│   │   └── mg8008e_node/            # Driver do atuador RS485 MG8008E
│   ├── processing/
│   │   ├── gait_estimation_pkg/     # Estimação de marcha, IA e sensor_visualizer_node (RViz2)
│   │   ├── convergence_node/        # Validação temporal e convergência
│   │   └── time_node/               # Telemetria e benchmarking de jitter
│   └── launch/bringup/              # Launch files e perfis RViz (.rviz)
└── README.md
```

---

## 🖥️ Monitoramento em Tempo Real no RViz2

O projeto conta com o nó nativo em C++ `sensor_visualizer_node` (`gait_estimation_pkg`) e configuração pré-definida no RViz2:
* **Mapa de Calor de Pressão (16 Canais)**: Color mapping Jet (azul a vermelho) proporcional à força em cada sensor plantar.
* **Centro de Pressão Dinâmico (CoP)**: Marcador esférico dourado com coordenadas $X/Y$ calculadas em tempo real.
* **Orientação 3D das IMUs**: Eixos tridimensionais (RGB) projetando a atitude e ângulos de Euler (Roll, Pitch, Yaw).
* **HUD Telemetry Banner**: Monitor flutuante com cálculo contínuo de frequência real em Hz ($\Delta t$) para cada palmilha e IMU.

Para iniciar a interface completa com um único comando:
```bash
./scripts/iniciar_rviz_sensores.sh
```

---

## ⚡ Política de Sincronização Temporal (1 kHz Insole Fast-Sync)

Como as palmilhas operam em taxas mais elevadas (~1 kHz) e as IMUs multiplexadas em frequências menores (~100 Hz):
* **Modo Fast-Sync Ativo (`use_fast_insole_sync = true`)**: A chegada de cada pacote de palmilha dispara imediatamente a publicação de `/sync/data`.
* **Zero-Order Hold (ZOH)**: A última leitura válida de IMU é retida e alinhada sem descartar quadros de pressão e sem introduzir latência de espera por novos frames I2C.
* **Multiplexação Binária das IMUs**: O `imu_node` realiza o chaveamento de endereçamento binário dos pinos (`00`, `01`, `10`, `11`) para leitura sequencial determinística dos sensores físicos.

---

## 🚀 Como Compilar e Executar

```bash
# 1. Compilar todo o workspace em C++
colcon build --symlink-install

# 2. Carregar o ambiente
source install/setup.bash

# 3. Executar o pipeline completo na Jetson Nano
ros2 launch bringup jetson_bringup.launch.py

# 4. Executar apenas o monitor visual do RViz2
ros2 launch bringup rviz_sensors.launch.py
```

