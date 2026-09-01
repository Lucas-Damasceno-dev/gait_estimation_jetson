#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

#pragma pack(push, 1)

// Cabeçalho de início de frame
constexpr uint8_t STM32_SOF1 = 0xAA;
constexpr uint8_t STM32_SOF2 = 0x55;

// Estrutura de dados de 1 IMU (quaternions normalizados)
struct IMUDataPayload {
    float q_w;
    float q_x;
    float q_y;
    float q_z;
};

// Estrutura completa de telemetria transmitida pelo STM32
struct STM32SensorFrame {
    uint8_t sof1;                  // 0xAA
    uint8_t sof2;                  // 0x55
    uint8_t device_id;             // 0 = Insole Left, 1 = Insole Right, etc.
    uint32_t timestamp_us;         // Timestamp interno do timer do STM32 (microssegundos)
    uint16_t pressure_channels[16];// 16 canais de pressão (0-4095 do ADC de 12 bits)
    uint8_t num_imus;              // Quantidade de IMUs no pacote (2 ou 3)
    IMUDataPayload imus[3];        // Até 3 IMUs
    uint16_t checksum;             // CRC16 ou soma de verificação
};

#pragma pack(pop)

// Função utilitária para cálculo de checksum CRC16-CCITT
inline uint16_t calculate_checksum16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
