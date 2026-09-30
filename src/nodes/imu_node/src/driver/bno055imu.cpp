#include "driver/bno055imu.hpp"

#include <stdio.h>      
#include <stdlib.h>   
#include <stdexcept>      
#include <iostream>       
#include <thread>         
#include <chrono>     
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

int I2CManager::get_fd(uint8_t address, const std::string& i2c_bus) {
    static std::map<uint8_t, int> fds_internal;

    if (fds_internal.find(address) == fds_internal.end()) {
        int fd = open(i2c_bus.c_str(), O_RDWR);
        if (fd < 0) {
            std::cerr << "[ERRO] Não foi possível abrir " << i2c_bus 
                      << " para o endereço 0x" << std::hex << (int)address << std::dec << std::endl;
            return -1;
        }

        if (ioctl(fd, I2C_SLAVE, address) < 0) {
            std::cerr << "[ERRO] ioctl I2C_SLAVE falhou para o endereço 0x" 
                      << std::hex << (int)address << std::dec << std::endl;
            close(fd);
            return -1;
        }

        fds_internal[address] = fd;
    }
    return fds_internal[address];
}

void I2CManager::close_all() {
    // mantido para gerenciamento de recursos se necessário
}

BNO055IMU::BNO055IMU(int32_t imu_id, int sensor_id, uint8_t address) : 
    bno_(IMU(imu_id, address)), sensor_id_(sensor_id) {
} 

void BNO055IMU::setup() {
    int shared_fd = I2CManager::get_fd(bno_.get_address());
    if (shared_fd < 0) {
        std::string msg = "[ERRO] BNO" + std::to_string(sensor_id_) + 
                          ": falha ao abrir descritor I2C para endereço " + std::to_string(bno_.get_address());
        throw std::runtime_error(msg);
    }

    bno_.set_fd(shared_fd);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (bno_.init_bno055() != 0)
    {
        std::string msg = "[ERRO] BNO" + std::to_string(sensor_id_) + 
                          " (endereço " + std::to_string(bno_.get_address()) + "): falha na inicialização do chip";
        throw std::runtime_error(msg);
    }
}

void BNO055IMU::get_euler_data(std::vector<float>& out_data) {
    out_data.resize(3);
    bno_.read_euler(out_data.data());

    // calibrando com referência relativa
    out_data[0] = out_data[0] - calibration_ref_[0]; // yaw
    out_data[1] = out_data[1] - calibration_ref_[1]; // pitch
    out_data[2] = out_data[2] - calibration_ref_[2]; // roll
}

void BNO055IMU::get_quaternions_data(std::vector<float>& out_data) {
    out_data.resize(4);
    bno_.read_quaternions(out_data.data());

    float rw =  calibration_ref_quaternions_[0];
    float rx = -calibration_ref_quaternions_[1];
    float ry = -calibration_ref_quaternions_[2];
    float rz = -calibration_ref_quaternions_[3];
    
    float cw = out_data[0];
    float cx = out_data[1];
    float cy = out_data[2];
    float cz = out_data[3];
    
    // Multiplicação de quatérnions: q_rel = q_ref_conjugate * q_current
    out_data[0] = rw*cw - rx*cx - ry*cy - rz*cz; // w
    out_data[1] = rw*cx + rx*cw + ry*cz - rz*cy; // x
    out_data[2] = rw*cy - rx*cz + ry*cw + rz*cx; // y
    out_data[3] = rw*cz + rx*cy - ry*cx + rz*cw; // z
}

void BNO055IMU::calibrate_euler() {
    bno_.read_euler(calibration_ref_);
}

void BNO055IMU::calibrate_quaternions() {
    float temp[4] = {0, 0, 0, 0};
    bno_.read_quaternions(temp);
    float norm_sq = temp[0]*temp[0] + temp[1]*temp[1] + temp[2]*temp[2] + temp[3]*temp[3];
    if (norm_sq > 0.5f && norm_sq < 1.5f) {
        calibration_ref_quaternions_[0] = temp[0];
        calibration_ref_quaternions_[1] = temp[1];
        calibration_ref_quaternions_[2] = temp[2];
        calibration_ref_quaternions_[3] = temp[3];
    } else {
        // Se a leitura inicial for inválida, mantém quatérnion identidade
        calibration_ref_quaternions_[0] = 1.0f;
        calibration_ref_quaternions_[1] = 0.0f;
        calibration_ref_quaternions_[2] = 0.0f;
        calibration_ref_quaternions_[3] = 0.0f;
    }
}
