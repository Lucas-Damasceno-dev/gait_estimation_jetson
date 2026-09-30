#include "driver/bno055imu.hpp"

#include <stdio.h>      
#include <stdlib.h>   
#include <stdexcept>      
#include <iostream>       
#include <thread>         
#include <chrono>     

#include <wiringPi.h>
#include <wiringPiI2C.h>

#define SEL_A 2
#define SEL_B 0

int I2CManager::get_fd(uint8_t address) {
static std::map<uint8_t, int> fds_internal;

    if (fds_internal.find(address) == fds_internal.end()) {
        
        if (fds_internal.empty()) {
            wiringPiSetup();
        }

        int new_fd = wiringPiI2CSetup(address);
        if (new_fd >= 0) {
            fds_internal[address] = new_fd;
        } else {
            return -1; 
        }
    }
    return fds_internal[address];
}

BNO055IMU::BNO055IMU(int32_t imu_id, int sensor_id, uint8_t address) : 
    bno_(IMU(imu_id, address)), sensor_id_(sensor_id) {
    setup_states();
} 

void BNO055IMU::setup() {
    // pega o fd único do singleton
    int shared_fd = I2CManager::get_fd(bno_.get_address());

    // injeta o fd no driver
    bno_.set_fd(shared_fd);

    pinMode(SEL_A, OUTPUT);
    pinMode(SEL_B, OUTPUT);
    digitalWrite(SEL_A, selA_state_);
    digitalWrite(SEL_B, selB_state_);

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if(bno_.init_bno055() != 0)
    {
        std::string msg = "[ERRO] BNO" + std::to_string(sensor_id_) + ": não inicializado";
        throw std::runtime_error(msg);
    }
}

void BNO055IMU::get_euler_data(std::vector<float>& out_data) {
    digitalWrite(SEL_A, selA_state_);
    digitalWrite(SEL_B, selB_state_);
    
    delay(1);

    out_data.resize(3);
    bno_.read_euler(out_data.data());
    delay(1);
    
    // calibrando
    out_data[0] = out_data[0] - calibration_ref_[0]; // yaw
    out_data[1] = out_data[1] - calibration_ref_[1]; // pitch
    out_data[2] = out_data[2] - calibration_ref_[2]; // roll
}

void BNO055IMU::get_quaternions_data(std::vector<float>& out_data) {
    digitalWrite(SEL_A, selA_state_);
    digitalWrite(SEL_B, selB_state_);
    
    delay(1);

    out_data.resize(4);
    bno_.read_quaternions(out_data.data());
    delay(1);
    

    float rw =  calibration_ref_quaternions_[0];
    float rx = -calibration_ref_quaternions_[1];
    float ry = -calibration_ref_quaternions_[2];
    float rz = -calibration_ref_quaternions_[3];
    
    float cw = out_data[0];
    float cx = out_data[1];
    float cy = out_data[2];
    float cz = out_data[3];
    
    out_data[0] = rw*cw - rx*cx - ry*cy - rz*cz; // novo w
    out_data[1] = rw*cx + rx*cw + ry*cz - rz*cy; // novo x
    out_data[2] = rw*cy - rx*cz + ry*cw + rz*cx; // novo y
    out_data[3] = rw*cz + rx*cy - ry*cx + rz*cw; // novo z
}

void BNO055IMU::calibrate_euler() {
    digitalWrite(SEL_A, selA_state_);
    digitalWrite(SEL_B, selB_state_);
    
    delay(1);

    bno_.read_euler(calibration_ref_);
    delay(1);
}

void BNO055IMU::calibrate_quaternions() {
    digitalWrite(SEL_A, selA_state_);
    digitalWrite(SEL_B, selB_state_);
    
    delay(1);

    bno_.read_quaternions(calibration_ref_quaternions_);
    delay(1);
}

void BNO055IMU::setup_states() {
    switch (sensor_id_) {
        case 1:
        case 2:
            selA_state_ = LOW;
            selB_state_ = LOW;
            break;
            
        case 3:
        case 4:
            selA_state_ = LOW;
            selB_state_ = HIGH;
            break;
            
        default:
            break;
    }
}
