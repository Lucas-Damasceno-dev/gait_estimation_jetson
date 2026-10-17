#ifndef IMU_LIB_HPP_
#define IMU_LIB_HPP_

#include <vector>
#include <cstdint>
#include <map>
#include <string>

#include "driver/imu.hpp"

class I2CManager {
public:
    static int get_fd(uint8_t address, const std::string& i2c_bus = "/dev/i2c-1");
    static void close_all();
};

class BNO055IMU {
public:
    BNO055IMU(int32_t bno_id, int sensor_id, uint8_t address);
    ~BNO055IMU() = default;

    void setup();
    bool get_euler_data(std::vector<float>& out_data);
    bool get_quaternions_data(std::vector<float>& out_data);
    void calibrate_euler();
    void calibrate_quaternions();
  
private:
    IMU bno_;
    int sensor_id_;
    float calibration_ref_[3] = {0.0f, 0.0f, 0.0f};
    float calibration_ref_quaternions_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
};

#endif // IMU_LIB_HPP_