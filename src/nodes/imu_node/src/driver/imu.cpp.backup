#include "driver/imu.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <cmath>

#include <wiringPi.h>
#include <wiringPiI2C.h>

IMU::IMU(int32_t id, uint8_t address, int shared_fd)
:  dev_(shared_fd), id_(id), address_(address) {}


int IMU::init_bno055()
{
    // modo configuracao
    if (wiringPiI2CWriteReg8(dev_, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG) < 0)
        return -1;
    usleep(30 * 1000); // 30ms

    // reset
    // 0x20 -> comando reset
    if (wiringPiI2CWriteReg8(dev_, BNO055_SYS_TRIGGER_ADDR, 0x20) < 0) 
        return -1;

    // espera até o id do chip ser válido (reset termina)
    int retry = 0;
    while (wiringPiI2CReadReg8(dev_, BNO055_CHIP_ID_ADDR) != BNO055_ID)
    {
        usleep(10 * 1000); // 10ms
        if (++retry > 50) return -1; // Sai após 500ms se o sensor não responder
    }

    usleep(50 * 1000); // 50ms

    // power mode normal
    if (wiringPiI2CWriteReg8(dev_, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL) < 0)
        return -1;

    // página 0
    if (wiringPiI2CWriteReg8(dev_, BNO055_PAGE_ID_ADDR, 0) < 0)
        return -1;

    // reset terminou
    if (wiringPiI2CWriteReg8(dev_, BNO055_SYS_TRIGGER_ADDR, 0x00) < 0)
        return -1;
    usleep(10 * 1000);

    // modo de operação NDOF
    if (wiringPiI2CWriteReg8(dev_, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF) < 0)
        return -1;
    usleep(50 * 1000);

    return 0;
}

void IMU::read_quaternions(float quaternion[4])
{
    if (dev_ < 0) return;

    uint8_t reg = BNO055_QUATERNION_DATA_W_LSB_ADDR;
    uint8_t buf[8];

    // estrutura de mensagens para o I2C
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset;

    // informa qual reg será lido
    msgs[0].addr  = address_; 
    msgs[0].flags = 0;        // 0 -> escrita
    msgs[0].len   = 1;
    msgs[0].buf   = &reg;

    // lê 8 bytes em sequência
    msgs[1].addr  = address_;
    msgs[1].flags = I2C_M_RD; // read flag
    msgs[1].len   = 8;
    msgs[1].buf   = buf;

    msgset.msgs = msgs;
    msgset.nmsgs = 2; // 2 operações sequenciais

    if (ioctl(dev_, I2C_RDWR, &msgset) < 0) {
        return; 
    }

    quaternion[0] = (float)((int16_t)(buf[0] | (buf[1] << 8))) / 16384.0f;
    quaternion[1] = (float)((int16_t)(buf[2] | (buf[3] << 8))) / 16384.0f;
    quaternion[2] = (float)((int16_t)(buf[4] | (buf[5] << 8))) / 16384.0f;
    quaternion[3] = (float)((int16_t)(buf[6] | (buf[7] << 8))) / 16384.0f;
}

void IMU::read_euler(float euler[3])
{
    if (dev_ < 0) return;

    uint8_t reg = BNO055_EULER_H_LSB_ADDR;
    uint8_t buf[6]; // 2 bytes para cada: roll, pitch e yaw

    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset;

    msgs[0].addr = address_; 
    msgs[0].flags = 0; 
    msgs[0].len = 1; 
    msgs[0].buf = &reg;

    msgs[1].addr = address_; 
    msgs[1].flags = I2C_M_RD; 
    msgs[1].len = 6; 
    msgs[1].buf = buf;
    msgset.msgs = msgs; msgset.nmsgs = 2;

    if (ioctl(dev_, I2C_RDWR, &msgset) < 0) return;

    int16_t h = (int16_t)(buf[0] | (buf[1] << 8)); // yaw
    int16_t r = (int16_t)(buf[2] | (buf[3] << 8)); // roll
    int16_t p = (int16_t)(buf[4] | (buf[5] << 8)); // pitch

    euler[0] = r / 16.0f; // roll
    euler[1] = p / 16.0f; // pitch
    euler[2] = h / 16.0f; // yaw 
}

