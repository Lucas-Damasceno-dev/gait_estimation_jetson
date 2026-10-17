#include "driver/imu.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <cmath>

IMU::IMU(int32_t id, uint8_t address, int shared_fd)
:  dev_(shared_fd), id_(id), address_(address) {}

int IMU::write_reg(uint8_t reg, uint8_t val)
{
    if (dev_ < 0) return -1;
    uint8_t buf[2] = {reg, val};
    if (write(dev_, buf, 2) != 2) return -1;
    return 0;
}

int IMU::read_reg(uint8_t reg)
{
    if (dev_ < 0) return -1;
    if (write(dev_, &reg, 1) != 1) return -1;
    uint8_t val = 0;
    if (read(dev_, &val, 1) != 1) return -1;
    return val;
}

int IMU::init_bno055()
{
    // modo configuracao
    if (write_reg(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG) < 0)
        return -1;
    usleep(30 * 1000); // 30ms

    // reset (0x20 -> comando reset)
    if (write_reg(BNO055_SYS_TRIGGER_ADDR, 0x20) < 0) 
        return -1;

    // espera até o id do chip ser válido (reset termina)
    int retry = 0;
    while (read_reg(BNO055_CHIP_ID_ADDR) != BNO055_ID)
    {
        usleep(10 * 1000); // 10ms
        if (++retry > 50) return -1; // Sai após 500ms se o sensor não responder
    }

    usleep(50 * 1000); // 50ms

    // power mode normal
    if (write_reg(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL) < 0)
        return -1;

    // página 0
    if (write_reg(BNO055_PAGE_ID_ADDR, 0) < 0)
        return -1;

    // reset terminou
    if (write_reg(BNO055_SYS_TRIGGER_ADDR, 0x00) < 0)
        return -1;
    usleep(10 * 1000);

    // modo de operação NDOF (fusão 9-DOF nativa do sensor)
    if (write_reg(BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF) < 0)
        return -1;
    usleep(50 * 1000);

    return 0;
}

int IMU::read_quaternions(float quaternion[4])
{
    if (dev_ < 0) return -1;

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
        return -1; 
    }

    quaternion[0] = (float)((int16_t)(buf[0] | (buf[1] << 8))) / 16384.0f;
    quaternion[1] = (float)((int16_t)(buf[2] | (buf[3] << 8))) / 16384.0f;
    quaternion[2] = (float)((int16_t)(buf[4] | (buf[5] << 8))) / 16384.0f;
    quaternion[3] = (float)((int16_t)(buf[6] | (buf[7] << 8))) / 16384.0f;
    return 0;
}

int IMU::read_euler(float euler[3])
{
    if (dev_ < 0) return -1;

    uint8_t reg = BNO055_EULER_H_LSB_ADDR;
    uint8_t buf[6]; // 2 bytes para cada: yaw, roll, pitch

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

    if (ioctl(dev_, I2C_RDWR, &msgset) < 0) return -1;

    int16_t h = (int16_t)(buf[0] | (buf[1] << 8)); // yaw
    int16_t r = (int16_t)(buf[2] | (buf[3] << 8)); // roll
    int16_t p = (int16_t)(buf[4] | (buf[5] << 8)); // pitch

    euler[0] = r / 16.0f; // roll
    euler[1] = p / 16.0f; // pitch
    euler[2] = h / 16.0f; // yaw 
    return 0;
}
