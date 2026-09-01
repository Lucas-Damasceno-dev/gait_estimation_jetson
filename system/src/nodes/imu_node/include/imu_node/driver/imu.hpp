#ifndef IMU_HPP
#define IMU_HPP

#include <cstdint>

#define BNO055_ADDRESS_A    (0x28)
#define BNO055_ADDRESS_B    (0x29)
#define BNO055_ID           (0xA0)

class IMU
{
    public:
        typedef enum 
        {
            BNO055_CHIP_ID_ADDR                     = 0x00,
            BNO055_PAGE_ID_ADDR                     = 0x07,

            BNO055_EULER_H_LSB_ADDR                 = 0x1A,
            BNO055_EULER_H_MSB_ADDR                 = 0x1B,
            BNO055_EULER_R_LSB_ADDR                 = 0x1C,
            BNO055_EULER_R_MSB_ADDR                 = 0x1D,
            BNO055_EULER_P_LSB_ADDR                 = 0x1E,
            BNO055_EULER_P_MSB_ADDR                 = 0x1F,

            BNO055_QUATERNION_DATA_W_LSB_ADDR       = 0x20,
            BNO055_QUATERNION_DATA_W_MSB_ADDR       = 0x21,
            BNO055_QUATERNION_DATA_X_LSB_ADDR       = 0x22,
            BNO055_QUATERNION_DATA_X_MSB_ADDR       = 0x23,
            BNO055_QUATERNION_DATA_Y_LSB_ADDR       = 0x24,
            BNO055_QUATERNION_DATA_Y_MSB_ADDR       = 0x25,
            BNO055_QUATERNION_DATA_Z_LSB_ADDR       = 0x26,
            BNO055_QUATERNION_DATA_Z_MSB_ADDR       = 0x27,

            BNO055_OPR_MODE_ADDR                    = 0x3D,
            BNO055_PWR_MODE_ADDR                    = 0x3E,
            BNO055_SYS_TRIGGER_ADDR                 = 0x3F
            
        } bno055_reg_addr;

        typedef enum 
        {
            OPERATION_MODE_CONFIG                   = 0x00,
            OPERATION_MODE_NDOF                     = 0x0C
        } bno055_oprmode;

        typedef enum
        {
            POWER_MODE_NORMAL                       = 0x00
        } bno055_pwrmode;

        typedef enum
        {
            VECTOR_EULER        = BNO055_EULER_H_LSB_ADDR
        } bno055_vector_type;

        IMU(int32_t id = -1, uint8_t address = BNO055_ADDRESS_A, int shared_fd = -1);
        void set_fd(int fd) { dev_ = fd; }
        uint8_t get_address() { return address_; }
        int init_bno055();
        void read_quaternions(float quaternion[4]);
        void read_quaternions_euler(float euler[3]);
        void read_euler(float euler[3]);

    private:
        int dev_ = -1;
        int32_t id_;
        uint8_t address_;
        
};

#endif // IMU_HPP