#ifndef DYNAMIXEL_DRIVER_HPP
#define DYNAMIXEL_DRIVER_HPP

#include "driver/actuator_driver.hpp"
#include "common_serial/serial_handler.hpp"

#include <vector>
#include <array>
#include <memory>

class SerialHandler;

class DynamixelDriver : public IActuatorDriver
{
    public:
        // TODO: mudar pra enum class
        typedef enum
        {
            PREAMBLE_POS            = 0,
            ID_POS                  = 2,
            LENGTH_POS              = 3,
            INSTRUCTION_POS         = 4,
            ERROR_POS               = 4,
            PARAMETER_POS           = 5,

            MIN_PAYLOAD_SIZE        = 5,
            RXPACKET_MAX_LEN        = (250)
        } dynamixel_packet;

        typedef enum
        {
            TORQUE_ADDR             = 0x18,
            GOAL_POS_ADDR           = 0x1E,
            CURRENT_POS_ADDR        = 0x24
        } dynamixel_protocol_addresses;

        typedef enum
        {
            READ_INSTR              = 0x02,
            WRITE_INSTR             = 0x03
        } dynamixel_protocol_instructions;

        explicit DynamixelDriver();
        virtual ~DynamixelDriver() = default;

        int init(std::string device, int baudrate) override;
        int set_torque(uint8_t id, uint8_t enable_torque) override;
        int set_goal_position(uint8_t id, uint16_t goal_position) override;
        int get_current_position(uint8_t id, uint16_t& current_position) override;
    
    private:
        struct StatusPacket
        {
            uint8_t id;
            uint8_t error;
            std::array<uint8_t, RXPACKET_MAX_LEN> params;
        };

        std::vector<uint8_t> get_packet(
            uint8_t id, uint8_t instr, const std::vector<uint8_t>& params);
        int read_packet(std::array<uint8_t, RXPACKET_MAX_LEN>& packet);
        int read_status(uint8_t id, StatusPacket& out);

        std::unique_ptr<SerialHandler> transport_;
        
};

#endif // DYNAMIXEL_DRIVER_HPP
