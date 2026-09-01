#ifndef ACTUATOR_DRIVER_HPP
#define ACTUATOR_DRIVER_HPP

#include <string>
#include <cstdint>

class IActuatorDriver
{
    public:
        virtual ~IActuatorDriver() = default;

        virtual int init(std::string device, int baudrate) = 0;
        virtual int set_torque(uint8_t id, uint8_t enable_torque) = 0;
        virtual int set_goal_position(uint8_t id, uint16_t goal_position) = 0;
        virtual int get_current_position(uint8_t id, uint16_t& current_position) = 0;
};

#endif // ACTUATOR_DRIVER_HPP