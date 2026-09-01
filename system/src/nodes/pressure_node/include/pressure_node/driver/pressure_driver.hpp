#ifndef PRESSURE_DRIVER_HPP
#define PRESSURE_DRIVER_HPP

#include "common_serial/serial_handler.hpp"

#include <cstdint>
#include <string>
#include <vector>
#include <memory>

class IPressureDriver
{
    public:
        virtual ~IPressureDriver() = default;

        virtual int init(std::string device, int baudrate) = 0;
        virtual int get_data(std::vector<uint16_t>& data) = 0;
};

#endif // PRESSURE_DRIVER_HPP