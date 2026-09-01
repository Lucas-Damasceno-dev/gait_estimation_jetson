#ifndef PRESSURE_FACTORY_HPP
#define PRESSURE_FACTORY_HPP

#include <memory>
#include <string>

#include "driver/pressure_driver.hpp"

class PressureFactory
{
    public:
        static std::unique_ptr<IPressureDriver> 
        create_pressure();
};

#endif // PRESSURE_FACTORY_HPP