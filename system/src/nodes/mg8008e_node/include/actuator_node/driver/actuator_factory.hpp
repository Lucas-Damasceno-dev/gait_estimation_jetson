#ifndef ACTUATOR_FACTORY_HPP
#define ACTUATOR_FACTORY_HPP

#include <memory>
#include <string>

#include "driver/actuator_driver.hpp"

class ActuatorFactory
{
    public:
        static std::unique_ptr<IActuatorDriver> 
        createMG8008E();
};

#endif // ACTUATOR_FACTORY_HPP