#include "driver/actuator_factory.hpp"
#include "driver/dynamixel/dynamixel_driver.hpp"

std::unique_ptr<IActuatorDriver> 
ActuatorFactory::createDynamixel()
{
    auto controller = std::make_unique<DynamixelDriver>();
    return controller;
}
