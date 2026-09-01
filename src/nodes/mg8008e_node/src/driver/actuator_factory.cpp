#include "driver/actuator_factory.hpp"
#include "driver/mg8008e/mg8008e_driver.hpp"


std::unique_ptr<IActuatorDriver> 
ActuatorFactory::createMG8008E()
{
    auto controller = std::make_unique<MG8008EDriver>();
    return controller;
}