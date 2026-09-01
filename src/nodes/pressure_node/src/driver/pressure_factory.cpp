#include "driver/pressure_factory.hpp"
#include "driver/insole/insole_driver.hpp"

std::unique_ptr<IPressureDriver> 
PressureFactory::create_pressure()
{
    auto controller = std::make_unique<InsoleDriver>();
    return controller;
}