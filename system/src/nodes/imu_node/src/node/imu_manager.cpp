#include "imu_manager.hpp"

#include <thread>

IMUManager::IMUManager(
    const std::shared_ptr<ParameterManager>& parameter_manager) 
:  parameter_manager_(parameter_manager) {}

int IMUManager::init(rclcpp::Node* node)
{
    if (create_imus() != 0) return -1;
    if (setup_imus() != 0) return -1;

    return 0;
}

int IMUManager::create_imus()
{
    imus_.clear();
    
    size_t max_size = parameter_manager_->get_ids().size();
    for (size_t i = 0; i < max_size; i++)
    {
        auto imu = std::make_shared<BNO055IMU>(
            parameter_manager_->get_multiplexer()[i], 
            parameter_manager_->get_ids()[i], 
            parameter_manager_->get_addresses()[i]);
        
        if (imus_.find(parameter_manager_->get_ids()[i]) != imus_.end())
        {
            RCLCPP_WARN(rclcpp::get_logger("IMUManager"), 
            "Id %d duplicado. Sobrescrevendo!", parameter_manager_->get_ids()[i]);
        }
        imus_[parameter_manager_->get_ids()[i]] = imu;
    }
    return 0;
}

int IMUManager::setup_imus()
{
    if (imus_.empty())
        return -1;

    for (const auto& [_, imu] : imus_) imu->setup();
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1ms

    for (const auto& [_, imu] : imus_) imu->calibrate_quaternions();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 1s;

    return 0;
}
