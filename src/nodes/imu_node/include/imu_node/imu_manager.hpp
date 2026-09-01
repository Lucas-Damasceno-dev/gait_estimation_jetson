#ifndef IMU_MANAGER_HPP
#define IMU_MANAGER_HPP

#include <map>
#include <memory>

#include "driver/bno055imu.hpp"

#include "rclcpp/rclcpp.hpp"
#include "control/node/parameter_manager.hpp"

class IMUManager
{
    public:
        IMUManager(const std::shared_ptr<ParameterManager>& parameter_manager);

        int init(rclcpp::Node* node);

        const std::map<int, std::shared_ptr<BNO055IMU>>&
        get_imus() const { return imus_; }
        
    private:
        int create_imus();
        int setup_imus();

        std::shared_ptr<ParameterManager> parameter_manager_;
        
        std::map<int, std::shared_ptr<BNO055IMU>> imus_;
};

#endif // IMU_MANAGER_HPP