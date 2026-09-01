#ifndef IMU_NODE_HPP
#define IMU_NODE_HPP

#include <memory>
#include <vector>
#include <fstream>
#include <chrono>

#include "imu_manager.hpp"
#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/imu_data.hpp"
#include "interfaces/msg/imu_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "interfaces/msg/time.hpp"

using IMUData = interfaces::msg::IMUData;
using IMUState = interfaces::msg::IMUState;
using Time = interfaces::msg::Time;

class IMUNode : public rclcpp::Node
{
    public:
        explicit IMUNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        virtual ~IMUNode();

    private:
        void init_driver();
        void setup_node();

        IMUState read_imu_data(Time& time_data);
        void publish_imu_state(); 

        template <typename Func>
        inline double measure_micros(Func&& func) {
            auto start = std::chrono::high_resolution_clock::now();
            func();
            auto end = std::chrono::high_resolution_clock::now();
            return static_cast<double>(
                std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
            );
        }

        rclcpp::Publisher<IMUState>::SharedPtr publisher_;
        rclcpp::Publisher<Time>::SharedPtr time_publisher_;

        std::shared_ptr<ParameterManager> parameter_manager_;
        std::shared_ptr<IMUManager> manager_;

        rclcpp::TimerBase::SharedPtr timer_;
};

#endif // IMU_NODE_HPP
