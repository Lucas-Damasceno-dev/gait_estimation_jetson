#ifndef PRESSURE_NODE_HPP
#define PRESSURE_NODE_HPP

#include <memory>
#include <map>
#include <vector>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/pressure_state.hpp"
#include "interfaces/msg/pressure_data.hpp"
#include "interfaces/msg/pressure_unit_sensor.hpp"
#include "interfaces/msg/time.hpp"

#include "control/node/parameter_manager.hpp"
#include "driver/pressure_driver.hpp"

using PressureState = interfaces::msg::PressureState;
using PressureData = interfaces::msg::PressureData;
using PressureUnitSensor = interfaces::msg::PressureUnitSensor;
using Time = interfaces::msg::Time;

class PressureNode : public rclcpp::Node
{
    public:
        explicit PressureNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~PressureNode();
        
    private:
        void init_driver();
        void setup_node();

        PressureState read_pressure_data(Time& time_data);
        void publish_pressure_state();

        template <typename Func>
        inline double measure_micros(Func&& func) {
            auto start = std::chrono::high_resolution_clock::now();
            func();
            auto end = std::chrono::high_resolution_clock::now();
            return static_cast<double>(
                std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
            );
        }


        std::vector<std::shared_ptr<IPressureDriver>> pressure_drivers_;
        std::shared_ptr<ParameterManager> parameter_manager_;

        rclcpp::Publisher<PressureState>::SharedPtr publisher_;
        rclcpp::Publisher<Time>::SharedPtr time_publisher_;
        
        rclcpp::TimerBase::SharedPtr timer_;
};

#endif // PRESSURE_NODE_HPP
