#ifndef ACTUATOR_NODE_HPP
#define ACTUATOR_NODE_HPP

#include <memory>
#include <mutex>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "driver/actuator_factory.hpp"

#include "interfaces/msg/mg8008_e_command.hpp"
#include "interfaces/msg/mg8008_e_state.hpp"
#include "interfaces/msg/time.hpp"

class ParameterManager;

using MG8008ECommand = interfaces::msg::MG8008ECommand;
using MG8008EState = interfaces::msg::MG8008EState;
using Time = interfaces::msg::Time;

struct ActuatorData {
    int8_t id;
    std::string name;
    int32_t angle;
    int32_t speed;
};

class ActuatorNode : public rclcpp::Node
{
    public:
        explicit ActuatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~ActuatorNode();

    private:
        void init_driver();
        void setup_node();

        MG8008EState read_actuator_data(Time& time_data);
        void set_angle(const std::vector<ActuatorData>& actuator_data);

        std::vector<ActuatorData> read_angle_msg(const MG8008ECommand::SharedPtr msg);
        void publish_actuator_state();

        template <typename Func>
        inline double measure_micros(Func&& func) {
            auto start = std::chrono::high_resolution_clock::now();
            func();
            auto end = std::chrono::high_resolution_clock::now();
            return static_cast<double>(
                std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
            );
        }

        std::mutex driver_mutex_;

        std::shared_ptr<IActuatorDriver> actuator_driver_;
        std::shared_ptr<ParameterManager> parameter_manager_;
        
        rclcpp::Subscription<MG8008ECommand>::SharedPtr actuator_subscriber_;
        rclcpp::Publisher<MG8008EState>::SharedPtr state_publisher_;
        rclcpp::Publisher<Time>::SharedPtr time_publisher_;
        
        rclcpp::TimerBase::SharedPtr timer_;
};

#endif // ACTUATOR_NODE_HPP
