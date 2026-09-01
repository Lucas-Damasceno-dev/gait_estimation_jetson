#ifndef CLIENT_HPP
#define CLIENT_HPP

#include <string>
#include <vector>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/actuator_state.hpp"

#include "interfaces/msg/mg8008_e_command.hpp"
#include "interfaces/msg/mg8008_e_state.hpp"

class ClientNode : public rclcpp::Node
{
    public:
        explicit ClientNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        virtual ~ClientNode();
                
        void run(int argc, char **argv);
        void execute_path(); 
        
    private:
        void send_angle(std::string name, int32_t angle, int32_t speed);
        void get_angle(const interfaces::msg::MG8008EState::SharedPtr msg);
        double seno(int amplitude, int period, int offset, double phase, double t);
        
        rclcpp::Publisher<interfaces::msg::MG8008ECommand>::SharedPtr actuator_publisher_;
        rclcpp::Subscription<interfaces::msg::MG8008EState>::SharedPtr actuator_subscriber_;
        
        int32_t amplitude_;
        int32_t period_;
        int32_t offset_;
        double phase_;
        int32_t samples_;
        int32_t speed_;
        int32_t loops_;
        int32_t read_samples_;
        
        std::ofstream file_;
        double current_t_;
        double current_read_t_;
        int32_t desired_angle_;
        
        std::atomic<int32_t> real_angle_{0};
};

#endif // CLIENT_HPP
