#include "client.hpp"
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cmath>

using namespace std::chrono_literals;

ClientNode::ClientNode(const rclcpp::NodeOptions& options)
    : Node("client_node", options)
{
    this->declare_parameter("amplitude", 500);
    this->declare_parameter("period", 5);
    this->declare_parameter("offset", 400);
    this->declare_parameter("phase", -M_PI / 2.0);
    this->declare_parameter("samples", 50);
    this->declare_parameter("speed", 360);
    this->declare_parameter("loops", 2);
    this->declare_parameter("read_samples", 40);
    
    amplitude_ = this->get_parameter("amplitude").as_int();
    period_ = this->get_parameter("period").as_int();
    offset_ = this->get_parameter("offset").as_int();
    phase_ = this->get_parameter("phase").as_double();
    samples_ = this->get_parameter("samples").as_int();
    speed_ = this->get_parameter("speed").as_int();
    loops_ = this->get_parameter("loops").as_int();
    read_samples_ = this->get_parameter("read_samples").as_int();
    
    if (samples_ <= 0 || read_samples_ <= 0)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "samples deve ser > 0");

        return;
    }
    
    actuator_publisher_ = this->create_publisher<interfaces::msg::MG8008ECommand>("/actuator/command", 10);
    actuator_subscriber_ = this->create_subscription<interfaces::msg::MG8008EState>(
        "/actuator/state", 
        1, 
        std::bind(&ClientNode::get_angle, this, std::placeholders::_1));
        
    file_.open("angles.csv");
    file_ << "t,desired,real\n";

    RCLCPP_INFO(this->get_logger(), "Nó ClientNode iniciado com sucesso.");
}

void ClientNode::run(int argc, char **argv)
{
    execute_path();
}

double ClientNode::seno(int amplitude, int period, int offset, double phase, double t)
{
    double freq = 1.0 / period;
    return amplitude * std::sin(2 * M_PI * freq * t + phase) + offset;
}

void ClientNode::get_angle(const interfaces::msg::MG8008EState::SharedPtr msg)
{
        real_angle_ = msg->angles[0];
}

void ClientNode::send_angle(std::string name, int32_t angle, int32_t speed) {
    interfaces::msg::MG8008ECommand actuator_msg;
    actuator_msg.names.push_back(name);
    actuator_msg.angles.push_back(angle);
    actuator_msg.speeds.push_back(speed);
    actuator_publisher_->publish(actuator_msg);
}

// Função principal de execução
void ClientNode::execute_path()
{  
    current_t_ = 0;
    double interval = (double)period_ / samples_; // m
    RCLCPP_INFO(this->get_logger(), "Intervalo %f.", interval);
    
    // tempo pra pegar a posicao inicial de 1s
    rclcpp::sleep_for(
    	std::chrono::milliseconds(
    		static_cast<int>(1000.0)
        )
    );
    // mandando pra posiço inicial
    for (size_t i = 0; i <= samples_*loops_; i++)
    {
        current_t_ = i * interval;
        desired_angle_ = static_cast<int32_t>(seno(amplitude_, period_, offset_, phase_, current_t_));
        
        RCLCPP_INFO(this->get_logger(), "Enviando angulo %d.", desired_angle_);
        
        send_angle(
            "joint_1", 
            desired_angle_, 
            speed_
        );
        
        double read_interval = interval / read_samples_;
        for (size_t j = 0; j < read_samples_; j++)
        {
            current_read_t_ = current_t_ + j * read_interval;
            rclcpp::sleep_for(
                std::chrono::microseconds(
                    static_cast<int>(read_interval * 1000000.0)
                )
            );
            
            file_ << current_read_t_
              << ","
              << desired_angle_
              << ","
              << real_angle_
              << "\n";
        }
    }
    RCLCPP_INFO(this->get_logger(), "Trajetoria concluida");
}

ClientNode::~ClientNode()
{
    if (file_.is_open())
        file_.close();
}
