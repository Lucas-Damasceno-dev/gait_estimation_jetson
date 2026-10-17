#include "convergence_node.hpp"

ConvergenceNode::ConvergenceNode() : Node("convergence_node")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
          .best_effort()
          .durability_volatile();
    sub_ = this->create_subscription<interfaces::msg::SyncedSensorData>(
        "/sync/data", qos,
        std::bind(&ConvergenceNode::time_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "ConvergenceNode iniciado");
}

void ConvergenceNode::time_callback(const interfaces::msg::SyncedSensorData::SharedPtr msg)
{
    // Calcula tempo relativo sem truncamento
    double current_time_s = rclcpp::Time(msg->header.stamp).seconds();
    if (start_time_s_ <= 0.0) start_time_s_ = current_time_s;
    double relative_time_s = current_time_s - start_time_s_;
    
    plotter_.set_window_name(1, "Joints");
    plotter_.set_window_name(2, "IMU 1");
    plotter_.set_window_name(3, "IMU 2");
    plotter_.set_window_name(4, "IMU 3");

    // Janela 1: Juntas (com verificação de tamanho para prevenir segfault)
    const auto& angles = msg->actuator_data.angles;
    for (size_t i = 0; i < angles.size() && i < 3; ++i) {
        plotter_.add_data(1, "joint_" + std::to_string(i + 1), relative_time_s, static_cast<double>(angles[i]));
    }
    
    // Janelas 2..4: IMUs (com verificação de tamanho para prevenir segfault)
    const auto& imus = msg->imu_data.imus;
    for (size_t i = 0; i < imus.size() && i < 3; ++i) {
        int window_idx = static_cast<int>(i) + 2;
        plotter_.add_data(window_idx, "q_x", relative_time_s, static_cast<double>(imus[i].q_x));
        plotter_.add_data(window_idx, "q_y", relative_time_s, static_cast<double>(imus[i].q_y));
        plotter_.add_data(window_idx, "q_z", relative_time_s, static_cast<double>(imus[i].q_z));
        plotter_.add_data(window_idx, "q_w", relative_time_s, static_cast<double>(imus[i].q_w));
    }

    msg_counter_++;
    if (msg_counter_ % 10 == 0) {
        plotter_.plot();
    }
}

ConvergenceNode::~ConvergenceNode()
{
    plotter_.show(); // exibe o gráfico após o fim da execução
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConvergenceNode>());
    rclcpp::shutdown();
    return 0;
}
