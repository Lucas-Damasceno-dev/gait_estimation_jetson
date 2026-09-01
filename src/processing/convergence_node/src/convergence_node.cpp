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
    // Calcula tempo relativo
    double current_time_s = rclcpp::Time(msg->header.stamp).seconds();
    if (start_time_ms_ == 0) start_time_ms_ = current_time_s;
    double relative_time_s = static_cast<double>(current_time_s - start_time_ms_);
    
    plotter_.set_window_name(1, "Joints");
    plotter_.set_window_name(2, "IMU 1");
    plotter_.set_window_name(3, "IMU 2");
    plotter_.set_window_name(4, "IMU 3");

    // Janela 1: Juntas (usando o índice 1)
    plotter_.add_data(1, "joint_1", relative_time_s, static_cast<double>(msg->actuator_data.angles[0]));
    plotter_.add_data(1, "joint_2", relative_time_s, static_cast<double>(msg->actuator_data.angles[1]));
    plotter_.add_data(1, "joint_3", relative_time_s, static_cast<double>(msg->actuator_data.angles[2]));
    
    // Janela 2: IMU (usando o índice 2)
    plotter_.add_data(2, "q_x", relative_time_s, static_cast<double>(msg->imu_data.imus[0].q_x));
    plotter_.add_data(2, "q_y", relative_time_s, static_cast<double>(msg->imu_data.imus[0].q_y));
    plotter_.add_data(2, "q_z", relative_time_s, static_cast<double>(msg->imu_data.imus[0].q_z));
    plotter_.add_data(2, "q_w", relative_time_s, static_cast<double>(msg->imu_data.imus[0].q_w));
    
    plotter_.add_data(3, "q_x", relative_time_s, static_cast<double>(msg->imu_data.imus[1].q_x));
    plotter_.add_data(3, "q_y", relative_time_s, static_cast<double>(msg->imu_data.imus[1].q_y));
    plotter_.add_data(3, "q_z", relative_time_s, static_cast<double>(msg->imu_data.imus[1].q_z));
    plotter_.add_data(3, "q_w", relative_time_s, static_cast<double>(msg->imu_data.imus[1].q_w));
    
    plotter_.add_data(4, "q_x", relative_time_s, static_cast<double>(msg->imu_data.imus[2].q_x));
    plotter_.add_data(4, "q_y", relative_time_s, static_cast<double>(msg->imu_data.imus[2].q_y));
    plotter_.add_data(4, "q_z", relative_time_s, static_cast<double>(msg->imu_data.imus[2].q_z));
    plotter_.add_data(4, "q_w", relative_time_s, static_cast<double>(msg->imu_data.imus[2].q_w));

    plotter_.plot();
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
