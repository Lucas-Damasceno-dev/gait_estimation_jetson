#include "imu_time_node.hpp"

IMUTimeNode::IMUTimeNode() : Node("imu_time_node")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
          .best_effort()
          .durability_volatile();
    sub_ = this->create_subscription<interfaces::msg::Time>(
        "/imu/time", qos,
        std::bind(&IMUTimeNode::time_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "IMUTimeNode iniciado");
}

void IMUTimeNode::time_callback(const interfaces::msg::Time::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(),
                "[IMU] Total time: %ld us | Names: %lu",
                msg->total_time, msg->names.size());

    // tempo total
    plotter_.add_data("imu_total_time", msg_counter_, static_cast<double>(msg->total_time));

    // tempos individuais
    for (size_t idx = 0; idx < msg->times.size(); idx++) 
        plotter_.add_data("imu_time_" + msg->names[idx], msg_counter_, static_cast<double>(msg->times[idx]));

    // tempo esperado (update_rate)
    // TODO: enviar pela msg
    plotter_.add_data("imu_update_rate", msg_counter_, static_cast<double>(15000.0)); // 15ms

    if (msg_counter_ % 10 == 0) {
        plotter_.plot();
    }

    // --- CSV ---
    CsvRow row;
    for (double t : msg->times)
        row.values.push_back(static_cast<double>(t));
    row.values.push_back(static_cast<double>(msg->total_time)); // última coluna = total

    // cabeçalho dinâmico
    std::vector<std::string> header;
    for (size_t idx = 0; idx < msg->times.size(); idx++)
        header.push_back(msg->names[idx]);
    header.push_back("TotalTime");

    csv_writer_.append_row("imu_times.csv", row, header);

    msg_counter_++;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUTimeNode>());
    rclcpp::shutdown();
    return 0;
}
