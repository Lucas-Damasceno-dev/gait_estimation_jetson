#include "pressure_time_node.hpp"

PressureTimeNode::PressureTimeNode() : Node("pressure_time_node")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
          .best_effort()
          .durability_volatile();
    sub_ = this->create_subscription<interfaces::msg::Time>(
        "/pressure/time", qos,
        std::bind(&PressureTimeNode::time_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "PressureTimeNode iniciado");
}

void PressureTimeNode::time_callback(const interfaces::msg::Time::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(),
                "[Pressure] Total time: %ld us | Names: %lu",
                msg->total_time, msg->names.size());

    // --- GRÁFICOS ---
    // tempo total
    plotter_.add_data("pressure_total_time", msg_counter_, static_cast<double>(msg->total_time));
    // tempos individuais
    for (size_t idx = 0; idx < msg->times.size(); idx++) 
        plotter_.add_data("pressure_time_" + msg->names[idx], msg_counter_, static_cast<double>(msg->times[idx]));
    // tempo esperado (update_rate)
    // TODO: enviar pela msg
    //plotter_.add_data("pressure_update_rate", msg_counter_, static_cast<double>(15000.0)); // 15ms
    plotter_.plot();

    // --- CSV ---
    CsvRow row;
    for (double t : msg->times)
        row.values.push_back(static_cast<double>(t));
    row.values.push_back(static_cast<double>(msg->total_time)); // última coluna = total
    csv_writer_.add_row(row);

    // cabeçalho dinâmico
    std::vector<std::string> header;
    for (size_t idx = 0; idx < msg->times.size(); idx++)
        header.push_back(msg->names[idx]);
    header.push_back("TotalTime");

    csv_writer_.save("pressure_times.csv", header);

    msg_counter_++;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PressureTimeNode>());
    rclcpp::shutdown();
    return 0;
}
