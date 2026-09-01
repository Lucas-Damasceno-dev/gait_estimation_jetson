#ifndef CONVERGENCE_NODE_HPP
#define CONVERGENCE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/synced_sensor_data.hpp"
#include "plotter.hpp"
#include "csv_writer.hpp"

class ConvergenceNode : public rclcpp::Node
{
    public:
        ConvergenceNode();
        ~ConvergenceNode();

    private:
        void time_callback(const interfaces::msg::SyncedSensorData::SharedPtr msg);
        rclcpp::Subscription<interfaces::msg::SyncedSensorData>::SharedPtr sub_;

        CsvWriter csv_writer_;
        Plotter plotter_;

        uint64_t start_time_ms_ = 0;
};

#endif //CONVERGENCE_NODE_HPP
