#ifndef SYNC_NODE_HPP
#define SYNC_NODE_HPP

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

// Interfaces
#include "interfaces/msg/actuator_state.hpp"
#include "interfaces/msg/pressure_state.hpp"
#include "interfaces/msg/imu_state.hpp"
#include "interfaces/msg/synced_sensor_data.hpp"
#include "interfaces/msg/mg8008_e_state.hpp"

using namespace interfaces::msg;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class SyncNode : public rclcpp::Node
{
public:
    SyncNode();
    virtual ~SyncNode();

private:
    void synced_callback(
        const IMUState::ConstSharedPtr& imu_msg,
        const PressureState::ConstSharedPtr& pressure_msg,
        const MG8008EState::ConstSharedPtr& actuator_msg);

    void watchdog_callback();

    typedef message_filters::sync_policies::ApproximateTime<
        IMUState, PressureState, MG8008EState> SyncPolicy;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    message_filters::Subscriber<IMUState> imu_sub_;
    message_filters::Subscriber<PressureState> pressure_sub_;
    message_filters::Subscriber<MG8008EState> actuator_sub_;

    rclcpp::Publisher<SyncedSensorData>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_pressure_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_actuator_time_{0, 0, RCL_ROS_TIME};

	IMUState::ConstSharedPtr last_imu_msg_;
	PressureState::ConstSharedPtr last_pressure_msg_;
	MG8008EState::ConstSharedPtr last_actuator_msg_;

    // limite de tempo pra considerar que um sensor
    // parou de enviar dados
    const rclcpp::Duration timeout_threshold_ = rclcpp::Duration::from_seconds(0.2);
};

#endif // SYNC_NODE_HPP
