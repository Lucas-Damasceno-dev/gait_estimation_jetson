#include "sync_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

SyncNode::SyncNode() : Node("sync_node")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    publisher_ = this->create_publisher<SyncedSensorData>("/sync/data", qos);

    imu_sub_.subscribe(this, "imu/state", qos.get_rmw_qos_profile());
    pressure_sub_.subscribe(this, "pressure/state", qos.get_rmw_qos_profile());
    actuator_sub_.subscribe(this, "actuator/state", qos.get_rmw_qos_profile());

    imu_sub_.registerCallback([this](const IMUState::ConstSharedPtr& msg) { 
        last_imu_time_ = this->now(); 
        last_imu_msg_ = msg;
    });
    pressure_sub_.registerCallback([this](const PressureState::ConstSharedPtr& msg) { 
        last_pressure_time_ = this->now(); 
        last_pressure_msg_ = msg;
    });
    actuator_sub_.registerCallback([this](const MG8008EState::ConstSharedPtr& msg) { 
        last_actuator_time_ = this->now(); 
        last_actuator_msg_ = msg;
    });

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), imu_sub_, pressure_sub_, actuator_sub_);

    sync_->registerCallback(std::bind(&SyncNode::synced_callback, this, _1, _2, _3));

    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&SyncNode::watchdog_callback, this));

    RCLCPP_INFO(this->get_logger(), "SyncNode inicializado com cache de dados.");
}

void SyncNode::watchdog_callback()
{
    auto now = this->now();

    // verifica se nenhum sensor está publicando
    bool imu_stale = (now - last_imu_time_) > timeout_threshold_;
    bool pressure_stale = (now - last_pressure_time_) > timeout_threshold_;
    bool actuator_stale = (now - last_actuator_time_) > timeout_threshold_;

    if (imu_stale && pressure_stale && actuator_stale) {
        return; 
    }
    
    // IMU
    if (imu_stale) {
        auto dummy = last_imu_msg_ ? std::make_shared<IMUState>(*last_imu_msg_) 
                                   : std::make_shared<IMUState>();
        dummy->header.stamp = now;
        dummy->header.frame_id = "stale"; // Marcamos como dado "congelado"
        sync_->add<0>(dummy); 
    }

    // Pressure
    if (pressure_stale) {
        auto dummy = last_pressure_msg_ ? std::make_shared<PressureState>(*last_pressure_msg_) 
                                         : std::make_shared<PressureState>();
        dummy->header.stamp = now;
        dummy->header.frame_id = "stale";
        sync_->add<1>(dummy);
    }

    // Actuator
    if (actuator_stale) {
        auto dummy = last_actuator_msg_ ? std::make_shared<MG8008EState>(*last_actuator_msg_) 
                                         : std::make_shared<MG8008EState>();
        dummy->header.stamp = now;
        dummy->header.frame_id = "stale";
        sync_->add<2>(dummy);
    }
}

void SyncNode::synced_callback(
    const IMUState::ConstSharedPtr& imu_msg,
    const PressureState::ConstSharedPtr& pressure_msg,
    const MG8008EState::ConstSharedPtr& actuator_msg)
{
    auto synced_msg = std::make_unique<SyncedSensorData>();

    synced_msg->header.stamp = this->now();
    
    synced_msg->imu_data = *imu_msg;
    synced_msg->pressure_data = *pressure_msg;
    synced_msg->actuator_data = *actuator_msg;

    publisher_->publish(std::move(synced_msg));
}

SyncNode::~SyncNode() = default;
