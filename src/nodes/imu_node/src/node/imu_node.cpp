#include "imu_node.hpp"

using namespace std::chrono_literals;

IMUNode::IMUNode(const rclcpp::NodeOptions & options) : Node("imu_node", options)
{
    init_driver();
    setup_node();
    
    RCLCPP_INFO(this->get_logger(), "Sucesso ao inicializar IMUNode");
}

void IMUNode::init_driver() 
{
    parameter_manager_ = std::make_shared<ParameterManager>(this);
    manager_ = std::make_unique<IMUManager>(parameter_manager_);

    if (manager_->init(this) < 0)
    {
        RCLCPP_FATAL(this->get_logger(), 
            "Falha na inicialização do hardware");
        throw std::runtime_error("Falha ao inicializar IMUNode");
    }
}

void IMUNode::setup_node() 
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();

    publisher_ = this->create_publisher<IMUState>(
        "imu/state", qos);
    time_publisher_ = this->create_publisher<Time>(
        "imu/time", qos);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(parameter_manager_->get_update_rate()), 
        [this]() {
            publish_imu_state();
        });
}

IMUState IMUNode::read_imu_data(Time& time_data)
{ 
    IMUState state_data;

    const auto& imus = manager_->get_imus();
    const auto& ids = parameter_manager_->get_ids();
    
    for (const auto& [id, imu] : imus)
    {
        std::vector<float> imu_quaternions_data;

        auto duration = measure_micros([&]() {
            imu->get_quaternions_data(imu_quaternions_data);
        });

        time_data.names.push_back(parameter_manager_->get_name(id));
        time_data.times.push_back(duration);

        // monta msg com dados do imu
        IMUData imu_data;
        imu_data.name = parameter_manager_->get_name(id);

        if (imu_quaternions_data.size() < 4) continue;

        imu_data.q_w = imu_quaternions_data[0];
        imu_data.q_x = imu_quaternions_data[1];
        imu_data.q_y = imu_quaternions_data[2];
        imu_data.q_z = imu_quaternions_data[3];
        
        state_data.imus.push_back(imu_data);
    }

    return state_data;
}

void IMUNode::publish_imu_state() 
{
    Time time_msg;
    IMUState state_msg;

    auto duration = measure_micros([&]() {
        state_msg = read_imu_data(time_msg);
    });
    
    // TODO: adicionar nomes
    if (state_msg.imus.empty()) return;

    state_msg.header.stamp = this->get_clock()->now();
    publisher_->publish(state_msg);

    time_msg.total_time = duration;
    time_publisher_->publish(time_msg);
}

IMUNode::~IMUNode() = default;
