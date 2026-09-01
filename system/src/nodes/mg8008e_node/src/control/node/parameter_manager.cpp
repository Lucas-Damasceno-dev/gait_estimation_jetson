#include "control/node/parameter_manager.hpp"

#include "rclcpp/rclcpp.hpp"

ParameterManager::ParameterManager(
    rclcpp::Node* node) : node_(node)
{
    declare_parameters();
    load_parameters();
}

void ParameterManager::declare_parameters()
{
    node_->declare_parameter("usb_port", "/dev/ttyUSB0");
    node_->declare_parameter("baudrate", 115200);
    node_->declare_parameter("ids", std::vector<int64_t>{1});
    node_->declare_parameter("names", std::vector<std::string>{"joint_1"});
    node_->declare_parameter("update_rate_ms", 15);
}

void ParameterManager::load_parameters()
{
    usb_port_ = node_->get_parameter("usb_port").as_string();
    baudrate_ = static_cast<uint32_t>(node_->get_parameter("baudrate").as_int());
    update_rate_ = node_->get_parameter("update_rate_ms").as_int();
    names_ = node_->get_parameter("names").as_string_array();
    std::vector<long> raw_ids = node_->get_parameter("ids").as_integer_array();
    
    // converte de long pra uint8_t
    ids_.clear();
    ids_.reserve(raw_ids.size()); 
    for (long id : raw_ids)
        if (id >= 0 && id <= 253) // valores suportados por uint8_t
            ids_.push_back(static_cast<uint8_t>(id)); 

    // associas nomes a ids no map
    name_map_.clear();
    for (size_t i = 0; i < names_.size() && i < ids_.size(); i++) {
        name_map_[names_[i]] = ids_[i];
    }
}
