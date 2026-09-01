#include "control/node/parameter_manager.hpp"

#include "rclcpp/rclcpp.hpp"

#include <algorithm>

ParameterManager::ParameterManager(
    rclcpp::Node* node) : node_(node)
{
    declare_parameters();
    load_parameters();
}

void ParameterManager::declare_parameters()
{
    node_->declare_parameter("names", std::vector<std::string>{"bottom", "middle", "top"});
    node_->declare_parameter("ids", std::vector<int64_t>{3, 2, 1});
    node_->declare_parameter("multiplexer", std::vector<int64_t>{0, 1, 0});
    node_->declare_parameter("addresses", std::vector<int64_t>{40, 41, 40});
    node_->declare_parameter("update_rate_ms", 15);
}

void ParameterManager::load_parameters()
{
    update_rate_ = node_->get_parameter("update_rate_ms").as_int();

    std::vector<std::string> raw_names = node_->get_parameter("names").as_string_array();
    std::vector<long> raw_ids = node_->get_parameter("ids").as_integer_array();
    std::vector<long> raw_multiplexer = node_->get_parameter("multiplexer").as_integer_array();
    std::vector<long> raw_addresses = node_->get_parameter("addresses").as_integer_array();
    
    // evita segfault
    size_t min_size = std::min(
        raw_ids.size(), std::min(raw_addresses.size(), raw_multiplexer.size()));

    ids_.clear();
    ids_.reserve(min_size);
    multiplexer_.clear();
    multiplexer_.reserve(min_size);
    addresses_.clear();
    addresses_.reserve(min_size);

    // converte de long pra uint8_t
    for (size_t i = 0; i < min_size; i++) {
        if (raw_ids[i] >= 0 && raw_ids[i] <= 253)
            ids_.push_back(static_cast<uint8_t>(raw_ids[i]));

        if (raw_multiplexer[i] >= 0 && raw_multiplexer[i] <= 253)
            multiplexer_.push_back(static_cast<uint8_t>(raw_multiplexer[i]));

        if (raw_addresses[i] >= 0 && raw_addresses[i] <= 253)
            addresses_.push_back(static_cast<uint8_t>(raw_addresses[i]));
    }

    // associa cada nome a um id de imu
    size_t name_size = std::min(ids_.size(), raw_names.size());
    for (size_t i = 0; i < name_size; i++) {
        names_[ids_[i]] = raw_names[i];
    }

}
