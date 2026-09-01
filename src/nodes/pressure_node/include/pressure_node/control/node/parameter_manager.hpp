#ifndef PARAMETER_MANAGER_HPP
#define PARAMETER_MANAGER_HPP

#include <string>
#include <vector>
#include <cstdint>
#include <memory>

namespace rclcpp {
class Node;
}

class ParameterManager {
    public:
        explicit ParameterManager(rclcpp::Node* node);
        virtual ~ParameterManager() = default;

        int get_update_rate() { return update_rate_; }
        int get_baudrate() { return baudrate_; }
        std::vector<std::string> get_usb_ports() { return usb_ports_; }
        std::vector<uint8_t> get_ids() {return ids_; }
        std::vector<std::string> get_names() {return names_; }

    private:
        void declare_parameters();
        void load_parameters();

        int update_rate_;
        int baudrate_;
        std::vector<std::string> usb_ports_;
        std::vector<uint8_t> ids_;
        std::vector<std::string> names_;

        rclcpp::Node* node_;
};

#endif // PARAMETER_MANAGER_HPP