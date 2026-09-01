#ifndef PARAMETER_MANAGER_HPP
#define PARAMETER_MANAGER_HPP

#include <string>
#include <vector>
#include <cstdint>
#include <memory>
#include <unordered_map>

namespace rclcpp {
class Node;
}

class ParameterManager {
    public:
        explicit ParameterManager(rclcpp::Node* node);
        virtual ~ParameterManager() = default;

        std::string get_usb_port() { return usb_port_; }
        int get_update_rate() { return update_rate_; }
        int get_baudrate() { return baudrate_; }
        std::vector<uint8_t> get_ids() { return ids_; }
        std::vector<std::string> get_names() { return names_; }
        int get_id_by_name(std::string name) { 
            auto it = name_map_.find(name);
            if (it == name_map_.end()) return -1;
            return it->second;
        }

    private:
        void declare_parameters();
        void load_parameters();

        std::string usb_port_;
        int update_rate_;
        int baudrate_;
        std::vector<uint8_t> ids_;
        std::vector<std::string> names_;

        // TODO: criar map que mepeia nomes -> ids
        std::unordered_map<std::string, uint8_t> name_map_;

        rclcpp::Node* node_;
};

#endif // PARAMETER_MANAGER_HPP