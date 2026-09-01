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

        int get_update_rate() { return update_rate_; }
        std::vector<uint8_t> get_ids() { return ids_; }
        std::vector<uint8_t> get_multiplexer() { return multiplexer_; }
        std::vector<uint8_t> get_addresses() { return addresses_; }

        std::string get_name(uint8_t id) {
            auto it = names_.find(id);
            if (it != names_.end())
                return it->second;
            return "";
        }


    private:
        void declare_parameters();
        void load_parameters();

        int update_rate_;
        std::vector<uint8_t> ids_;
        std::vector<uint8_t> multiplexer_;
        std::vector<uint8_t> addresses_;
        std::unordered_map<int, std::string> names_;

        rclcpp::Node* node_;
};

#endif // PARAMETER_MANAGER_HPP