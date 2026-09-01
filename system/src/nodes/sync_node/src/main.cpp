#include "rclcpp/rclcpp.hpp"
#include "sync_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SyncNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}