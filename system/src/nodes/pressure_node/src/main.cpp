#include "rclcpp/rclcpp.hpp"
#include "pressure_node.hpp" 

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PressureNode>());
    rclcpp::shutdown();
    return 0;
}