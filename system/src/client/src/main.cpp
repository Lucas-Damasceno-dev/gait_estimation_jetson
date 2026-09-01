#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "client.hpp" 

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ClientNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread executor_thread([&executor]() {
        executor.spin();
    });

    node->run(argc, argv);

    rclcpp::shutdown();
    
    if (executor_thread.joinable()) {
        executor_thread.join();
    }
    return 0;
}
