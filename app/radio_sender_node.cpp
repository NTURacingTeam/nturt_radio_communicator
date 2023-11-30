// std include
#include <memory>

// ros2 include
#include "rclcpp/rclcpp.hpp"

// nturt include
#include "nturt_radio_communicator/radio_communicator.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    rclcpp::executors::StaticSingleThreadedExecutor executor;
    rclcpp::NodeOptions options;

    auto radio_sender_node =
        std::make_shared<RadioSender>(options);
    radio_sender_node->register_can_callback();

    executor.add_node(radio_sender_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}