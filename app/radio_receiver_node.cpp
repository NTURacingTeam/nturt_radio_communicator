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

    auto radio_receiver_node =
        std::make_shared<RadioReceiver>(options);

    executor.add_node(radio_receiver_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}