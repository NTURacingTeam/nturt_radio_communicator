/**
 * @file radio_communicator.hpp
 * @author Majorly by Jack b10502016@ntu.edu.tw, Minorly by Chris b10902069@ntu.edu.tw
 * @brief ROS2 package having two executable, one is a sender who gets data from CAN sensor and GPS sensor
 * then sends it to the receiver, the other is the receiver.
 */

#ifndef NTURT_RADIO_COMMUNICATOR_HPP
#define NTURT_RADIO_COMMUNICATOR_HPP

// glibc include
#include <stdint.h>
#include <string.h>
// tty control include
#include <errno.h>
#include <fcntl.h> 
#include <termios.h>
#include <unistd.h>

// std include
#include <array>
#include <memory>
#include <sstream>
#include <string>

// ros2 include
#include <rclcpp/rclcpp.hpp>

// ros2 message include
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// nturt include
#include "nturt_can_config.h"
#include "nturt_can_config/battery_utils.hpp"
#include "nturt_can_config_logger-binutil.h"
#include "nturt_ros_interface/msg/system_stats.hpp"

// define the tty device
#define TERMINAL "/dev/ttyUSB0"
#define BAUDRATE B115200

/**
 * @author Jack b10502016@ntu.edu.tw, modified by Chris b10902069@ntu.edu.tw
 * @brief Class for sending data to the receiver
 */
class RadioSender : public rclcpp::Node {
    public:
        /// @brief Constructor for RadioSender  
        RadioSender(rclcpp::NodeOptions options);

        /// @brief Register codedbc callback function for can signal
        void register_can_callback();

    private:
        /// @brief Callback function when receiving message from "/from_can_bus".
        void onCan(const std::shared_ptr<can_msgs::msg::Frame> msg);

        /// @brief Callback function when receiving message from "/fix".
        void onGpsFix(const std::shared_ptr<sensor_msgs::msg::NavSatFix> msg);

        /// @brief Callback function when receiving message from "/vel".
        void onGpsVel(const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg);

        /// @brief Callback function when receiving message from "/system_stats".
        void onSystemStats(
            const std::shared_ptr<nturt_ros_interface::msg::SystemStats> msg);

        /// @brief Timed callback function for periodically checking can receive
        /// timeout.
        void check_can_timer_callback();

        /// @brief Timed callback function for sending fast data to control tower.
        void send_fast_data_timer_callback();

        /// @brief Timed callback function for sending slow data to control tower.
        void send_slow_data_timer_callback();

        /* coder dbc callback function ---------------------------------------------*/
        uint32_t get_tick();

        /// @brief ROS2 sbscriber to "/from_can_bus", for receiving can signal.
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

        /// @brief ROS2 sbscriber to "/fix", for receiving GPS signal.
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_sub_;

        /// @brief ROS2 sbscriber to "/vel", for receiving GPS signal.
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
            gps_vel_sub_;

        /// @brief ROS2 sbscriber to "/system_stats", for receiving system stats
        /// information.
        rclcpp::Subscription<nturt_ros_interface::msg::SystemStats>::SharedPtr
            system_stats_sub_;

        /// @brief ROS2 timer for periodically checking can receive timeout.
        rclcpp::TimerBase::SharedPtr check_can_timer_;

        /// @brief ROS2 timer for sending fast data to control tower.
        rclcpp::TimerBase::SharedPtr send_fast_data_timer_;

        /// @brief ROS2 timer for sending slow data to control tower.
        rclcpp::TimerBase::SharedPtr send_slow_data_timer_;

        /// @brief Struct for storing can frame data.
        nturt_can_config_logger_rx_t can_rx_;

        /// @brief Struct for storing "/fix" message data.
        sensor_msgs::msg::NavSatFix gps_fix_;

        /// @brief Struct for storing "/vel" message data.
        geometry_msgs::msg::TwistStamped gps_vel_;

        /// @brief Struct for storing "/system_stats" message data.
        nturt_ros_interface::msg::SystemStats system_stats_;

        /// @brief Struct for storing battery data.
        BatteryData battery_data_;

        /// @brief protocol array sending data to control tower.
        uint32_t *protocol_fast_data_;
        uint32_t *protocol_slow_data_;

        /// @brief file descriptor for device file
        int fd_;

        /// @brief port name
        char portname_[30];

        /// @brief  slow data function counter
        int slow_data_function_counter;
};


/**
 * @author Chris b10902069@ntu.edu.tw
 * @brief Class for recieving data from sender
*/
class RadioReceiver : public rclcpp::Node {
    public:
        /// @brief Constructor for RadioReceiver
        RadioReceiver(rclcpp::NodeOptions options);
    private:
        /// @brief Callback function when receiving data from radio usb device
        void receive_data_timer_callback();        

        /// @brief ROS2 timer for periodically checking radio receive timeout
        rclcpp::TimerBase::SharedPtr receive_data_timer_;

        /// @brief Struct for storing can frame data.
        nturt_can_config_logger_rx_t can_rx_;

        /// @brief Struct for storing "/fix" message data.
        sensor_msgs::msg::NavSatFix gps_fix_;

        /// @brief Struct for storing "/vel" message data.
        geometry_msgs::msg::TwistStamped gps_vel_;

        /// @brief Struct for storing "/system_stats" message data.
        nturt_ros_interface::msg::SystemStats system_stats_;

        /// @brief Struct for storing battery data.
        BatteryData battery_data_;

        /// @brief protocol buffer array receiving data
        uint32_t *protocol_receive_data_;

        /// @brief protocol buffer array pointer, specifying the head
        int protocol_receive_data_head_;

        /// @brief file descriptor for device file
        int fd_;

        /// @brief port name
        char portname_[30];
};

#endif // NTURT_RADIO_COMMUNICATOR_HPP