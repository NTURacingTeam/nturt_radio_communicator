#include "nturt_radio_communicator/radio_communicator.hpp"
#include "nturt_radio_communicator/radio_communicator_protocol.hpp"

// glibc include
#include <stdint.h>
#include <string.h>
// tty control include
#include <errno.h>
#include <fcntl.h> 
#include <termios.h>
#include <unistd.h>

// std include
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
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
#include "nturt_can_config/can_callback_register.hpp"
#include "nturt_can_config/can_timeout_monitor.hpp"
#include "nturt_can_config_logger-binutil.h"
#include "nturt_ros_interface/msg/system_stats.hpp"

// for s, ms literal operator
using namespace std::chrono_literals;

RadioReceiver::RadioReceiver(rclcpp::NodeOptions options)
    : Node("nturt_radio_receiver_node", options),
        receive_data_timer_(this->create_wall_timer(
            200ms,
            std::bind(&RadioReceiver::receive_data_timer_callback, this))
        ) {
    // initialize the buffer head to 0
    protocol_receive_data_head_ = 0;

    // NUM_BATTERY_SEGMENT*NUM_BATTERY_CELL_PER_SEGMENT*2+2 > FAST_DATA_FORMAT_PROTOCOL_LEN
    protocol_receive_data_ = (uint64_t*)malloc(RECEIVE_DATA_BUFFER_SIZE*sizeof(uint64_t));
    protocol_receive_data_[PFAST_DATA_IDENTIFIER] = ~0UL;

    // init can_rx_
    memset(&can_rx_, 0, sizeof(can_rx_));
    nturt_can_config_logger_Check_Receive_Timeout_Init(&can_rx_);

    // set file descriptor for tty device
    strcpy(portname_, TERMINAL);
    if((fd_ = open(portname_, O_RDWR | O_NOCTTY | O_SYNC)) < 0) {
        RCLCPP_ERROR(get_logger(), 
        "Error opening %s: %s\n", portname_, strerror(errno));
    } else {
        RCLCPP_INFO(get_logger(), "Successfully opened the tty device %s at file descriptor %d\n", portname_, fd_);
    }

    // setup the tty device with correct setting
    struct termios tty;

    if (tcgetattr(fd_, &tty) < 0) {
        RCLCPP_ERROR(get_logger(), "Error from tcgetattr: %s\n", strerror(errno));
    }

    cfsetospeed(&tty, (speed_t)BAUDRATE);
    cfsetispeed(&tty, (speed_t)BAUDRATE);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(get_logger(), "Error from tcsetattr: %s\n", strerror(errno));
    }
}

void RadioReceiver::receive_data_timer_callback() {
    int rdlen = 0;   
    // read a full packet of fast data or slow data
    do{
        // read the data from fd_ to buffer
        rdlen = read(fd_, protocol_receive_data_ + protocol_receive_data_head_, 
                     (RECEIVE_DATA_BUFFER_SIZE - protocol_receive_data_head_)*sizeof(uint64_t));
        protocol_receive_data_head_ += rdlen;

    }while(!((protocol_receive_data_[PFAST_DATA_IDENTIFIER] == FAST_DATA_IDENTIFIER 
            && protocol_receive_data_head_ >= FAST_DATA_FORMAT_PROTOCOL_LEN) 
            || (protocol_receive_data_[PFAST_DATA_IDENTIFIER] == SLOW_DATA_IDENTIFIER
            && protocol_receive_data_head_ >= SLOW_DATA_FORMAT_PROTOCOL_LEN)));

    // print the data
    if(protocol_receive_data_[PFAST_DATA_IDENTIFIER] == FAST_DATA_IDENTIFIER){
        std::cout << "=======================================\n";
        for(int i=0; i<FAST_DATA_FORMAT_PROTOCOL_LEN; i++)
            std::cout << protocol_receive_data_[i];
        std::cout << "\n";
        protocol_receive_data_head_ -= FAST_DATA_FORMAT_PROTOCOL_LEN;
    }else if(protocol_receive_data_[PFAST_DATA_IDENTIFIER] == SLOW_DATA_IDENTIFIER){
        std::cout << "=======================================\n";
        for(int i=0; i<SLOW_DATA_FORMAT_PROTOCOL_LEN; i++)
            std::cout << protocol_receive_data_[i];
        std::cout << "\n";
        protocol_receive_data_head_ -= SLOW_DATA_FORMAT_PROTOCOL_LEN;
    }
}