#include "nturt_radio_communicator/radio_communicator.hpp"

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

RadioSender::RadioSender(rclcpp::NodeOptions options)
    : Node("nturt_radio_sender_node", options),
        can_sub_(this->create_subscription<can_msgs::msg::Frame>(
            "/from_can_bus", 50,
            std::bind(&RadioSender::onCan, this, std::placeholders::_1))
        ),
        gps_fix_sub_(this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/fix", 10,
            std::bind(&RadioSender::onGpsFix, this,
                        std::placeholders::_1))
        ),
        gps_vel_sub_(this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/vel", 10,
            std::bind(&RadioSender::onGpsVel, this,
                        std::placeholders::_1))
        ),
        system_stats_sub_(
            this->create_subscription<nturt_ros_interface::msg::SystemStats>(
                "/system_stats", 10,
                std::bind(&RadioSender::onSystemStats, this,
                            std::placeholders::_1))
        ),
        check_can_timer_(this->create_wall_timer(
            100ms,
            std::bind(&RadioSender::check_can_timer_callback, this))
        ),
        send_fast_data_timer_(this->create_wall_timer(
            200ms,
            std::bind(&RadioSender::send_fast_data_timer_callback, this))
        ),
        send_slow_data_timer_(this->create_wall_timer(
            1s,
            std::bind(&RadioSender::send_slow_data_timer_callback, this))
        ) {
    
    // init can_rx_
    memset(&can_rx_, 0, sizeof(can_rx_));
    nturt_can_config_logger_Check_Receive_Timeout_Init(&can_rx_);

    // set file descriptor for tty device
    strcpy(portname, TERMINAL);
    if((fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC)) < 0) {
        RCLCPP_ERROR(get_logger(), 
        "Error opening %s: %s\n", portname, strerror(errno));
    } else {
        RCLCPP_INFO(get_logger(), "Successfully opened the tty device %s at file descriptor %d\n", portname, fd);
    }

    // setup the tty device with correct setting
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
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

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(get_logger(), "Error from tcsetattr: %s\n", strerror(errno));
    }
};

void RadioSender::register_can_callback() {
    CanCallbackRegieter::register_callback(
        static_cast<get_tick_t>(std::bind(&RadioSender::get_tick, this)));
}

void RadioSender::onCan(
        const std::shared_ptr<can_msgs::msg::Frame> msg) {
    uint32_t id = nturt_can_config_logger_Receive(&can_rx_, msg->data.data(),
                                                    msg->id, msg->dlc);

    if (id == BMS_Cell_Stats_CANID) {
        battery_data_.update(&can_rx_.BMS_Cell_Stats);
    }
}

void RadioSender::onGpsFix(
        const std::shared_ptr<sensor_msgs::msg::NavSatFix> msg) {
    gps_fix_ = *msg;
}

void RadioSender::onGpsVel(
        const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) {
    gps_vel_ = *msg;
}

void RadioSender::onSystemStats(
        const std::shared_ptr<nturt_ros_interface::msg::SystemStats> msg) {
    system_stats_ = *msg;
}

void RadioSender::check_can_timer_callback() {
    nturt_can_config_logger_Check_Receive_Timeout(&can_rx_);
}

void RadioSender::send_fast_data_timer_callback() {
    ss_ << "{\"batch\":{";

    // can rx timeout
    uint32_t can_rx_error_node = 0;
    if (can_timeout_monior::can_rx_error & FRAME_FRONT_MASK) {
        can_rx_error_node |= 0x1;
    }
    if (can_timeout_monior::can_rx_error & FRAME_REAR_MASK) {
        can_rx_error_node |= 0x2;
    }
    if (can_timeout_monior::can_rx_error & FRAME_BMS_MASK) {
        can_rx_error_node |= 0x4;
    }
    if (can_timeout_monior::can_rx_error & FRAME_INVERTER_MASK) {
        can_rx_error_node |= 0x8;
    }
    if (can_timeout_monior::can_rx_error & FRAME_IMU_MASK) {
        can_rx_error_node |= 0x10;
    }

    ss_ << "\"can_rx_timeout\":" << can_rx_error_node;

    // vcu_status
    VCU_Status_t* vcu_status = &can_rx_.VCU_Status;
    ss_ << ",\"vcu_status\":" << static_cast<int>(vcu_status->VCU_Status)
        << ",\"vcu_error_code\":" << vcu_status->VCU_Error_Code;

    // front_sensor_1
    FRONT_SENSOR_1_t* front_sensor_1 = &can_rx_.FRONT_SENSOR_1;
    ss_ << ",\"brake\":" << front_sensor_1->FRONT_SENSOR_Brake_phys
        << ",\"accelerator_1\":"
        << front_sensor_1->FRONT_SENSOR_Accelerator_1_phys
        << ",\"accelerator_2\":"
        << front_sensor_1->FRONT_SENSOR_Accelerator_2_phys
        << ",\"steer_angle\":" << front_sensor_1->FRONT_SENSOR_Steer_Angle
        << ",\"brake_micro\":"
        << static_cast<int>(front_sensor_1->FRONT_SENSOR_Brake_Micro)
        << ",\"accelerator_micro\":"
        << static_cast<int>(front_sensor_1->FRONT_SENSOR_Accelerator_Micro);

    // front_sensor_2
    FRONT_SENSOR_2_t* front_sensor_2 = &can_rx_.FRONT_SENSOR_2;
    ss_ << ",\"front_left_wheel_speed\":"
        << front_sensor_2->FRONT_SENSOR_Front_Left_Wheel_Speed_phys
        << ",\"front_right_wheel_speed\":"
        << front_sensor_2->FRONT_SENSOR_Front_Right_Wheel_Speed_phys
        << ",\"front_brake_pressure\":"
        << front_sensor_2->FRONT_SENSOR_Front_Brake_Pressure_phys
        << ",\"rear_brake_pressure\":"
        << front_sensor_2->FRONT_SENSOR_Rear_Brake_Pressure_phys
        << ",\"front_left_suspension\":"
        << front_sensor_2->FRONT_SENSOR_Front_Left_Suspension_phys
        << ",\"front_right_suspension\":"
        << front_sensor_2->FRONT_SENSOR_Front_Right_Suspension_phys;

    // front_sensor_3
    FRONT_SENSOR_3_t* front_sensor_3 = &can_rx_.FRONT_SENSOR_3;
    ss_ << ",\"front_left_tire_temperature_1\":"
        << front_sensor_3->FRONT_SENSOR_Front_Left_Tire_Temperature_1_phys
        << ",\"front_left_tire_temperature_2\":"
        << front_sensor_3->FRONT_SENSOR_Front_Left_Tire_Temperature_2_phys
        << ",\"front_left_tire_temperature_3\":"
        << front_sensor_3->FRONT_SENSOR_Front_Left_Tire_Temperature_3_phys
        << ",\"front_left_tire_temperature_4\":"
        << front_sensor_3->FRONT_SENSOR_Front_Left_Tire_Temperature_4_phys
        << ",\"front_right_tire_temperature_1\":"
        << front_sensor_3->FRONT_SENSOR_Front_Right_Tire_Temperature_1_phys
        << ",\"front_right_tire_temperature_2\":"
        << front_sensor_3->FRONT_SENSOR_Front_Right_Tire_Temperature_2_phys
        << ",\"front_right_tire_temperature_3\":"
        << front_sensor_3->FRONT_SENSOR_Front_Right_Tire_Temperature_3_phys
        << ",\"front_right_tire_temperature_4\":"
        << front_sensor_3->FRONT_SENSOR_Front_Right_Tire_Temperature_4_phys;

    // rear_sensor_1
    REAR_SENSOR_1_t* rear_sensor_1 = &can_rx_.REAR_SENSOR_1;
    ss_ << ",\"rear_left_wheel_speed\":"
        << rear_sensor_1->REAR_SENSOR_Rear_Left_Wheel_Speed_phys
        << ",\"rear_right_wheel_speed\":"
        << rear_sensor_1->REAR_SENSOR_Rear_Right_Wheel_Speed_phys
        << ",\"rear_left_suspension\":"
        << rear_sensor_1->REAR_SENSOR_Rear_Left_Suspension_phys
        << ",\"rear_right_suspension\":"
        << rear_sensor_1->REAR_SENSOR_Rear_Right_Suspension_phys;

    // rear_sensor_2
    REAR_SENSOR_2_t* rear_sensor_2 = &can_rx_.REAR_SENSOR_2;
    ss_ << ",\"rear_left_tire_temperature_1\":"
        << rear_sensor_2->REAR_SENSOR_Rear_Left_Tire_Temperature_1_phys
        << ",\"rear_left_tire_temperature_2\":"
        << rear_sensor_2->REAR_SENSOR_Rear_Left_Tire_Temperature_2_phys
        << ",\"rear_left_tire_temperature_3\":"
        << rear_sensor_2->REAR_SENSOR_Rear_Left_Tire_Temperature_3_phys
        << ",\"rear_left_tire_temperature_4\":"
        << rear_sensor_2->REAR_SENSOR_Rear_Left_Tire_Temperature_4_phys
        << ",\"rear_right_tire_temperature_1\":"
        << rear_sensor_2->REAR_SENSOR_Rear_Right_Tire_Temperature_1_phys
        << ",\"rear_right_tire_temperature_2\":"
        << rear_sensor_2->REAR_SENSOR_Rear_Right_Tire_Temperature_2_phys
        << ",\"rear_right_tire_temperature_3\":"
        << rear_sensor_2->REAR_SENSOR_Rear_Right_Tire_Temperature_3_phys
        << ",\"rear_right_tire_temperature_4\":"
        << rear_sensor_2->REAR_SENSOR_Rear_Right_Tire_Temperature_4_phys;

    // rear_sensor_status
    REAR_SENSOR_Status_t* rear_sensor_status = &can_rx_.REAR_SENSOR_Status;
    ss_ << ",\"rear_sensor_status\":"
        << static_cast<int>(rear_sensor_status->REAR_SENSOR_Status)
        << ",\"rear_sensor_error_code\":"
        << rear_sensor_status->REAR_SENSOR_Error_Code;

    // bms_status
    ss_ << ",\"bms_error_code\":"
        << static_cast<int>(can_rx_.BMS_Status.BMS_Error_Code);

    // battery info
    double voltage = battery_data_.average_voltage();
    double current = can_rx_.INV_Current_Info.INV_DC_Bus_Current_phys;
    ss_ << ",\"state_of_charge\":" << state_of_charge(voltage, current);

    // inverter_temperature
    ss_ << ",\"inverter_control_board_temperature\":"
        << can_rx_.INV_Temperature_Set_2.INV_Control_Board_Temp_phys
        << ",\"inverter_hot_spot_temperature\":"
        << can_rx_.INV_Temperature_Set_3.INV_Hot_Spot_Temp_phys
        << ",\"motor_temperature\":"
        << can_rx_.INV_Temperature_Set_3.INV_Motor_Temp_phys;

    // inverter_fault_codes
    INV_Fault_Codes_t* inverter_fault_codes = &can_rx_.INV_Fault_Codes;
    ss_ << ",\"inverter_post_fault_lo\":"
        << inverter_fault_codes->INV_Post_Fault_Lo
        << ",\"inverter_post_fault_hi\":"
        << inverter_fault_codes->INV_Post_Fault_Hi
        << ",\"inverter_run_fault_lo\":" << inverter_fault_codes->INV_Run_Fault_Lo
        << ",\"inverter_run_fault_hi\":"
        << inverter_fault_codes->INV_Run_Fault_Hi;

    // inverter_fast_info
    INV_Fast_Info_t* inverter_fast_info = &can_rx_.INV_Fast_Info;
    ss_ << ",\"torque_command\":"
        << inverter_fast_info->INV_Fast_Torque_Command_phys
        << ",\"torque_feedback\":"
        << inverter_fast_info->INV_Fast_Torque_Feedback_phys
        << ",\"motor_speed\":" << inverter_fast_info->INV_Fast_Motor_Speed
        << ",\"inverter_dc_bus_voltage\":"
        << inverter_fast_info->INV_Fast_DC_Bus_Voltage_phys;

    // inverter other
    ss_ << ",\"inverter_vsm_state\":"
        << static_cast<int>(can_rx_.INV_Internal_States.INV_VSM_State)
        << ",\"inverter_state\":"
        << static_cast<int>(can_rx_.INV_Internal_States.INV_Inverter_State)
        << ",\"inverter_dc_bus_current\":"
        << can_rx_.INV_Current_Info.INV_DC_Bus_Current_phys;

    // imu_acceleration
    IMU_Acceleration_t* imu_acceleration = &can_rx_.IMU_Acceleration;
    ss_ << ",\"imu_acceleration_x\":" << imu_acceleration->IMU_Acceleration_X_phys
        << ",\"imu_acceleration_y\":" << imu_acceleration->IMU_Acceleration_Y_phys
        << ",\"imu_acceleration_z\":"
        << imu_acceleration->IMU_Acceleration_Z_phys;

    // imu_angular_velocity
    IMU_Angular_Velocity_t* imu_angular_velocity = &can_rx_.IMU_Angular_Velocity;
    ss_ << ",\"imu_angular_velocity_x\":"
        << imu_angular_velocity->IMU_Angular_Velocity_X_phys
        << ",\"imu_angular_velocity_y\":"
        << imu_angular_velocity->IMU_Angular_Velocity_Y_phys
        << ",\"imu_angular_velocity_z\":"
        << imu_angular_velocity->IMU_Angular_Velocity_Z_phys;

    // imu_quaternion
    IMU_Quaternion_t* imu_quaternion = &can_rx_.IMU_Quaternion;
    ss_ << ",\"imu_quaternion_w\":" << imu_quaternion->IMU_Quaternion_W_phys
        << ",\"imu_quaternion_x\":" << imu_quaternion->IMU_Quaternion_X_phys
        << ",\"imu_quaternion_y\":" << imu_quaternion->IMU_Quaternion_Y_phys
        << ",\"imu_quaternion_z\":" << imu_quaternion->IMU_Quaternion_Z_phys;

    // gps_fix
    if (gps_fix_.status.status == 0) {
        ss_ << ",\"gps_fix_longitude\":null"
            << ",\"gps_fix_latitude\":null"
            << ",\"gps_fix_altitude\":null";
    } else {
        ss_ << ",\"gps_fix_longitude\":" << gps_fix_.longitude
            << ",\"gps_fix_latitude\":" << gps_fix_.latitude
            << ",\"gps_fix_altitude\":" << gps_fix_.altitude;
    }

    // gps_vel
    ss_ << ",\"gps_vel_linear_x\":"
        << (std::isnan(gps_vel_.twist.linear.x) ? 0 : gps_vel_.twist.linear.x)
        << ",\"gps_vel_linear_y\":"
        << (std::isnan(gps_vel_.twist.linear.y) ? 0 : gps_vel_.twist.linear.y);

    // system stats
    ss_ << ",\"cpu_usage\":" << system_stats_.cpu_usage
        << ",\"memory_usage\":" << system_stats_.memory_usage
        << ",\"swap_usage\":" << system_stats_.swap_usage
        << ",\"disk_usage\":" << system_stats_.disk_usage
        << ",\"cpu_temperature\":" << system_stats_.cpu_temperature;

    ss_ << "},\"timestamp\":" << std::fixed << get_clock()->now().seconds()
        << std::defaultfloat << "}";

    RCLCPP_DEBUG(get_logger(), "Sending data to control tower: %s",
                ss_.str().c_str());

    // sender send to reciever
    const char *ss_c_str = ss_.str().c_str();
    int msg_len = strlen(ss_c_str);
    int written_len;
    if((written_len = write(fd, ss_c_str, msg_len)) != msg_len) {
        RCLCPP_ERROR(get_logger(), 
            "Error from write: expected written len:%d, written len:%d\n", 
            msg_len,
            written_len
        );
    }
    tcdrain(fd); /*delay for output*/

    // clear stringstream
    ss_.clear();
    ss_.str("");
};

void RadioSender::send_slow_data_timer_callback() {
    ss_ << "{\"accumulator\":{";

    // battery cell voltage
    ss_ << "\"accumulator_voltage\":[";
    for (int i = 0; i < NUM_BATTERY_SEGMENT; i++) {
        ss_ << "[";
        for (int j = 0; j < NUM_BATTERY_CELL_PER_SEGMENT; j++) {
            ss_ << battery_data_.voltage[i][j];
            if (j != NUM_BATTERY_CELL_PER_SEGMENT - 1) {
                ss_ << ",";
            }
        }

        ss_ << "]";
        if (i != NUM_BATTERY_SEGMENT - 1) {
            ss_ << ",";
        }
    }

    // battery cell temperature
    ss_ << "],\"accumulator_temperature\":[";
    for (int i = 0; i < NUM_BATTERY_SEGMENT; i++) {
        ss_ << "[";
        for (int j = 0; j < NUM_BATTERY_CELL_PER_SEGMENT; j++) {
            ss_ << battery_data_.temperature[i][j];
            if (j != NUM_BATTERY_CELL_PER_SEGMENT - 1) {
                ss_ << ",";
            }
        }
        
        ss_ << "]";
        if (i != NUM_BATTERY_SEGMENT - 1) {
            ss_ << ",";
        }
    }

    ss_ << "]},\"timestamp\":" << std::fixed << get_clock()->now().seconds()
        << std::defaultfloat << "}";

    RCLCPP_DEBUG(get_logger(), "Sending data to control tower: %s",
                ss_.str().c_str());

    // sender send to reciever
    const char *ss_c_str = ss_.str().c_str();
    int msg_len = strlen(ss_c_str);
    int written_len;
    if((written_len = write(fd, ss_c_str, msg_len)) != msg_len) {
        RCLCPP_ERROR(get_logger(), 
            "Error from write: expected written len:%d, written len:%d\n", 
            msg_len,
            written_len
        );
    }
    tcdrain(fd); /*delay for output*/

    // clear stringstream
    ss_.clear();
    ss_.str("");
}

uint32_t RadioSender::get_tick() {
  return static_cast<uint32_t>(now().nanoseconds() / 1000000);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(RadioSender)