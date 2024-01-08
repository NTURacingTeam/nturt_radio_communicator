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

int counter = 0;

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
            1s,
            std::bind(&RadioSender::send_fast_data_timer_callback, this))
        ) {
    
    // allocate protocol data array memory
    protocol_fast_data_ = (uint32_t*)malloc(FAST_DATA_FORMAT_PROTOCOL_LEN*sizeof(uint32_t));
    protocol_slow_data_ = (uint32_t*)malloc(SLOW_DATA_FORMAT_PROTOCOL_LEN*sizeof(uint32_t));

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

    slow_data_function_counter = 0;
    tcdrain(fd_);
}

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
    protocol_fast_data_[PFAST_DATA_IDENTIFIER] = FAST_DATA_IDENTIFIER;
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

    protocol_fast_data_[PCAN_RX_ERROR_NODE] = can_rx_error_node;

    // vcu_status
    VCU_Status_t* vcu_status = &can_rx_.VCU_Status;
    protocol_fast_data_[PVCU_STATUS] = static_cast<int>(vcu_status->VCU_Status);
    protocol_fast_data_[PVCU_ERROR_CODE] = vcu_status->VCU_Error_Code;

    // front_sensor_1
    FRONT_SENSOR_1_t* front_sensor_1 = &can_rx_.FRONT_SENSOR_1;
    protocol_fast_data_[PBRAKE] = (float_t)front_sensor_1->FRONT_SENSOR_Brake_phys;
    protocol_fast_data_[PACCELERATOR_1] = (float_t)front_sensor_1->FRONT_SENSOR_Accelerator_1_phys;
    protocol_fast_data_[PACCELERATOR_2] = (float_t)front_sensor_1->FRONT_SENSOR_Accelerator_2_phys;
    protocol_fast_data_[PSTEER_ANGLE] = front_sensor_1->FRONT_SENSOR_Steer_Angle;
    protocol_fast_data_[PBRAKE_MICRO] = static_cast<int>(front_sensor_1->FRONT_SENSOR_Brake_Micro);
    protocol_fast_data_[PACCELERATOR_MICRO] = static_cast<int>(front_sensor_1->FRONT_SENSOR_Accelerator_Micro);

    // front_sensor_2
    FRONT_SENSOR_2_t* front_sensor_2 = &can_rx_.FRONT_SENSOR_2;
    protocol_fast_data_[PFRONT_LEFT_WHEEL_SPEED] = (float_t)front_sensor_2->FRONT_SENSOR_Front_Left_Wheel_Speed_phys;
    protocol_fast_data_[PFRONT_RIGHT_WHEEL_SPEED] = (float_t)front_sensor_2->FRONT_SENSOR_Front_Right_Wheel_Speed_phys;
    protocol_fast_data_[PFRONT_BRAKE_PRESSURE] = (float_t)front_sensor_2->FRONT_SENSOR_Front_Brake_Pressure_phys;
    protocol_fast_data_[PREAR_BRAKE_PRESSURE] = (float_t)front_sensor_2->FRONT_SENSOR_Rear_Brake_Pressure_phys;
    protocol_fast_data_[PFRONT_LEFT_SUSPENSION] = (float_t)front_sensor_2->FRONT_SENSOR_Front_Left_Suspension_phys;
    protocol_fast_data_[PFRONT_RIGHT_SUSPENSION] = (float_t)front_sensor_2->FRONT_SENSOR_Front_Right_Suspension_phys;

    // front_sensor_3
    FRONT_SENSOR_3_t* front_sensor_3 = &can_rx_.FRONT_SENSOR_3;
    protocol_fast_data_[PFRONT_LEFT_TIRE_TEMPERATURE_1] = (float_t)front_sensor_3->FRONT_SENSOR_Front_Left_Tire_Temperature_1_phys;
    protocol_fast_data_[PFRONT_LEFT_TIRE_TEMPERATURE_2] = (float_t)front_sensor_3->FRONT_SENSOR_Front_Left_Tire_Temperature_2_phys;
    protocol_fast_data_[PFRONT_LEFT_TIRE_TEMPERATURE_3] = (float_t)front_sensor_3->FRONT_SENSOR_Front_Left_Tire_Temperature_3_phys;
    protocol_fast_data_[PFRONT_LEFT_TIRE_TEMPERATURE_4] = (float_t)front_sensor_3->FRONT_SENSOR_Front_Left_Tire_Temperature_4_phys;
    protocol_fast_data_[PFRONT_RIGHT_TIRE_TEMPERATURE_1] = (float_t)front_sensor_3->FRONT_SENSOR_Front_Right_Tire_Temperature_1_phys;
    protocol_fast_data_[PFRONT_RIGHT_TIRE_TEMPERATURE_2] = (float_t)front_sensor_3->FRONT_SENSOR_Front_Right_Tire_Temperature_2_phys;
    protocol_fast_data_[PFRONT_RIGHT_TIRE_TEMPERATURE_3] = (float_t)front_sensor_3->FRONT_SENSOR_Front_Right_Tire_Temperature_3_phys;
    protocol_fast_data_[PFRONT_RIGHT_TIRE_TEMPERATURE_4] = (float_t)front_sensor_3->FRONT_SENSOR_Front_Right_Tire_Temperature_4_phys;

    // rear_sensor_1
    REAR_SENSOR_1_t* rear_sensor_1 = &can_rx_.REAR_SENSOR_1;
    protocol_fast_data_[PREAR_LEFT_WHEEL_SPEED] = (float_t)rear_sensor_1->REAR_SENSOR_Rear_Left_Wheel_Speed_phys;
    protocol_fast_data_[PREAR_RIGHT_WHEEL_SPEED] = (float_t)rear_sensor_1->REAR_SENSOR_Rear_Right_Wheel_Speed_phys;
    protocol_fast_data_[PREAR_LEFT_SUSPENSION] = (float_t)rear_sensor_1->REAR_SENSOR_Rear_Left_Suspension_phys;
    protocol_fast_data_[PREAR_RIGHT_SUSPENSION] = (float_t)rear_sensor_1->REAR_SENSOR_Rear_Right_Suspension_phys;

    // rear_sensor_2
    REAR_SENSOR_2_t* rear_sensor_2 = &can_rx_.REAR_SENSOR_2;
    protocol_fast_data_[PREAR_LEFT_TIRE_TEMPERATURE_1] = (float_t)rear_sensor_2->REAR_SENSOR_Rear_Left_Tire_Temperature_1_phys;
    protocol_fast_data_[PREAR_LEFT_TIRE_TEMPERATURE_2] = (float_t)rear_sensor_2->REAR_SENSOR_Rear_Left_Tire_Temperature_2_phys;
    protocol_fast_data_[PREAR_LEFT_TIRE_TEMPERATURE_3] = (float_t)rear_sensor_2->REAR_SENSOR_Rear_Left_Tire_Temperature_3_phys;
    protocol_fast_data_[PREAR_LEFT_TIRE_TEMPERATURE_4] = (float_t)rear_sensor_2->REAR_SENSOR_Rear_Left_Tire_Temperature_4_phys;
    protocol_fast_data_[PREAR_RIGHT_TIRE_TEMPERATURE_1] = (float_t)rear_sensor_2->REAR_SENSOR_Rear_Right_Tire_Temperature_1_phys;
    protocol_fast_data_[PREAR_RIGHT_TIRE_TEMPERATURE_2] = (float_t)rear_sensor_2->REAR_SENSOR_Rear_Right_Tire_Temperature_2_phys;
    protocol_fast_data_[PREAR_RIGHT_TIRE_TEMPERATURE_3] = (float_t)rear_sensor_2->REAR_SENSOR_Rear_Right_Tire_Temperature_3_phys;
    protocol_fast_data_[PREAR_RIGHT_TIRE_TEMPERATURE_4] = (float_t)rear_sensor_2->REAR_SENSOR_Rear_Right_Tire_Temperature_4_phys;

    // rear_sensor_status
    REAR_SENSOR_Status_t* rear_sensor_status = &can_rx_.REAR_SENSOR_Status;
    protocol_fast_data_[PREAR_SENSOR_STATUS] = static_cast<int>(rear_sensor_status->REAR_SENSOR_Status);
    protocol_fast_data_[PREAR_SENSOR_ERROR_CODE] = rear_sensor_status->REAR_SENSOR_Error_Code;

    // bms_status
    protocol_fast_data_[PBMS_ERROR_CODE] = static_cast<int>(can_rx_.BMS_Status.BMS_Error_Code);

    // battery info
    double voltage = battery_data_.average_voltage();
    double current = can_rx_.INV_Current_Info.INV_DC_Bus_Current_phys;
    protocol_fast_data_[PSTATE_OF_CHARGE] = (float_t)state_of_charge(voltage, current);

    // inverter_temperature
    protocol_fast_data_[PINVERTER_CONTROL_BOARD_TEMPERATURE] = (float_t)can_rx_.INV_Temperature_Set_2.INV_Control_Board_Temp_phys;
    protocol_fast_data_[PINVERTER_HOT_SPOT_TEMPERATURE] = (float_t)can_rx_.INV_Temperature_Set_3.INV_Hot_Spot_Temp_phys;
    protocol_fast_data_[PMOTOR_TEMPERATURE] = (float_t)can_rx_.INV_Temperature_Set_3.INV_Motor_Temp_phys;

    // inverter_fault_codes
    INV_Fault_Codes_t* inverter_fault_codes = &can_rx_.INV_Fault_Codes;
    protocol_fast_data_[PINVERTER_POST_FAULT_LO] = inverter_fault_codes->INV_Post_Fault_Lo;
    protocol_fast_data_[PINVERTER_POST_FAULT_HI] = inverter_fault_codes->INV_Post_Fault_Hi;
    protocol_fast_data_[PINVERTER_RUN_FAULT_LO] = inverter_fault_codes->INV_Run_Fault_Lo;
    protocol_fast_data_[PINVERTER_RUN_FAULT_HI] = inverter_fault_codes->INV_Run_Fault_Hi;

    // inverter_fast_info
    INV_Fast_Info_t* inverter_fast_info = &can_rx_.INV_Fast_Info;
    protocol_fast_data_[PTORQUE_COMMAND] = (float_t)inverter_fast_info->INV_Fast_Torque_Command_phys;
    protocol_fast_data_[PTORQUE_FEEDBACK] = (float_t)inverter_fast_info->INV_Fast_Torque_Feedback_phys;
    protocol_fast_data_[PMOTOR_SPEED] = inverter_fast_info->INV_Fast_Motor_Speed;
    protocol_fast_data_[PINVERTER_DC_BUS_VOLTAGE] = (float_t)inverter_fast_info->INV_Fast_DC_Bus_Voltage_phys;

    // inverter other
    protocol_fast_data_[PINVERTER_VSM_STATE] = static_cast<int>(can_rx_.INV_Internal_States.INV_VSM_State);
    protocol_fast_data_[PINVERTER_STATE] = static_cast<int>(can_rx_.INV_Internal_States.INV_Inverter_State);
    protocol_fast_data_[PINVERTER_DC_BUS_CURRENT] = (float_t)can_rx_.INV_Current_Info.INV_DC_Bus_Current_phys;

    // imu_acceleration
    IMU_Acceleration_t* imu_acceleration = &can_rx_.IMU_Acceleration;
    protocol_fast_data_[PIMU_ACCELERATION_X] = (float_t)imu_acceleration->IMU_Acceleration_X_phys;
    protocol_fast_data_[PIMU_ACCELERATION_Y] = (float_t)imu_acceleration->IMU_Acceleration_Y_phys;
    protocol_fast_data_[PIMU_ACCELERATION_Z] = (float_t)imu_acceleration->IMU_Acceleration_Z_phys;

    // imu_angular_velocity
    IMU_Angular_Velocity_t* imu_angular_velocity = &can_rx_.IMU_Angular_Velocity;
    protocol_fast_data_[PIMU_ANGULAR_VELOCITY_X] = (float_t)imu_angular_velocity->IMU_Angular_Velocity_X_phys;
    protocol_fast_data_[PIMU_ANGULAR_VELOCITY_Y] = (float_t)imu_angular_velocity->IMU_Angular_Velocity_Y_phys;
    protocol_fast_data_[PIMU_ANGULAR_VELOCITY_Z] = (float_t)imu_angular_velocity->IMU_Angular_Velocity_Z_phys;

    // imu_quaternion
    IMU_Quaternion_t* imu_quaternion = &can_rx_.IMU_Quaternion;
    protocol_fast_data_[PIMU_QUATERNION_W] = (float_t)imu_quaternion->IMU_Quaternion_W_phys;
    protocol_fast_data_[PIMU_QUATERNION_X] = (float_t)imu_quaternion->IMU_Quaternion_X_phys;
    protocol_fast_data_[PIMU_QUATERNION_Y] = (float_t)imu_quaternion->IMU_Quaternion_Y_phys;
    protocol_fast_data_[PIMU_QUATERNION_Z] = (float_t)imu_quaternion->IMU_Quaternion_Z_phys;

    // gps_fix
    if (gps_fix_.status.status == 0) {
        protocol_fast_data_[PGPS_FIX_LONGTITUDE] = 0;
        protocol_fast_data_[PGPS_FIX_LATITUDE] = 0;
        protocol_fast_data_[PGPS_FIX_ALTITUDE] = 0;
    } else {
        protocol_fast_data_[PGPS_FIX_LONGTITUDE] = (float_t)gps_fix_.longitude;
        protocol_fast_data_[PGPS_FIX_LATITUDE] = (float_t)gps_fix_.latitude;
        protocol_fast_data_[PGPS_FIX_ALTITUDE] = (float_t)gps_fix_.altitude;
    }

    // gps_vel
    protocol_fast_data_[PGPS_VEL_LINEAR_X] = (std::isnan(gps_vel_.twist.linear.x) ? 0 : gps_vel_.twist.linear.x);
    protocol_fast_data_[PGPS_VEL_LINEAR_Y] = (std::isnan(gps_vel_.twist.linear.y) ? 0 : gps_vel_.twist.linear.y);

    // system stats
    protocol_fast_data_[PCPU_USAGE] = system_stats_.cpu_usage;
    protocol_fast_data_[PMEMORY_USAGE] = system_stats_.memory_usage;
    protocol_fast_data_[PSWAP_USAGE] = system_stats_.swap_usage;
    protocol_fast_data_[PDISK_USAGE] = system_stats_.disk_usage;
    protocol_fast_data_[PCPU_TEMPERATURE] = system_stats_.cpu_temperature;

    protocol_fast_data_[PTIME_STAMP] = (uint32_t)get_clock()->now().seconds();

    char fast_data_format_log_message[1024];
    for(int i=0, l=0; i<FAST_DATA_FORMAT_PROTOCOL_LEN; i++){
        sprintf(fast_data_format_log_message+l, " %u", protocol_fast_data_[i]);
        while(fast_data_format_log_message[l] != '\0')
            l++;
    }

    RCLCPP_INFO(get_logger(), "%d Sending data to control tower: %s", counter++, fast_data_format_log_message);

    // sender send to receiver
    int written_len;
    for(uint32_t unit = 0U; unit < 2U; unit ++){
        if((written_len = write(fd_, protocol_fast_data_+FAST_DATA_FORMAT_SEGMENT_ACCLEN[unit], 
                FAST_DATA_FORMAT_SEGMENT_LEN[unit])) != FAST_DATA_FORMAT_SEGMENT_LEN[unit]) {
            RCLCPP_ERROR(get_logger(), 
                "Error from write: expected written len:%d, written len:%d\n", 
                FAST_DATA_FORMAT_SEGMENT_LEN[unit],
                written_len
            );
        }
        tcdrain(fd_); /*delay for output*/
        usleep(100000);
    }

    if(++slow_data_function_counter == 5){
        send_slow_data_timer_callback();
        slow_data_function_counter = 0;
    }
};

void RadioSender::send_slow_data_timer_callback() {
    // battery cell voltage
    int slow_data_acc_len = 0;
    protocol_slow_data_[slow_data_acc_len++] = SLOW_DATA_IDENTIFIER;
    for (int i = 0; i < NUM_BATTERY_SEGMENT; i++) 
        for (int j = 0; j < NUM_BATTERY_CELL_PER_SEGMENT; j++) 
            protocol_slow_data_[slow_data_acc_len++] = (float_t)battery_data_.voltage[i][j];

    // battery cell temperature
    for (int i = 0; i < NUM_BATTERY_SEGMENT; i++) 
        for (int j = 0; j < NUM_BATTERY_CELL_PER_SEGMENT; j++) 
            protocol_slow_data_[slow_data_acc_len++] = (float_t)battery_data_.temperature[i][j];

    protocol_slow_data_[slow_data_acc_len++] = (uint32_t)get_clock()->now().seconds();

    char slow_data_format_log_message[1024];
    for(int i=0, l=0; i<slow_data_acc_len; i++){
        sprintf(slow_data_format_log_message+l, " %u", protocol_slow_data_[i]);
        while(slow_data_format_log_message[l] != '\0')
            l++;
    }
    RCLCPP_INFO(get_logger(), "%d Sending %d data to control tower: %s", counter++, slow_data_acc_len, slow_data_format_log_message);

    // sender send to receiver
    int written_len;

    for(uint32_t unit = 0U; unit < 4U; unit ++){
        if((written_len = write(fd_, protocol_slow_data_+SLOW_DATA_FORMAT_SEGMENT_ACCLEN[unit], 
                SLOW_DATA_FORMAT_SEGMENT_LEN[unit])) != SLOW_DATA_FORMAT_SEGMENT_LEN[unit]) {
            RCLCPP_ERROR(get_logger(), 
                "Error from write: expected written len:%d, written len:%d\n", 
                SLOW_DATA_FORMAT_SEGMENT_LEN[unit],
                written_len
            );
        }
        tcdrain(fd_); /*delay for output*/
        usleep(100000);
    }
}

uint32_t RadioSender::get_tick() {
  return static_cast<uint32_t>(now().nanoseconds() / 1000000);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(RadioSender)