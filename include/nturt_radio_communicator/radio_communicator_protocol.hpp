/**
 * @file radio_communicator_protocol.hpp
 * @author Chris b10902069@ntu.edu.tw
 * @brief define the radio communicator protocol
 */

// define the data format
#define FAST_DATA_FORMAT_PROTOCOL_LEN 75
#define SLOW_DATA_FORMAT_PROTOCOL_LEN (NUM_BATTERY_SEGMENT*NUM_BATTERY_CELL_PER_SEGMENT*2+2)
#define FAST_DATA_IDENTIFIER 0
#define SLOW_DATA_IDENTIFIER 1

enum fast_data_format_protocol{
    PFAST_DATA_IDENTIFIER,

    // can rx error node
    PCAN_RX_ERROR_NODE,

    // vcu status
    PVCU_STATUS,
    PVCU_ERROR_CODE,

    // front sensor 1
    PBRAKE,
    PACCELERATOR_1,
    PACCELERATOR_2,
    PSTEER_ANGLE,
    PBRAKE_MICRO,
    PACCELERATOR_MICRO,
    
    // front sensor 2
    PFRONT_LEFT_WHEEL_SPEED,
    PFRONT_RIGHT_WHEEL_SPEED,
    PFRONT_BRAKE_PRESSURE,
    PREAR_BRAKE_PRESSURE,
    PFRONT_LEFT_SUSPENSION,
    PFRONT_RIGHT_SUSPENSION,

    // front sensor 3
    PFRONT_LEFT_TIRE_TEMPERATURE_1,
    PFRONT_LEFT_TIRE_TEMPERATURE_2,
    PFRONT_LEFT_TIRE_TEMPERATURE_3,
    PFRONT_LEFT_TIRE_TEMPERATURE_4,
    PFRONT_RIGHT_TIRE_TEMPERATURE_1,
    PFRONT_RIGHT_TIRE_TEMPERATURE_2,
    PFRONT_RIGHT_TIRE_TEMPERATURE_3,
    PFRONT_RIGHT_TIRE_TEMPERATURE_4,

    // rear sensor 1
    PREAR_LEFT_WHEEL_SPEED,
    PREAR_RIGHT_WHEEL_SPEED,
    PREAR_LEFT_SUSPENSION,
    PREAR_RIGHT_SUSPENSION,

    // rear sensor 2
    PREAR_LEFT_TIRE_TEMPERATURE_1,
    PREAR_LEFT_TIRE_TEMPERATURE_2,
    PREAR_LEFT_TIRE_TEMPERATURE_3,
    PREAR_LEFT_TIRE_TEMPERATURE_4,
    PREAR_RIGHT_TIRE_TEMPERATURE_1,
    PREAR_RIGHT_TIRE_TEMPERATURE_2,
    PREAR_RIGHT_TIRE_TEMPERATURE_3,
    PREAR_RIGHT_TIRE_TEMPERATURE_4,
    
    // rear sensor status
    PREAR_SENSOR_STATUS,
    PREAR_SENSOR_ERROR_CODE,

    // bms status
    PBMS_ERROR_CODE,

    // battery info
    PSTATE_OF_CHARGE,

    // inverter temperature
    PINVERTER_CONTROL_BOARD_TEMPERATURE,
    PINVERTER_HOT_SPOT_TEMPERATURE,
    PMOTOR_TEMPERATURE,

    // inverter fault codes
    PINVERTER_POST_FAULT_LO,
    PINVERTER_POST_FAULT_HI,
    PINVERTER_RUN_FAULT_LO,
    PINVERTER_RUN_FAULT_HI,

    // inverter fast info
    PTORQUE_COMMAND,
    PTORQUE_FEEDBACK,
    PMOTOR_SPEED,
    PINVERTER_DC_BUS_VOLTAGE,

    // inverter other
    PINVERTER_VSM_STATE,
    PINVERTER_STATE,
    PINVERTER_DC_BUS_CURRENT,

    // imu acceleration
    PIMU_ACCELERATION_X,
    PIMU_ACCELERATION_Y,
    PIMU_ACCELERATION_Z,

    // imu angular velocity
    PIMU_ANGULAR_VELOCITY_X,
    PIMU_ANGULAR_VELOCITY_Y,
    PIMU_ANGULAR_VELOCITY_Z,

    // imu quaternion
    PIMU_QUATERNION_W,
    PIMU_QUATERNION_X,
    PIMU_QUATERNION_Y,
    PIMU_QUATERNION_Z,
    
    // gps fix
    PGPS_FIX_LONGTITUDE,
    PGPS_FIX_LATITUDE,
    PGPS_FIX_ALTITUDE,

    // gps vel
    PGPS_VEL_LINEAR_X,
    PGPS_VEL_LINEAR_Y,

    // system status
    PCPU_USAGE,
    PMEMORY_USAGE,
    PSWAP_USAGE,
    PDISK_USAGE,
    PCPU_TEMPERATURE,

    // time stamp
    PTIME_STAMP,
}; 

// define the receive data buffer size
#define RECEIVE_DATA_BUFFER_SIZE 2048