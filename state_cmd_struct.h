#ifndef STATE_CMD_STRUCT_H
#define STATE_CMD_STRUCT_H
#include <stdint.h>

struct ScoutState
{
    enum MotorID
    {
        FRONT_RIGHT = 0,
        FRONT_LEFT = 1,
        REAR_LEFT = 2,
        REAR_RIGHT = 3
    };

    struct ActuatorState
    {
        double motor_current = 0;  // in A
        double motor_rpm = 0;
        uint16_t motor_pulses = 0;
        double motor_temperature = 0;

        double driver_voltage = 0;
        double driver_temperature = 0;
        uint8_t driver_state = 0;
    };

    struct LightState
    {
        uint8_t mode = 0;
        uint8_t custom_value = 0;
    };

    // base state
    uint8_t base_state = 0;
    uint8_t control_mode = 0;
    uint8_t fault_code = 0;
    double battery_voltage = 0.0;

    // motor state
    static constexpr uint8_t motor_num = 4;
    ActuatorState actuator_states[motor_num];

    // light state
    bool light_control_enabled = false;
    LightState front_light_state;
    LightState rear_light_state;

    // motion state
    double linear_velocity = 0;
    double angular_velocity = 0;

    // odometer state
    double left_odometry = 0;
    double right_odometry = 0;

    // BMS date
    uint8_t SOC;
    uint8_t SOH;
    double bms_battery_voltage = 0.0;
    double battery_current = 0.0;
    double battery_temperature = 0.0;

    // BMS state
    uint8_t Alarm_Status_1;
    uint8_t Alarm_Status_2;
    uint8_t Warning_Status_1;
    uint8_t Warning_Status_2;

};

struct ScoutLightCmd
{
    enum class LightMode
    {
        CONST_OFF = 0x00,
        CONST_ON = 0x01,
        BREATH = 0x02,
        CUSTOM = 0x03
    };

    ScoutLightCmd() = default;
    ScoutLightCmd(LightMode f_mode, uint8_t f_value, LightMode r_mode,
                  uint8_t r_value)
        : enable_ctrl(true),
          front_mode(f_mode),
          front_custom_value(f_value),
          rear_mode(r_mode),
          rear_custom_value(r_value) {}

    bool enable_ctrl = false;
    LightMode front_mode;
    uint8_t front_custom_value;
    LightMode rear_mode;
    uint8_t rear_custom_value;
};

struct ScoutMotionCmd
{
    enum class FaultClearFlag
    {
        NO_FAULT = 0x00,
        BAT_UNDER_VOL = 0x01,
        BAT_OVER_VOL = 0x02,
        MOTOR1_COMM = 0x03,
        MOTOR2_COMM = 0x04,
        MOTOR3_COMM = 0x05,
        MOTOR4_COMM = 0x06,
        MOTOR_DRV_OVERHEAT = 0x07,
        MOTOR_OVERCURRENT = 0x08
    };

    ScoutMotionCmd(double linear = 0.0, double angular = 0.0,
                   FaultClearFlag fault_clr_flag = FaultClearFlag::NO_FAULT)
        : linear_velocity(linear), angular_velocity(angular),
          fault_clear_flag(fault_clr_flag) {}

    double linear_velocity;
    double angular_velocity;
    double lateral_velocity;
    FaultClearFlag fault_clear_flag;

};

struct ScoutCmdLimits
{
    static constexpr double max_linear_velocity = 1.5;       // 1.5 m/s
    static constexpr double min_linear_velocity = -1.5;      // -1.5 m/s
    static constexpr double max_angular_velocity = 0.5235;   // 0.5235 rad/s
    static constexpr double min_angular_velocity = -0.5235;  // -0.5235 rad/s
};

struct ScoutMiniCmdLimits {//输出速度控制指令的限制
  static constexpr double max_linear_velocity = 3.0;       // 3.0 m/s
  static constexpr double min_linear_velocity = -3.0;      // -3.0 m/s
  static constexpr double max_angular_velocity = 2.5235;   // 2.5235 rad/s
  static constexpr double min_angular_velocity = -2.5235;  // -2.5235 rad/s
  static constexpr double max_lateral_velocity = 2.0;   // 2.0 m/s
  static constexpr double min_lateral_velocity = -2.0;  // -2.0 m/s
};

#endif