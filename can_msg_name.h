#ifndef CAN_MSG_NAME_H
#define CAN_MSG_NAME_H
#include <can_msg_define.h>

#pragma pack(push, 1)
typedef enum  //使用枚举变量定义通信信息种类
{
    AgxMsgUnkonwn = 0x00,
    // command
    AgxMsgMotionCommand = 0x01,
    AgxMsgLightCommand = 0x02,
    AgxMsgCtrlModeSelect = 0x03,
    AgxMsgFaultByteReset = 0x04,
    AgxMsgParkModeSelect = 0x05,
    // state feedback
    AgxMsgSystemState = 0x21,
    AgxMsgMotionState = 0x22,
    AgxMsgLightState = 0x23,
    AgxMsgRcState = 0x24,
    AgxMsgActuatorHSState = 0x25,
    AgxMsgActuatorLSState = 0x26,
    AgxMsgOdometry = 0x27,
    AgxMsgVersionQuery = 0x28,
    AgxMsgPlatformVersion = 0x29,
    AgxMsgBmsDate = 0x30,
    AgxMsgBmsStatus = 0x31
} MsgType;

typedef struct
{
    MsgType type;
    union  //使用联合控制某一时间只有一个信息有值
    {
        // command
        MotionCommandMessage motion_command_msg;
        LightCommandMessage light_command_msg;
        CtrlModeSelectMessage ctrl_mode_select_msg;
        StateResetMessage state_reset_msg;
        ParkControlMessage park_control_msg;
        // state feedback
        SystemStateMessage system_state_msg;
        MotionStateMessage motion_state_msg;
        LightStateMessage light_state_msg;
        RcStateMessage rc_state_msg;
        ActuatorHSStateMessage actuator_hs_state_msg;
        ActuatorLSStateMessage actuator_ls_state_msg;
        OdometryMessage odometry_msg;
        VersionQueryMessage version_query_msg;
        PlatformVersionMessage platform_version_msg;
        BMSDateMessage bms_date_msg;
        BMSStatusMessage bms_status_msg;

    } body;
} AgxMessage;
#pragma pack(pop)

#endif