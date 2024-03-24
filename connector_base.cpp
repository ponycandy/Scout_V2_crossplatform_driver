#include <connector_base.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <sys/types.h>
#include "boost/thread.hpp"
// #include "QDebug"
Connector::Connector(int baud):io_context_()
{
    m_sockfd=0;
    baud_rate=baud;//如500k/bit则输入500
    switch (baud)
    {
    case 500:
    {
        recsize=8192;//循环时使用
    }

    }

}



Connector::~Connector()
{
}

void Connector::handleRead(const boost::system::error_code &error, std::size_t bytes_transferred)
{
    static int counter=0;
    if (!error)
    {
        //啥都不干
        std::string command(data_);
        memset(data_, 0x00, max_length);
        Core_socket->async_read_some(boost::asio::buffer(data_, max_length),
                                     boost::bind(&Connector::handleRead, this,
                                                 boost::asio::placeholders::error,
                                                 boost::asio::placeholders::bytes_transferred));
        // std::cout<<"trigger a callback"<<counter++<<std::endl;
    }
    else {
        // Handle error
    }
}


int Connector::init(std::string Ipaddress, int port)
{
    Core_socket = new boost::asio::ip::tcp::socket(io_context_);
    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(Ipaddress), port);

    Core_socket->connect(endpoint);
    internel_socket = Core_socket;

    Core_socket->async_read_some(boost::asio::buffer(data_, max_length),
                                 boost::bind(&Connector::handleRead, this,
                                             boost::asio::placeholders::error,
                                             boost::asio::placeholders::bytes_transferred));




    unsigned char strbuffer[13] = { 0x08,0x00,0x00,0x04,0x21,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
    Send(strbuffer, 13);
    unsigned char strbuffer0[13] = { 0x08,0x00,0x00,0x01,0x21,0x01,0x01,0x05,0x01,0x05,0x00,0x00,0x00 };

    unsigned char strbuffer1[13] = { 0x08,0x00,0x00,0x01,0x21,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };

    unsigned char strbuffer2[13] = { 0x08,0x00,0x00,0x01,0x21,0x01,0x01,0x05,0x01,0x05,0x00,0x00,0x00 };

    //char strbuf1[8] {\000,\000,\000,\000,\000,\000,\000,\b};
    Send(strbuffer0, 13);
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    Send(strbuffer1, 13);
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    Send(strbuffer2, 13);
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    printf("initialization completed. \n");

    boost::thread Iothread(boost::bind(&Connector::runIoContext, this));
    boost::thread Iothread_control(boost::bind(&Connector::Control_thread, this));

    return 1;
}

int Connector::Send(const void *buf, const int buflen)
{
    boost::asio::const_buffer buffer(buf, buflen);
    // return boost::asio::write(Core_socket, buffer);
    return Core_socket->write_some(buffer);
}

void Connector::runIoContext()
{
    io_context_.run();
}



void Connector::Control_thread( )
{
    while(true)
    {
        try {

            // Try to create a const_buffer
            snd_buffermutex.lock();
            boost::asio::const_buffer buffer(snd_buffer, 13);
            internel_socket->write_some(buffer);
            snd_buffermutex.unlock();
            // Use the buffer here
            // ...
            boost::this_thread::sleep(boost::posix_time::milliseconds(20));
            //50HZ的控制更新频率

        }
        catch(const std::exception& e) {
            std::cerr << "An error occurred: " << e.what() << std::endl;
            // qDebug()<<"An error occurred: " << e.what() ;
        }
    }
}
void Connector::EncodeCanFrame(const AgxMessage *msg, struct can_frame *tx_frame)
{
    //将msg中的信息传入到要发送的can_frame中
    switch (msg->type)  //判定通信种类，，根据不同的信息种类，将msg中的不同信息传递到can_frame中，并位canid赋值
    {
    // command frame
    case AgxMsgMotionCommand:
    {
        tx_frame->can_id = CAN_MSG_MOTION_COMMAND_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.motion_command_msg.raw,
               tx_frame->can_dlc);
        break;
    }
    case AgxMsgLightCommand:
    {
        tx_frame->can_id = CAN_MSG_LIGHT_COMMAND_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.light_command_msg.raw,
               tx_frame->can_dlc);
        break;
    }
    case AgxMsgCtrlModeSelect:  //根据can通讯的种类确定canid的种类
    {
        tx_frame->can_id = CAN_MSG_CTRL_MODE_SELECT_ID;
        tx_frame->can_dlc = 8;//帧长度
        memcpy(tx_frame->data, msg->body.ctrl_mode_select_msg.raw,
               tx_frame->can_dlc);//为数组赋值，将要传递的信息录入can框架中
        break;
    }
    case AgxMsgFaultByteReset:
    {
        tx_frame->can_id = CAN_MSG_STATE_RESET_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.state_reset_msg.raw, tx_frame->can_dlc);
        break;
    }
    // state feedback frame
    case AgxMsgSystemState:
    {
        tx_frame->can_id = CAN_MSG_SYSTEM_STATE_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.system_state_msg.raw, tx_frame->can_dlc);
        break;
    }
    case AgxMsgMotionState:
    {
        tx_frame->can_id = CAN_MSG_MOTION_STATE_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.motion_state_msg.raw, tx_frame->can_dlc);
        break;
    }
    case AgxMsgLightState:
    {
        tx_frame->can_id = CAN_MSG_LIGHT_STATE_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.light_state_msg.raw, tx_frame->can_dlc);
        break;
    }
    case AgxMsgOdometry:
    {
        tx_frame->can_id = CAN_MSG_ODOMETRY_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.odometry_msg.raw, tx_frame->can_dlc);
        break;
    }
    case AgxMsgActuatorHSState:
    {
        switch (msg->body.actuator_hs_state_msg.motor_id)
        {
        case ACTUATOR1_ID:
        {
            tx_frame->can_id = CAN_MSG_ACTUATOR1_HS_STATE_ID;
            break;
        }
        case ACTUATOR2_ID:
        {
            tx_frame->can_id = CAN_MSG_ACTUATOR2_HS_STATE_ID;
            break;
        }
        case ACTUATOR3_ID:
        {
            tx_frame->can_id = CAN_MSG_ACTUATOR3_HS_STATE_ID;
            break;
        }
        case ACTUATOR4_ID:
        {
            tx_frame->can_id = CAN_MSG_ACTUATOR4_HS_STATE_ID;
            break;
        }
        }
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.actuator_hs_state_msg.data.raw,
               tx_frame->can_dlc);
        break;
    }
    case AgxMsgActuatorLSState:
    {
        switch (msg->body.actuator_ls_state_msg.motor_id)
        {
        case ACTUATOR1_ID:
        {
            tx_frame->can_id = CAN_MSG_ACTUATOR1_LS_STATE_ID;
            break;
        }
        case ACTUATOR2_ID:
        {
            tx_frame->can_id = CAN_MSG_ACTUATOR2_LS_STATE_ID;
            break;
        }
        case ACTUATOR3_ID:
        {
            tx_frame->can_id = CAN_MSG_ACTUATOR3_LS_STATE_ID;
            break;
        }
        case ACTUATOR4_ID:
        {
            tx_frame->can_id = CAN_MSG_ACTUATOR4_LS_STATE_ID;
            break;
        }
        }
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.actuator_ls_state_msg.data.raw,
               tx_frame->can_dlc);
        break;
    }
    case AgxMsgParkModeSelect:
    {
        tx_frame->can_id = CAN_MSG_PARK_COMMAND_ID;
        tx_frame->can_dlc = 8;
        memcpy(tx_frame->data, msg->body.park_control_msg.raw, tx_frame->can_dlc);
        break;
    }
    default:
        break;
    }
    //   tx_frame->data[7] =
    //       CalcCanFrameChecksum(tx_frame->can_id, tx_frame->data,
    //       tx_frame->can_dlc);
}

void Connector::EncodeCanFrameV1(const AgxMessage *msg, can_frame *tx_frame)
{
    switch (msg->type) {
    case AgxMsgMotionCommand:
    {
        static uint8_t count = 0;
        tx_frame->can_id = (uint32_t)0x130;
        tx_frame->can_dlc = 8;
        MotionCommandFrame frame;
        frame.control_mode = CTRL_MODE_CMD_CAN;
        frame.error_clear_byte = (uint8_t)0x00;
        frame.linear_percentage =
            (int8_t)(linear_percentage*100);
        frame.angular_percentage =
            (int8_t)(Omega_percentage*100);
        frame.lateral_percentage =
            (int8_t)(0);
        frame.reserved0 = 0;
        frame.count = count++;
        memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
        tx_frame->data[7] = CalcCanFrameChecksumV1(
            tx_frame->can_id, tx_frame->data, tx_frame->can_dlc);
        break;
    }

    }
    return ;
}

uint8_t Connector::CalcCanFrameChecksumV1(uint16_t id, uint8_t *data, uint8_t dlc) {
    uint8_t checksum = 0x00;
    checksum = (uint8_t)(id & 0x00ff) + (uint8_t)(id >> 8) + dlc;
    for (int i = 0; i < (dlc - 1); ++i) checksum += data[i];
    return checksum;
}





bool Connector::DecodeCanFrame(const struct can_frame *rx_frame, AgxMessage *msg)
{
    msg->type = AgxMsgUnkonwn;

    switch (rx_frame->can_id)
    {
    // command frame
    case CAN_MSG_MOTION_COMMAND_ID:
    {
        msg->type = AgxMsgMotionCommand;
        memcpy(msg->body.motion_command_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_LIGHT_COMMAND_ID:
    {
        msg->type = AgxMsgLightCommand;
        memcpy(msg->body.light_command_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_RC_STATE_ID:
    {
        msg->type = AgxMsgRcState;
        memcpy(msg->body.rc_state_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_CTRL_MODE_SELECT_ID:
    {
        msg->type = AgxMsgCtrlModeSelect;
        memcpy(msg->body.ctrl_mode_select_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_STATE_RESET_ID:
    {
        msg->type = AgxMsgFaultByteReset;
        memcpy(msg->body.state_reset_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    // state feedback frame
    case CAN_MSG_SYSTEM_STATE_ID:
    {
        msg->type = AgxMsgSystemState;
        memcpy(msg->body.system_state_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_MOTION_STATE_ID:
    {
        msg->type = AgxMsgMotionState;
        memcpy(msg->body.motion_state_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_LIGHT_STATE_ID:
    {
        msg->type = AgxMsgLightState;
        memcpy(msg->body.light_state_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_ODOMETRY_ID:
    {
        msg->type = AgxMsgOdometry;
        memcpy(msg->body.odometry_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_ACTUATOR1_HS_STATE_ID:
    case CAN_MSG_ACTUATOR2_HS_STATE_ID:
    case CAN_MSG_ACTUATOR3_HS_STATE_ID:
    case CAN_MSG_ACTUATOR4_HS_STATE_ID:
    {
        msg->type = AgxMsgActuatorHSState;
        msg->body.actuator_hs_state_msg.motor_id =
            (uint8_t)(rx_frame->can_id - CAN_MSG_ACTUATOR1_HS_STATE_ID);
        memcpy(msg->body.actuator_hs_state_msg.data.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_ACTUATOR1_LS_STATE_ID:
    case CAN_MSG_ACTUATOR2_LS_STATE_ID:
    case CAN_MSG_ACTUATOR3_LS_STATE_ID:
    case CAN_MSG_ACTUATOR4_LS_STATE_ID:
    {
        msg->type = AgxMsgActuatorLSState;
        msg->body.actuator_ls_state_msg.motor_id =
            (uint8_t)(rx_frame->can_id - CAN_MSG_ACTUATOR1_LS_STATE_ID);
        memcpy(msg->body.actuator_ls_state_msg.data.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_BMS_DATE_ID:
    {
        msg->type = AgxMsgBmsDate;
        memcpy(msg->body.bms_date_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_BMS_STATUES_ID:
    {
        msg->type = AgxMsgBmsStatus;
        memcpy(msg->body.bms_status_msg.raw, rx_frame->data,
               rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    default:
        break;
    }

    return true;
}




void Connector::copy_to_buffer(const can_frame *tx_frame,uint8_t *msg)
{
    msg[0]=tx_frame->can_dlc;
    uint32_t canid=tx_frame->can_id;
    msg[1]=(canid>>24) & 0xff;
    msg[2]=(canid>>16) & 0xff;
    msg[3]=(canid>>8) & 0xff;
    msg[4]=canid & 0xff;

    for (uint8_t i = 0; i < sizeof(tx_frame->data); i++)
    {
        msg[i + 5] = tx_frame->data[i];
    }
}





void Connector::copy_to_can_frame(can_frame *rx_frame,const uint8_t *msg)
{
    rx_frame->can_dlc=msg[0];
    rx_frame->can_id=(
        static_cast<uint32_t>(msg[1]) << 24 |
        static_cast<uint32_t>(msg[2]) << 16 |
        static_cast<uint32_t>(msg[3]) <<  8 |
        static_cast<uint32_t>(msg[4])   );

    for(uint8_t count{0};count<sizeof(rx_frame->data);++count)
        rx_frame->data[count]=msg[count+5];
}




void Connector::convert_data_once(const AgxMessage &status_msg,ScoutState &state)//将status转换为scout类型到数据
{
    switch (status_msg.type)  //根据msg的种类确定进行什么操作
    {
    case AgxMsgSystemState:  //如果回馈的是系统信息
    {
        // std::cout << "system status feedback received" << std::endl;
        const SystemStateMessage &msg = status_msg.body.system_state_msg;//定义一个msg结构继承传来的信息内容
        state.control_mode = msg.state.control_mode;//用得到的信息更新系统状态
        state.base_state = msg.state.vehicle_state;
        state.battery_voltage =
            (static_cast<uint16_t>(msg.state.battery_voltage.low_byte) |
             static_cast<uint16_t>(msg.state.battery_voltage.high_byte) << 8) /
            10.0;//把八位的两字节电池典雅信息转变为电池的实际电压
        state.fault_code = msg.state.fault_code;
        break;
    }
    case AgxMsgMotionState:  //如果回馈的是运动信息
    {
        // std::cout << "motion control feedback received" << std::endl;
        const MotionStateMessage &msg = status_msg.body.motion_state_msg;
        state.linear_velocity =//更新速度信息
            static_cast<int16_t>(
                static_cast<uint16_t>(msg.state.linear_velocity.low_byte) |
                static_cast<uint16_t>(msg.state.linear_velocity.high_byte) << 8) /
            1000.0;
        state.angular_velocity =//更新角速度信息
            static_cast<int16_t>(
                static_cast<uint16_t>(msg.state.angular_velocity.low_byte) |
                static_cast<uint16_t>(msg.state.angular_velocity.high_byte)
                    << 8) /
            1000.0;
        break;
    }
    case AgxMsgLightState:  //如果回馈的是灯的控制信息
    {
        // std::cout << "light control feedback received" << std::endl;
        const LightStateMessage &msg = status_msg.body.light_state_msg;
        if (msg.state.light_ctrl_enabled == LIGHT_CTRL_DISABLE)
            state.light_control_enabled = false;
        else
            state.light_control_enabled = true;//更新状态信息
        state.front_light_state.mode = msg.state.front_light_mode;
        state.front_light_state.custom_value = msg.state.front_light_custom;
        state.rear_light_state.mode = msg.state.rear_light_mode;
        state.rear_light_state.custom_value = msg.state.rear_light_custom;
        break;
    }
    case AgxMsgActuatorHSState:  //如果回馈的是电机的信息
    {
        // std::cout << "actuator hs feedback received" << std::endl;
        const ActuatorHSStateMessage &msg = status_msg.body.actuator_hs_state_msg;
        state.actuator_states[msg.motor_id].motor_current =//更新电机电流
            (static_cast<uint16_t>(msg.data.state.current.low_byte) |
             static_cast<uint16_t>(msg.data.state.current.high_byte) << 8) /
            10.0;
        state.actuator_states[msg.motor_id].motor_rpm = static_cast<int16_t>(//更新电机转速
            static_cast<uint16_t>(msg.data.state.rpm.low_byte) |
            static_cast<uint16_t>(msg.data.state.rpm.high_byte) << 8);
        state.actuator_states[msg.motor_id].motor_pulses = static_cast<int32_t>(
            static_cast<uint32_t>(msg.data.state.pulse_count.lsb) |
            static_cast<uint32_t>(msg.data.state.pulse_count.low_byte) << 8 |
            static_cast<uint32_t>(msg.data.state.pulse_count.high_byte) << 16 |
            static_cast<uint32_t>(msg.data.state.pulse_count.msb) << 24);
        break;
    }
    case AgxMsgActuatorLSState:  //如果回馈的是电机信息
    {
        // std::cout << "actuator ls feedback received" << std::endl;
        const ActuatorLSStateMessage &msg = status_msg.body.actuator_ls_state_msg;
        for (int i = 0; i < 2; ++i)
        {
            state.actuator_states[msg.motor_id].driver_voltage =//更新电机驱动电压
                (static_cast<uint16_t>(msg.data.state.driver_voltage.low_byte) |
                 static_cast<uint16_t>(msg.data.state.driver_voltage.high_byte)
                     << 8) /
                10.0;
            state.actuator_states[msg.motor_id]
                .driver_temperature = static_cast<int16_t>(//更新电机温度
                    static_cast<uint16_t>(msg.data.state.driver_temperature.low_byte) |
                    static_cast<uint16_t>(msg.data.state.driver_temperature.high_byte)
                        << 8);
            state.actuator_states[msg.motor_id].motor_temperature =
                msg.data.state.motor_temperature;
            state.actuator_states[msg.motor_id].driver_state =
                msg.data.state.driver_state;
        }
        break;
    }
    case AgxMsgOdometry:  //如果获取的是里程计信息
    {
        // std::cout << "Odometer msg feedback received" << std::endl;
        const OdometryMessage &msg = status_msg.body.odometry_msg;
        state.right_odometry = static_cast<int32_t>(
            (static_cast<uint32_t>(msg.state.right_wheel.lsb)) |
            (static_cast<uint32_t>(msg.state.right_wheel.low_byte) << 8) |
            (static_cast<uint32_t>(msg.state.right_wheel.high_byte) << 16) |
            (static_cast<uint32_t>(msg.state.right_wheel.msb) << 24));
        state.left_odometry = static_cast<int32_t>(
            (static_cast<uint32_t>(msg.state.left_wheel.lsb)) |
            (static_cast<uint32_t>(msg.state.left_wheel.low_byte) << 8) |
            (static_cast<uint32_t>(msg.state.left_wheel.high_byte) << 16) |
            (static_cast<uint32_t>(msg.state.left_wheel.msb) << 24));
        break;
    }
    case AgxMsgBmsDate:  //如果获取的电池信息
    {
        // std::cout << "Odometer msg feedback received" << std::endl;
        const BMSDateMessage &msg = status_msg.body.bms_date_msg;
        state.SOC = msg.state.battery_SOC;
        state.SOH = msg.state.battery_SOH;
        state.bms_battery_voltage = static_cast<int16_t>(
            (static_cast<uint16_t>(msg.state.battery_voltage.low_byte)) |
            (static_cast<uint16_t>(msg.state.battery_voltage.high_byte) << 8));
        state.battery_current = static_cast<int16_t>(
            (static_cast<uint16_t>(msg.state.battery_current.low_byte)) |
            (static_cast<uint16_t>(msg.state.battery_current.high_byte) << 8));
        state.battery_temperature = static_cast<int16_t>(
            (static_cast<uint16_t>(msg.state.battery_temperature.low_byte)) |
            (static_cast<uint16_t>(msg.state.battery_temperature.high_byte) << 8));
        break;
    }
    case AgxMsgBmsStatus:  //如果获取的使滇池的报警信息
    {
        // std::cout << "Odometer msg feedback received" << std::endl;
        const BMSStatusMessage &msg = status_msg.body.bms_status_msg;
        state.Alarm_Status_1 = msg.state.Alarm_Status_1;
        state.Alarm_Status_2 = msg.state.Alarm_Status_2;
        state.Warning_Status_1 = msg.state.Warning_Status_1;
        state.Warning_Status_2 = msg.state.Warning_Status_2;
    }

    }
}




void Connector::convert_msg_once()
{
    //status_msg.header.stamp = ros::Time::now();

    //status_msg.linear_velocity = scout_state.linear_velocity;
    //status_msg.angular_velocity = scout_state.angular_velocity;

    //status_msg.base_state = scout_state.base_state;
    //status_msg.control_mode = scout_state.control_mode;
    //status_msg.fault_code = scout_state.fault_code;
    //status_msg.battery_voltage = scout_state.battery_voltage;

    //for (int i = 0; i < 4; ++i)
    //{
    //  status_msg.motor_states[i].current = scout_state.actuator_states[i].motor_current;
    //  status_msg.motor_states[i].rpm = scout_state.actuator_states[i].motor_rpm;
    //  status_msg.motor_states[i].temperature = scout_state.actuator_states[i].motor_temperature;
    //  status_msg.motor_states[i].motor_pose = scout_state.actuator_states[i].motor_pulses;
    //  status_msg.driver_states[i].driver_state = scout_state.actuator_states[i].driver_state;
    //  status_msg.driver_states[i].driver_voltage = scout_state.actuator_states[i].driver_voltage;
    //  status_msg.driver_states[i].driver_temperature = scout_state.actuator_states[i].driver_temperature;
    //}

    //status_msg.light_control_enabled = scout_state.light_control_enabled;
    //status_msg.front_light_state.mode = scout_state.front_light_state.mode;
    //status_msg.front_light_state.custom_value =
    //    scout_state.front_light_state.custom_value;
    //status_msg.rear_light_state.mode = scout_state.rear_light_state.mode;
    //status_msg.rear_light_state.custom_value =
    //    scout_state.front_light_state.custom_value;

}



void Connector::SetMotionCommand(double linear_vel, double lateral_velocity, double angular_vel, ScoutMotionCmd::FaultClearFlag fault_clr_flag)
{
    linear_percentage=linear_vel/1.5;
    Omega_percentage=angular_vel/0.5235;
    lateral_percentage=0;
    //限制运动控制指令的指令值
    // make sure cmd thread is started before attempting to send commands
    //if (!cmd_thread_started_) StartCmdThread();//如果控制进程未开启，那么开启控制进程
#ifdef  SCOUT_V2_PROTCOL
    if (linear_vel < ScoutCmdLimits::min_linear_velocity)
        linear_vel = ScoutCmdLimits::min_linear_velocity;
    if (linear_vel > ScoutCmdLimits::max_linear_velocity)
        linear_vel = ScoutCmdLimits::max_linear_velocity;
    if (angular_vel < ScoutCmdLimits::min_angular_velocity)
        angular_vel = ScoutCmdLimits::min_angular_velocity;
    if (angular_vel > ScoutCmdLimits::max_angular_velocity)
        angular_vel = ScoutCmdLimits::max_angular_velocity;

    current_motion_cmd_.linear_velocity = linear_vel;//把限制好的数值传递给cmd
    current_motion_cmd_.angular_velocity = angular_vel;
    current_motion_cmd_.lateral_velocity = lateral_velocity;
    current_motion_cmd_.fault_clear_flag = fault_clr_flag;
#else
    if (linear_vel < ScoutMiniCmdLimits::min_linear_velocity)
        linear_vel = ScoutMiniCmdLimits::min_linear_velocity;
    if (linear_vel > ScoutMiniCmdLimits::max_linear_velocity)
        linear_vel = ScoutMiniCmdLimits::max_linear_velocity;
    if (angular_vel < ScoutMiniCmdLimits::min_angular_velocity)
        angular_vel = ScoutMiniCmdLimits::min_angular_velocity;
    if (angular_vel > ScoutMiniCmdLimits::max_angular_velocity)
        angular_vel = ScoutMiniCmdLimits::max_angular_velocity;
    if (lateral_velocity < ScoutMiniCmdLimits::min_lateral_velocity)//为各个控制指令增加限制
        lateral_velocity = ScoutMiniCmdLimits::min_lateral_velocity;
    if (lateral_velocity > ScoutMiniCmdLimits::max_lateral_velocity)
        lateral_velocity = ScoutMiniCmdLimits::max_lateral_velocity;

    current_motion_cmd_.linear_velocity = linear_vel;
    current_motion_cmd_.angular_velocity = angular_vel;
    current_motion_cmd_.lateral_velocity = lateral_velocity;
    current_motion_cmd_.fault_clear_flag = fault_clr_flag;
#endif
}




void Connector::SendMotionCmd()
{
    //定义运动控制指令发送函数
    // motion control message
    AgxMessage m_msg;//定义msg
    m_msg.type = AgxMsgMotionCommand;//选择can'通信种类为运动控制
    // memset(m_msg.body.motion_command_msg.raw, 0, 8);//初始化参信息

    //motion_cmd_mutex_.lock();//上锁保证在进行如下操作的时候相关的变量值不会被其他进程修改
    int16_t linear_cmd =
        static_cast<int16_t>(current_motion_cmd_.linear_velocity * 1000);//将线速度和角速度转变为can协议定义的数值
    int16_t angular_cmd =
        static_cast<int16_t>(current_motion_cmd_.angular_velocity * 1000);
    int16_t lateral_cmd =
        static_cast<int16_t>(current_motion_cmd_.lateral_velocity * 1000);
    //motion_cmd_mutex_.unlock();//解索

    // SendControlCmd();
    m_msg.body.motion_command_msg.cmd.linear_velocity.high_byte =//把上面得到的十六位速度信息转变为量子节的把为速度信息
        (static_cast<uint16_t>(linear_cmd) >> 8) & 0x00ff;
    m_msg.body.motion_command_msg.cmd.linear_velocity.low_byte =
        (static_cast<uint16_t>(linear_cmd) >> 0) & 0x00ff;
    m_msg.body.motion_command_msg.cmd.angular_velocity.high_byte =
        (static_cast<uint16_t>(angular_cmd) >> 8) & 0x00ff;
    m_msg.body.motion_command_msg.cmd.angular_velocity.low_byte =
        (static_cast<uint16_t>(angular_cmd) >> 0) & 0x00ff;
    m_msg.body.motion_command_msg.cmd.lateral_velocity.high_byte =
        (static_cast<uint16_t>(lateral_cmd) >> 8) & 0x00ff;
    m_msg.body.motion_command_msg.cmd.lateral_velocity.low_byte =
        (static_cast<uint16_t>(lateral_cmd) >> 0) & 0x00ff;

    // send to can bus
    can_frame m_frame;//定义can框架对象
#ifdef SCOUT_V2_PROTCOL
    EncodeCanFrame(&m_msg, &m_frame);//打包can信息
#else
    EncodeCanFrameV1(&m_msg, &m_frame);
#endif
    snd_buffermutex.lock();
    copy_to_buffer(&m_frame,snd_buffer);
    snd_buffermutex.unlock();

}





void Connector::SetLightCommand(const ScoutLightCmd &cmd)
{
    static uint8_t light_cmd_count = 0;
    SendLightCmd(cmd, light_cmd_count++);
}





void Connector::SendLightCmd(const ScoutLightCmd &lcmd, uint8_t count)
{
    //将灯的控制命令传递给msg，然后填充入can框架中，最后以流的形式发送
    AgxMessage l_msg;//定义一个msg类对象
    l_msg.type = AgxMsgLightCommand;//定义通信功能种类
    memset(l_msg.body.light_command_msg.raw, 0, 8);//初始化数组

    if (lcmd.enable_ctrl)  //如果can控制使能
    {
        l_msg.body.light_command_msg.cmd.light_ctrl_enabled = LIGHT_CTRL_ENABLE;

        l_msg.body.light_command_msg.cmd.front_light_mode =//将cmd中的控制指令传递到msg中
            static_cast<uint8_t>(lcmd.front_mode);
        l_msg.body.light_command_msg.cmd.front_light_custom =
            lcmd.front_custom_value;
        l_msg.body.light_command_msg.cmd.rear_light_mode =
            static_cast<uint8_t>(lcmd.rear_mode);
        l_msg.body.light_command_msg.cmd.rear_light_custom = lcmd.rear_custom_value;
    }
    else
    {
        l_msg.body.light_command_msg.cmd.light_ctrl_enabled = LIGHT_CTRL_DISABLE;
    }

    l_msg.body.light_command_msg.cmd.count = count;

    // send to can bus
    can_frame l_frame;//把msg填充到参框架中
#ifdef SCOUT_V2_PROTCOL
    EncodeCanFrame(&l_msg, &l_frame);//打包can信息
#else
    EncodeCanFrameV1(&l_msg, &l_frame);
#endif
    snd_buffermutex.lock();
    copy_to_buffer(&l_frame,snd_buffer);//将can信息以流的形式发送
    snd_buffermutex.unlock();

}






















