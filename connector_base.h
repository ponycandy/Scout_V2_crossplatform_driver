#ifndef CONNECTOR_BASE_H
#define CONNECTOR_BASE_H

#include <stdint.h>
#include <can_msg_name.h>
#include <state_cmd_struct.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

struct can_frame {//定义can结构
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t data[8] ;
};

#endif

struct ScoutParams//定义一些硬件参数
{
  /* Scout Parameters */
  static constexpr double max_steer_angle = 30.0; // in degree最大转角

  static constexpr double track = 0.58306;      // in meter (left & right wheel distance)轮距
  static constexpr double wheelbase = 0.498;    // in meter (front & rear wheel distance)轴距
  static constexpr double wheel_radius = 0.165; // in meter轮半径

  // from user manual v1.2.8 P18
  // max linear velocity: 1.5 m/s
  // max angular velocity: 0.7853 rad/s
  static constexpr double max_linear_speed = 1.5;     // in m/s//最大速度
  static constexpr double max_angular_speed = 0.7853; // in rad/s//最大角速度
  static constexpr double max_speed_cmd = 10.0;       // in rad/s//
};

class Connector
{
public:

    boost::asio::io_context io_context_;
    boost::asio::ip::tcp::socket* Core_socket;
    boost::mutex snd_buffermutex;

    int m_sockfd;
    int baud_rate;
    int wait_time = 10;
    int recsize;
    int line_num=10;
    int control_period_ms {10};

    enum { max_length = 1024 };
    char data_[max_length];

    uint8_t rec_buffer0[13] {};
    uint8_t rec_buffer[8192] {};
    uint8_t snd_buffer[13] {};

    can_frame canframe;
    can_frame* can_frame_pt{ &canframe };
    AgxMessage agx_msg;
    AgxMessage* agx_msg_pt{ &agx_msg };
    ScoutState scout_state;
    ScoutState* scout_state_pt{ &scout_state };
    ScoutLightCmd scout_light_cmd;
    ScoutMotionCmd current_motion_cmd_;
    ScoutCmdLimits scout_cmd_limits;

    Connector(int baud);
    ~Connector();
    void handleRead(const boost::system::error_code& error, std::size_t bytes_transferred);
    int init(std::string Ipaddress, int port);
    // 向对端发送报文
    int  Send(const void *buf,const int buflen);
    void runIoContext();
    void Control_thread( );
    void Read_thread();
    void copy_to_buffer(const can_frame *tx_frame,uint8_t *msg);
    void copy_to_can_frame(can_frame *rx_frame,const uint8_t *msg);
    void EncodeCanFrame(const AgxMessage *msg, struct can_frame *tx_frame);
    bool DecodeCanFrame(const struct can_frame *rx_frame, AgxMessage *msg);
    void convert_data_once(const AgxMessage &status_msg,ScoutState &state);
    void  convert_msg_once();
    void SetMotionCommand(double linear_vel, double lateral_velocity, double angular_vel, ScoutMotionCmd::FaultClearFlag fault_clr_flag);
    void SendMotionCmd();
    void SetLightCommand(const ScoutLightCmd &cmd);
    void SendLightCmd(const ScoutLightCmd &lcmd, uint8_t count);
    

    boost::asio::ip::tcp::socket* internel_socket;

    
   
    
    
    


    
    
protected:

private:
};
